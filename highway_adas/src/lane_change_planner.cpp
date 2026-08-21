#include "highway_adas/lane_change_planner.hpp"

#include "highway_adas/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace highway_adas {
namespace {

double longitudinalSpeed(const TrackedObject& object, double heading_rad) {
  return object.velocity_map_mps.x * std::cos(heading_rad) +
         object.velocity_map_mps.y * std::sin(heading_rad);
}

double smoothStepQuintic(double ratio) {
  const double u = clamp(ratio, 0.0, 1.0);
  return 10.0 * u * u * u - 15.0 * u * u * u * u +
         6.0 * u * u * u * u * u;
}

double requiredDeceleration(double closing_speed_mps,
                            double gap_m,
                            double minimum_gap_m) {
  if (closing_speed_mps <= 0.0) return 0.0;
  const double braking_room = std::max(0.1, gap_m - minimum_gap_m);
  return closing_speed_mps * closing_speed_mps / (2.0 * braking_room);
}

}  // namespace

void LaneChangePlanner::reset() {
  state_ = LaneChangeState::KEEP_LANE;
  source_lane_id_ = -1;
  target_lane_id_ = -1;
  safe_since_sec_ = -1.0;
  settled_since_sec_ = -1.0;
  lane_change_length_m_ = 0.0;
  execute_reference_speed_mps_ = 0.0;
  returning_to_source_ = false;
  selected_gap_ = SelectedGap();
  locked_path_map_.clear();
}

const LanePath* LaneChangePlanner::findLane(const std::vector<LanePath>& lanes,
                                            int lane_id) const {
  for (const LanePath& lane : lanes) {
    if (lane.id == lane_id) return &lane;
  }
  return nullptr;
}

bool LaneChangePlanner::adjacent(const LanePath& source,
                                 int target_lane_id) const {
  return source.left_neighbor_id == target_lane_id ||
         source.right_neighbor_id == target_lane_id;
}

bool LaneChangePlanner::laneChangeReady(const EgoState& ego,
                                        const LanePath& source) const {
  const Polyline path(source.centerline_map);
  const PathProjection projection = path.project(ego.position_map);
  if (!projection.valid) return false;
  return std::abs(projection.lateral_m) <= config_.ready_lateral_m &&
         std::abs(normalizeAngle(ego.yaw_rad - projection.heading_rad)) <=
             config_.ready_heading_rad &&
         std::abs(ego.yaw_rate_radps) <= config_.ready_yaw_rate_radps;
}

std::vector<LaneChangePlanner::LaneTrack> LaneChangePlanner::collectLaneTracks(
    const LanePath& lane, const std::vector<TrackedObject>& tracks) const {
  const Polyline path(lane.centerline_map);
  std::vector<LaneTrack> result;
  if (!path.valid()) return result;

  for (const TrackedObject& track : tracks) {
    const PathProjection projection = path.project(track.position_map);
    if (!projection.valid) continue;
    const double lane_gate = 0.5 * lane.width_m + 0.5 * track.width_m +
                             config_.lane_boundary_margin_m;
    if (std::abs(projection.lateral_m) > lane_gate) continue;
    result.push_back(
        {&track, projection.s_m,
         std::max(0.0, longitudinalSpeed(track, projection.heading_rad))});
  }

  std::sort(result.begin(), result.end(),
            [](const LaneTrack& first, const LaneTrack& second) {
              return first.s_m < second.s_m;
            });
  return result;
}

double LaneChangePlanner::targetLaneFlowSpeed(
    const std::vector<LaneTrack>& lane_tracks,
    double desired_speed_mps) const {
  if (lane_tracks.empty()) return std::max(0.0, desired_speed_mps);
  std::vector<double> speeds;
  speeds.reserve(lane_tracks.size());
  for (const LaneTrack& lane_track : lane_tracks) {
    speeds.push_back(lane_track.speed_mps);
  }
  std::sort(speeds.begin(), speeds.end());
  const std::size_t middle = speeds.size() / 2;
  if (speeds.size() % 2 == 1) return speeds[middle];
  return 0.5 * (speeds[middle - 1] + speeds[middle]);
}

SelectedGap LaneChangePlanner::selectGap(
    const EgoState& ego,
    const LanePath& target,
    const std::vector<TrackedObject>& tracks,
    double desired_speed_mps,
    double now_sec,
    double prediction_horizon_sec,
    const SelectedGap* preferred_gap,
    bool current_slot_only) const {
  SelectedGap best;
  double best_score = std::numeric_limits<double>::infinity();
  const Polyline target_path(target.centerline_map);
  const PathProjection ego_projection = target_path.project(ego.position_map);
  if (!target_path.valid() || !ego_projection.valid) return best;

  const std::vector<LaneTrack> lane_tracks = collectLaneTracks(target, tracks);
  const double flow_speed = targetLaneFlowSpeed(lane_tracks, desired_speed_mps);
  const double horizon = std::max(0.1, prediction_horizon_sec);
  const double reachable_min =
      std::max(0.0, ego.speed_mps - config_.gap_shaping_decel_mps2 * horizon);
  const double reachable_max =
      ego.speed_mps + config_.gap_shaping_accel_mps2 * horizon;

  bool preferred_found = false;
  SelectedGap preferred_candidate;
  for (std::size_t slot = 0; slot <= lane_tracks.size(); ++slot) {
    const LaneTrack* rear = slot == 0 ? nullptr : &lane_tracks[slot - 1];
    const LaneTrack* front = slot == lane_tracks.size() ? nullptr : &lane_tracks[slot];

    if (current_slot_only) {
      const bool rear_is_ahead =
          rear != nullptr && rear->s_m > ego_projection.s_m;
      const bool front_is_behind =
          front != nullptr && front->s_m < ego_projection.s_m;
      if (rear_is_ahead || front_is_behind) continue;
    }

    const int front_id = front == nullptr ? -1 : front->track->id;
    const int rear_id = rear == nullptr ? -1 : rear->track->id;
    const double front_speed = front == nullptr ? 0.0 : front->speed_mps;
    const double rear_speed = rear == nullptr ? 0.0 : rear->speed_mps;
    const double front_gap =
        front == nullptr
            ? std::numeric_limits<double>::infinity()
            : front->s_m - ego_projection.s_m - 0.5 * ego.length_m -
                  0.5 * front->track->length_m;
    const double rear_gap =
        rear == nullptr
            ? std::numeric_limits<double>::infinity()
            : ego_projection.s_m - rear->s_m - 0.5 * ego.length_m -
                  0.5 * rear->track->length_m;
    const double front_closing = std::max(0.0, ego.speed_mps - front_speed);
    const double rear_closing = std::max(0.0, rear_speed - ego.speed_mps);
    const double required_front =
        std::max(config_.minimum_gap_m,
                 std::max(config_.steady_flow_headway_sec * ego.speed_mps,
                          config_.closing_base_gap_m +
                              config_.front_closing_time_sec * front_closing));
    const double required_rear =
        std::max(config_.minimum_gap_m,
                 std::max(config_.steady_flow_headway_sec * rear_speed,
                          config_.closing_base_gap_m +
                              config_.rear_closing_time_sec * rear_closing));

    const double slot_speed_min =
        rear == nullptr
            ? 0.0
            : rear_speed + (required_rear - rear_gap) / horizon;
    const double slot_speed_max =
        front == nullptr
            ? std::numeric_limits<double>::infinity()
            : front_speed + (front_gap - required_front) / horizon;
    const double minimum_flow_speed =
        config_.minimum_flow_speed_ratio * std::max(0.0, flow_speed);
    const double feasible_min =
        std::max(0.0, std::max(slot_speed_min,
                               std::max(reachable_min, minimum_flow_speed)));
    const double feasible_max =
        std::min(slot_speed_max,
                 std::min(reachable_max, std::max(0.0, desired_speed_mps)));

    SelectedGap candidate;
    candidate.valid = true;
    candidate.speed_feasible = feasible_min <= feasible_max;
    candidate.front_track_id = front_id;
    candidate.rear_track_id = rear_id;
    candidate.selected_since_sec = now_sec;
    candidate.target_lane_flow_speed_mps = flow_speed;
    candidate.minimum_speed_mps = feasible_min;
    candidate.maximum_speed_mps = feasible_max;
    if (candidate.speed_feasible) {
      candidate.target_speed_mps =
          clamp(flow_speed, feasible_min, feasible_max);
    }

    if (preferred_gap != nullptr && preferred_gap->valid &&
        preferred_gap->front_track_id == front_id &&
        preferred_gap->rear_track_id == rear_id) {
      candidate.selected_since_sec = preferred_gap->selected_since_sec;
      preferred_candidate = candidate;
      preferred_found = true;
    }

    // Once lateral motion has started, the slot currently surrounding Ego is
    // the physical situation to assess even when speed shaping can no longer
    // make that slot feasible.
    if (current_slot_only) return candidate;

    if (!candidate.speed_feasible) continue;
    double slot_distance = 0.0;
    if (front != nullptr && rear != nullptr) {
      slot_distance =
          std::abs(0.5 * (front->s_m + rear->s_m) - ego_projection.s_m);
    }
    const double score = std::abs(candidate.target_speed_mps - ego.speed_mps) +
                         0.02 * slot_distance;
    if (score < best_score) {
      best = candidate;
      best_score = score;
    }
  }

  if (preferred_found) return preferred_candidate;
  return best;
}

bool LaneChangePlanner::selectedGapAvailable(
    const SelectedGap& selected,
    const std::vector<TrackedObject>& tracks) const {
  if (!selected.valid) return false;
  bool front_found = selected.front_track_id < 0;
  bool rear_found = selected.rear_track_id < 0;
  for (const TrackedObject& track : tracks) {
    if (track.id == selected.front_track_id) front_found = true;
    if (track.id == selected.rear_track_id) rear_found = true;
  }
  return front_found && rear_found;
}

GapAssessment LaneChangePlanner::assessGap(
    const EgoState& ego,
    const LanePath& target,
    const std::vector<TrackedObject>& tracks,
    double perception_stamp_sec,
    const std::vector<Point2>& candidate_path,
    const SelectedGap& selected_gap,
    double prediction_horizon_sec) const {
  GapAssessment result;
  result.prediction_horizon_sec = std::max(0.0, prediction_horizon_sec);
  result.perception_fresh =
      perception_stamp_sec > 0.0 && ego.stamp_sec >= perception_stamp_sec &&
      ego.stamp_sec - perception_stamp_sec <= config_.perception_timeout_sec;
  if (!result.perception_fresh || !selected_gap.valid) return result;

  const Polyline target_path(target.centerline_map);
  const Polyline corridor(candidate_path);
  const PathProjection ego_projection = target_path.project(ego.position_map);
  const PathProjection ego_corridor_projection = corridor.project(ego.position_map);
  if (!target_path.valid() || !corridor.valid() || !ego_projection.valid ||
      !ego_corridor_projection.valid) {
    return result;
  }

  bool expected_front_found = selected_gap.front_track_id < 0;
  bool expected_rear_found = selected_gap.rear_track_id < 0;
  result.corridor_clear = true;
  for (const TrackedObject& track : tracks) {
    const PathProjection projection = target_path.project(track.position_map);
    if (projection.valid && track.id == selected_gap.front_track_id) {
      expected_front_found = true;
      result.has_front = true;
      result.front_track_id = track.id;
      result.front_gap_m = projection.s_m - ego_projection.s_m -
                           0.5 * ego.length_m - 0.5 * track.length_m;
      result.front_speed_mps =
          std::max(0.0, longitudinalSpeed(track, projection.heading_rad));
    }
    if (projection.valid && track.id == selected_gap.rear_track_id) {
      expected_rear_found = true;
      result.has_rear = true;
      result.rear_track_id = track.id;
      result.rear_gap_m = ego_projection.s_m - projection.s_m -
                          0.5 * ego.length_m - 0.5 * track.length_m;
      result.rear_speed_mps =
          std::max(0.0, longitudinalSpeed(track, projection.heading_rad));
    }

    for (double prediction_time = 0.0;
         prediction_time <= result.prediction_horizon_sec + 1e-6;
         prediction_time += config_.prediction_step_sec) {
      const Point2 predicted_object{
          track.position_map.x + track.velocity_map_mps.x * prediction_time,
          track.position_map.y + track.velocity_map_mps.y * prediction_time};
      const PathProjection corridor_projection = corridor.project(predicted_object);
      if (!corridor_projection.valid) continue;

      const double predicted_ego_s =
          clamp(ego_corridor_projection.s_m + ego.speed_mps * prediction_time,
                0.0, corridor.length());
      const double lateral_clearance = 0.5 * ego.width_m +
                                       0.5 * track.width_m +
                                       config_.corridor_margin_m;
      const double longitudinal_clearance =
          0.5 * ego.length_m + 0.5 * track.length_m;
      if (std::abs(corridor_projection.lateral_m) <= lateral_clearance &&
          std::abs(corridor_projection.s_m - predicted_ego_s) <=
              longitudinal_clearance) {
        result.corridor_clear = false;
        break;
      }
    }
  }

  if (!expected_front_found || !expected_rear_found) return result;

  const double front_closing =
      std::max(0.0, ego.speed_mps - result.front_speed_mps);
  const double rear_closing =
      std::max(0.0, result.rear_speed_mps - ego.speed_mps);
  result.required_front_gap_m =
      std::max(config_.minimum_gap_m,
               std::max(config_.steady_flow_headway_sec * ego.speed_mps,
                        config_.closing_base_gap_m +
                            config_.front_closing_time_sec * front_closing));
  result.required_rear_gap_m =
      std::max(config_.minimum_gap_m,
               std::max(config_.steady_flow_headway_sec * result.rear_speed_mps,
                        config_.closing_base_gap_m +
                            config_.rear_closing_time_sec * rear_closing));

  if (result.has_front) {
    result.front_ttc_sec =
        front_closing > 1e-3
            ? result.front_gap_m / front_closing
            : std::numeric_limits<double>::infinity();
    result.front_required_decel_mps2 = requiredDeceleration(
        front_closing, result.front_gap_m, config_.minimum_gap_m);
    result.predicted_front_gap_m =
        result.front_gap_m +
        (result.front_speed_mps - ego.speed_mps) *
            result.prediction_horizon_sec;
  }
  if (result.has_rear) {
    result.rear_ttc_sec =
        rear_closing > 1e-3
            ? result.rear_gap_m / rear_closing
            : std::numeric_limits<double>::infinity();
    result.rear_required_decel_mps2 = requiredDeceleration(
        rear_closing, result.rear_gap_m, config_.minimum_gap_m);
    result.predicted_rear_gap_m =
        result.rear_gap_m +
        (ego.speed_mps - result.rear_speed_mps) *
            result.prediction_horizon_sec;
  }

  result.front_safe =
      !result.has_front ||
      (result.front_gap_m >= result.required_front_gap_m &&
       result.predicted_front_gap_m >= config_.minimum_gap_m &&
       result.front_ttc_sec >= config_.minimum_front_ttc_sec &&
       result.front_required_decel_mps2 <=
           config_.maximum_start_required_decel_mps2);
  result.rear_safe =
      !result.has_rear ||
      (result.rear_gap_m >= result.required_rear_gap_m &&
       result.predicted_rear_gap_m >= config_.minimum_gap_m &&
       result.rear_ttc_sec >= config_.minimum_rear_ttc_sec &&
       result.rear_required_decel_mps2 <=
           config_.maximum_start_required_decel_mps2);
  result.safe = result.front_safe && result.rear_safe && result.corridor_clear;
  return result;
}

bool LaneChangePlanner::hardInvalid(const GapAssessment& gap) const {
  if (!gap.perception_fresh) return false;
  return !gap.corridor_clear ||
         gap.front_ttc_sec < config_.hard_invalid_ttc_sec ||
         gap.rear_ttc_sec < config_.hard_invalid_ttc_sec ||
         gap.front_required_decel_mps2 >
             config_.hard_invalid_required_decel_mps2 ||
         gap.rear_required_decel_mps2 >
             config_.hard_invalid_required_decel_mps2 ||
         gap.predicted_front_gap_m < 0.0 || gap.predicted_rear_gap_m < 0.0;
}

bool LaneChangePlanner::bodyCrossedSourceBoundary(
    const EgoState& ego, const LanePath& source) const {
  const Polyline source_path(source.centerline_map);
  const PathProjection projection = source_path.project(ego.position_map);
  if (!projection.valid) return true;
  return std::abs(projection.lateral_m) + 0.5 * ego.width_m >=
         0.5 * source.width_m;
}

std::vector<Point2> LaneChangePlanner::buildTransitionPath(
    const EgoState& ego,
    const std::vector<Point2>& source_reference,
    const LanePath& target,
    double* transition_length_m) const {
  const Polyline source_path(source_reference);
  const Polyline target_path(target.centerline_map);
  const PathProjection source_projection = source_path.project(ego.position_map);
  const PathProjection target_projection = target_path.project(ego.position_map);
  std::vector<Point2> result;
  if (!source_projection.valid || !target_projection.valid) return result;

  const double transition_length =
      clamp(ego.speed_mps * config_.nominal_duration_sec,
            config_.minimum_path_length_m, config_.maximum_path_length_m);
  if (transition_length_m != nullptr) *transition_length_m = transition_length;
  for (double distance_ahead = 0.0;
       distance_ahead <= config_.output_horizon_m;
       distance_ahead += config_.path_sample_step_m) {
    const PathSample source_sample =
        source_path.sample(source_projection.s_m + distance_ahead);
    const PathSample target_sample =
        target_path.sample(target_projection.s_m + distance_ahead);
    if (!source_sample.valid || !target_sample.valid) break;
    const double weight = smoothStepQuintic(distance_ahead / transition_length);
    result.push_back(
        {source_sample.point.x +
             weight * (target_sample.point.x - source_sample.point.x),
         source_sample.point.y +
             weight * (target_sample.point.y - source_sample.point.y)});
  }
  return result;
}

std::vector<Point2> LaneChangePlanner::trimPathAhead(
    const EgoState& ego, const std::vector<Point2>& path_points) const {
  const Polyline path(path_points);
  const PathProjection ego_projection = path.project(ego.position_map);
  std::vector<Point2> result;
  if (!path.valid() || !ego_projection.valid) return result;

  for (double distance_ahead = 0.0;
       distance_ahead <= config_.output_horizon_m;
       distance_ahead += config_.path_sample_step_m) {
    const PathSample sample = path.sample(ego_projection.s_m + distance_ahead);
    if (!sample.valid) break;
    result.push_back(sample.point);
    if (ego_projection.s_m + distance_ahead >= path.length()) break;
  }
  return result;
}

LaneChangeStep LaneChangePlanner::update(
    const EgoState& ego,
    int physical_lane_id,
    const std::vector<LanePath>& lanes,
    const std::vector<TrackedObject>& tracks,
    double perception_stamp_sec,
    const LaneRequest& request,
    double desired_speed_mps) {
  LaneChangeStep result;
  result.state = state_;
  result.source_lane_id = source_lane_id_;
  result.target_lane_id = target_lane_id_;

  const LanePath* physical_lane = findLane(lanes, physical_lane_id);
  if (physical_lane == nullptr) {
    result.request_rejected = true;
    result.reason = "physical lane is unknown";
    return result;
  }

  if (state_ == LaneChangeState::KEEP_LANE) {
    source_lane_id_ = physical_lane_id;
    target_lane_id_ = physical_lane_id;
    locked_path_map_.clear();
    safe_since_sec_ = -1.0;
    settled_since_sec_ = -1.0;
    selected_gap_ = SelectedGap();
    returning_to_source_ = false;

    if (request.type == LaneRequestType::KEEP_LANE ||
        request.target_lane_id == physical_lane_id) {
      result.path_map = trimPathAhead(ego, physical_lane->centerline_map);
      result.source_lane_id = physical_lane_id;
      result.target_lane_id = physical_lane_id;
      return result;
    }

    const LanePath* requested_lane = findLane(lanes, request.target_lane_id);
    if (requested_lane == nullptr ||
        !adjacent(*physical_lane, request.target_lane_id)) {
      result.request_rejected = true;
      result.reason = "lane change target must be an adjacent known lane";
      result.path_map = trimPathAhead(ego, physical_lane->centerline_map);
      return result;
    }

    source_lane_id_ = physical_lane_id;
    target_lane_id_ = request.target_lane_id;
    state_ = LaneChangeState::CHECK_GAP;
  }

  const LanePath* source = findLane(lanes, source_lane_id_);
  const LanePath* target = findLane(lanes, target_lane_id_);
  if (source == nullptr || target == nullptr) {
    reset();
    result.request_rejected = true;
    result.reason = "active lane-change path disappeared";
    result.path_map = trimPathAhead(ego, physical_lane->centerline_map);
    return result;
  }

  if (state_ == LaneChangeState::CHECK_GAP &&
      (request.type == LaneRequestType::KEEP_LANE ||
       request.target_lane_id != target_lane_id_)) {
    result.path_map = trimPathAhead(ego, source->centerline_map);
    result.reason = "lane change request was withdrawn";
    reset();
    result.state = state_;
    return result;
  }

  if (state_ == LaneChangeState::CHECK_GAP) {
    locked_path_map_ = buildTransitionPath(
        ego, source->centerline_map, *target, &lane_change_length_m_);
    const double horizon = lane_change_length_m_ / std::max(ego.speed_mps, 1.0);
    const SelectedGap* preferred =
        selectedGapAvailable(selected_gap_, tracks) ? &selected_gap_ : nullptr;
    SelectedGap candidate = selectGap(
        ego, *target, tracks, desired_speed_mps, ego.stamp_sec, horizon,
        preferred);

    if (candidate.valid && !candidate.speed_feasible &&
        candidate.selected_since_sec >= 0.0 &&
        ego.stamp_sec - candidate.selected_since_sec >=
            config_.selected_gap_hold_sec) {
      candidate = selectGap(ego, *target, tracks, desired_speed_mps,
                            ego.stamp_sec, horizon);
    }
    selected_gap_ = candidate;
    result.gap = assessGap(ego, *target, tracks, perception_stamp_sec,
                           locked_path_map_, selected_gap_, horizon);

    if (hardInvalid(result.gap)) {
      selected_gap_ = selectGap(ego, *target, tracks, desired_speed_mps,
                                ego.stamp_sec, horizon);
      result.gap = assessGap(ego, *target, tracks, perception_stamp_sec,
                             locked_path_map_, selected_gap_, horizon);
    }

    result.path_map = trimPathAhead(ego, source->centerline_map);
    result.selected_gap = selected_gap_;
    if (selected_gap_.valid && selected_gap_.speed_feasible) {
      result.has_behavior_target_speed = true;
      result.behavior_target_speed_mps = selected_gap_.target_speed_mps;
    }

    if (!laneChangeReady(ego, *source)) {
      safe_since_sec_ = -1.0;
      result.safety_action = SafetyAction::START_BLOCKED;
      result.reason = "ego lateral state is not ready";
    } else if (!result.gap.perception_fresh) {
      safe_since_sec_ = -1.0;
      result.safety_action = SafetyAction::STALE_PERCEPTION_HOLD;
      result.reason = "perception is stale";
    } else if (!selected_gap_.speed_feasible || !result.gap.safe) {
      safe_since_sec_ = -1.0;
      result.safety_action = SafetyAction::START_BLOCKED;
      result.reason = "selected target-lane gap is not ready";
    } else {
      if (safe_since_sec_ < 0.0) safe_since_sec_ = ego.stamp_sec;
      if (ego.stamp_sec - safe_since_sec_ >= config_.gap_hold_sec) {
        state_ = LaneChangeState::EXECUTE;
        execute_reference_speed_mps_ = selected_gap_.target_speed_mps;
      }
    }
  }

  if (state_ == LaneChangeState::EXECUTE) {
    const LanePath* active_target = returning_to_source_ ? source : target;
    const Polyline locked_path(locked_path_map_);
    const PathProjection progress_projection = locked_path.project(ego.position_map);
    result.progress =
        progress_projection.valid && lane_change_length_m_ > 1e-6
            ? clamp(progress_projection.s_m / lane_change_length_m_, 0.0, 1.0)
            : 0.0;
    const double full_duration =
        lane_change_length_m_ / std::max(ego.speed_mps, 1.0);
    const double remaining_horizon =
        std::max(config_.prediction_step_sec,
                 (1.0 - result.progress) * full_duration);

    if (!selectedGapAvailable(selected_gap_, tracks)) {
      selected_gap_ = selectGap(ego, *active_target, tracks,
                                desired_speed_mps, ego.stamp_sec,
                                remaining_horizon, nullptr, true);
    } else {
      selected_gap_ = selectGap(ego, *active_target, tracks,
                                desired_speed_mps, ego.stamp_sec,
                                remaining_horizon, &selected_gap_, true);
    }
    result.gap = assessGap(ego, *active_target, tracks, perception_stamp_sec,
                           locked_path_map_, selected_gap_, remaining_horizon);
    result.selected_gap = selected_gap_;
    result.has_behavior_target_speed = true;
    result.behavior_target_speed_mps = execute_reference_speed_mps_;
    result.path_map = trimPathAhead(ego, locked_path_map_);

    if (!result.gap.perception_fresh) {
      result.safety_action = SafetyAction::STALE_PERCEPTION_HOLD;
      result.behavior_target_speed_mps =
          std::min(execute_reference_speed_mps_, ego.speed_mps);
    } else if (!result.gap.safe) {
      const bool early = result.progress < config_.early_cancel_progress &&
                         !bodyCrossedSourceBoundary(ego, *source) &&
                         !returning_to_source_;
      if (early) {
        double return_length = 0.0;
        const std::vector<Point2> return_path = buildTransitionPath(
            ego, locked_path_map_, *source, &return_length);
        const double return_horizon =
            return_length / std::max(ego.speed_mps, 1.0);
        const SelectedGap source_gap = selectGap(
            ego, *source, tracks, desired_speed_mps, ego.stamp_sec,
            return_horizon, nullptr, true);
        const GapAssessment source_assessment = assessGap(
            ego, *source, tracks, perception_stamp_sec, return_path,
            source_gap, return_horizon);
        if (!return_path.empty() && source_assessment.safe) {
          locked_path_map_ = return_path;
          lane_change_length_m_ = return_length;
          returning_to_source_ = true;
          selected_gap_ = source_gap;
          execute_reference_speed_mps_ =
              std::min(execute_reference_speed_mps_, ego.speed_mps);
          result.safety_action = SafetyAction::EARLY_CANCEL;
          result.reason = "target gap became unsafe; returning to source lane";
          result.path_map = trimPathAhead(ego, locked_path_map_);
          result.progress = 0.0;
        }
      }

      if (result.safety_action == SafetyAction::NONE) {
        if (!result.gap.front_safe) {
          result.safety_action = SafetyAction::CONTINUE_FRONT_DECELERATE;
        } else {
          result.safety_action = SafetyAction::CONTINUE_HOLD_SPEED;
          result.behavior_target_speed_mps =
              std::max(execute_reference_speed_mps_, ego.speed_mps);
        }
      }
    }

    active_target = returning_to_source_ ? source : target;
    const Polyline active_target_path(active_target->centerline_map);
    const PathProjection target_projection =
        active_target_path.project(ego.position_map);
    if (result.progress >= config_.minimum_completion_progress &&
        target_projection.valid &&
        std::abs(target_projection.lateral_m) <=
            config_.completion_lateral_m &&
        std::abs(normalizeAngle(ego.yaw_rad - target_projection.heading_rad)) <=
            config_.completion_heading_rad) {
      state_ = LaneChangeState::SETTLE;
      settled_since_sec_ = ego.stamp_sec;
    }
  }

  if (state_ == LaneChangeState::SETTLE) {
    const LanePath* active_target = returning_to_source_ ? source : target;
    result.path_map = trimPathAhead(ego, active_target->centerline_map);
    const Polyline target_path(active_target->centerline_map);
    const PathProjection target_projection = target_path.project(ego.position_map);
    const bool settled =
        target_projection.valid &&
        std::abs(target_projection.lateral_m) <= config_.completion_lateral_m &&
        std::abs(normalizeAngle(ego.yaw_rad - target_projection.heading_rad)) <=
            config_.completion_heading_rad;
    if (!settled) {
      settled_since_sec_ = -1.0;
    } else if (settled_since_sec_ < 0.0) {
      settled_since_sec_ = ego.stamp_sec;
    } else if (ego.stamp_sec - settled_since_sec_ >=
               config_.settle_hold_sec) {
      reset();
    }
  }

  result.state = state_;
  result.source_lane_id = source_lane_id_;
  result.target_lane_id = returning_to_source_ ? source_lane_id_ : target_lane_id_;
  return result;
}

}  // namespace highway_adas
