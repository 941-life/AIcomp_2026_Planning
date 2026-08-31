#include "highway_adas/adas_planner.hpp"

#include "highway_adas/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace highway_adas {
namespace {

int severity(LongitudinalMode mode) {
  return static_cast<int>(mode);
}

double curvatureSpeedLimit(const std::vector<Point2>& points,
                           const Point2& ego_position,
                           const AdasConfig& config) {
  if (!config.curvature_speed_limit_enabled) {
    return std::numeric_limits<double>::infinity();
  }

  const Polyline path(points);
  const PathProjection ego = path.project(ego_position);
  if (!ego.valid || config.curvature_sample_distance_m <= 0.0 ||
      config.max_lateral_accel_mps2 <= 0.0 ||
      config.curvature_lookahead_distance_m < 0.0 ||
      config.curvature_planned_deceleration_mps2 <= 0.0) {
    return 0.0;
  }

  double limit = std::numeric_limits<double>::infinity();
  for (double ahead = 0.0;
       ahead <= config.curvature_lookahead_distance_m;
       ahead += config.curvature_sample_distance_m) {
    const double center_s = ego.s_m + ahead;
    const PathSample p0 = path.sample(
        center_s - config.curvature_sample_distance_m);
    const PathSample p1 = path.sample(center_s);
    const PathSample p2 = path.sample(
        center_s + config.curvature_sample_distance_m);
    if (!p0.valid || !p1.valid || !p2.valid) return 0.0;

    const double denominator =
        distance(p0.point, p1.point) * distance(p1.point, p2.point) *
        distance(p0.point, p2.point);
    if (denominator <= 1e-9) continue;

    const double cross =
        (p1.point.x - p0.point.x) * (p2.point.y - p0.point.y) -
        (p1.point.y - p0.point.y) * (p2.point.x - p0.point.x);
    const double curvature = std::abs(2.0 * cross / denominator);
    if (curvature <= 1e-6) continue;

    const double curve_speed =
        std::sqrt(config.max_lateral_accel_mps2 / curvature);
    limit = std::min(
        limit,
        std::sqrt(curve_speed * curve_speed +
                  2.0 * config.curvature_planned_deceleration_mps2 * ahead));
  }
  return limit;
}

}  // namespace

AdasPlanner::AdasPlanner(const AdasConfig& config)
    : config_(config),
      tracker_(config.tracker),
      following_(config.following),
      lane_change_(config.lane_change) {}

void AdasPlanner::reset() {
  tracker_.reset();
  lane_change_.reset();
  last_detection_stamp_sec_ = -1.0;
  emergency_brake_until_sec_ = -1.0;
  current_lane_id_ = -1;
  lane_candidate_id_ = -1;
  lane_candidate_since_sec_ = -1.0;
}

const LanePath* AdasPlanner::findLane(const std::vector<LanePath>& lanes, int lane_id) const {
  for (const LanePath& lane : lanes) {
    if (lane.id == lane_id) return &lane;
  }
  return nullptr;
}

int AdasPlanner::currentLaneId(const EgoState& ego,
                               const std::vector<LanePath>& lanes) {
  int best_lane_id = -1;
  double best_distance = std::numeric_limits<double>::infinity();
  double current_distance = std::numeric_limits<double>::infinity();
  for (const LanePath& lane : lanes) {
    const Polyline path(lane.centerline_map);
    const PathProjection projection = path.project(ego.position_map);
    if (!projection.valid) continue;
    const double gate = 0.5 * lane.width_m + config_.lane_classification_margin_m;
    if (projection.distance_m <= gate && projection.distance_m < best_distance) {
      best_distance = projection.distance_m;
      best_lane_id = lane.id;
    }
    if (lane.id == current_lane_id_) current_distance = projection.distance_m;
  }
  if (best_lane_id < 0) {
    lane_candidate_id_ = -1;
    lane_candidate_since_sec_ = -1.0;
    return -1;
  }
  if (current_lane_id_ < 0 || !std::isfinite(current_distance)) {
    current_lane_id_ = best_lane_id;
    lane_candidate_id_ = -1;
    lane_candidate_since_sec_ = -1.0;
    return current_lane_id_;
  }
  if (best_lane_id == current_lane_id_ ||
      best_distance + config_.lane_switch_advantage_m >= current_distance) {
    lane_candidate_id_ = -1;
    lane_candidate_since_sec_ = -1.0;
    return current_lane_id_;
  }
  if (lane_candidate_id_ != best_lane_id) {
    lane_candidate_id_ = best_lane_id;
    lane_candidate_since_sec_ = ego.stamp_sec;
    return current_lane_id_;
  }
  if (ego.stamp_sec - lane_candidate_since_sec_ >=
      config_.lane_switch_hold_sec) {
    current_lane_id_ = best_lane_id;
    lane_candidate_id_ = -1;
    lane_candidate_since_sec_ = -1.0;
  }
  return current_lane_id_;
}

LongitudinalResult AdasPlanner::moreRestrictive(
    const LongitudinalResult& first, const LongitudinalResult& second) const {
  if (second.speed_cap_mps < first.speed_cap_mps) return second;
  if (first.speed_cap_mps < second.speed_cap_mps) return first;
  return severity(second.mode) > severity(first.mode) ? second : first;
}

AdasOutput AdasPlanner::update(const AdasInput& input) {
  AdasOutput output;
  if (!std::isfinite(input.ego.stamp_sec) || input.ego.stamp_sec <= 0.0 ||
      !std::isfinite(input.ego.speed_mps) || input.ego.speed_mps < 0.0 ||
      input.cruise_speed_mps < 0.0) {
    output.reason = "invalid ego or cruise input";
    return output;
  }

  const int physical_lane_id = currentLaneId(input.ego, input.lanes);
  const LanePath* physical_lane = findLane(input.lanes, physical_lane_id);
  if (physical_lane == nullptr) {
    output.reason = "ego is not on a known lane";
    return output;
  }

  if (input.perception_healthy && input.detection_stamp_sec > 0.0 &&
      input.detection_stamp_sec > last_detection_stamp_sec_) {
    if (!std::isfinite(input.detection_ego.stamp_sec) ||
        std::abs(input.detection_ego.stamp_sec - input.detection_stamp_sec) > 0.05) {
      output.reason = "detection-synchronized ego pose is required";
      return output;
    }
    tracker_.update(input.detection_ego, input.detections, input.detection_stamp_sec);
    last_detection_stamp_sec_ = input.detection_stamp_sec;
  }
  const std::vector<TrackedObject> confirmed_tracks =
      tracker_.tracks(input.ego.stamp_sec, true);
  const std::vector<TrackedObject> all_tracks =
      tracker_.tracks(input.ego.stamp_sec, false);

  const double effective_perception_stamp_sec =
      input.perception_healthy ? input.detection_stamp_sec : 0.0;
  LaneChangeStep lateral = lane_change_.update(
      input.ego, physical_lane_id, input.lanes, confirmed_tracks,
      effective_perception_stamp_sec, input.lane_request,
      input.cruise_speed_mps);

  LongitudinalResult longitudinal = following_.plan(
      input.ego, *physical_lane, confirmed_tracks, input.cruise_speed_mps);

  // During a lane change, both source and target corridors constrain speed.
  if (lateral.state == LaneChangeState::EXECUTE ||
      lateral.state == LaneChangeState::SETTLE) {
    const LanePath* source = findLane(input.lanes, lateral.source_lane_id);
    const LanePath* target = findLane(input.lanes, lateral.target_lane_id);
    if (source != nullptr) {
      longitudinal = moreRestrictive(
          longitudinal,
          following_.plan(input.ego, *source, confirmed_tracks, input.cruise_speed_mps));
    }
    if (target != nullptr) {
      longitudinal = moreRestrictive(
          longitudinal,
          following_.plan(input.ego, *target, confirmed_tracks, input.cruise_speed_mps));
    }
  }

  // A new, unconfirmed detection may only escalate to emergency braking.
  const LongitudinalResult immediate_hazard = following_.plan(
      input.ego, *physical_lane, all_tracks, input.cruise_speed_mps);
  if (immediate_hazard.mode == LongitudinalMode::EMERGENCY_BRAKE) {
    longitudinal = immediate_hazard;
  }

  // Do not release AEB on a single dropped/noisy perception frame. Once an
  // emergency is observed, zero speed remains authoritative for the hold
  // interval even if the corresponding track temporarily disappears.
  if (longitudinal.mode == LongitudinalMode::EMERGENCY_BRAKE) {
    emergency_brake_until_sec_ = std::max(
        emergency_brake_until_sec_,
        input.ego.stamp_sec + config_.emergency_brake_hold_sec);
  } else if (input.ego.stamp_sec < emergency_brake_until_sec_) {
    longitudinal.mode = LongitudinalMode::EMERGENCY_BRAKE;
    longitudinal.speed_cap_mps = 0.0;
  }

  const bool perception_fresh = input.perception_healthy &&
                                input.detection_stamp_sec > 0.0 &&
                                input.ego.stamp_sec >= input.detection_stamp_sec &&
                                input.ego.stamp_sec - input.detection_stamp_sec <=
                                    config_.lane_change.perception_timeout_sec;
  double final_speed_cap = longitudinal.speed_cap_mps;
  if (lateral.has_behavior_target_speed) {
    final_speed_cap =
        std::min(final_speed_cap, lateral.behavior_target_speed_mps);
  }
  if (!perception_fresh) {
    const bool lane_change_in_progress =
        lateral.state == LaneChangeState::EXECUTE ||
        lateral.state == LaneChangeState::SETTLE;
    const double stale_cap = lane_change_in_progress
                                 ? input.ego.speed_mps
                                 : config_.stale_perception_speed_mps;
    final_speed_cap = std::min(final_speed_cap, stale_cap);
    if (lateral.safety_action == SafetyAction::NONE) {
      lateral.safety_action = SafetyAction::STALE_PERCEPTION_HOLD;
    }
  }

  const double curvature_speed_cap =
      curvatureSpeedLimit(lateral.path_map, input.ego.position_map, config_);
  final_speed_cap = std::min(final_speed_cap, curvature_speed_cap);

  output.valid = !lateral.path_map.empty();
  output.request_rejected = lateral.request_rejected;
  output.reason = lateral.reason;
  output.current_lane_id = physical_lane_id;
  output.target_lane_id = lateral.target_lane_id >= 0 ? lateral.target_lane_id
                                                      : physical_lane_id;
  output.lane_change_state = lateral.state;
  output.safety_action = lateral.safety_action;
  output.longitudinal = longitudinal;
  output.target_gap = lateral.gap;
  output.speed_cap_mps = std::max(0.0, final_speed_cap);
  output.target_path_map = lateral.path_map;
  return output;
}

}  // namespace highway_adas
