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
}

const LanePath* AdasPlanner::findLane(const std::vector<LanePath>& lanes, int lane_id) const {
  for (const LanePath& lane : lanes) {
    if (lane.id == lane_id) return &lane;
  }
  return nullptr;
}

int AdasPlanner::currentLaneId(const EgoState& ego,
                               const std::vector<LanePath>& lanes) const {
  int best_lane_id = -1;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const LanePath& lane : lanes) {
    const Polyline path(lane.centerline_map);
    const PathProjection projection = path.project(ego.position_map);
    if (!projection.valid) continue;
    const double gate = 0.5 * lane.width_m + config_.lane_classification_margin_m;
    if (projection.distance_m <= gate && projection.distance_m < best_distance) {
      best_distance = projection.distance_m;
      best_lane_id = lane.id;
    }
  }
  return best_lane_id;
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

  if (input.detection_stamp_sec > last_detection_stamp_sec_) {
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

  LaneChangeStep lateral = lane_change_.update(
      input.ego, physical_lane_id, input.lanes, confirmed_tracks,
      input.detection_stamp_sec, input.lane_request, input.cruise_speed_mps);

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

  const bool perception_fresh = input.detection_stamp_sec > 0.0 &&
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
