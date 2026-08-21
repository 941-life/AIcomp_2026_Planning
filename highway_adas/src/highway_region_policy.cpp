#include "highway_adas/highway_region_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace highway_adas {

double HighwayRegionPolicy::stoppingSpeedCap(
    double ego_speed_mps, double distance_to_stop_m) const {
  if (!std::isfinite(distance_to_stop_m)) {
    return std::numeric_limits<double>::infinity();
  }
  const double usable_distance =
      std::max(0.0, distance_to_stop_m - config_.stop_margin_m -
                        std::max(0.0, ego_speed_mps) *
                            config_.response_time_sec);
  return std::sqrt(2.0 * config_.guard_deceleration_mps2 *
                   usable_distance);
}

double HighwayRegionPolicy::nextLimitSpeedCap(
    double distance_to_limit_m) const {
  if (!std::isfinite(distance_to_limit_m)) {
    return std::numeric_limits<double>::infinity();
  }
  const double distance = std::max(0.0, distance_to_limit_m);
  return std::sqrt(config_.next_limit_speed_mps *
                       config_.next_limit_speed_mps +
                   2.0 * config_.next_limit_deceleration_mps2 * distance);
}

HighwayRegionOutput HighwayRegionPolicy::acquirePolicy(
    const HighwayRegionInput& input,
    int source_lane_id,
    int target_lane_id,
    double target_speed_mps,
    double speed_cap_mps,
    bool guard) const {
  HighwayRegionOutput output;
  output.target_speed_mps = target_speed_mps;
  output.speed_cap_mps = speed_cap_mps;
  output.required_lane_id = target_lane_id;
  output.guard_active = guard && input.current_lane_id == source_lane_id;

  if (input.current_lane_id == target_lane_id) {
    output.valid = true;
    output.lane_request = {LaneRequestType::KEEP_LANE, target_lane_id};
    return output;
  }
  if (input.current_lane_id != source_lane_id) {
    output.reason = "current lane is outside this acquire policy";
    return output;
  }

  output.valid = true;
  output.lane_request = {LaneRequestType::MANDATORY, target_lane_id};
  if (output.guard_active) {
    output.speed_cap_mps =
        std::min(output.speed_cap_mps,
                 stoppingSpeedCap(input.ego_speed_mps,
                                  input.distance_to_guard_stop_m));
  }
  return output;
}

HighwayRegionOutput HighwayRegionPolicy::evaluate(
    const HighwayRegionInput& input) const {
  switch (input.region) {
    case HighwayRegion::HW_ENTRY_ACQUIRE:
      return acquirePolicy(input, config_.third_lane_id,
                           config_.second_lane_id,
                           config_.entry_target_speed_mps,
                           config_.entry_speed_cap_mps, false);
    case HighwayRegion::HW_ENTRY_GUARD:
      return acquirePolicy(input, config_.third_lane_id,
                           config_.second_lane_id,
                           config_.entry_target_speed_mps,
                           config_.entry_speed_cap_mps, true);
    case HighwayRegion::HW_MAIN_ACQUIRE:
      return acquirePolicy(input, config_.second_lane_id,
                           config_.first_lane_id,
                           config_.main_target_speed_mps,
                           config_.main_speed_cap_mps, false);
    case HighwayRegion::HW_MAIN_GUARD:
      return acquirePolicy(input, config_.second_lane_id,
                           config_.first_lane_id,
                           config_.main_target_speed_mps,
                           config_.main_speed_cap_mps, true);
    case HighwayRegion::HW_SINGLE_LANE: {
      HighwayRegionOutput output;
      if (input.current_lane_id != config_.first_lane_id) {
        output.reason = "single-lane region requires lane 1 acquisition";
        return output;
      }
      output.valid = true;
      output.required_lane_id = config_.first_lane_id;
      output.target_speed_mps = config_.main_target_speed_mps;
      output.speed_cap_mps =
          std::min(config_.main_speed_cap_mps,
                   nextLimitSpeedCap(input.distance_to_next_limit_m));
      output.lane_request = {LaneRequestType::KEEP_LANE,
                             config_.first_lane_id};
      return output;
    }
    case HighwayRegion::NONE:
      break;
  }

  HighwayRegionOutput output;
  output.reason = "highway region is not active";
  return output;
}

}  // namespace highway_adas
