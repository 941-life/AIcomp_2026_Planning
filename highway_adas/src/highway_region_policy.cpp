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
    double ego_speed_mps, double distance_to_limit_m) const {
  if (!std::isfinite(distance_to_limit_m)) {
    return std::numeric_limits<double>::infinity();
  }
  const double distance = std::max(
      0.0, distance_to_limit_m - config_.stop_margin_m -
               std::max(0.0, ego_speed_mps) * config_.response_time_sec);
  return std::sqrt(config_.next_limit_speed_mps *
                       config_.next_limit_speed_mps +
                   2.0 * config_.next_limit_deceleration_mps2 * distance);
}

HighwayRegionOutput HighwayRegionPolicy::baseOutput(
    double target_speed_mps,
    double speed_cap_mps,
    int required_lane_id) const {
  HighwayRegionOutput output;
  output.valid = true;
  output.target_speed_mps = target_speed_mps;
  output.speed_cap_mps = speed_cap_mps;
  output.required_lane_id = required_lane_id;
  return output;
}

LaneRequest HighwayRegionPolicy::requestToward(
    int current_lane_id, int target_lane_id, LaneRequestType type) const {
  if (current_lane_id <= target_lane_id) {
    return {LaneRequestType::KEEP_LANE, current_lane_id};
  }
  return {type, current_lane_id - 1};
}

HighwayRegionOutput HighwayRegionPolicy::evaluate(
    const HighwayRegionInput& input) const {
  switch (input.region) {
    case HighwayRegion::HW_ENTRY:
    case HighwayRegion::HW_ENTRY_GUARD: {
      HighwayRegionOutput output = baseOutput(
          config_.entry_target_speed_mps, config_.entry_speed_cap_mps,
          config_.third_lane_id);
      if (input.current_lane_id < config_.first_lane_id ||
          input.current_lane_id > config_.fourth_lane_id) {
        output.valid = false;
        output.reason = "entry region requires lane 1 through 4";
        return output;
      }
      output.lane_request = requestToward(
          input.current_lane_id, config_.third_lane_id,
          LaneRequestType::MANDATORY);
      output.guard_active =
          input.region == HighwayRegion::HW_ENTRY_GUARD &&
          input.current_lane_id > config_.third_lane_id;
      if (output.guard_active) {
        output.speed_cap_mps = std::min(
            output.speed_cap_mps,
            stoppingSpeedCap(input.ego_speed_mps,
                             input.distance_to_guard_stop_m));
      }
      return output;
    }
    case HighwayRegion::HW_MAIN: {
      HighwayRegionOutput output = baseOutput(
          config_.main_target_speed_mps, config_.main_speed_cap_mps,
          config_.first_lane_id);
      if (input.current_lane_id < config_.first_lane_id ||
          input.current_lane_id > config_.third_lane_id) {
        output.valid = false;
        output.reason = "main region requires lane 1 through 3";
        return output;
      }
      output.lane_request = requestToward(
          input.current_lane_id, config_.first_lane_id,
          LaneRequestType::STRATEGIC);
      return output;
    }
    case HighwayRegion::HW_MAIN_GUARD_1: {
      HighwayRegionOutput output = baseOutput(
          config_.main_target_speed_mps, config_.main_speed_cap_mps,
          config_.second_lane_id);
      if (input.current_lane_id < config_.first_lane_id ||
          input.current_lane_id > config_.third_lane_id) {
        output.valid = false;
        output.reason = "main guard 1 requires lane 1 through 3";
        return output;
      }
      const bool must_leave_lane_3 =
          input.current_lane_id > config_.second_lane_id;
      output.lane_request = requestToward(
          input.current_lane_id,
          must_leave_lane_3 ? config_.second_lane_id : config_.first_lane_id,
          must_leave_lane_3 ? LaneRequestType::MANDATORY
                            : LaneRequestType::STRATEGIC);
      output.guard_active = must_leave_lane_3;
      if (output.guard_active) {
        output.speed_cap_mps = std::min(
            output.speed_cap_mps,
            stoppingSpeedCap(input.ego_speed_mps,
                             input.distance_to_guard_stop_m));
      }
      return output;
    }
    case HighwayRegion::HW_MAIN_GUARD_2: {
      HighwayRegionOutput output = baseOutput(
          config_.main_target_speed_mps, config_.main_speed_cap_mps,
          config_.first_lane_id);
      if (input.current_lane_id < config_.first_lane_id ||
          input.current_lane_id > config_.third_lane_id) {
        output.valid = false;
        output.reason = "main guard 2 requires lane 1 through 3";
        return output;
      }
      output.lane_request = requestToward(
          input.current_lane_id, config_.first_lane_id,
          LaneRequestType::MANDATORY);
      output.guard_active = input.current_lane_id > config_.first_lane_id;
      if (output.guard_active) {
        output.speed_cap_mps = std::min(
            output.speed_cap_mps,
            stoppingSpeedCap(input.ego_speed_mps,
                             input.distance_to_guard_stop_m));
      }
      return output;
    }
    case HighwayRegion::HW_TOLL: {
      HighwayRegionOutput output = baseOutput(
          config_.main_target_speed_mps, config_.main_speed_cap_mps,
          config_.first_lane_id);
      if (input.current_lane_id != config_.first_lane_id) {
        output.valid = false;
        output.reason = "toll region requires the single lane corridor";
        return output;
      }
      output.lane_request = {LaneRequestType::KEEP_LANE,
                             config_.first_lane_id};
      output.speed_cap_mps = std::min(
          output.speed_cap_mps,
          nextLimitSpeedCap(input.ego_speed_mps,
                            input.distance_to_next_limit_m));
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
