#include "highway_adas/highway_decision_module.hpp"

#include <algorithm>

namespace highway_adas {

void HighwayDecisionModule::reset() {
  adas_.reset();
}

HighwayDecisionOutput HighwayDecisionModule::update(
    const HighwayDecisionInput& input) {
  HighwayDecisionOutput output;
  const int current_lane_id =
      adas_.currentLaneId(input.adas.ego, input.adas.lanes);
  if (current_lane_id < 0) {
    output.reason = "ego is not on a known highway lane";
    return output;
  }

  HighwayRegionInput region_input;
  region_input.region = input.region;
  region_input.current_lane_id = current_lane_id;
  region_input.ego_speed_mps = input.adas.ego.speed_mps;
  region_input.distance_to_guard_stop_m = input.distance_to_guard_stop_m;
  region_input.distance_to_next_limit_m = input.distance_to_next_limit_m;
  output.region = region_policy_.evaluate(region_input);
  if (!output.region.valid) {
    output.reason = output.region.reason;
    return output;
  }

  AdasInput adas_input = input.adas;
  adas_input.lane_request = output.region.lane_request;
  adas_input.cruise_speed_mps = output.region.target_speed_mps;
  output.adas = adas_.update(adas_input);
  if (!output.adas.valid) {
    output.reason = output.adas.reason;
    return output;
  }

  output.target_speed_mps =
      std::min(output.region.speed_cap_mps, output.adas.speed_cap_mps);
  output.valid = true;
  return output;
}

}  // namespace highway_adas
