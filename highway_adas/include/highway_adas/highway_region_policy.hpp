#ifndef HIGHWAY_ADAS_HIGHWAY_REGION_POLICY_HPP
#define HIGHWAY_ADAS_HIGHWAY_REGION_POLICY_HPP

#include "highway_adas/types.hpp"

#include <limits>
#include <string>

namespace highway_adas {

enum class HighwayRegion {
  NONE = 0,
  HW_ENTRY_ACQUIRE,
  HW_ENTRY_GUARD,
  HW_MAIN_ACQUIRE,
  HW_MAIN_GUARD,
  HW_SINGLE_LANE
};

struct HighwayRegionConfig {
  int first_lane_id = 1;
  int second_lane_id = 2;
  int third_lane_id = 3;

  double entry_target_speed_mps = 70.0 / 3.6;
  double entry_speed_cap_mps = 80.0 / 3.6;
  double main_target_speed_mps = 100.0 / 3.6;
  double main_speed_cap_mps = 100.0 / 3.6;
  double next_limit_speed_mps = 60.0 / 3.6;

  double guard_deceleration_mps2 = 3.0;
  double next_limit_deceleration_mps2 = 2.5;
  double response_time_sec = 0.5;
  double stop_margin_m = 5.0;
};

struct HighwayRegionInput {
  HighwayRegion region = HighwayRegion::NONE;
  int current_lane_id = -1;
  double ego_speed_mps = 0.0;
  double distance_to_guard_stop_m = std::numeric_limits<double>::infinity();
  double distance_to_next_limit_m = std::numeric_limits<double>::infinity();
};

struct HighwayRegionOutput {
  bool valid = false;
  std::string reason;
  LaneRequest lane_request;
  int required_lane_id = -1;
  bool guard_active = false;
  double target_speed_mps = 0.0;
  double speed_cap_mps = 0.0;
};

class HighwayRegionPolicy {
 public:
  explicit HighwayRegionPolicy(
      const HighwayRegionConfig& config = HighwayRegionConfig())
      : config_(config) {}

  HighwayRegionOutput evaluate(const HighwayRegionInput& input) const;

 private:
  HighwayRegionOutput acquirePolicy(const HighwayRegionInput& input,
                                    int source_lane_id,
                                    int target_lane_id,
                                    double target_speed_mps,
                                    double speed_cap_mps,
                                    bool guard) const;
  double stoppingSpeedCap(double ego_speed_mps,
                          double distance_to_stop_m) const;
  double nextLimitSpeedCap(double distance_to_limit_m) const;

  HighwayRegionConfig config_;
};

}  // namespace highway_adas

#endif  // HIGHWAY_ADAS_HIGHWAY_REGION_POLICY_HPP
