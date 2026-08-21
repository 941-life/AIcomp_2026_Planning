#ifndef HIGHWAY_ADAS_HIGHWAY_DECISION_MODULE_HPP
#define HIGHWAY_ADAS_HIGHWAY_DECISION_MODULE_HPP

#include "highway_adas/adas_planner.hpp"
#include "highway_adas/highway_region_policy.hpp"

#include <limits>
#include <string>

namespace highway_adas {

struct HighwayDecisionInput {
  AdasInput adas;
  HighwayRegion region = HighwayRegion::NONE;
  double distance_to_guard_stop_m =
      std::numeric_limits<double>::infinity();
  double distance_to_next_limit_m =
      std::numeric_limits<double>::infinity();
};

struct HighwayDecisionOutput {
  bool valid = false;
  std::string reason;
  HighwayRegionOutput region;
  AdasOutput adas;
  double target_speed_mps = 0.0;
};

class HighwayDecisionModule {
 public:
  HighwayDecisionModule(
      const AdasConfig& adas_config = AdasConfig(),
      const HighwayRegionConfig& region_config = HighwayRegionConfig())
      : adas_(adas_config), region_policy_(region_config) {}

  void reset();
  HighwayDecisionOutput update(const HighwayDecisionInput& input);

 private:
  AdasPlanner adas_;
  HighwayRegionPolicy region_policy_;
};

}  // namespace highway_adas

#endif  // HIGHWAY_ADAS_HIGHWAY_DECISION_MODULE_HPP
