#ifndef HIGHWAY_ADAS_FOLLOWING_CONTROLLER_HPP
#define HIGHWAY_ADAS_FOLLOWING_CONTROLLER_HPP

#include "highway_adas/geometry.hpp"
#include "highway_adas/object_tracker.hpp"
#include "highway_adas/types.hpp"

#include <vector>

namespace highway_adas {

struct FollowingConfig {
  double minimum_gap_m = 5.0;
  double time_headway_sec = 1.8;
  double gap_gain_per_sec = 0.35;
  double follow_entry_margin_m = 10.0;

  double hard_brake_ttc_sec = 3.0;
  double emergency_ttc_sec = 1.5;
  double hard_brake_decel_mps2 = 4.0;
  double emergency_decel_mps2 = 6.0;
  double lane_boundary_margin_m = 0.25;
};

class FollowingController {
 public:
  explicit FollowingController(const FollowingConfig& config = FollowingConfig())
      : config_(config) {}

  LongitudinalResult plan(const EgoState& ego,
                          const LanePath& lane,
                          const std::vector<TrackedObject>& tracks,
                          double cruise_speed_mps) const;

 private:
  FollowingConfig config_;
};

}  // namespace highway_adas

#endif  // HIGHWAY_ADAS_FOLLOWING_CONTROLLER_HPP

