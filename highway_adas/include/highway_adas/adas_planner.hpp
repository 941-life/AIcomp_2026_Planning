#ifndef HIGHWAY_ADAS_ADAS_PLANNER_HPP
#define HIGHWAY_ADAS_ADAS_PLANNER_HPP

#include "highway_adas/following_controller.hpp"
#include "highway_adas/lane_change_planner.hpp"
#include "highway_adas/object_tracker.hpp"
#include "highway_adas/types.hpp"

namespace highway_adas {

struct AdasConfig {
  TrackerConfig tracker;
  FollowingConfig following;
  LaneChangeConfig lane_change;
  double lane_classification_margin_m = 0.5;
  double lane_switch_advantage_m = 0.4;
  double lane_switch_hold_sec = 0.3;
  double stale_perception_speed_mps = 60.0 / 3.6;
};

class AdasPlanner {
 public:
  explicit AdasPlanner(const AdasConfig& config = AdasConfig());

  void reset();
  int currentLaneId(const EgoState& ego,
                    const std::vector<LanePath>& lanes);
  AdasOutput update(const AdasInput& input);

 private:
  const LanePath* findLane(const std::vector<LanePath>& lanes, int lane_id) const;
  LongitudinalResult moreRestrictive(const LongitudinalResult& first,
                                     const LongitudinalResult& second) const;

  AdasConfig config_;
  ObjectTracker tracker_;
  FollowingController following_;
  LaneChangePlanner lane_change_;
  double last_detection_stamp_sec_ = -1.0;
  int current_lane_id_ = -1;
  int lane_candidate_id_ = -1;
  double lane_candidate_since_sec_ = -1.0;
};

}  // namespace highway_adas

#endif  // HIGHWAY_ADAS_ADAS_PLANNER_HPP
