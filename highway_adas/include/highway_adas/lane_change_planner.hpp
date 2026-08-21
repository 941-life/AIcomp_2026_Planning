#ifndef HIGHWAY_ADAS_LANE_CHANGE_PLANNER_HPP
#define HIGHWAY_ADAS_LANE_CHANGE_PLANNER_HPP

#include "highway_adas/object_tracker.hpp"
#include "highway_adas/types.hpp"

#include <vector>

namespace highway_adas {

struct LaneChangeConfig {
  double nominal_duration_sec = 3.0;
  double minimum_path_length_m = 45.0;
  double maximum_path_length_m = 85.0;
  double output_horizon_m = 120.0;
  double path_sample_step_m = 1.0;

  double minimum_gap_m = 6.0;
  double closing_base_gap_m = 5.0;
  double steady_flow_headway_sec = 0.6;
  double front_closing_time_sec = 1.2;
  double rear_closing_time_sec = 1.5;
  double minimum_front_ttc_sec = 4.0;
  double minimum_rear_ttc_sec = 4.0;
  double maximum_start_required_decel_mps2 = 2.5;
  double hard_invalid_ttc_sec = 2.5;
  double hard_invalid_required_decel_mps2 = 4.0;
  double corridor_margin_m = 0.5;
  double lane_boundary_margin_m = 0.25;
  double prediction_step_sec = 0.25;

  double perception_timeout_sec = 0.3;
  double gap_hold_sec = 0.5;
  double selected_gap_hold_sec = 1.5;
  double settle_hold_sec = 0.5;
  double completion_lateral_m = 0.25;
  double completion_heading_rad = 3.0 * kPi / 180.0;
  double minimum_completion_progress = 0.85;
  double early_cancel_progress = 0.35;

  double gap_shaping_decel_mps2 = 2.0;
  double gap_shaping_accel_mps2 = 1.0;
  double minimum_flow_speed_ratio = 0.70;

  double ready_lateral_m = 0.35;
  double ready_heading_rad = 4.0 * kPi / 180.0;
  double ready_yaw_rate_radps = 6.0 * kPi / 180.0;
};

struct LaneChangeStep {
  LaneChangeState state = LaneChangeState::KEEP_LANE;
  SafetyAction safety_action = SafetyAction::NONE;
  int source_lane_id = -1;
  int target_lane_id = -1;
  bool request_rejected = false;
  std::string reason;
  GapAssessment gap;
  SelectedGap selected_gap;
  bool has_behavior_target_speed = false;
  double behavior_target_speed_mps = 0.0;
  double progress = 0.0;
  std::vector<Point2> path_map;
};

class LaneChangePlanner {
 public:
  explicit LaneChangePlanner(const LaneChangeConfig& config = LaneChangeConfig())
      : config_(config) {}

  void reset();
  LaneChangeStep update(const EgoState& ego,
                        int physical_lane_id,
                        const std::vector<LanePath>& lanes,
                        const std::vector<TrackedObject>& tracks,
                        double perception_stamp_sec,
                        const LaneRequest& request,
                        double desired_speed_mps);

 private:
  struct LaneTrack {
    const TrackedObject* track = nullptr;
    double s_m = 0.0;
    double speed_mps = 0.0;
  };

  const LanePath* findLane(const std::vector<LanePath>& lanes, int lane_id) const;
  bool adjacent(const LanePath& source, int target_lane_id) const;
  bool laneChangeReady(const EgoState& ego, const LanePath& source) const;
  std::vector<LaneTrack> collectLaneTracks(
      const LanePath& lane, const std::vector<TrackedObject>& tracks) const;
  double targetLaneFlowSpeed(const std::vector<LaneTrack>& lane_tracks,
                             double desired_speed_mps) const;
  SelectedGap selectGap(const EgoState& ego,
                        const LanePath& target,
                        const std::vector<TrackedObject>& tracks,
                        double desired_speed_mps,
                        double now_sec,
                        double prediction_horizon_sec,
                        const SelectedGap* preferred_gap = nullptr,
                        bool current_slot_only = false) const;
  bool selectedGapAvailable(const SelectedGap& selected,
                            const std::vector<TrackedObject>& tracks) const;
  GapAssessment assessGap(const EgoState& ego,
                          const LanePath& target,
                          const std::vector<TrackedObject>& tracks,
                          double perception_stamp_sec,
                          const std::vector<Point2>& candidate_path,
                          const SelectedGap& selected_gap,
                          double prediction_horizon_sec) const;
  bool hardInvalid(const GapAssessment& gap) const;
  bool bodyCrossedSourceBoundary(const EgoState& ego,
                                 const LanePath& source) const;
  std::vector<Point2> buildTransitionPath(
      const EgoState& ego,
      const std::vector<Point2>& source_reference,
      const LanePath& target,
      double* transition_length_m) const;
  std::vector<Point2> trimPathAhead(const EgoState& ego,
                                    const std::vector<Point2>& path) const;

  LaneChangeConfig config_;
  LaneChangeState state_ = LaneChangeState::KEEP_LANE;
  int source_lane_id_ = -1;
  int target_lane_id_ = -1;
  double safe_since_sec_ = -1.0;
  double settled_since_sec_ = -1.0;
  double lane_change_length_m_ = 0.0;
  double execute_reference_speed_mps_ = 0.0;
  bool returning_to_source_ = false;
  SelectedGap selected_gap_;
  std::vector<Point2> locked_path_map_;
};

}  // namespace highway_adas

#endif  // HIGHWAY_ADAS_LANE_CHANGE_PLANNER_HPP
