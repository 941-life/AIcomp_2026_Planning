#ifndef HIGHWAY_ADAS_TYPES_HPP
#define HIGHWAY_ADAS_TYPES_HPP

#include <limits>
#include <string>
#include <vector>

namespace highway_adas {

constexpr double kPi = 3.14159265358979323846;

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

struct EgoState {
  Point2 position_map;
  double yaw_rad = 0.0;
  double yaw_rate_radps = 0.0;
  double speed_mps = 0.0;
  double stamp_sec = 0.0;
  double width_m = 1.892;
  double length_m = 4.635;
};

// Perception contract. Position is base_link (x forward, y left).
// No object id, lane id, or velocity is required.
struct ObjectDetection {
  double x_m = 0.0;
  double y_m = 0.0;
  double width_m = 0.0;
  double length_m = 0.0;
};

struct LanePath {
  int id = -1;
  int left_neighbor_id = -1;
  int right_neighbor_id = -1;
  double width_m = 3.5;
  std::vector<Point2> centerline_map;
};

enum class LaneRequestType {
  KEEP_LANE = 0,
  STRATEGIC,
  MANDATORY,
  OVERTAKE
};

struct LaneRequest {
  LaneRequestType type = LaneRequestType::KEEP_LANE;
  int target_lane_id = -1;
};

struct AdasInput {
  EgoState ego;
  std::vector<ObjectDetection> detections;
  // A fresh empty frame is different from a missing perception frame.
  double detection_stamp_sec = 0.0;
  // Ego pose interpolated at detection_stamp_sec. Required for base_link -> map conversion.
  EgoState detection_ego;
  std::vector<LanePath> lanes;
  LaneRequest lane_request;
  double cruise_speed_mps = 0.0;
};

enum class LongitudinalMode {
  CRUISE = 0,
  FOLLOW,
  HARD_BRAKE,
  EMERGENCY_BRAKE
};

enum class LaneChangeState {
  KEEP_LANE = 0,
  CHECK_GAP,
  EXECUTE,
  SETTLE
};

enum class SafetyAction {
  NONE = 0,
  START_BLOCKED,
  EARLY_CANCEL,
  CONTINUE_FRONT_DECELERATE,
  CONTINUE_HOLD_SPEED,
  STALE_PERCEPTION_HOLD
};

struct LeadVehicleInfo {
  bool present = false;
  int track_id = -1;
  double gap_m = std::numeric_limits<double>::infinity();
  double speed_mps = 0.0;
  double relative_speed_mps = 0.0;
  double ttc_sec = std::numeric_limits<double>::infinity();
  double required_decel_mps2 = 0.0;
};

struct LongitudinalResult {
  LongitudinalMode mode = LongitudinalMode::CRUISE;
  double speed_cap_mps = 0.0;
  LeadVehicleInfo lead;
};

struct GapAssessment {
  bool perception_fresh = false;
  bool front_safe = false;
  bool rear_safe = false;
  bool corridor_clear = false;
  bool safe = false;

  bool has_front = false;
  bool has_rear = false;
  int front_track_id = -1;
  int rear_track_id = -1;
  double front_gap_m = std::numeric_limits<double>::infinity();
  double rear_gap_m = std::numeric_limits<double>::infinity();
  double required_front_gap_m = 0.0;
  double required_rear_gap_m = 0.0;
  double predicted_front_gap_m = std::numeric_limits<double>::infinity();
  double predicted_rear_gap_m = std::numeric_limits<double>::infinity();
  double front_ttc_sec = std::numeric_limits<double>::infinity();
  double rear_ttc_sec = std::numeric_limits<double>::infinity();
  double front_required_decel_mps2 = 0.0;
  double rear_required_decel_mps2 = 0.0;
  double front_speed_mps = 0.0;
  double rear_speed_mps = 0.0;
  double prediction_horizon_sec = 0.0;
};

struct SelectedGap {
  bool valid = false;
  bool speed_feasible = false;
  int front_track_id = -1;
  int rear_track_id = -1;
  double selected_since_sec = -1.0;
  double target_lane_flow_speed_mps = 0.0;
  double minimum_speed_mps = 0.0;
  double maximum_speed_mps = std::numeric_limits<double>::infinity();
  double target_speed_mps = 0.0;
};

struct AdasOutput {
  bool valid = false;
  bool request_rejected = false;
  std::string reason;

  int current_lane_id = -1;
  int target_lane_id = -1;
  LaneChangeState lane_change_state = LaneChangeState::KEEP_LANE;
  SafetyAction safety_action = SafetyAction::NONE;

  LongitudinalResult longitudinal;
  GapAssessment target_gap;
  double speed_cap_mps = 0.0;
  std::vector<Point2> target_path_map;
};

}  // namespace highway_adas

#endif  // HIGHWAY_ADAS_TYPES_HPP
