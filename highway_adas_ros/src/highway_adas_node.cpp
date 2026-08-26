#include "highway_adas/highway_decision_module.hpp"
#include "highway_adas_ros/ros_adapter.hpp"

#include <gadis_perception_msgs/ObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace adas = highway_adas;
namespace adapter = highway_adas_ros;

class HighwayAdasNode {
 public:
  HighwayAdasNode()
      : nh_(),
        pnh_("~"),
        decision_(loadAdasConfig(), loadRegionConfig()) {
    loadNodeParams();
    setupRos();
    ROS_INFO("[highway_adas] ready: ROS adapter -> %s",
             output_path_topic_.c_str());
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  adas::HighwayDecisionModule decision_;
  adapter::EgoPoseHistory pose_history_;
  adapter::ObjectConverter object_converter_;

  ros::Subscriber objects_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber region_sub_;
  ros::Subscriber guard_distance_sub_;
  ros::Subscriber next_limit_distance_sub_;
  ros::Subscriber lane1_sub_;
  ros::Subscriber lane2_sub_;
  ros::Subscriber lane3_sub_;
  ros::Subscriber lane4_sub_;

  ros::Publisher path_pub_;
  ros::Publisher target_speed_mps_pub_;
  ros::Publisher target_speed_kph_pub_;
  ros::Publisher fsm_pub_;
  ros::Publisher safety_pub_;
  ros::Publisher behavior_pub_;
  ros::Publisher current_lane_pub_;
  ros::Publisher target_lane_pub_;
  ros::Publisher front_ttc_pub_;
  ros::Publisher rear_ttc_pub_;
  ros::Publisher front_gap_pub_;
  ros::Publisher rear_gap_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Publisher active_pub_;
  ros::Timer timer_;

  std::string objects_topic_ = "/objects";
  std::string odom_topic_ = "/odometry/filtered";
  std::string speed_topic_ = "/current_speed";
  std::string status_topic_ = "/vehicle/status_valid";
  std::string region_topic_ = "/highway/region";
  std::string guard_distance_topic_ = "/highway/distance_to_guard_stop";
  std::string next_limit_distance_topic_ = "/highway/distance_to_next_limit";
  std::string lane1_topic_ = "/highway/lane1_centerline";
  std::string lane2_topic_ = "/highway/lane2_centerline";
  std::string lane3_topic_ = "/highway/lane3_centerline";
  std::string lane4_topic_ = "/highway/lane4_centerline";
  std::string output_path_topic_ = "/highway/selected_path_with_speed";
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
  std::string lidar_frame_ = "velodyne";

  double rate_hz_ = 20.0;
  double input_timeout_sec_ = 0.3;
  double pose_sync_tolerance_sec_ = 0.05;
  double odom_history_sec_ = 2.0;
  double lane_width_m_ = 3.5;
  double lidar_to_base_x_m_ = 1.144;
  double lidar_to_base_y_m_ = 0.0;
  double lidar_to_base_yaw_rad_ = 0.0;
  double minimum_object_dimension_m_ = 0.2;
  double maximum_object_dimension_m_ = 20.0;
  bool require_vehicle_status_ = true;

  bool odom_received_ = false;
  bool speed_received_ = false;
  bool status_received_ = false;
  bool status_valid_ = false;
  bool objects_received_ = false;
  bool latest_sensor_healthy_ = false;
  bool region_valid_ = false;
  ros::Time last_odom_receipt_;
  ros::Time last_speed_receipt_;
  ros::Time last_objects_receipt_;
  ros::Time last_region_receipt_;
  ros::Time last_guard_distance_receipt_;
  ros::Time last_next_limit_distance_receipt_;
  adas::EgoState current_ego_;
  double current_speed_mps_ = 0.0;
  std::map<int, adas::LanePath> lanes_;
  adas::HighwayRegion current_region_ = adas::HighwayRegion::NONE;
  std::string current_region_name_ = "NONE";
  double distance_to_guard_stop_m_ = std::numeric_limits<double>::infinity();
  double distance_to_next_limit_m_ = std::numeric_limits<double>::infinity();

  double accepted_detection_stamp_sec_ = 0.0;
  adas::EgoState accepted_detection_ego_;
  std::vector<adas::ObjectDetection> accepted_detections_;

  adas::AdasConfig loadAdasConfig() {
    adas::AdasConfig config;
    pnh_.param("tracker/association_gate_m", config.tracker.association_gate_m,
                config.tracker.association_gate_m);
    pnh_.param("tracker/size_cost_weight", config.tracker.size_cost_weight,
                config.tracker.size_cost_weight);
    pnh_.param("tracker/position_gain", config.tracker.position_gain,
                config.tracker.position_gain);
    pnh_.param("tracker/velocity_gain", config.tracker.velocity_gain,
                config.tracker.velocity_gain);
    pnh_.param("tracker/track_timeout_sec", config.tracker.track_timeout_sec,
                config.tracker.track_timeout_sec);
    pnh_.param("tracker/confirmation_hits", config.tracker.confirmation_hits,
                config.tracker.confirmation_hits);
    pnh_.param("lane_classification_margin_m",
                config.lane_classification_margin_m,
                config.lane_classification_margin_m);
    pnh_.param("lane_switch_advantage_m", config.lane_switch_advantage_m,
                config.lane_switch_advantage_m);
    pnh_.param("lane_switch_hold_sec", config.lane_switch_hold_sec,
                config.lane_switch_hold_sec);
    double stale_perception_speed_kph =
        config.stale_perception_speed_mps * 3.6;
    pnh_.param("stale_perception_speed_kph", stale_perception_speed_kph,
                stale_perception_speed_kph);
    config.stale_perception_speed_mps =
        std::max(0.0, stale_perception_speed_kph / 3.6);

    pnh_.param("following/minimum_gap_m", config.following.minimum_gap_m,
                config.following.minimum_gap_m);
    pnh_.param("following/time_headway_sec", config.following.time_headway_sec,
                config.following.time_headway_sec);
    pnh_.param("following/gap_gain_per_sec", config.following.gap_gain_per_sec,
                config.following.gap_gain_per_sec);
    pnh_.param("following/follow_entry_margin_m",
                config.following.follow_entry_margin_m,
                config.following.follow_entry_margin_m);
    pnh_.param("following/hard_brake_ttc_sec",
                config.following.hard_brake_ttc_sec,
                config.following.hard_brake_ttc_sec);
    pnh_.param("following/emergency_ttc_sec",
                config.following.emergency_ttc_sec,
                config.following.emergency_ttc_sec);
    pnh_.param("following/hard_brake_decel_mps2",
                config.following.hard_brake_decel_mps2,
                config.following.hard_brake_decel_mps2);
    pnh_.param("following/emergency_decel_mps2",
                config.following.emergency_decel_mps2,
                config.following.emergency_decel_mps2);

    pnh_.param("lane_change/nominal_duration_sec",
                config.lane_change.nominal_duration_sec,
                config.lane_change.nominal_duration_sec);
    pnh_.param("lane_change/minimum_path_length_m",
                config.lane_change.minimum_path_length_m,
                config.lane_change.minimum_path_length_m);
    pnh_.param("lane_change/maximum_path_length_m",
                config.lane_change.maximum_path_length_m,
                config.lane_change.maximum_path_length_m);
    pnh_.param("lane_change/output_horizon_m",
                config.lane_change.output_horizon_m,
                config.lane_change.output_horizon_m);
    pnh_.param("lane_change/minimum_gap_m", config.lane_change.minimum_gap_m,
                config.lane_change.minimum_gap_m);
    pnh_.param("lane_change/minimum_front_ttc_sec",
                config.lane_change.minimum_front_ttc_sec,
                config.lane_change.minimum_front_ttc_sec);
    pnh_.param("lane_change/minimum_rear_ttc_sec",
                config.lane_change.minimum_rear_ttc_sec,
                config.lane_change.minimum_rear_ttc_sec);
    pnh_.param("lane_change/maximum_start_required_decel_mps2",
                config.lane_change.maximum_start_required_decel_mps2,
                config.lane_change.maximum_start_required_decel_mps2);
    pnh_.param("lane_change/perception_timeout_sec",
                config.lane_change.perception_timeout_sec,
                config.lane_change.perception_timeout_sec);
    pnh_.param("lane_change/gap_hold_sec", config.lane_change.gap_hold_sec,
                config.lane_change.gap_hold_sec);
    pnh_.param("lane_change/selected_gap_hold_sec",
                config.lane_change.selected_gap_hold_sec,
                config.lane_change.selected_gap_hold_sec);
    pnh_.param("lane_change/settle_hold_sec",
                config.lane_change.settle_hold_sec,
                config.lane_change.settle_hold_sec);
    pnh_.param("lane_change/early_cancel_progress",
                config.lane_change.early_cancel_progress,
                config.lane_change.early_cancel_progress);
    pnh_.param("lane_change/gap_shaping_decel_mps2",
                config.lane_change.gap_shaping_decel_mps2,
                config.lane_change.gap_shaping_decel_mps2);
    pnh_.param("lane_change/gap_shaping_accel_mps2",
                config.lane_change.gap_shaping_accel_mps2,
                config.lane_change.gap_shaping_accel_mps2);
    pnh_.param("lane_change/minimum_flow_speed_ratio",
                config.lane_change.minimum_flow_speed_ratio,
                config.lane_change.minimum_flow_speed_ratio);
    return config;
  }

  adas::HighwayRegionConfig loadRegionConfig() {
    adas::HighwayRegionConfig config;
    double value_kph = 0.0;
    pnh_.param("region/entry_target_speed_kph", value_kph, 70.0);
    config.entry_target_speed_mps = value_kph / 3.6;
    pnh_.param("region/entry_speed_cap_kph", value_kph, 80.0);
    config.entry_speed_cap_mps = value_kph / 3.6;
    pnh_.param("region/main_target_speed_kph", value_kph, 100.0);
    config.main_target_speed_mps = value_kph / 3.6;
    pnh_.param("region/main_speed_cap_kph", value_kph, 100.0);
    config.main_speed_cap_mps = value_kph / 3.6;
    pnh_.param("region/next_limit_speed_kph", value_kph, 60.0);
    config.next_limit_speed_mps = value_kph / 3.6;
    pnh_.param("region/guard_deceleration_mps2",
                config.guard_deceleration_mps2,
                config.guard_deceleration_mps2);
    pnh_.param("region/next_limit_deceleration_mps2",
                config.next_limit_deceleration_mps2,
                config.next_limit_deceleration_mps2);
    pnh_.param("region/response_time_sec", config.response_time_sec,
                config.response_time_sec);
    pnh_.param("region/stop_margin_m", config.stop_margin_m,
                config.stop_margin_m);
    return config;
  }

  void loadNodeParams() {
    pnh_.param("objects_topic", objects_topic_, objects_topic_);
    pnh_.param("odom_topic", odom_topic_, odom_topic_);
    pnh_.param("speed_topic", speed_topic_, speed_topic_);
    pnh_.param("status_valid_topic", status_topic_, status_topic_);
    pnh_.param("region_topic", region_topic_, region_topic_);
    pnh_.param("guard_distance_topic", guard_distance_topic_,
                guard_distance_topic_);
    pnh_.param("next_limit_distance_topic", next_limit_distance_topic_,
                next_limit_distance_topic_);
    pnh_.param("lane1_topic", lane1_topic_, lane1_topic_);
    pnh_.param("lane2_topic", lane2_topic_, lane2_topic_);
    pnh_.param("lane3_topic", lane3_topic_, lane3_topic_);
    pnh_.param("lane4_topic", lane4_topic_, lane4_topic_);
    pnh_.param("output_path_topic", output_path_topic_, output_path_topic_);
    pnh_.param("map_frame", map_frame_, map_frame_);
    pnh_.param("base_frame", base_frame_, base_frame_);
    pnh_.param("lidar_frame", lidar_frame_, lidar_frame_);
    pnh_.param("rate_hz", rate_hz_, rate_hz_);
    pnh_.param("input_timeout_sec", input_timeout_sec_, input_timeout_sec_);
    pnh_.param("pose_sync_tolerance_sec", pose_sync_tolerance_sec_,
                pose_sync_tolerance_sec_);
    pnh_.param("odom_history_sec", odom_history_sec_, odom_history_sec_);
    pnh_.param("lane_width_m", lane_width_m_, lane_width_m_);
    pnh_.param("lidar_to_base_x_m", lidar_to_base_x_m_, lidar_to_base_x_m_);
    pnh_.param("lidar_to_base_y_m", lidar_to_base_y_m_, lidar_to_base_y_m_);
    pnh_.param("lidar_to_base_yaw_rad", lidar_to_base_yaw_rad_,
                lidar_to_base_yaw_rad_);
    pnh_.param("minimum_object_dimension_m", minimum_object_dimension_m_,
                minimum_object_dimension_m_);
    pnh_.param("maximum_object_dimension_m", maximum_object_dimension_m_,
                maximum_object_dimension_m_);
    pnh_.param("require_vehicle_status", require_vehicle_status_,
                require_vehicle_status_);
    pose_history_.configure(odom_history_sec_, pose_sync_tolerance_sec_);
    object_converter_.configure(
        lidar_to_base_x_m_, lidar_to_base_y_m_, lidar_to_base_yaw_rad_,
        minimum_object_dimension_m_, maximum_object_dimension_m_);
  }

  void setupRos() {
    objects_sub_ = nh_.subscribe(objects_topic_, 2,
                                 &HighwayAdasNode::objectsCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 20,
                              &HighwayAdasNode::odomCallback, this);
    speed_sub_ = nh_.subscribe(speed_topic_, 20,
                               &HighwayAdasNode::speedCallback, this);
    status_sub_ = nh_.subscribe(status_topic_, 2,
                                &HighwayAdasNode::statusCallback, this);
    region_sub_ = nh_.subscribe(region_topic_, 2,
                                &HighwayAdasNode::regionCallback, this);
    guard_distance_sub_ = nh_.subscribe(
        guard_distance_topic_, 2, &HighwayAdasNode::guardDistanceCallback, this);
    next_limit_distance_sub_ = nh_.subscribe(
        next_limit_distance_topic_, 2,
        &HighwayAdasNode::nextLimitDistanceCallback, this);
    lane1_sub_ = nh_.subscribe(lane1_topic_, 1,
                               &HighwayAdasNode::lane1Callback, this);
    lane2_sub_ = nh_.subscribe(lane2_topic_, 1,
                               &HighwayAdasNode::lane2Callback, this);
    lane3_sub_ = nh_.subscribe(lane3_topic_, 1,
                               &HighwayAdasNode::lane3Callback, this);
    lane4_sub_ = nh_.subscribe(lane4_topic_, 1,
                               &HighwayAdasNode::lane4Callback, this);

    path_pub_ = nh_.advertise<nav_msgs::Path>(output_path_topic_, 1);
    target_speed_mps_pub_ =
        nh_.advertise<std_msgs::Float64>("/highway/target_speed_mps", 1);
    target_speed_kph_pub_ =
        nh_.advertise<std_msgs::Float64>("/highway/target_speed_kph", 1);
    fsm_pub_ = nh_.advertise<std_msgs::String>("/highway/lane_change_state", 1);
    safety_pub_ = nh_.advertise<std_msgs::String>("/highway/safety_action", 1);
    behavior_pub_ = nh_.advertise<std_msgs::String>("/highway/longitudinal_mode", 1);
    current_lane_pub_ = nh_.advertise<std_msgs::Int32>("/highway/current_lane", 1);
    target_lane_pub_ = nh_.advertise<std_msgs::Int32>("/highway/target_lane", 1);
    front_ttc_pub_ = nh_.advertise<std_msgs::Float64>("/highway/front_ttc", 1);
    rear_ttc_pub_ = nh_.advertise<std_msgs::Float64>("/highway/rear_ttc", 1);
    front_gap_pub_ = nh_.advertise<std_msgs::Float64>("/highway/front_gap", 1);
    rear_gap_pub_ = nh_.advertise<std_msgs::Float64>("/highway/rear_gap", 1);
    diagnostics_pub_ = nh_.advertise<std_msgs::String>("/highway/diagnostics", 1);
    active_pub_ = nh_.advertise<std_msgs::Bool>("/highway/active", 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, rate_hz_)),
                             &HighwayAdasNode::timerCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (msg->header.stamp.isZero() || msg->header.frame_id != map_frame_) {
      ROS_WARN_THROTTLE(1.0, "[highway_adas] odom must be stamped in map frame");
      return;
    }
    adas::EgoState ego;
    ego.position_map = {msg->pose.pose.position.x, msg->pose.pose.position.y};
    ego.yaw_rad = adapter::yawFromQuaternion(msg->pose.pose.orientation);
    ego.yaw_rate_radps = msg->twist.twist.angular.z;
    ego.stamp_sec = msg->header.stamp.toSec();
    current_ego_ = ego;
    odom_received_ = true;
    last_odom_receipt_ = ros::Time::now();
    pose_history_.add(msg->header.stamp, ego);
  }

  void speedCallback(const std_msgs::Float64::ConstPtr& msg) {
    if (!std::isfinite(msg->data) || msg->data < 0.0) return;
    current_speed_mps_ = msg->data;
    speed_received_ = true;
    last_speed_receipt_ = ros::Time::now();
  }

  void statusCallback(const std_msgs::Bool::ConstPtr& msg) {
    status_received_ = true;
    status_valid_ = msg->data;
  }

  void regionCallback(const std_msgs::String::ConstPtr& msg) {
    adas::HighwayRegion parsed = adas::HighwayRegion::NONE;
    region_valid_ = adapter::parseRegion(msg->data, &parsed);
    if (region_valid_) {
      current_region_ = parsed;
      current_region_name_ = msg->data;
      last_region_receipt_ = ros::Time::now();
    } else {
      ROS_ERROR_THROTTLE(1.0, "[highway_adas] unknown region: %s",
                         msg->data.c_str());
    }
  }

  void guardDistanceCallback(const std_msgs::Float64::ConstPtr& msg) {
    if (std::isfinite(msg->data) && msg->data >= 0.0) {
      distance_to_guard_stop_m_ = msg->data;
      last_guard_distance_receipt_ = ros::Time::now();
    }
  }

  void nextLimitDistanceCallback(const std_msgs::Float64::ConstPtr& msg) {
    if (std::isfinite(msg->data) && msg->data >= 0.0) {
      distance_to_next_limit_m_ = msg->data;
      last_next_limit_distance_receipt_ = ros::Time::now();
    }
  }

  void lane1Callback(const nav_msgs::Path::ConstPtr& msg) {
    laneCallback(msg, 1);
  }

  void lane2Callback(const nav_msgs::Path::ConstPtr& msg) {
    laneCallback(msg, 2);
  }

  void lane3Callback(const nav_msgs::Path::ConstPtr& msg) {
    laneCallback(msg, 3);
  }

  void lane4Callback(const nav_msgs::Path::ConstPtr& msg) {
    laneCallback(msg, 4);
  }

  void laneCallback(const nav_msgs::Path::ConstPtr& msg, int lane_id) {
    if (msg->header.frame_id != map_frame_ || msg->poses.size() < 2) {
      ROS_WARN_THROTTLE(1.0,
                        "[highway_adas] lane %d must contain a map-frame path",
                        lane_id);
      return;
    }
    adas::LanePath lane;
    lane.id = lane_id;
    lane.width_m = lane_width_m_;
    lane.left_neighbor_id = lane_id == 1 ? -1 : lane_id - 1;
    lane.right_neighbor_id = lane_id == 4 ? -1 : lane_id + 1;
    lane.centerline_map.reserve(msg->poses.size());
    for (const geometry_msgs::PoseStamped& pose : msg->poses) {
      lane.centerline_map.push_back(
          {pose.pose.position.x, pose.pose.position.y});
    }
    lanes_[lane_id] = lane;
  }

  void objectsCallback(const gadis_perception_msgs::ObjectArray::ConstPtr& msg) {
    objects_received_ = true;
    last_objects_receipt_ = ros::Time::now();
    latest_sensor_healthy_ =
        msg->sensor_healthy && msg->sensor_stale_s <= input_timeout_sec_;
    if (!latest_sensor_healthy_) return;
    if (msg->header.stamp.isZero() || msg->header.frame_id != lidar_frame_) {
      latest_sensor_healthy_ = false;
      ROS_WARN_THROTTLE(1.0,
                        "[highway_adas] objects must be stamped in velodyne frame");
      return;
    }
    adas::EgoState detection_ego;
    if (!pose_history_.interpolate(msg->header.stamp, &detection_ego)) {
      latest_sensor_healthy_ = false;
      ROS_WARN_THROTTLE(1.0,
                        "[highway_adas] no ego pose at object measurement time");
      return;
    }
    std::vector<adas::ObjectDetection> detections;
    detections.reserve(msg->objects.size());
    for (const gadis_perception_msgs::Object& object : msg->objects) {
      adas::ObjectDetection detection;
      if (object_converter_.convert(object, &detection)) {
        detections.push_back(detection);
      }
    }
    accepted_detection_stamp_sec_ = msg->header.stamp.toSec();
    accepted_detection_ego_ = detection_ego;
    accepted_detections_ = detections;
  }

  bool inputsReady(const ros::Time& now, std::string* reason) const {
    if (!odom_received_ || !speed_received_ || !objects_received_) {
      *reason = "waiting for odom, speed, and objects";
      return false;
    }
    if (lanes_.size() != 4 || !region_valid_) {
      *reason = "waiting for four lane centerlines and a valid region";
      return false;
    }
    if (require_vehicle_status_ && (!status_received_ || !status_valid_)) {
      *reason = "vehicle status is not valid";
      return false;
    }
    if ((now - last_odom_receipt_).toSec() > input_timeout_sec_ ||
        (now - last_speed_receipt_).toSec() > input_timeout_sec_) {
      *reason = "ego input is stale";
      return false;
    }
    if (last_region_receipt_.isZero() ||
        (now - last_region_receipt_).toSec() > input_timeout_sec_) {
      *reason = "highway region input is stale";
      return false;
    }
    const bool guard_region =
        current_region_ == adas::HighwayRegion::HW_ENTRY_GUARD ||
        current_region_ == adas::HighwayRegion::HW_MAIN_GUARD_1 ||
        current_region_ == adas::HighwayRegion::HW_MAIN_GUARD_2;
    if (guard_region &&
        (!std::isfinite(distance_to_guard_stop_m_) ||
         last_guard_distance_receipt_.isZero() ||
         (now - last_guard_distance_receipt_).toSec() > input_timeout_sec_)) {
      *reason = "guard region requires a fresh distance_to_guard_stop";
      return false;
    }
    if (current_region_ == adas::HighwayRegion::HW_TOLL &&
        (!std::isfinite(distance_to_next_limit_m_) ||
         last_next_limit_distance_receipt_.isZero() ||
         (now - last_next_limit_distance_receipt_).toSec() >
             input_timeout_sec_)) {
      *reason = "toll region requires a fresh distance_to_next_limit";
      return false;
    }
    return true;
  }

  std::vector<adas::LanePath> laneVector() const {
    return {lanes_.at(1), lanes_.at(2), lanes_.at(3), lanes_.at(4)};
  }

  nav_msgs::Path toLocalPath(const std::vector<adas::Point2>& map_path,
                             const adas::EgoState& ego,
                             double speed_kph,
                             const ros::Time& stamp) const {
    nav_msgs::Path output;
    output.header.stamp = stamp;
    output.header.frame_id = base_frame_;
    output.poses.reserve(map_path.size());
    const double c = std::cos(ego.yaw_rad);
    const double s = std::sin(ego.yaw_rad);
    for (const adas::Point2& point : map_path) {
      const double dx = point.x - ego.position_map.x;
      const double dy = point.y - ego.position_map.y;
      geometry_msgs::PoseStamped pose;
      pose.header = output.header;
      pose.pose.position.x = c * dx + s * dy;
      pose.pose.position.y = -s * dx + c * dy;
      pose.pose.position.z = speed_kph;
      pose.pose.orientation.w = 1.0;
      output.poses.push_back(pose);
    }
    return output;
  }

  void publishInvalid(const std::string& reason) {
    publishActive(true);
    std_msgs::Float64 speed;
    speed.data = 0.0;
    target_speed_mps_pub_.publish(speed);
    target_speed_kph_pub_.publish(speed);
    std_msgs::String diagnostics;
    diagnostics.data = "valid=0 reason=" + reason;
    diagnostics_pub_.publish(diagnostics);
    ROS_WARN_THROTTLE(1.0, "[highway_adas] %s", reason.c_str());
  }

  void publishInactive() {
    publishActive(false);
    std_msgs::String diagnostics;
    diagnostics.data = "valid=0 active=0 region=NONE";
    diagnostics_pub_.publish(diagnostics);
  }

  void publishScalar(ros::Publisher& publisher, double value) {
    std_msgs::Float64 message;
    message.data = value;
    publisher.publish(message);
  }

  void publishActive(bool active) {
    std_msgs::Bool message;
    message.data = active;
    active_pub_.publish(message);
  }

  void publishDecision(const adas::HighwayDecisionOutput& output,
                       const ros::Time& stamp) {
    publishActive(true);
    const double target_speed_kph = output.target_speed_mps * 3.6;
    path_pub_.publish(toLocalPath(output.adas.target_path_map, current_ego_,
                                  target_speed_kph, stamp));
    publishScalar(target_speed_mps_pub_, output.target_speed_mps);
    publishScalar(target_speed_kph_pub_, target_speed_kph);
    publishScalar(front_ttc_pub_, output.adas.target_gap.front_ttc_sec);
    publishScalar(rear_ttc_pub_, output.adas.target_gap.rear_ttc_sec);
    publishScalar(front_gap_pub_, output.adas.target_gap.front_gap_m);
    publishScalar(rear_gap_pub_, output.adas.target_gap.rear_gap_m);

    std_msgs::String text;
    text.data = adapter::laneChangeStateName(output.adas.lane_change_state);
    fsm_pub_.publish(text);
    text.data = adapter::safetyActionName(output.adas.safety_action);
    safety_pub_.publish(text);
    text.data = adapter::longitudinalModeName(output.adas.longitudinal.mode);
    behavior_pub_.publish(text);

    std_msgs::Int32 lane;
    lane.data = output.adas.current_lane_id;
    current_lane_pub_.publish(lane);
    lane.data = output.adas.target_lane_id;
    target_lane_pub_.publish(lane);

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "valid=1 region=" << current_region_name_
           << " lane=" << output.adas.current_lane_id << "->"
           << output.adas.target_lane_id
           << " fsm="
           << adapter::laneChangeStateName(output.adas.lane_change_state)
           << " action=" << adapter::safetyActionName(output.adas.safety_action)
           << " longitudinal="
           << adapter::longitudinalModeName(output.adas.longitudinal.mode)
           << " speed_kph=" << target_speed_kph
           << " front_gap=" << output.adas.target_gap.front_gap_m
           << " rear_gap=" << output.adas.target_gap.rear_gap_m;
    text.data = stream.str();
    diagnostics_pub_.publish(text);
    ROS_INFO_THROTTLE(1.0, "%s", text.data.c_str());
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (region_valid_ && current_region_ == adas::HighwayRegion::NONE) {
      decision_.reset();
      publishInactive();
      return;
    }
    std::string reason;
    if (!inputsReady(now, &reason)) {
      publishInvalid(reason);
      return;
    }

    const bool perception_fresh = latest_sensor_healthy_ &&
        (now - last_objects_receipt_).toSec() <= input_timeout_sec_ &&
        accepted_detection_stamp_sec_ > 0.0;

    adas::HighwayDecisionInput input;
    input.region = current_region_;
    input.distance_to_guard_stop_m = distance_to_guard_stop_m_;
    input.distance_to_next_limit_m = distance_to_next_limit_m_;
    input.adas.ego = current_ego_;
    input.adas.ego.speed_mps = current_speed_mps_;
    input.adas.ego.stamp_sec = now.toSec();
    input.adas.perception_healthy = perception_fresh;
    input.adas.detection_stamp_sec = accepted_detection_stamp_sec_;
    input.adas.detection_ego = accepted_detection_ego_;
    input.adas.detections = accepted_detections_;
    input.adas.lanes = laneVector();

    const adas::HighwayDecisionOutput output = decision_.update(input);
    if (!output.valid) {
      publishInvalid(output.reason);
      return;
    }
    publishDecision(output, now);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "highway_adas");
  HighwayAdasNode node;
  ros::spin();
  return 0;
}
