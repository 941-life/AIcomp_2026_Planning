#ifndef HIGHWAY_ADAS_ROS_ROS_ADAPTER_HPP
#define HIGHWAY_ADAS_ROS_ROS_ADAPTER_HPP

#include "highway_adas/types.hpp"
#include "highway_adas/highway_region_policy.hpp"

#include <gadis_perception_msgs/Object.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>

#include <deque>
#include <string>

namespace highway_adas_ros {

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
bool parseRegion(const std::string& text,
                 highway_adas::HighwayRegion* region);
std::string laneChangeStateName(highway_adas::LaneChangeState state);
std::string safetyActionName(highway_adas::SafetyAction action);
std::string longitudinalModeName(highway_adas::LongitudinalMode mode);

class EgoPoseHistory {
 public:
  void configure(double history_sec, double sync_tolerance_sec);
  void add(const ros::Time& stamp, const highway_adas::EgoState& ego);
  bool interpolate(const ros::Time& stamp,
                   highway_adas::EgoState* ego) const;

 private:
  struct Sample {
    ros::Time stamp;
    highway_adas::EgoState ego;
  };

  double history_sec_ = 2.0;
  double sync_tolerance_sec_ = 0.05;
  std::deque<Sample> samples_;
};

class ObjectConverter {
 public:
  void configure(double lidar_to_base_x_m,
                 double lidar_to_base_y_m,
                 double lidar_to_base_yaw_rad,
                 double minimum_dimension_m,
                 double maximum_dimension_m);
  bool convert(const gadis_perception_msgs::Object& object,
               highway_adas::ObjectDetection* detection) const;

 private:
  double lidar_to_base_x_m_ = 1.144;
  double lidar_to_base_y_m_ = 0.0;
  double lidar_to_base_yaw_rad_ = 0.0;
  double minimum_dimension_m_ = 0.2;
  double maximum_dimension_m_ = 20.0;
};

}  // namespace highway_adas_ros

#endif  // HIGHWAY_ADAS_ROS_ROS_ADAPTER_HPP
