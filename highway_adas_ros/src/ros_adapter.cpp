#include "highway_adas_ros/ros_adapter.hpp"

#include <geometry_msgs/Point32.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace adas = highway_adas;

namespace highway_adas_ros {
namespace {

double normalizeAngle(double angle) {
  while (angle > adas::kPi) angle -= 2.0 * adas::kPi;
  while (angle < -adas::kPi) angle += 2.0 * adas::kPi;
  return angle;
}

}  // namespace

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
  const double sin_yaw =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

bool parseRegion(const std::string& text, adas::HighwayRegion* region) {
  if (text == "NONE") {
    *region = adas::HighwayRegion::NONE;
  } else if (text == "HW_ENTRY") {
    *region = adas::HighwayRegion::HW_ENTRY;
  } else if (text == "HW_ENTRY_GUARD") {
    *region = adas::HighwayRegion::HW_ENTRY_GUARD;
  } else if (text == "HW_MAIN") {
    *region = adas::HighwayRegion::HW_MAIN;
  } else if (text == "HW_MAIN_GUARD_1") {
    *region = adas::HighwayRegion::HW_MAIN_GUARD_1;
  } else if (text == "HW_MAIN_GUARD_2") {
    *region = adas::HighwayRegion::HW_MAIN_GUARD_2;
  } else if (text == "HW_TOLL") {
    *region = adas::HighwayRegion::HW_TOLL;
  } else {
    return false;
  }
  return true;
}

std::string laneChangeStateName(adas::LaneChangeState state) {
  switch (state) {
    case adas::LaneChangeState::KEEP_LANE: return "KEEP_LANE";
    case adas::LaneChangeState::CHECK_GAP: return "CHECK_GAP";
    case adas::LaneChangeState::EXECUTE: return "EXECUTE";
    case adas::LaneChangeState::SETTLE: return "SETTLE";
  }
  return "UNKNOWN";
}

std::string safetyActionName(adas::SafetyAction action) {
  switch (action) {
    case adas::SafetyAction::NONE: return "NONE";
    case adas::SafetyAction::START_BLOCKED: return "START_BLOCKED";
    case adas::SafetyAction::EARLY_CANCEL: return "EARLY_CANCEL";
    case adas::SafetyAction::CONTINUE_FRONT_DECELERATE:
      return "CONTINUE_FRONT_DECELERATE";
    case adas::SafetyAction::CONTINUE_HOLD_SPEED:
      return "CONTINUE_HOLD_SPEED";
    case adas::SafetyAction::STALE_PERCEPTION_HOLD:
      return "STALE_PERCEPTION_HOLD";
  }
  return "UNKNOWN";
}

std::string longitudinalModeName(adas::LongitudinalMode mode) {
  switch (mode) {
    case adas::LongitudinalMode::CRUISE: return "CRUISE";
    case adas::LongitudinalMode::FOLLOW: return "FOLLOW";
    case adas::LongitudinalMode::HARD_BRAKE: return "HARD_BRAKE";
    case adas::LongitudinalMode::EMERGENCY_BRAKE: return "EMERGENCY_BRAKE";
  }
  return "UNKNOWN";
}

void EgoPoseHistory::configure(double history_sec,
                               double sync_tolerance_sec) {
  history_sec_ = std::max(0.1, history_sec);
  sync_tolerance_sec_ = std::max(0.0, sync_tolerance_sec);
}

void EgoPoseHistory::add(const ros::Time& stamp, const adas::EgoState& ego) {
  samples_.push_back({stamp, ego});
  while (!samples_.empty() &&
         (stamp - samples_.front().stamp).toSec() > history_sec_) {
    samples_.pop_front();
  }
}

bool EgoPoseHistory::interpolate(const ros::Time& stamp,
                                 adas::EgoState* ego) const {
  if (samples_.empty()) return false;
  if (stamp <= samples_.front().stamp) {
    if ((samples_.front().stamp - stamp).toSec() > sync_tolerance_sec_) {
      return false;
    }
    *ego = samples_.front().ego;
    ego->stamp_sec = stamp.toSec();
    return true;
  }
  if (stamp >= samples_.back().stamp) {
    if ((stamp - samples_.back().stamp).toSec() > sync_tolerance_sec_) {
      return false;
    }
    *ego = samples_.back().ego;
    ego->stamp_sec = stamp.toSec();
    return true;
  }
  for (std::size_t i = 1; i < samples_.size(); ++i) {
    const Sample& first = samples_[i - 1];
    const Sample& second = samples_[i];
    if (stamp > second.stamp) continue;
    const double duration = (second.stamp - first.stamp).toSec();
    if (duration <= 0.0) return false;
    const double ratio = (stamp - first.stamp).toSec() / duration;
    ego->position_map.x = first.ego.position_map.x +
        ratio * (second.ego.position_map.x - first.ego.position_map.x);
    ego->position_map.y = first.ego.position_map.y +
        ratio * (second.ego.position_map.y - first.ego.position_map.y);
    ego->yaw_rad = normalizeAngle(
        first.ego.yaw_rad + ratio * normalizeAngle(
            second.ego.yaw_rad - first.ego.yaw_rad));
    ego->yaw_rate_radps = first.ego.yaw_rate_radps +
        ratio * (second.ego.yaw_rate_radps - first.ego.yaw_rate_radps);
    ego->stamp_sec = stamp.toSec();
    return true;
  }
  return false;
}

void ObjectConverter::configure(double lidar_to_base_x_m,
                                double lidar_to_base_y_m,
                                double lidar_to_base_yaw_rad,
                                double minimum_dimension_m,
                                double maximum_dimension_m) {
  lidar_to_base_x_m_ = lidar_to_base_x_m;
  lidar_to_base_y_m_ = lidar_to_base_y_m;
  lidar_to_base_yaw_rad_ = lidar_to_base_yaw_rad;
  minimum_dimension_m_ = minimum_dimension_m;
  maximum_dimension_m_ = maximum_dimension_m;
}

bool ObjectConverter::convert(const gadis_perception_msgs::Object& object,
                              adas::ObjectDetection* detection) const {
  if (object.is_predicted) return false;
  const double cosine = std::cos(lidar_to_base_yaw_rad_);
  const double sine = std::sin(lidar_to_base_yaw_rad_);
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const geometry_msgs::Point32& point : object.bbox3d_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
    const double x = lidar_to_base_x_m_ + cosine * point.x - sine * point.y;
    const double y = lidar_to_base_y_m_ + sine * point.x + cosine * point.y;
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
    sum_x += x;
    sum_y += y;
  }
  const double length = max_x - min_x;
  const double width = max_y - min_y;
  if (length < minimum_dimension_m_ || width < minimum_dimension_m_ ||
      length > maximum_dimension_m_ || width > maximum_dimension_m_) {
    return false;
  }
  detection->x_m = sum_x / object.bbox3d_points.size();
  detection->y_m = sum_y / object.bbox3d_points.size();
  detection->length_m = length;
  detection->width_m = width;
  return true;
}

}  // namespace highway_adas_ros
