#include "highway_adas/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace highway_adas {

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double normalizeAngle(double angle_rad) {
  while (angle_rad > kPi) angle_rad -= 2.0 * kPi;
  while (angle_rad < -kPi) angle_rad += 2.0 * kPi;
  return angle_rad;
}

double distance(const Point2& a, const Point2& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point2 baseLinkToMap(const EgoState& ego, const Point2& point_base) {
  const double c = std::cos(ego.yaw_rad);
  const double s = std::sin(ego.yaw_rad);
  return {ego.position_map.x + c * point_base.x - s * point_base.y,
          ego.position_map.y + s * point_base.x + c * point_base.y};
}

Polyline::Polyline(const std::vector<Point2>& points) {
  points_.reserve(points.size());
  for (const Point2& point : points) {
    if (points_.empty() || distance(points_.back(), point) > 1e-4) {
      points_.push_back(point);
    }
  }

  accumulated_s_.assign(points_.size(), 0.0);
  for (std::size_t i = 1; i < points_.size(); ++i) {
    length_m_ += distance(points_[i - 1], points_[i]);
    accumulated_s_[i] = length_m_;
  }
}

PathProjection Polyline::project(const Point2& point) const {
  PathProjection result;
  if (!valid()) return result;

  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
    const Point2 a = points_[i];
    const Point2 b = points_[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq <= 1e-8) continue;

    const double t = clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) /
                               length_sq,
                           0.0, 1.0);
    const Point2 nearest{a.x + t * dx, a.y + t * dy};
    const double error_x = point.x - nearest.x;
    const double error_y = point.y - nearest.y;
    const double distance_sq = error_x * error_x + error_y * error_y;
    if (distance_sq >= best_distance_sq) continue;

    const double segment_length = std::sqrt(length_sq);
    const double tangent_x = dx / segment_length;
    const double tangent_y = dy / segment_length;
    best_distance_sq = distance_sq;
    result.valid = true;
    result.s_m = accumulated_s_[i] + t * segment_length;
    result.lateral_m = tangent_x * error_y - tangent_y * error_x;
    result.distance_m = std::sqrt(distance_sq);
    result.heading_rad = std::atan2(dy, dx);
    result.segment_index = i;
  }
  return result;
}

PathSample Polyline::sample(double s_m) const {
  PathSample result;
  if (!valid()) return result;

  const double bounded_s = clamp(s_m, 0.0, length_m_);
  const auto upper = std::upper_bound(accumulated_s_.begin(), accumulated_s_.end(), bounded_s);
  std::size_t index = 0;
  if (upper == accumulated_s_.begin()) {
    index = 0;
  } else if (upper == accumulated_s_.end()) {
    index = points_.size() - 2;
  } else {
    index = static_cast<std::size_t>(upper - accumulated_s_.begin() - 1);
  }

  const Point2 a = points_[index];
  const Point2 b = points_[index + 1];
  const double segment_length = accumulated_s_[index + 1] - accumulated_s_[index];
  const double t = segment_length > 1e-8
                       ? (bounded_s - accumulated_s_[index]) / segment_length
                       : 0.0;
  result.valid = true;
  result.point = {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
  result.heading_rad = std::atan2(b.y - a.y, b.x - a.x);
  return result;
}

}  // namespace highway_adas
