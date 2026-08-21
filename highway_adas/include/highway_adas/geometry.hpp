#ifndef HIGHWAY_ADAS_GEOMETRY_HPP
#define HIGHWAY_ADAS_GEOMETRY_HPP

#include "highway_adas/types.hpp"

#include <cstddef>
#include <vector>

namespace highway_adas {

double clamp(double value, double low, double high);
double normalizeAngle(double angle_rad);
double distance(const Point2& a, const Point2& b);
Point2 baseLinkToMap(const EgoState& ego, const Point2& point_base);

struct PathProjection {
  bool valid = false;
  double s_m = 0.0;
  double lateral_m = 0.0;
  double distance_m = 0.0;
  double heading_rad = 0.0;
  std::size_t segment_index = 0;
};

struct PathSample {
  bool valid = false;
  Point2 point;
  double heading_rad = 0.0;
};

class Polyline {
 public:
  explicit Polyline(const std::vector<Point2>& points);

  bool valid() const { return points_.size() >= 2 && length_m_ > 0.0; }
  double length() const { return length_m_; }
  const std::vector<Point2>& points() const { return points_; }

  PathProjection project(const Point2& point) const;
  PathSample sample(double s_m) const;

 private:
  std::vector<Point2> points_;
  std::vector<double> accumulated_s_;
  double length_m_ = 0.0;
};

}  // namespace highway_adas

#endif  // HIGHWAY_ADAS_GEOMETRY_HPP

