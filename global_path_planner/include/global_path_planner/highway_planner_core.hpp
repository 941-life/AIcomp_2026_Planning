#pragma once

#include <limits>
#include <string>
#include <vector>

namespace highway_planner
{

struct Point2D
{
    double x = 0.0;
    double y = 0.0;
};

struct RoutePoint
{
    double x = 0.0;
    double y = 0.0;
    double s = 0.0;
    double yaw = 0.0;
    double curvature = 0.0;
    double curve_speed = 0.0;
};

struct Projection
{
    bool valid = false;
    int index = -1;
    double s = 0.0;
    double distance = std::numeric_limits<double>::infinity();
};

class RouteModel
{
public:
    bool setPath(const std::vector<Point2D>& points,
                 bool circular,
                 int curvature_half_window,
                 double max_lateral_accel,
                 double maximum_speed);

    Projection project(double x,
                       double y,
                       double vehicle_yaw,
                       int previous_index,
                       int search_radius,
                       double relocalization_distance,
                       double heading_weight) const;

    double curveSpeedLimit(int start_index,
                           double lookahead_distance,
                           double planned_deceleration) const;

    double forwardDistance(double from_s, double to_s) const;
    double length() const { return length_; }
    bool circular() const { return circular_; }
    bool empty() const { return points_.empty(); }
    const std::vector<RoutePoint>& points() const { return points_; }

private:
    int wrapIndex(int index) const;
    Projection search(double x,
                      double y,
                      double vehicle_yaw,
                      int start_index,
                      int end_index,
                      bool wrap,
                      double heading_weight) const;

    std::vector<RoutePoint> points_;
    bool circular_ = false;
    double length_ = 0.0;
};

enum class ZoneState
{
    APPROACH,
    ENTRY_CONFIRM,
    HIGH_SPEED,
    BRAKE_TO_LIMIT,
    LIMITED
};

const char* toString(ZoneState state);

struct ZoneConfig
{
    bool enabled = false;
    double entry_s = 0.0;
    double high_speed_start_s = 0.0;
    double brake_start_s = 0.0;
    double limit_start_s = 0.0;
    double approach_distance = 30.0;
    double limited_speed = 16.0;
    double highway_speed = 25.0;
    double planned_deceleration = 2.5;
    double limit_margin = 10.0;
    double system_delay = 0.3;
};

struct ZoneResult
{
    ZoneState state = ZoneState::LIMITED;
    double zone_speed = 0.0;
    double tollgate_speed = 0.0;
    double distance_to_limit = 0.0;
};

class ZonePlanner
{
public:
    bool configure(const ZoneConfig& config, const RouteModel& route, std::string& error);
    ZoneResult evaluate(double current_s, double ego_speed, const RouteModel& route) const;

private:
    ZoneConfig config_;
    bool configured_ = false;
    double entry_to_high_speed_ = 0.0;
    double entry_to_brake_ = 0.0;
    double entry_to_limit_ = 0.0;
};

struct LeaderObservation
{
    bool valid = false;
    double distance = 0.0;
    double relative_speed = 0.0;  // leader speed - ego speed
};

struct AccConfig
{
    double standstill_gap = 5.0;
    double time_headway = 1.5;
    double gap_gain = 0.25;
    double relative_speed_gain = 0.8;
    double response_time = 1.0;
    double maximum_acceleration = 2.0;
    double comfortable_deceleration = 2.5;
    double emergency_deceleration = 5.0;
    double minimum_gap = 3.0;
    double emergency_ttc = 1.5;
};

struct LongitudinalResult
{
    double target_speed = 0.0;
    double ttc = std::numeric_limits<double>::infinity();
    double required_deceleration = 0.0;
    bool following = false;
    bool emergency = false;
};

class LongitudinalPlanner
{
public:
    explicit LongitudinalPlanner(const AccConfig& config) : config_(config) {}

    LongitudinalResult plan(double ego_speed,
                            double free_flow_speed,
                            const LeaderObservation& leader) const;

private:
    AccConfig config_;
};

struct SpeedFilterConfig
{
    double maximum_acceleration = 2.0;
    double maximum_deceleration = 3.0;
    double maximum_jerk = 1.5;
    double response_time = 0.8;
};

class SpeedCommandFilter
{
public:
    explicit SpeedCommandFilter(const SpeedFilterConfig& config) : config_(config) {}

    void reset(double speed);
    double update(double raw_target_speed, double dt);
    double speed() const { return speed_; }

private:
    SpeedFilterConfig config_;
    bool initialized_ = false;
    double speed_ = 0.0;
    double acceleration_ = 0.0;
};

}  // namespace highway_planner
