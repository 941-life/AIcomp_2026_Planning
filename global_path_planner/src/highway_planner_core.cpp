#include "global_path_planner/highway_planner_core.hpp"

#include <algorithm>
#include <cmath>

namespace highway_planner
{
namespace
{

double normalizeAngle(const double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double clamp(const double value, const double minimum, const double maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

double distance(const Point2D& a, const Point2D& b)
{
    return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

bool RouteModel::setPath(const std::vector<Point2D>& input,
                         const bool circular,
                         const int curvature_half_window,
                         const double max_lateral_accel,
                         const double maximum_speed)
{
    points_.clear();
    length_ = 0.0;
    circular_ = circular;

    if (input.size() < 3 ||
        curvature_half_window < 1 ||
        max_lateral_accel <= 0.0 ||
        maximum_speed <= 0.0) {
        return false;
    }

    points_.resize(input.size());
    for (std::size_t i = 0; i < input.size(); ++i) {
        points_[i].x = input[i].x;
        points_[i].y = input[i].y;
        if (i > 0) {
            length_ += distance(input[i - 1], input[i]);
        }
        points_[i].s = length_;
    }

    if (circular_) {
        length_ += distance(input.back(), input.front());
    }
    if (length_ <= 0.0) {
        points_.clear();
        return false;
    }

    const int size = static_cast<int>(points_.size());
    for (int i = 0; i < size; ++i) {
        const int previous = circular_
                                 ? wrapIndex(i - curvature_half_window)
                                 : std::max(0, i - curvature_half_window);
        const int next = circular_
                             ? wrapIndex(i + curvature_half_window)
                             : std::min(size - 1, i + curvature_half_window);

        const RoutePoint& p0 = points_[previous];
        const RoutePoint& p1 = points_[i];
        const RoutePoint& p2 = points_[next];

        points_[i].yaw = std::atan2(p2.y - p0.y, p2.x - p0.x);

        const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
        const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
        const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);
        const double denominator = a * b * c;
        if (denominator > 1e-9) {
            const double cross =
                (p1.x - p0.x) * (p2.y - p0.y) -
                (p1.y - p0.y) * (p2.x - p0.x);
            points_[i].curvature = 2.0 * cross / denominator;
        }

        const double absolute_curvature = std::fabs(points_[i].curvature);
        points_[i].curve_speed =
            absolute_curvature > 1e-6
                ? std::min(maximum_speed, std::sqrt(max_lateral_accel / absolute_curvature))
                : maximum_speed;
    }

    return true;
}

int RouteModel::wrapIndex(const int index) const
{
    const int size = static_cast<int>(points_.size());
    const int wrapped = index % size;
    return wrapped < 0 ? wrapped + size : wrapped;
}

Projection RouteModel::search(const double x,
                              const double y,
                              const double vehicle_yaw,
                              int start_index,
                              int end_index,
                              const bool wrap,
                              const double heading_weight) const
{
    Projection best;
    double best_cost = std::numeric_limits<double>::infinity();
    const int size = static_cast<int>(points_.size());

    if (wrap) {
        const int count = std::min(size, end_index - start_index + 1);
        for (int offset = 0; offset < count; ++offset) {
            const int index = wrapIndex(start_index + offset);
            const RoutePoint& point = points_[index];
            const double dx = point.x - x;
            const double dy = point.y - y;
            const double distance_sq = dx * dx + dy * dy;
            const double heading_error = normalizeAngle(point.yaw - vehicle_yaw);
            const double cost = distance_sq + heading_weight * heading_error * heading_error;
            if (cost < best_cost) {
                best_cost = cost;
                best.valid = true;
                best.index = index;
                best.s = point.s;
                best.distance = std::sqrt(distance_sq);
            }
        }
        return best;
    }

    start_index = std::max(0, start_index);
    end_index = std::min(size - 1, end_index);
    for (int index = start_index; index <= end_index; ++index) {
        const RoutePoint& point = points_[index];
        const double dx = point.x - x;
        const double dy = point.y - y;
        const double distance_sq = dx * dx + dy * dy;
        const double heading_error = normalizeAngle(point.yaw - vehicle_yaw);
        const double cost = distance_sq + heading_weight * heading_error * heading_error;
        if (cost < best_cost) {
            best_cost = cost;
            best.valid = true;
            best.index = index;
            best.s = point.s;
            best.distance = std::sqrt(distance_sq);
        }
    }
    return best;
}

Projection RouteModel::project(const double x,
                               const double y,
                               const double vehicle_yaw,
                               const int previous_index,
                               const int search_radius,
                               const double relocalization_distance,
                               const double heading_weight) const
{
    if (points_.empty()) {
        return Projection();
    }

    if (previous_index < 0 || previous_index >= static_cast<int>(points_.size())) {
        return search(
            x, y, vehicle_yaw, 0, static_cast<int>(points_.size()) - 1, false, heading_weight);
    }

    Projection result = search(
        x,
        y,
        vehicle_yaw,
        previous_index - search_radius,
        previous_index + search_radius,
        circular_,
        heading_weight);

    if (!result.valid || result.distance > relocalization_distance) {
        result = search(
            x, y, vehicle_yaw, 0, static_cast<int>(points_.size()) - 1, false, heading_weight);
    }
    return result;
}

double RouteModel::curveSpeedLimit(const int start_index,
                                   const double lookahead_distance,
                                   const double planned_deceleration) const
{
    if (points_.empty() || start_index < 0 ||
        start_index >= static_cast<int>(points_.size())) {
        return 0.0;
    }

    double limit = points_[start_index].curve_speed;
    double ahead = 0.0;
    int index = start_index;
    const int maximum_steps = circular_
                                  ? static_cast<int>(points_.size())
                                  : static_cast<int>(points_.size()) - start_index - 1;

    for (int step = 0; step < maximum_steps && ahead < lookahead_distance; ++step) {
        const int next = circular_ ? wrapIndex(index + 1) : index + 1;
        if (next >= static_cast<int>(points_.size())) {
            break;
        }

        ahead += std::hypot(
            points_[next].x - points_[index].x,
            points_[next].y - points_[index].y);
        const double allowable = std::sqrt(
            points_[next].curve_speed * points_[next].curve_speed +
            2.0 * planned_deceleration * ahead);
        limit = std::min(limit, allowable);
        index = next;
    }
    return limit;
}

double RouteModel::forwardDistance(const double from_s, const double to_s) const
{
    if (length_ <= 0.0) {
        return 0.0;
    }
    if (!circular_ || to_s >= from_s) {
        return std::max(0.0, to_s - from_s);
    }
    return length_ - from_s + to_s;
}

const char* toString(const ZoneState state)
{
    switch (state) {
    case ZoneState::APPROACH:
        return "APPROACH";
    case ZoneState::ENTRY_CONFIRM:
        return "ENTRY_CONFIRM";
    case ZoneState::HIGH_SPEED:
        return "HIGH_SPEED";
    case ZoneState::BRAKE_TO_LIMIT:
        return "BRAKE_TO_LIMIT";
    case ZoneState::LIMITED:
        return "LIMITED";
    }
    return "LIMITED";
}

bool ZonePlanner::configure(const ZoneConfig& config,
                            const RouteModel& route,
                            std::string& error)
{
    config_ = config;
    configured_ = false;
    error.clear();

    if (config_.limited_speed <= 0.0 ||
        config_.highway_speed < config_.limited_speed ||
        config_.planned_deceleration <= 0.0 ||
        config_.limit_margin < 0.0 ||
        config_.system_delay < 0.0) {
        error = "invalid speed or deceleration parameter";
        return false;
    }

    if (!config_.enabled) {
        configured_ = true;
        return true;
    }

    const double length = route.length();
    const auto valid_s = [length](const double s) { return s >= 0.0 && s < length; };
    if (!valid_s(config_.entry_s) ||
        !valid_s(config_.high_speed_start_s) ||
        !valid_s(config_.brake_start_s) ||
        !valid_s(config_.limit_start_s)) {
        error = "zone s parameter is outside the route";
        return false;
    }

    entry_to_high_speed_ =
        route.forwardDistance(config_.entry_s, config_.high_speed_start_s);
    entry_to_brake_ = route.forwardDistance(config_.entry_s, config_.brake_start_s);
    entry_to_limit_ = route.forwardDistance(config_.entry_s, config_.limit_start_s);

    if (!(entry_to_high_speed_ > 0.0 &&
          entry_to_brake_ > entry_to_high_speed_ &&
          entry_to_limit_ > entry_to_brake_)) {
        error = "zone order must be entry -> high_speed -> brake -> limit";
        return false;
    }

    configured_ = true;
    return true;
}

ZoneResult ZonePlanner::evaluate(const double current_s,
                                 const double ego_speed,
                                 const RouteModel& route) const
{
    ZoneResult result;
    result.zone_speed = config_.limited_speed;
    result.tollgate_speed = config_.limited_speed;

    if (!configured_ || !config_.enabled) {
        return result;
    }

    const double phase = route.forwardDistance(config_.entry_s, current_s);
    const double distance_to_entry = route.forwardDistance(current_s, config_.entry_s);
    result.distance_to_limit = route.forwardDistance(current_s, config_.limit_start_s);

    if (phase <= entry_to_high_speed_) {
        result.state = ZoneState::ENTRY_CONFIRM;
    } else if (phase < entry_to_brake_) {
        result.state = ZoneState::HIGH_SPEED;
    } else if (phase < entry_to_limit_) {
        result.state = ZoneState::BRAKE_TO_LIMIT;
    } else if (distance_to_entry <= config_.approach_distance) {
        result.state = ZoneState::APPROACH;
    } else {
        result.state = ZoneState::LIMITED;
    }

    const bool high_speed_allowed =
        result.state == ZoneState::HIGH_SPEED ||
        result.state == ZoneState::BRAKE_TO_LIMIT;
    result.zone_speed = high_speed_allowed
                            ? config_.highway_speed
                            : config_.limited_speed;

    if (high_speed_allowed) {
        const double effective_distance = std::max(
            0.0,
            result.distance_to_limit -
                config_.limit_margin -
                ego_speed * config_.system_delay);
        result.tollgate_speed = std::sqrt(
            config_.limited_speed * config_.limited_speed +
            2.0 * config_.planned_deceleration * effective_distance);
    }

    return result;
}

LongitudinalResult LongitudinalPlanner::plan(
    const double ego_speed,
    const double free_flow_speed,
    const LeaderObservation& leader) const
{
    LongitudinalResult result;
    result.target_speed = std::max(0.0, free_flow_speed);

    if (!leader.valid) {
        return result;
    }

    result.following = true;
    const double gap = std::max(0.0, leader.distance);
    const double closing_speed = std::max(0.0, -leader.relative_speed);
    const double leader_speed = std::max(0.0, ego_speed + leader.relative_speed);
    const double desired_gap = config_.standstill_gap + config_.time_headway * ego_speed;

    if (closing_speed > 1e-3) {
        result.ttc = gap / closing_speed;
        const double braking_gap = std::max(0.1, gap - config_.minimum_gap);
        result.required_deceleration =
            closing_speed * closing_speed / (2.0 * braking_gap);
    }

    result.emergency =
        gap <= config_.minimum_gap ||
        result.ttc <= config_.emergency_ttc ||
        result.required_deceleration >= config_.emergency_deceleration;
    if (result.emergency) {
        result.target_speed = 0.0;
        return result;
    }

    double acceleration =
        config_.gap_gain * (gap - desired_gap) +
        config_.relative_speed_gain * leader.relative_speed;
    acceleration = clamp(
        acceleration,
        -config_.comfortable_deceleration,
        config_.maximum_acceleration);

    const double acc_speed = std::max(
        0.0,
        std::min(
            leader_speed + config_.gap_gain * std::max(0.0, gap - desired_gap),
            ego_speed + acceleration * config_.response_time));
    result.target_speed = std::min(result.target_speed, acc_speed);
    return result;
}

void SpeedCommandFilter::reset(const double speed)
{
    initialized_ = true;
    speed_ = std::max(0.0, speed);
    acceleration_ = 0.0;
}

double SpeedCommandFilter::update(const double raw_target_speed, const double dt)
{
    const double target = std::max(0.0, raw_target_speed);
    if (!initialized_) {
        reset(target);
        return speed_;
    }
    if (dt <= 0.0) {
        return speed_;
    }

    double desired_acceleration =
        (target - speed_) / std::max(0.05, config_.response_time);
    desired_acceleration = clamp(
        desired_acceleration,
        -config_.maximum_deceleration,
        config_.maximum_acceleration);

    const double maximum_acceleration_change = config_.maximum_jerk * dt;
    acceleration_ += clamp(
        desired_acceleration - acceleration_,
        -maximum_acceleration_change,
        maximum_acceleration_change);

    const double next_speed = std::max(0.0, speed_ + acceleration_ * dt);
    if ((target - speed_) * (target - next_speed) <= 0.0) {
        speed_ = target;
        acceleration_ = 0.0;
    } else {
        speed_ = next_speed;
    }
    return speed_;
}

}  // namespace highway_planner
