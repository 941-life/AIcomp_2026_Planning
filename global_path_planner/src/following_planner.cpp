#include "global_path_planner/following_planner.hpp"

#include <algorithm>
#include <cmath>

namespace highway_planner
{
namespace
{

double clamp(const double value, const double minimum, const double maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

}  // namespace

FollowingPlanner::FollowingPlanner(const FollowingConfig& config)
    : config_(config)
{
    config_.relative_speed_alpha =
        clamp(config_.relative_speed_alpha, 0.0, 1.0);
}

void FollowingPlanner::observe(
    const std::vector<ObjectObservation>& objects,
    const int active_path,
    const double stamp)
{
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const ObjectObservation& object : objects) {
        if (object.path_number == active_path &&
            object.distance > 0.0 &&
            object.distance <= config_.maximum_distance) {
            nearest_distance = std::min(nearest_distance, object.distance);
        }
    }

    if (!std::isfinite(nearest_distance)) {
        return;
    }

    double relative_speed = 0.0;
    if (leader_valid_ && leader_path_ == active_path) {
        const double dt = stamp - leader_stamp_;
        if (dt > 1e-3 && dt <= config_.hold_time) {
            const double predicted_distance =
                leader_distance_ + leader_relative_speed_ * dt;
            const bool same_leader =
                std::fabs(nearest_distance - predicted_distance) <=
                config_.switch_distance;
            if (same_leader) {
                const double measured_relative_speed = clamp(
                    (nearest_distance - leader_distance_) / dt,
                    -config_.maximum_relative_speed,
                    config_.maximum_relative_speed);
                relative_speed =
                    leader_relative_speed_ +
                    config_.relative_speed_alpha *
                        (measured_relative_speed - leader_relative_speed_);
            }
        }
    }

    leader_valid_ = true;
    leader_path_ = active_path;
    leader_distance_ = nearest_distance;
    leader_relative_speed_ = relative_speed;
    leader_stamp_ = stamp;
}

LeaderObservation FollowingPlanner::leaderObservation(
    const int active_path,
    const double now) const
{
    LeaderObservation leader;
    if (!leader_valid_ || leader_path_ != active_path) {
        return leader;
    }

    const double age = now - leader_stamp_;
    if (age < 0.0 || age > config_.hold_time) {
        return leader;
    }

    leader.valid = true;
    leader.relative_speed = leader_relative_speed_;
    leader.distance = std::max(
        0.0,
        leader_distance_ + leader_relative_speed_ * age);
    return leader;
}

FollowingResult FollowingPlanner::plan(
    const int active_path,
    const double ego_speed,
    const double free_flow_speed,
    const double now) const
{
    const LeaderObservation leader = leaderObservation(active_path, now);
    return planWithLeader(ego_speed, free_flow_speed, leader);
}

FollowingResult FollowingPlanner::planWithLeader(
    const double ego_speed,
    const double free_flow_speed,
    const LeaderObservation& leader) const
{
    FollowingResult result;
    result.target_speed = std::max(0.0, free_flow_speed);

    if (!leader.valid) {
        return result;
    }

    result.following = true;
    result.leader_distance = std::max(0.0, leader.distance);
    result.relative_speed = leader.relative_speed;

    const double closing_speed = std::max(0.0, -leader.relative_speed);
    const double leader_speed = std::max(0.0, ego_speed + leader.relative_speed);
    const double desired_gap =
        config_.acc.standstill_gap + config_.acc.time_headway * ego_speed;

    if (closing_speed > 1e-3) {
        result.ttc = result.leader_distance / closing_speed;
        const double braking_gap =
            std::max(0.1, result.leader_distance - config_.acc.minimum_gap);
        result.required_deceleration =
            closing_speed * closing_speed / (2.0 * braking_gap);
    }

    result.emergency =
        result.leader_distance <= config_.acc.minimum_gap ||
        result.ttc <= config_.acc.emergency_ttc ||
        result.required_deceleration >= config_.acc.emergency_deceleration;
    if (result.emergency) {
        result.target_speed = 0.0;
        return result;
    }

    double acceleration =
        config_.acc.gap_gain * (result.leader_distance - desired_gap) +
        config_.acc.relative_speed_gain * leader.relative_speed;
    acceleration = clamp(
        acceleration,
        -config_.acc.comfortable_deceleration,
        config_.acc.maximum_acceleration);

    const double acc_speed = std::max(
        0.0,
        std::min(
            leader_speed +
                config_.acc.gap_gain *
                    std::max(0.0, result.leader_distance - desired_gap),
            ego_speed + acceleration * config_.acc.response_time));
    result.target_speed = std::min(result.target_speed, acc_speed);
    return result;
}

void FollowingPlanner::reset()
{
    leader_valid_ = false;
    leader_path_ = 0;
    leader_distance_ = 0.0;
    leader_relative_speed_ = 0.0;
    leader_stamp_ = 0.0;
}

}  // namespace highway_planner
