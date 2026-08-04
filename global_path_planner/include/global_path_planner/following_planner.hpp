#pragma once

#include <limits>
#include <vector>

namespace highway_planner
{

struct ObjectObservation
{
    int path_number = 0;
    double distance = 0.0;
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

struct FollowingConfig
{
    double maximum_distance = 120.0;
    double hold_time = 0.3;
    double switch_distance = 8.0;
    double relative_speed_alpha = 0.3;
    double maximum_relative_speed = 30.0;
    AccConfig acc;
};

struct FollowingResult
{
    double target_speed = 0.0;
    double leader_distance = 0.0;
    double relative_speed = 0.0;
    double ttc = std::numeric_limits<double>::infinity();
    double required_deceleration = 0.0;
    bool following = false;
    bool emergency = false;
};

class FollowingPlanner
{
public:
    explicit FollowingPlanner(const FollowingConfig& config);

    void observe(const std::vector<ObjectObservation>& objects,
                 int active_path,
                 double stamp);

    FollowingResult plan(int active_path,
                         double ego_speed,
                         double free_flow_speed,
                         double now) const;

    void reset();

private:
    FollowingResult planWithLeader(double ego_speed,
                                   double free_flow_speed,
                                   const LeaderObservation& leader) const;
    LeaderObservation leaderObservation(int active_path, double now) const;

    FollowingConfig config_;
    bool leader_valid_ = false;
    int leader_path_ = 0;
    double leader_distance_ = 0.0;
    double leader_relative_speed_ = 0.0;
    double leader_stamp_ = 0.0;
};

}  // namespace highway_planner
