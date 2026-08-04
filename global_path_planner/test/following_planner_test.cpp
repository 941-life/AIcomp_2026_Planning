#include "global_path_planner/following_planner.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace hp = highway_planner;

TEST(FollowingPlanner, SelectsNearestLeaderOnActivePath)
{
    hp::FollowingConfig config;
    hp::FollowingPlanner planner(config);
    const std::vector<hp::ObjectObservation> objects = {
        {1, 40.0},
        {1, 20.0},
        {2, 5.0}};

    planner.observe(objects, 1, 1.0);
    const hp::FollowingResult result = planner.plan(1, 20.0, 25.0, 1.0);

    EXPECT_TRUE(result.following);
    EXPECT_DOUBLE_EQ(result.leader_distance, 20.0);
    EXPECT_DOUBLE_EQ(result.relative_speed, 0.0);
}

TEST(FollowingPlanner, EstimatesRelativeSpeedAndAppliesAcc)
{
    hp::FollowingConfig config;
    config.relative_speed_alpha = 1.0;
    hp::FollowingPlanner planner(config);

    planner.observe({{1, 20.0}}, 1, 1.0);
    planner.observe({{1, 19.8}}, 1, 1.1);
    const hp::FollowingResult result = planner.plan(1, 20.0, 25.0, 1.1);

    EXPECT_TRUE(result.following);
    EXPECT_FALSE(result.emergency);
    EXPECT_NEAR(result.relative_speed, -2.0, 1e-9);
    EXPECT_NEAR(result.ttc, 9.9, 1e-9);
    EXPECT_LT(result.target_speed, 20.0);
}

TEST(FollowingPlanner, HoldsBriefDropoutAndExpiresLeader)
{
    hp::FollowingConfig config;
    config.hold_time = 0.3;
    hp::FollowingPlanner planner(config);

    planner.observe({{1, 20.0}}, 1, 1.0);
    planner.observe({}, 1, 1.1);

    EXPECT_TRUE(planner.plan(1, 20.0, 25.0, 1.2).following);
    EXPECT_FALSE(planner.plan(1, 20.0, 25.0, 1.31).following);
}

TEST(FollowingPlanner, DoesNotReuseLeaderAfterPathChange)
{
    hp::FollowingPlanner planner{hp::FollowingConfig()};
    planner.observe({{1, 20.0}}, 1, 1.0);

    const hp::FollowingResult result = planner.plan(2, 20.0, 25.0, 1.1);

    EXPECT_FALSE(result.following);
    EXPECT_DOUBLE_EQ(result.target_speed, 25.0);
}

TEST(FollowingPlanner, DoesNotReuseExpiredRelativeSpeed)
{
    hp::FollowingConfig config;
    config.hold_time = 0.3;
    config.relative_speed_alpha = 1.0;
    hp::FollowingPlanner planner(config);

    planner.observe({{1, 20.0}}, 1, 1.0);
    planner.observe({{1, 10.0}}, 1, 2.0);
    const hp::FollowingResult result = planner.plan(1, 20.0, 25.0, 2.0);

    EXPECT_TRUE(result.following);
    EXPECT_DOUBLE_EQ(result.relative_speed, 0.0);
}

TEST(FollowingPlanner, StopsForEmergencyGap)
{
    hp::FollowingPlanner planner{hp::FollowingConfig()};
    planner.observe({{1, 2.5}}, 1, 1.0);

    const hp::FollowingResult result = planner.plan(1, 20.0, 25.0, 1.0);

    EXPECT_TRUE(result.emergency);
    EXPECT_DOUBLE_EQ(result.target_speed, 0.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
