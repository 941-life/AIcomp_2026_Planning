#include "global_path_planner/highway_planner_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace hp = highway_planner;

namespace
{

hp::RouteModel makeSquareRoute()
{
    hp::RouteModel route;
    const std::vector<hp::Point2D> points = {
        {0.0, 0.0},
        {10.0, 0.0},
        {10.0, 10.0},
        {0.0, 10.0}};
    EXPECT_TRUE(route.setPath(points, true, 1, 2.5, 25.0));
    return route;
}

}  // namespace

TEST(RouteModel, TracksProgressAndCircularDistance)
{
    hp::RouteModel route = makeSquareRoute();

    EXPECT_NEAR(route.length(), 40.0, 1e-9);
    EXPECT_NEAR(route.forwardDistance(35.0, 5.0), 10.0, 1e-9);

    const hp::Projection projection =
        route.project(9.5, 0.2, 0.0, -1, 10, 5.0, 5.0);
    ASSERT_TRUE(projection.valid);
    EXPECT_EQ(projection.index, 1);
    EXPECT_NEAR(projection.s, 10.0, 1e-9);
}

TEST(ZonePlanner, HandlesCircularZoneOrder)
{
    hp::RouteModel route = makeSquareRoute();
    hp::ZoneConfig config;
    config.enabled = true;
    config.entry_s = 35.0;
    config.high_speed_start_s = 2.0;
    config.brake_start_s = 10.0;
    config.limit_start_s = 20.0;
    config.approach_distance = 5.0;
    config.limited_speed = 16.0;
    config.highway_speed = 25.0;
    config.planned_deceleration = 2.5;
    config.limit_margin = 0.0;
    config.system_delay = 0.0;

    hp::ZonePlanner planner;
    std::string error;
    ASSERT_TRUE(planner.configure(config, route, error)) << error;

    EXPECT_EQ(planner.evaluate(34.0, 20.0, route).state, hp::ZoneState::APPROACH);
    EXPECT_EQ(planner.evaluate(0.0, 20.0, route).state, hp::ZoneState::ENTRY_CONFIRM);
    EXPECT_EQ(planner.evaluate(5.0, 20.0, route).state, hp::ZoneState::HIGH_SPEED);

    const hp::ZoneResult braking = planner.evaluate(15.0, 20.0, route);
    EXPECT_EQ(braking.state, hp::ZoneState::BRAKE_TO_LIMIT);
    EXPECT_NEAR(
        braking.tollgate_speed,
        std::sqrt(16.0 * 16.0 + 2.0 * 2.5 * 5.0),
        1e-9);

    EXPECT_EQ(planner.evaluate(25.0, 20.0, route).state, hp::ZoneState::LIMITED);
}

TEST(ZonePlanner, RejectsAmbiguousZoneOrder)
{
    hp::RouteModel route = makeSquareRoute();
    hp::ZoneConfig config;
    config.enabled = true;
    config.entry_s = 5.0;
    config.high_speed_start_s = 20.0;
    config.brake_start_s = 10.0;
    config.limit_start_s = 30.0;

    hp::ZonePlanner planner;
    std::string error;
    EXPECT_FALSE(planner.configure(config, route, error));
    EXPECT_FALSE(error.empty());
}

TEST(SpeedCommandFilter, LimitsAccelerationChange)
{
    hp::SpeedFilterConfig config;
    config.maximum_acceleration = 2.0;
    config.maximum_deceleration = 3.0;
    config.maximum_jerk = 1.5;
    config.response_time = 0.8;

    hp::SpeedCommandFilter filter(config);
    filter.reset(10.0);

    EXPECT_NEAR(filter.update(0.0, 100.0, 1.0), 8.5, 1e-9);
    EXPECT_NEAR(filter.update(0.0, 100.0, 1.0), 5.5, 1e-9);
}

TEST(SpeedCommandFilter, NeverExceedsHardSpeedLimit)
{
    hp::SpeedFilterConfig config;
    hp::SpeedCommandFilter filter(config);
    filter.reset(25.0);

    EXPECT_DOUBLE_EQ(filter.update(25.0, 16.0, 0.05), 16.0);
    filter.reset(25.0);
    EXPECT_DOUBLE_EQ(filter.update(25.0, 16.0, 0.0), 16.0);
    EXPECT_LT(filter.update(25.0, 25.0, 0.05), 16.01);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
