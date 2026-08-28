#include "highway_adas/adas_planner.hpp"
#include "highway_adas/following_controller.hpp"
#include "highway_adas/highway_decision_module.hpp"
#include "highway_adas/highway_region_policy.hpp"
#include "highway_adas/lane_change_planner.hpp"
#include "highway_adas/object_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace adas = highway_adas;

namespace {

int failures = 0;

void expectTrue(bool condition, const std::string& message) {
  if (condition) return;
  ++failures;
  std::cerr << "FAIL: " << message << '\n';
}

void expectNear(double actual, double expected, double tolerance,
                const std::string& message) {
  if (std::abs(actual - expected) <= tolerance) return;
  ++failures;
  std::cerr << "FAIL: " << message << " actual=" << actual
            << " expected=" << expected << '\n';
}

adas::LanePath straightLane(int id, double y, int left, int right) {
  adas::LanePath lane;
  lane.id = id;
  lane.left_neighbor_id = left;
  lane.right_neighbor_id = right;
  lane.width_m = 3.5;
  for (int x = 0; x <= 300; x += 2) {
    lane.centerline_map.push_back({double(x), y});
  }
  return lane;
}

adas::TrackedObject track(int id, double x, double y, double vx) {
  adas::TrackedObject object;
  object.id = id;
  object.position_map = {x, y};
  object.velocity_map_mps = {vx, 0.0};
  object.width_m = 1.9;
  object.length_m = 4.6;
  object.confirmed = true;
  object.hit_count = 3;
  object.last_seen_sec = 1.0;
  return object;
}

adas::EgoState ego(double x, double y, double speed, double stamp) {
  adas::EgoState state;
  state.position_map = {x, y};
  state.speed_mps = speed;
  state.stamp_sec = stamp;
  return state;
}

std::vector<adas::LanePath> twoLanes() {
  return {straightLane(1, 0.0, 2, -1),
          straightLane(2, 3.5, -1, 1)};
}

adas::LaneChangeStep startExecution(adas::LaneChangePlanner* planner,
                                    adas::EgoState* state) {
  const auto lanes = twoLanes();
  const adas::LaneRequest request{adas::LaneRequestType::MANDATORY, 2};
  planner->update(*state, 1, lanes, {}, state->stamp_sec, request,
                  100.0 / 3.6);
  state->stamp_sec += 0.6;
  return planner->update(*state, 1, lanes, {}, state->stamp_sec, request,
                         100.0 / 3.6);
}

void testTrackerWithoutPerceptionIds() {
  adas::TrackerConfig config;
  config.confirmation_hits = 1;
  config.position_gain = 1.0;
  config.velocity_gain = 1.0;
  adas::ObjectTracker tracker(config);

  const adas::EgoState state = ego(0.0, 0.0, 0.0, 1.0);
  tracker.update(state,
                 {{20.0, 0.0, 1.9, 4.6}, {40.0, 0.0, 1.9, 4.6}},
                 1.0);
  tracker.update(state,
                 {{41.0, 0.0, 1.9, 4.6}, {21.0, 0.0, 1.9, 4.6}},
                 1.1);

  auto tracks = tracker.tracks(1.1, true);
  std::sort(tracks.begin(), tracks.end(),
            [](const adas::TrackedObject& first,
               const adas::TrackedObject& second) {
              return first.id < second.id;
            });
  expectTrue(tracks.size() == 2, "tracker keeps both objects after order swap");
  if (tracks.size() == 2) {
    expectNear(tracks[0].position_map.x, 21.0, 1e-6,
               "first track follows the nearby measurement");
    expectNear(tracks[1].position_map.x, 41.0, 1e-6,
               "second track follows the nearby measurement");
  }
}

void testCurvatureCapIsCombinedWithAdasSpeed() {
  adas::AdasConfig config;
  config.curvature_sample_distance_m = 5.0;
  config.max_lateral_accel_mps2 = 2.0;
  config.curvature_lookahead_distance_m = 30.0;
  config.curvature_planned_deceleration_mps2 = 1.0;
  adas::AdasPlanner planner(config);

  adas::LanePath lane;
  lane.id = 1;
  lane.centerline_map = {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0},
                         {20.0, 10.0}, {20.0, 20.0}};
  adas::AdasInput input;
  input.ego = ego(0.0, 0.0, 10.0, 1.0);
  input.perception_healthy = true;
  input.detection_stamp_sec = 1.0;
  input.detection_ego = input.ego;
  input.lanes = {lane};
  input.cruise_speed_mps = 30.0;

  const adas::AdasOutput output = planner.update(input);
  expectTrue(output.valid, "ADAS returns the curved keep-lane path");
  expectTrue(output.speed_cap_mps < input.cruise_speed_mps,
             "curvature cap restricts the ADAS target speed");
}

void testFollowingRules() {
  const adas::LanePath lane = straightLane(1, 0.0, 2, -1);
  adas::FollowingController controller;
  const adas::EgoState state = ego(0.0, 0.0, 100.0 / 3.6, 1.0);

  auto result = controller.plan(
      state, lane, {track(1, 60.0, 0.0, 20.0)}, 100.0 / 3.6);
  expectTrue(result.mode == adas::LongitudinalMode::FOLLOW,
             "leader inside desired headway activates FOLLOW");
  expectTrue(result.speed_cap_mps < state.speed_mps,
             "FOLLOW lowers target speed for a slower leader");

  result = controller.plan(
      state, lane, {track(2, 25.0, 0.0, 0.0)}, 100.0 / 3.6);
  expectTrue(result.mode == adas::LongitudinalMode::EMERGENCY_BRAKE,
             "short TTC activates emergency braking");
}

void testSameSpeedTrafficUsesRelativeGap() {
  adas::LaneChangePlanner planner;
  const auto lanes = twoLanes();
  const double speed = 100.0 / 3.6;
  adas::EgoState state = ego(50.0, 0.0, speed, 1.0);
  const double center_offset = 0.5 * (state.length_m + 4.6) + 17.0;
  const std::vector<adas::TrackedObject> tracks = {
      track(1, state.position_map.x + center_offset, 3.5, speed),
      track(2, state.position_map.x - center_offset, 3.5, speed)};
  const adas::LaneRequest request{adas::LaneRequestType::MANDATORY, 2};

  auto result = planner.update(state, 1, lanes, tracks, 1.0, request, speed);
  expectTrue(result.gap.safe,
             "17 m front and rear gaps are valid in equal-speed 100 kph flow");
  expectNear(result.gap.required_front_gap_m, 0.6 * speed, 1e-6,
             "equal-speed front gap uses the 0.6 second floor");

  state.stamp_sec = 1.6;
  result = planner.update(state, 1, lanes, tracks, 1.6, request, speed);
  expectTrue(result.state == adas::LaneChangeState::EXECUTE,
             "safe equal-speed slot starts after the 0.5 second hold");
}

void testGapShapingCanAccelerateForRearVehicle() {
  adas::LaneChangePlanner planner;
  const auto lanes = twoLanes();
  adas::EgoState state = ego(50.0, 0.0, 25.0, 1.0);
  const double center_offset = 0.5 * (state.length_m + 4.6) + 18.0;
  const adas::LaneRequest request{adas::LaneRequestType::MANDATORY, 2};

  const auto result = planner.update(
      state, 1, lanes,
      {track(7, state.position_map.x - center_offset, 3.5, 28.0)},
      1.0, request, 100.0 / 3.6);
  expectTrue(result.selected_gap.valid && result.selected_gap.speed_feasible,
             "a reachable rear-vehicle slot is selected");
  expectTrue(result.has_behavior_target_speed &&
                 result.behavior_target_speed_mps > state.speed_mps,
             "rear approach may create a limited acceleration target");
}

void testPendingRequestCanBeWithdrawn() {
  adas::LaneChangePlanner planner;
  const auto lanes = twoLanes();
  adas::EgoState state = ego(10.0, 0.0, 25.0, 1.0);

  auto result = planner.update(
      state, 1, lanes, {track(1, 5.0, 3.5, 35.0)}, 1.0,
      {adas::LaneRequestType::MANDATORY, 2}, 100.0 / 3.6);
  expectTrue(result.state == adas::LaneChangeState::CHECK_GAP,
             "unsafe request waits in CHECK_GAP");

  state.stamp_sec = 1.1;
  result = planner.update(state, 1, lanes, {}, 1.1,
                          {adas::LaneRequestType::KEEP_LANE, -1},
                          100.0 / 3.6);
  expectTrue(result.state == adas::LaneChangeState::KEEP_LANE,
             "withdrawn request cancels before lateral motion");
}

void testEarlyCancelRequiresSafeSourceLane() {
  const auto lanes = twoLanes();
  const adas::LaneRequest request{adas::LaneRequestType::MANDATORY, 2};

  adas::LaneChangePlanner safe_return_planner;
  adas::EgoState safe_state = ego(10.0, 0.0, 25.0, 1.0);
  auto result = startExecution(&safe_return_planner, &safe_state);
  expectTrue(result.state == adas::LaneChangeState::EXECUTE,
             "setup enters EXECUTE");
  safe_state.position_map = {12.0, 0.05};
  safe_state.stamp_sec = 1.7;
  result = safe_return_planner.update(
      safe_state, 1, lanes, {track(10, 18.0, 3.5, 0.0)}, 1.7,
      request, 100.0 / 3.6);
  expectTrue(result.safety_action == adas::SafetyAction::EARLY_CANCEL,
             "early target hazard returns through a smooth source path");
  expectTrue(result.state == adas::LaneChangeState::EXECUTE,
             "early cancel remains in EXECUTE until the return path is traversed");

  adas::LaneChangePlanner blocked_return_planner;
  adas::EgoState blocked_state = ego(10.0, 0.0, 25.0, 2.0);
  result = startExecution(&blocked_return_planner, &blocked_state);
  blocked_state.position_map = {12.0, 0.05};
  blocked_state.stamp_sec = 2.7;
  result = blocked_return_planner.update(
      blocked_state, 1, lanes,
      {track(20, 18.0, 3.5, 0.0), track(21, 5.0, 0.0, 35.0)},
      2.7, request, 100.0 / 3.6);
  expectTrue(result.safety_action != adas::SafetyAction::EARLY_CANCEL,
             "unsafe source rear gap blocks early return");
}

void testCommittedLaneChangeDoesNotSnapBack() {
  adas::LaneChangePlanner planner;
  const auto lanes = twoLanes();
  const adas::LaneRequest request{adas::LaneRequestType::MANDATORY, 2};
  adas::EgoState state = ego(10.0, 0.0, 25.0, 1.0);
  startExecution(&planner, &state);

  state.position_map = {40.0, 1.11};
  state.stamp_sec = 1.7;
  const auto result = planner.update(
      state, 1, lanes, {track(30, 45.0, 3.5, 0.0)}, 1.7,
      request, 100.0 / 3.6);
  expectTrue(result.progress >= 0.35,
             "test vehicle is past the early-cancel progress boundary");
  expectTrue(result.safety_action != adas::SafetyAction::EARLY_CANCEL,
             "committed lane change does not snap back to source");
  expectTrue(result.gap.prediction_horizon_sec < 3.0,
             "EXECUTE prediction uses the remaining lane-change duration");
}

void testCommittedRearThreatDoesNotRequestDeceleration() {
  adas::LaneChangePlanner planner;
  const auto lanes = twoLanes();
  const adas::LaneRequest request{adas::LaneRequestType::MANDATORY, 2};
  adas::EgoState state = ego(10.0, 0.0, 25.0, 1.0);
  startExecution(&planner, &state);

  state.position_map = {40.0, 1.11};
  state.stamp_sec = 1.7;
  const double rear_center =
      state.position_map.x - 0.5 * (state.length_m + 4.6) - 18.0;
  const auto result = planner.update(
      state, 1, lanes, {track(40, rear_center, 3.5, 35.0)}, 1.7,
      request, 100.0 / 3.6);
  expectTrue(result.safety_action == adas::SafetyAction::CONTINUE_HOLD_SPEED,
             "committed fast-rear threat keeps the lane-change path; action=" +
                 std::to_string(static_cast<int>(result.safety_action)));
  expectTrue(result.behavior_target_speed_mps >= state.speed_mps,
             "rear threat alone does not create a deceleration target");
}

void testHighwayRegionPolicy() {
  adas::HighwayRegionPolicy policy;
  adas::HighwayRegionInput input;
  input.region = adas::HighwayRegion::HW_ENTRY;
  input.current_lane_id = 4;
  input.ego_speed_mps = 70.0 / 3.6;
  auto output = policy.evaluate(input);
  expectTrue(output.valid &&
                 output.lane_request.type == adas::LaneRequestType::MANDATORY &&
                 output.lane_request.target_lane_id == 3,
             "entry acquire requests lane 3 from lane 4");

  input.current_lane_id = 3;
  output = policy.evaluate(input);
  expectTrue(output.lane_request.type == adas::LaneRequestType::STRATEGIC &&
                 output.lane_request.target_lane_id == 2,
             "entry uses an available gap to move from lane 3 toward lane 1");

  input.current_lane_id = 2;
  output = policy.evaluate(input);
  expectTrue(output.lane_request.type == adas::LaneRequestType::STRATEGIC &&
                 output.lane_request.target_lane_id == 1,
             "entry continues strategic lane-1 acquisition one lane at a time");

  input.region = adas::HighwayRegion::HW_ENTRY_GUARD;
  input.current_lane_id = 4;
  input.distance_to_guard_stop_m = 50.0;
  output = policy.evaluate(input);
  expectTrue(output.guard_active && output.speed_cap_mps < 70.0 / 3.6,
             "entry guard applies a stop-line speed profile after failure");

  input.region = adas::HighwayRegion::HW_MAIN;
  input.current_lane_id = 3;
  input.ego_speed_mps = 100.0 / 3.6;
  output = policy.evaluate(input);
  expectTrue(output.lane_request.type == adas::LaneRequestType::STRATEGIC &&
                 output.lane_request.target_lane_id == 2,
             "main moves strategically from lane 3 toward lane 1");

  input.current_lane_id = 2;
  output = policy.evaluate(input);
  expectTrue(output.lane_request.type == adas::LaneRequestType::STRATEGIC &&
                 output.lane_request.target_lane_id == 1,
             "main acquire requests lane 1 after settling in lane 2");

  input.region = adas::HighwayRegion::HW_MAIN_GUARD_1;
  input.current_lane_id = 3;
  input.distance_to_guard_stop_m = 100.0;
  output = policy.evaluate(input);
  expectTrue(output.guard_active &&
                 output.lane_request.type == adas::LaneRequestType::MANDATORY &&
                 output.lane_request.target_lane_id == 2 &&
                 output.speed_cap_mps < 100.0 / 3.6,
             "main guard 1 makes the 3 to 2 merge mandatory");

  input.current_lane_id = 2;
  output = policy.evaluate(input);
  expectTrue(!output.guard_active &&
                 output.lane_request.type == adas::LaneRequestType::STRATEGIC &&
                 output.lane_request.target_lane_id == 1,
             "main guard 1 keeps lane 1 acquisition strategic after lane 3 exit");

  input.region = adas::HighwayRegion::HW_MAIN_GUARD_2;
  output = policy.evaluate(input);
  expectTrue(output.guard_active &&
                 output.lane_request.type == adas::LaneRequestType::MANDATORY &&
                 output.lane_request.target_lane_id == 1,
             "main guard 2 makes the 2 to 1 merge mandatory");

  input.current_lane_id = 3;
  output = policy.evaluate(input);
  expectTrue(output.guard_active &&
                 output.lane_request.type == adas::LaneRequestType::MANDATORY &&
                 output.lane_request.target_lane_id == 2,
             "main guard 2 recovers sequentially without a two-lane jump");

  input.region = adas::HighwayRegion::HW_TOLL;
  input.current_lane_id = 1;
  input.distance_to_next_limit_m = 0.0;
  output = policy.evaluate(input);
  expectNear(output.speed_cap_mps, 60.0 / 3.6, 1e-9,
             "toll region reaches the next 60 kph limit");
  expectTrue(output.lane_request.type == adas::LaneRequestType::KEEP_LANE,
             "toll region locks the single lane corridor");

  input.current_lane_id = 2;
  output = policy.evaluate(input);
  expectTrue(!output.valid,
             "toll region rejects an uncompleted lane-1 acquisition");
}

void testLaneClassificationHysteresis() {
  adas::AdasConfig config;
  config.lane_switch_advantage_m = 0.4;
  config.lane_switch_hold_sec = 0.3;
  adas::AdasPlanner planner(config);
  const auto lanes = twoLanes();

  adas::EgoState state = ego(10.0, 0.0, 20.0, 1.0);
  expectTrue(planner.currentLaneId(state, lanes) == 1,
             "lane classifier initializes on the nearest lane");

  state.position_map.y = 2.2;
  state.stamp_sec = 1.1;
  expectTrue(planner.currentLaneId(state, lanes) == 1,
             "lane classifier does not switch on one sample");

  state.stamp_sec = 1.45;
  expectTrue(planner.currentLaneId(state, lanes) == 2,
             "lane classifier switches after a sustained distance advantage");

  state.position_map.y = 20.0;
  state.stamp_sec = 1.6;
  expectTrue(planner.currentLaneId(state, lanes) == -1,
             "lane classifier rejects a pose outside every lane corridor");
}

void testInvalidRequestAndStalePerception() {
  adas::LaneChangePlanner lane_change;
  const std::vector<adas::LanePath> lanes = {
      straightLane(1, 0.0, 2, -1), straightLane(2, 3.5, 3, 1),
      straightLane(3, 7.0, -1, 2)};
  const adas::EgoState state = ego(10.0, 0.0, 20.0, 1.0);
  const auto rejected = lane_change.update(
      state, 1, lanes, {}, 1.0,
      {adas::LaneRequestType::MANDATORY, 3}, 100.0 / 3.6);
  expectTrue(rejected.request_rejected,
             "planner rejects a two-lane jump");

  adas::AdasPlanner planner;
  adas::AdasInput input;
  input.ego = ego(10.0, 0.0, 100.0 / 3.6, 2.0);
  input.perception_healthy = true;
  input.detection_stamp_sec = 1.0;
  input.detection_ego = ego(0.0, 0.0, 100.0 / 3.6, 1.0);
  input.lanes = lanes;
  input.cruise_speed_mps = 100.0 / 3.6;
  const adas::AdasOutput output = planner.update(input);
  expectTrue(output.valid,
             "stale perception still returns the known keep-lane path");
  expectNear(output.speed_cap_mps, 60.0 / 3.6, 1e-9,
             "stale perception blocks new high-speed acceleration");
  expectTrue(output.safety_action ==
                 adas::SafetyAction::STALE_PERCEPTION_HOLD,
             "stale perception action is explicit");
}

void testBaselineFacingDecisionModule() {
  adas::HighwayDecisionModule module;
  adas::HighwayDecisionInput input;
  input.region = adas::HighwayRegion::HW_MAIN;
  input.adas.ego = ego(10.0, 3.5, 80.0 / 3.6, 1.0);
  input.adas.perception_healthy = true;
  input.adas.detection_stamp_sec = 1.0;
  input.adas.detection_ego = input.adas.ego;
  input.adas.lanes = twoLanes();

  const auto output = module.update(input);
  expectTrue(output.valid,
             "baseline-facing wrapper returns a complete highway decision");
  expectTrue(output.region.lane_request.type ==
                 adas::LaneRequestType::STRATEGIC &&
                 output.region.lane_request.target_lane_id == 1,
             "wrapper converts HW_MAIN into a strategic lane-1 request");
  expectTrue(output.adas.lane_change_state ==
                 adas::LaneChangeState::CHECK_GAP,
             "wrapper passes the region request into the lane-change FSM");
}

void testUnhealthyPerceptionCannotStartLaneChange() {
  adas::HighwayDecisionModule module;
  adas::HighwayDecisionInput input;
  input.region = adas::HighwayRegion::HW_MAIN;
  input.adas.ego = ego(10.0, 3.5, 80.0 / 3.6, 1.0);
  input.adas.perception_healthy = false;
  input.adas.detection_stamp_sec = 1.0;
  input.adas.detection_ego = input.adas.ego;
  input.adas.lanes = twoLanes();

  module.update(input);
  input.adas.ego.stamp_sec = 1.6;
  input.adas.detection_stamp_sec = 1.6;
  input.adas.detection_ego = input.adas.ego;
  const auto output = module.update(input);
  expectTrue(output.valid,
             "unhealthy perception keeps a known-lane path available");
  expectTrue(output.adas.lane_change_state ==
                 adas::LaneChangeState::CHECK_GAP,
             "unhealthy perception cannot approve lane-change execution");
  expectTrue(output.adas.safety_action ==
                 adas::SafetyAction::STALE_PERCEPTION_HOLD,
             "unhealthy perception is reported explicitly");
}

}  // namespace

int main() {
  testTrackerWithoutPerceptionIds();
  testCurvatureCapIsCombinedWithAdasSpeed();
  testFollowingRules();
  testSameSpeedTrafficUsesRelativeGap();
  testGapShapingCanAccelerateForRearVehicle();
  testPendingRequestCanBeWithdrawn();
  testEarlyCancelRequiresSafeSourceLane();
  testCommittedLaneChangeDoesNotSnapBack();
  testCommittedRearThreatDoesNotRequestDeceleration();
  testHighwayRegionPolicy();
  testLaneClassificationHysteresis();
  testInvalidRequestAndStalePerception();
  testBaselineFacingDecisionModule();
  testUnhealthyPerceptionCannotStartLaneChange();

  if (failures != 0) {
    std::cerr << failures << " test assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All highway ADAS tests passed\n";
  return EXIT_SUCCESS;
}
