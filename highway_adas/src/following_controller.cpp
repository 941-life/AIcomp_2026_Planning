#include "highway_adas/following_controller.hpp"

#include "highway_adas/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace highway_adas {
namespace {

double longitudinalSpeed(const TrackedObject& object, double heading_rad) {
  return object.velocity_map_mps.x * std::cos(heading_rad) +
         object.velocity_map_mps.y * std::sin(heading_rad);
}

}  // namespace

LongitudinalResult FollowingController::plan(
    const EgoState& ego,
    const LanePath& lane,
    const std::vector<TrackedObject>& tracks,
    double cruise_speed_mps) const {
  LongitudinalResult result;
  result.speed_cap_mps = std::max(0.0, cruise_speed_mps);

  const Polyline path(lane.centerline_map);
  const PathProjection ego_projection = path.project(ego.position_map);
  if (!path.valid() || !ego_projection.valid) return result;

  const TrackedObject* lead = nullptr;
  PathProjection lead_projection;
  double lead_gap = std::numeric_limits<double>::infinity();
  for (const TrackedObject& track : tracks) {
    const PathProjection projection = path.project(track.position_map);
    if (!projection.valid) continue;
    const double lane_gate = 0.5 * lane.width_m + 0.5 * track.width_m +
                             config_.lane_boundary_margin_m;
    if (std::abs(projection.lateral_m) > lane_gate) continue;

    const double center_delta = projection.s_m - ego_projection.s_m;
    const double gap = center_delta - 0.5 * ego.length_m - 0.5 * track.length_m;
    const double no_longitudinal_overlap =
        -0.5 * (ego.length_m + track.length_m);
    if (center_delta <= no_longitudinal_overlap || gap >= lead_gap) continue;
    lead = &track;
    lead_projection = projection;
    lead_gap = std::max(0.0, gap);
  }

  if (lead == nullptr) return result;

  const double lead_speed = std::max(0.0, longitudinalSpeed(*lead, lead_projection.heading_rad));
  const double closing_speed = std::max(0.0, ego.speed_mps - lead_speed);
  const double ttc = closing_speed > 1e-3
                         ? lead_gap / closing_speed
                         : std::numeric_limits<double>::infinity();
  const double braking_room = std::max(0.1, lead_gap - config_.minimum_gap_m);
  const double required_decel = closing_speed > 0.0
                                    ? closing_speed * closing_speed / (2.0 * braking_room)
                                    : 0.0;

  result.lead.present = true;
  result.lead.track_id = lead->id;
  result.lead.gap_m = lead_gap;
  result.lead.speed_mps = lead_speed;
  result.lead.relative_speed_mps = lead_speed - ego.speed_mps;
  result.lead.ttc_sec = ttc;
  result.lead.required_decel_mps2 = required_decel;

  if (lead_gap <= config_.minimum_gap_m || ttc <= config_.emergency_ttc_sec ||
      required_decel >= config_.emergency_decel_mps2) {
    result.mode = LongitudinalMode::EMERGENCY_BRAKE;
    result.speed_cap_mps = 0.0;
    return result;
  }

  const double desired_gap = config_.minimum_gap_m + config_.time_headway_sec * ego.speed_mps;
  const double follow_speed = std::max(
      0.0, lead_speed + config_.gap_gain_per_sec * (lead_gap - desired_gap));

  if (ttc <= config_.hard_brake_ttc_sec ||
      required_decel >= config_.hard_brake_decel_mps2) {
    result.mode = LongitudinalMode::HARD_BRAKE;
    result.speed_cap_mps = std::min(result.speed_cap_mps, follow_speed);
  } else if (lead_gap <= desired_gap + config_.follow_entry_margin_m ||
             lead_speed < cruise_speed_mps) {
    result.mode = LongitudinalMode::FOLLOW;
    result.speed_cap_mps = std::min(result.speed_cap_mps, follow_speed);
  }

  return result;
}

}  // namespace highway_adas
