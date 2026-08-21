#include "highway_adas/object_tracker.hpp"

#include "highway_adas/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace highway_adas {
namespace {

// Real rows and columns each receive dummy alternatives, so an invalid pair is
// never forced merely because the input matrix is square.
std::vector<int> hungarianAssignment(
    const std::vector<std::vector<double>>& input_cost,
    double unmatched_cost) {
  const int original_rows = static_cast<int>(input_cost.size());
  const int original_cols = original_rows == 0 ? 0 : static_cast<int>(input_cost.front().size());
  const int size = original_rows + original_cols;
  if (size == 0) return {};

  std::vector<std::vector<double>> cost(
      size + 1, std::vector<double>(size + 1, 0.0));
  for (int row = 0; row < original_rows; ++row) {
    for (int col = 0; col < original_cols; ++col) {
      cost[row + 1][col + 1] = input_cost[row][col];
    }
    for (int col = original_cols; col < size; ++col) {
      cost[row + 1][col + 1] = unmatched_cost;
    }
  }
  for (int row = original_rows; row < size; ++row) {
    for (int col = 0; col < original_cols; ++col) {
      cost[row + 1][col + 1] = unmatched_cost;
    }
  }

  std::vector<double> u(size + 1, 0.0), v(size + 1, 0.0);
  std::vector<int> column_to_row(size + 1, 0), previous_column(size + 1, 0);
  for (int row = 1; row <= size; ++row) {
    column_to_row[0] = row;
    int column0 = 0;
    std::vector<double> min_value(size + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(size + 1, false);
    do {
      used[column0] = true;
      const int row0 = column_to_row[column0];
      double delta = std::numeric_limits<double>::infinity();
      int column1 = 0;
      for (int column = 1; column <= size; ++column) {
        if (used[column]) continue;
        const double current = cost[row0][column] - u[row0] - v[column];
        if (current < min_value[column]) {
          min_value[column] = current;
          previous_column[column] = column0;
        }
        if (min_value[column] < delta) {
          delta = min_value[column];
          column1 = column;
        }
      }
      for (int column = 0; column <= size; ++column) {
        if (used[column]) {
          u[column_to_row[column]] += delta;
          v[column] -= delta;
        } else {
          min_value[column] -= delta;
        }
      }
      column0 = column1;
    } while (column_to_row[column0] != 0);

    do {
      const int column1 = previous_column[column0];
      column_to_row[column0] = column_to_row[column1];
      column0 = column1;
    } while (column0 != 0);
  }

  std::vector<int> assignment(original_rows, -1);
  for (int column = 1; column <= size; ++column) {
    const int row = column_to_row[column];
    if (row >= 1 && row <= original_rows && column <= original_cols) {
      assignment[row - 1] = column - 1;
    }
  }
  return assignment;
}

Point2 predictPosition(const TrackedObject& object, double dt) {
  return {object.position_map.x + object.velocity_map_mps.x * dt,
          object.position_map.y + object.velocity_map_mps.y * dt};
}

}  // namespace

void ObjectTracker::reset() {
  tracks_.clear();
  next_id_ = 1;
}

void ObjectTracker::update(const EgoState& ego,
                           const std::vector<ObjectDetection>& detections,
                           double detection_stamp_sec) {
  if (!std::isfinite(detection_stamp_sec) || detection_stamp_sec <= 0.0) return;

  std::vector<TrackedObject> measured;
  measured.reserve(detections.size());
  for (const ObjectDetection& detection : detections) {
    if (!std::isfinite(detection.x_m) || !std::isfinite(detection.y_m) ||
        detection.width_m <= 0.0 || detection.length_m <= 0.0) {
      continue;
    }
    TrackedObject object;
    object.position_map = baseLinkToMap(ego, {detection.x_m, detection.y_m});
    object.width_m = detection.width_m;
    object.length_m = detection.length_m;
    object.last_seen_sec = detection_stamp_sec;
    measured.push_back(object);
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const TrackState& track) {
                  return detection_stamp_sec - track.object.last_seen_sec >
                         config_.track_timeout_sec;
                }),
                tracks_.end());

  std::vector<std::vector<double>> cost(
      tracks_.size(), std::vector<double>(measured.size(), 0.0));
  for (std::size_t row = 0; row < tracks_.size(); ++row) {
    const double dt = std::max(0.0, detection_stamp_sec - tracks_[row].state_stamp_sec);
    const Point2 predicted = predictPosition(tracks_[row].object, dt);
    for (std::size_t column = 0; column < measured.size(); ++column) {
      const double position_cost =
          distance(predicted, measured[column].position_map);
      const double size_cost =
          config_.size_cost_weight *
          (std::abs(tracks_[row].object.width_m - measured[column].width_m) +
           std::abs(tracks_[row].object.length_m - measured[column].length_m));
      cost[row][column] =
          position_cost <= config_.association_gate_m
              ? position_cost + size_cost
              : 1e6;
    }
  }

  const std::vector<int> assignment =
      hungarianAssignment(cost, config_.association_gate_m);
  std::vector<bool> detection_used(measured.size(), false);
  for (std::size_t row = 0; row < tracks_.size(); ++row) {
    TrackState& track = tracks_[row];
    const double dt = std::max(1e-3, detection_stamp_sec - track.state_stamp_sec);
    const Point2 predicted = predictPosition(track.object, dt);
    const int column = row < assignment.size() ? assignment[row] : -1;
    if (column < 0 || static_cast<std::size_t>(column) >= measured.size() ||
        cost[row][column] > config_.association_gate_m) {
      track.object.position_map = predicted;
      track.state_stamp_sec = detection_stamp_sec;
      continue;
    }

    const TrackedObject& observation = measured[column];
    detection_used[column] = true;
    const Point2 residual{observation.position_map.x - predicted.x,
                          observation.position_map.y - predicted.y};
    track.object.position_map = {
        predicted.x + config_.position_gain * residual.x,
        predicted.y + config_.position_gain * residual.y};
    track.object.velocity_map_mps.x += config_.velocity_gain * residual.x / dt;
    track.object.velocity_map_mps.y += config_.velocity_gain * residual.y / dt;
    track.object.width_m = observation.width_m;
    track.object.length_m = observation.length_m;
    track.object.last_seen_sec = detection_stamp_sec;
    ++track.object.hit_count;
    track.object.confirmed = track.object.hit_count >= config_.confirmation_hits;
    track.state_stamp_sec = detection_stamp_sec;
  }

  for (std::size_t column = 0; column < measured.size(); ++column) {
    if (detection_used[column]) continue;
    TrackState track;
    track.object = measured[column];
    track.object.id = next_id_++;
    track.object.hit_count = 1;
    track.object.confirmed = config_.confirmation_hits <= 1;
    track.state_stamp_sec = detection_stamp_sec;
    tracks_.push_back(track);
  }
}

std::vector<TrackedObject> ObjectTracker::tracks(double now_sec, bool confirmed_only) const {
  std::vector<TrackedObject> result;
  for (const TrackState& track : tracks_) {
    if (now_sec - track.object.last_seen_sec > config_.track_timeout_sec) continue;
    if (confirmed_only && !track.object.confirmed) continue;
    TrackedObject predicted = track.object;
    predicted.position_map = predictPosition(track.object, std::max(0.0, now_sec - track.state_stamp_sec));
    result.push_back(predicted);
  }
  return result;
}

}  // namespace highway_adas
