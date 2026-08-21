#ifndef HIGHWAY_ADAS_OBJECT_TRACKER_HPP
#define HIGHWAY_ADAS_OBJECT_TRACKER_HPP

#include "highway_adas/types.hpp"

#include <vector>

namespace highway_adas {

struct TrackerConfig {
  double association_gate_m = 4.0;
  double size_cost_weight = 0.5;
  double position_gain = 0.70;
  double velocity_gain = 0.20;
  double track_timeout_sec = 0.6;
  int confirmation_hits = 3;
};

struct TrackedObject {
  int id = -1;
  Point2 position_map;
  Point2 velocity_map_mps;
  double width_m = 0.0;
  double length_m = 0.0;
  double last_seen_sec = 0.0;
  int hit_count = 0;
  bool confirmed = false;
};

class ObjectTracker {
 public:
  explicit ObjectTracker(const TrackerConfig& config = TrackerConfig()) : config_(config) {}

  void reset();
  void update(const EgoState& ego,
              const std::vector<ObjectDetection>& detections,
              double detection_stamp_sec);
  std::vector<TrackedObject> tracks(double now_sec, bool confirmed_only = true) const;

 private:
  struct TrackState {
    TrackedObject object;
    double state_stamp_sec = 0.0;
  };

  TrackerConfig config_;
  int next_id_ = 1;
  std::vector<TrackState> tracks_;
};

}  // namespace highway_adas

#endif  // HIGHWAY_ADAS_OBJECT_TRACKER_HPP
