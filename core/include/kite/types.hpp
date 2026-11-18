#pragma once

namespace kite {

struct Vec2 {
  double x;
  double y;
};

struct GroundTruthState {
  double t;
  int ego_id;
  int actor_id;
  Vec2 position;
  Vec2 velocity;
};

struct Measurement {
  double t;
  int ego_id;
  int sensor_id;
  Vec2 position;
};

struct TrackState {
  double t;
  int ego_id;
  int track_id;
  Vec2 position;
  Vec2 velocity;
};

inline bool operator==(const Vec2& a, const Vec2& b) {
  return a.x == b.x && a.y == b.y;
}

inline bool operator==(const GroundTruthState& a, const GroundTruthState& b) {
  return a.t == b.t && a.ego_id == b.ego_id && a.actor_id == b.actor_id &&
         a.position == b.position && a.velocity == b.velocity;
}

inline bool operator==(const Measurement& a, const Measurement& b) {
  return a.t == b.t && a.ego_id == b.ego_id && a.sensor_id == b.sensor_id &&
         a.position == b.position;
}

inline bool operator==(const TrackState& a, const TrackState& b) {
  return a.t == b.t && a.ego_id == b.ego_id && a.track_id == b.track_id &&
         a.position == b.position && a.velocity == b.velocity;
}

}
