#pragma once

#include <vector>

#include "kite/tracker.hpp"

namespace kite {

// No-filter baseline: every measurement becomes a track at the measured position, with no
// smoothing and no prediction. Velocity is the displacement from the matched previous track
// divided by dt (zero for a new track).
//
// Matching is greedy against the previous update's tracks only. Pairs within the gate
// (distance <= gate, metres) are taken in ascending distance, ties broken by track id and then
// measurement index. Unmatched measurements get a new id (counting up from 0, in measurement
// order, never reused); unmatched tracks are dropped. Output is sorted by track id.
//
// Throws std::invalid_argument for a negative or non-finite gate, and from update() when t is not
// greater than the previous update's t.
class NearestNeighbourBaseline : public Tracker {
 public:
  explicit NearestNeighbourBaseline(double gate);

  std::vector<TrackState> update(double t, const std::vector<Measurement>& measurements) override;

 private:
  double gate_;
  bool has_previous_ = false;
  double previous_t_ = 0.0;
  int next_id_ = 0;
  std::vector<TrackState> previous_;
};

}
