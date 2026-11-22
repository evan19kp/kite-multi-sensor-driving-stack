#pragma once

#include <algorithm>
#include <vector>

#include "kite/tracker.hpp"
#include "kite/types.hpp"

namespace kite::tool_track {

// Feeds the measurements to the tracker one time step at a time, in time order, and returns every
// track it outputs. A step is the measurements that share one t, in the order they came in. It
// takes measurements only, so ground truth cannot reach the tracker. A step with no measurement
// has no record, so the tracker never sees it.
inline std::vector<TrackState> run_tracker(Tracker& tracker, std::vector<Measurement> measurements) {
  std::stable_sort(measurements.begin(), measurements.end(),
                   [](const Measurement& a, const Measurement& b) { return a.t < b.t; });
  std::vector<TrackState> tracks;
  auto first = measurements.begin();
  while (first != measurements.end()) {
    const double t = first->t;
    const auto last = std::find_if(first, measurements.end(),
                                   [t](const Measurement& m) { return m.t != t; });
    const std::vector<TrackState> step_tracks = tracker.update(t, std::vector<Measurement>(first, last));
    tracks.insert(tracks.end(), step_tracks.begin(), step_tracks.end());
    first = last;
  }
  return tracks;
}

}
