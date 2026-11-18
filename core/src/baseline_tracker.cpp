#include "kite/baseline_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <tuple>

namespace kite {

namespace {

struct Candidate {
  double distance;
  int track_id;
  std::size_t measurement;
  std::size_t track;
};

bool closer(const Candidate& a, const Candidate& b) {
  return std::tie(a.distance, a.track_id, a.measurement) <
         std::tie(b.distance, b.track_id, b.measurement);
}

}

NearestNeighbourBaseline::NearestNeighbourBaseline(double gate) : gate_(gate) {
  if (!std::isfinite(gate) || gate < 0.0) {
    throw std::invalid_argument("gate must be finite and non-negative");
  }
}

std::vector<TrackState> NearestNeighbourBaseline::update(
    double t, const std::vector<Measurement>& measurements) {
  double dt = 0.0;
  if (has_previous_) {
    if (!(t > previous_t_)) {
      throw std::invalid_argument("update time must increase");
    }
    dt = t - previous_t_;
  }

  std::vector<Candidate> candidates;
  for (std::size_t m = 0; m < measurements.size(); ++m) {
    for (std::size_t p = 0; p < previous_.size(); ++p) {
      const double distance =
          std::hypot(measurements[m].position.x - previous_[p].position.x,
                     measurements[m].position.y - previous_[p].position.y);
      if (distance <= gate_) {
        candidates.push_back({distance, previous_[p].track_id, m, p});
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(), closer);

  std::vector<const TrackState*> match(measurements.size(), nullptr);
  std::vector<bool> track_taken(previous_.size(), false);
  for (const Candidate& c : candidates) {
    if (match[c.measurement] == nullptr && !track_taken[c.track]) {
      match[c.measurement] = &previous_[c.track];
      track_taken[c.track] = true;
    }
  }

  std::vector<TrackState> tracks;
  tracks.reserve(measurements.size());
  for (std::size_t m = 0; m < measurements.size(); ++m) {
    const Measurement& z = measurements[m];
    TrackState track{t, z.ego_id, 0, z.position, {0.0, 0.0}};
    if (match[m] != nullptr) {
      track.track_id = match[m]->track_id;
      track.velocity = {(z.position.x - match[m]->position.x) / dt,
                        (z.position.y - match[m]->position.y) / dt};
    } else {
      track.track_id = next_id_++;
    }
    tracks.push_back(track);
  }
  std::sort(tracks.begin(), tracks.end(),
            [](const TrackState& a, const TrackState& b) { return a.track_id < b.track_id; });

  has_previous_ = true;
  previous_t_ = t;
  previous_ = tracks;
  return tracks;
}

}
