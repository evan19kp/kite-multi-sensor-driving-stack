#include "kite/kalman_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

#include "kite/assignment.hpp"

namespace kite {

KalmanTracker::KalmanTracker(double q, double measurement_sigma, double initial_velocity_sigma,
                             double gate_chi2, int confirm_hits, int confirm_window,
                             int max_misses)
    : new_filter_(q, measurement_sigma),
      initial_velocity_sigma_(initial_velocity_sigma),
      gate_chi2_(gate_chi2),
      confirm_hits_(confirm_hits),
      confirm_window_(confirm_window),
      max_misses_(max_misses) {
  if (!(initial_velocity_sigma >= 0.0) ||
      !std::isfinite(initial_velocity_sigma * initial_velocity_sigma)) {
    throw std::invalid_argument("initial_velocity_sigma must be >= 0 with a finite square");
  }
  if (!std::isfinite(gate_chi2) || !(gate_chi2 > 0.0)) {
    throw std::invalid_argument("gate_chi2 must be finite and > 0");
  }
  if (confirm_hits < 1 || confirm_window < confirm_hits) {
    throw std::invalid_argument("need 1 <= confirm_hits <= confirm_window");
  }
  if (max_misses < 1) {
    throw std::invalid_argument("max_misses must be >= 1");
  }
}

void KalmanTracker::record(Track& track, bool hit) const {
  track.recent_hits.push_back(hit);
  if (track.recent_hits.size() > static_cast<std::size_t>(confirm_window_)) {
    track.recent_hits.pop_front();
  }
  track.consecutive_misses = hit ? 0 : track.consecutive_misses + 1;
  const auto hits = std::count(track.recent_hits.begin(), track.recent_hits.end(), true);
  if (track.status == Status::tentative && hits >= confirm_hits_) {
    track.status = Status::confirmed;
  }
}

std::vector<TrackState> KalmanTracker::update(double t,
                                              const std::vector<Measurement>& measurements) {
  if (!std::isfinite(t)) {
    throw std::invalid_argument("update time must be finite");
  }
  double dt = 0.0;
  if (previous_t_) {
    if (!(t > *previous_t_)) {
      throw std::invalid_argument("update time must increase");
    }
    dt = t - *previous_t_;
  }
  for (const Measurement& z : measurements) {
    if (!std::isfinite(z.position.x) || !std::isfinite(z.position.y)) {
      throw std::invalid_argument("measurement position must be finite");
    }
  }

  std::vector<Track> tracks = tracks_;
  for (Track& track : tracks) {
    track.filter.predict(dt);
  }

  CostMatrix cost{tracks.size(), measurements.size(), {}};
  cost.values.reserve(tracks.size() * measurements.size());
  for (const Track& track : tracks) {
    for (const Measurement& z : measurements) {
      cost.values.push_back(track.filter.innovation_mahalanobis2(z.position));
    }
  }

  std::vector<bool> track_hit(tracks.size(), false);
  std::vector<bool> measurement_used(measurements.size(), false);
  for (const Assignment& pair : solve_assignment(cost, gate_chi2_)) {
    const Measurement& z = measurements[pair.column];
    tracks[pair.row].filter.update(z.position);
    tracks[pair.row].ego_id = z.ego_id;
    track_hit[pair.row] = true;
    measurement_used[pair.column] = true;
  }
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    record(tracks[i], track_hit[i]);
  }
  tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
                              [this](const Track& track) {
                                return track.consecutive_misses >= max_misses_;
                              }),
               tracks.end());

  int next_id = next_id_;
  for (std::size_t m = 0; m < measurements.size(); ++m) {
    if (measurement_used[m]) {
      continue;
    }
    Track track{next_id++, measurements[m].ego_id, new_filter_, Status::tentative, {}, 0};
    track.filter.initialize(measurements[m].position, initial_velocity_sigma_);
    record(track, true);
    tracks.push_back(track);
  }

  std::vector<TrackState> output;
  for (const Track& track : tracks) {
    if (track.status == Status::confirmed) {
      const Matrix<4, 1>& x = track.filter.state();
      output.push_back({t, track.ego_id, track.id, {x(0, 0), x(1, 0)}, {x(2, 0), x(3, 0)}});
    }
  }

  tracks_ = std::move(tracks);
  previous_t_ = t;
  next_id_ = next_id;
  return output;
}

}
