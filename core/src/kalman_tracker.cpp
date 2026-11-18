#include "kite/kalman_tracker.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <variant>

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

bool KalmanTracker::record(Track& track, bool hit) const {
  if (Tentative* tentative = std::get_if<Tentative>(&track.lifecycle)) {
    ++(hit ? tentative->hits : tentative->misses);
    if (tentative->hits >= confirm_hits_) {
      track.lifecycle = Confirmed{};
      return true;
    }
    return tentative->misses <= confirm_window_ - confirm_hits_;
  }
  Confirmed& confirmed = std::get<Confirmed>(track.lifecycle);
  confirmed.consecutive_misses = hit ? 0 : confirmed.consecutive_misses + 1;
  return confirmed.consecutive_misses < max_misses_;
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

  std::vector<bool> track_hit(tracks.size(), false);
  std::vector<bool> measurement_used(measurements.size(), false);
  for (const bool confirmed_stage : {true, false}) {
    std::vector<std::size_t> rows;
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      if (std::holds_alternative<Confirmed>(tracks[i].lifecycle) == confirmed_stage) {
        rows.push_back(i);
      }
    }
    std::vector<std::size_t> columns;
    for (std::size_t m = 0; m < measurements.size(); ++m) {
      if (!measurement_used[m]) {
        columns.push_back(m);
      }
    }
    CostMatrix cost{rows.size(), columns.size(), {}};
    cost.values.reserve(rows.size() * columns.size());
    for (const std::size_t i : rows) {
      for (const std::size_t m : columns) {
        cost.values.push_back(tracks[i].filter.innovation_mahalanobis2(measurements[m].position));
      }
    }
    for (const Assignment& pair : solve_assignment(cost, gate_chi2_)) {
      Track& track = tracks[rows[pair.row]];
      const Measurement& z = measurements[columns[pair.column]];
      track.filter.update(z.position);
      track.ego_id = z.ego_id;
      track_hit[rows[pair.row]] = true;
      measurement_used[columns[pair.column]] = true;
    }
  }
  std::vector<Track> kept;
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    if (record(tracks[i], track_hit[i])) {
      kept.push_back(std::move(tracks[i]));
    }
  }

  int next_id = next_id_;
  for (std::size_t m = 0; m < measurements.size(); ++m) {
    if (measurement_used[m]) {
      continue;
    }
    Track track{next_id++, measurements[m].ego_id, new_filter_, Tentative{}};
    track.filter.initialize(measurements[m].position, initial_velocity_sigma_);
    record(track, true);
    kept.push_back(std::move(track));
  }

  std::vector<TrackState> output;
  for (const Track& track : kept) {
    if (std::holds_alternative<Confirmed>(track.lifecycle)) {
      const Matrix<4, 1>& x = track.filter.state();
      output.push_back({t, track.ego_id, track.id, {x(0, 0), x(1, 0)}, {x(2, 0), x(3, 0)}});
    }
  }

  tracks_ = std::move(kept);
  previous_t_ = t;
  next_id_ = next_id;
  return output;
}

}
