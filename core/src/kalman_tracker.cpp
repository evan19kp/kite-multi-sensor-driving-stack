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
                             int max_misses, AssociationMode association)
    : new_filter_(q, measurement_sigma),
      initial_velocity_sigma_(initial_velocity_sigma),
      gate_chi2_(gate_chi2),
      confirm_hits_(confirm_hits),
      confirm_window_(confirm_window),
      max_misses_(max_misses),
      association_(association) {
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

void KalmanTracker::record_steps(bool on) {
  record_steps_ = on;
  if (!on) {
    step_record_.reset();
  }
}

const std::optional<StepRecord>& KalmanTracker::step_record() const { return step_record_; }

bool KalmanTracker::survives_step(Track& track, bool hit) const {
  ++(hit ? track.hits : track.misses);
  if (std::holds_alternative<Tentative>(track.lifecycle)) {
    if (track.hits >= confirm_hits_) {
      track.lifecycle = Confirmed{};
      return true;
    }
    return track.misses <= confirm_window_ - confirm_hits_;
  }
  Confirmed& confirmed = std::get<Confirmed>(track.lifecycle);
  confirmed.consecutive_misses = hit ? 0 : confirmed.consecutive_misses + 1;
  return confirmed.consecutive_misses < max_misses_;
}

std::vector<std::vector<std::size_t>> KalmanTracker::association_passes(
    const std::vector<Track>& tracks) const {
  std::vector<std::size_t> confirmed;
  std::vector<std::size_t> tentative;
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    (std::holds_alternative<Confirmed>(tracks[i].lifecycle) ? confirmed : tentative).push_back(i);
  }
  switch (association_) {
    case AssociationMode::two_stage:
      return {confirmed, tentative};
    case AssociationMode::joint: {
      std::vector<std::size_t> all;
      for (std::size_t i = 0; i < tracks.size(); ++i) {
        all.push_back(i);
      }
      return {all};
    }
  }
  throw std::logic_error("unknown association mode");
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

  std::optional<StepRecord> record;
  if (record_steps_) {
    record.emplace();
    record->started_track.resize(measurements.size());
    for (const Track& track : tracks) {
      const Matrix<4, 1>& x = track.filter.state();
      record->tracks.push_back(
          {track.id,
           std::holds_alternative<Confirmed>(track.lifecycle) ? TrackStage::confirmed
                                                               : TrackStage::tentative,
           track.hits, track.misses, {x(0, 0), x(1, 0)}, track.filter.innovation_covariance(),
           std::nullopt});
    }
  }

  std::vector<bool> track_hit(tracks.size(), false);
  std::vector<bool> measurement_used(measurements.size(), false);
  for (const std::vector<std::size_t>& rows : association_passes(tracks)) {
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
      if (record) {
        record->tracks[rows[pair.row]].measurement = columns[pair.column];
      }
    }
  }
  std::vector<Track> kept;
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    if (survives_step(tracks[i], track_hit[i])) {
      kept.push_back(std::move(tracks[i]));
    }
  }

  int next_id = next_id_;
  for (std::size_t m = 0; m < measurements.size(); ++m) {
    if (measurement_used[m]) {
      continue;
    }
    if (record) {
      record->started_track[m] = next_id;
    }
    Track track{next_id++, measurements[m].ego_id, new_filter_, Tentative{}};
    track.filter.initialize(measurements[m].position, initial_velocity_sigma_);
    if (survives_step(track, true)) {
      kept.push_back(std::move(track));
    }
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
  step_record_ = std::move(record);
  return output;
}

}
