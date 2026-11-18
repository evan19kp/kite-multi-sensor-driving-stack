#pragma once

#include <cstddef>
#include <optional>
#include <variant>
#include <vector>

#include "kite/assignment.hpp"
#include "kite/kalman.hpp"
#include "kite/matrix.hpp"
#include "kite/tracker.hpp"
#include "kite/types.hpp"

namespace kite {

// two_stage: confirmed tracks are assigned first, then tentative tracks get the measurements left
// over, on the squared Mahalanobis distance d^2. joint: one assignment over every track on d^2.
// joint_likelihood: one assignment over every track on d^2 + ln(det S / det R). joint and
// joint_likelihood are for studies only.
//
// The joint_likelihood cost comes from the Gaussian likelihood. A track predicts its measurement
// z with mean H x and covariance S = H P H^T + R, so
// -2 ln N(z; H x, S) = d^2 + ln det S + 2 ln(2 pi).
// Subtracting ln det R + 2 ln(2 pi), the same for every pair, leaves d^2 + ln(det S / det R). That
// is twice the negative log-likelihood up to a constant, so among assignments with the most pairs
// the least total is the most likely one. H P H^T is positive semi-definite and R is positive
// definite, so det S >= det R and every cost is >= 0.
//
// Every mode gates on d^2 <= gate_chi2 alone. A pair with d^2 above the gate is never matched, and
// the ln term never keeps a pair inside the gate from matching.
enum class AssociationMode { two_stage, joint, joint_likelihood };

enum class TrackStage { tentative, confirmed };

// One track in one update, after predict and before the measurement update.
struct TrackRecord {
  int track_id;
  TrackStage stage;
  int hits;
  int misses;
  Vec2 predicted;
  Matrix<2, 2> innovation_covariance;
  std::optional<std::size_t> measurement;

  int age_steps() const { return hits + misses; }
};

// One update. tracks: every track that existed before it, in id order; hits, misses and stage are
// as of the end of the previous update (the step that started a track counts as a hit), and
// measurement is the index into this update's measurements that the track was matched to.
// started_track: one entry per measurement, the id of the track it started, or nullopt when it
// was matched to an existing track.
struct StepRecord {
  std::vector<TrackRecord> tracks;
  std::vector<std::optional<int>> started_track;
};

// Multi-target tracker: one ConstantVelocityKalman per track, global nearest-neighbour
// association, and a tentative/confirmed track lifecycle.
//
// Each update at time t:
//   1. Predicts every track to t (dt = t minus the previous update's t; 0 on the first update).
//   2. Gates: a track and a measurement may pair only when the squared Mahalanobis distance of the
//      innovation is <= gate_chi2 (9.21 is the 99 % point of chi-square with 2 degrees of freedom).
//   3. Associates with solve_assignment (most pairs within the gate, then the least total cost).
//      With AssociationMode::two_stage (the default), confirmed tracks first, then tentative tracks
//      with the measurements left over; with joint and joint_likelihood, all tracks in one
//      assignment. The cost is d^2, or d^2 + ln(det S / det R) with joint_likelihood.
//   4. Updates each matched track with its measurement. Every track records a hit or a miss for
//      this step.
//   5. A tentative track is confirmed at its confirm_hits-th hit and dropped at its
//      (confirm_window - confirm_hits + 1)-th miss, the first point at which it can no longer reach
//      confirm_hits hits within its first confirm_window steps. The step that created it counts as
//      hit 1, so every tentative track is confirmed or dropped within confirm_window steps.
//   6. A confirmed track stays confirmed and is deleted after max_misses consecutive misses.
//   7. Each unmatched measurement starts a tentative track with the next id. Ids count up from 0
//      and are never reused.
// Output: one TrackState per confirmed track, at t, with the filter's position and velocity and
// the ego_id of the track's latest measurement, sorted by track id. Coasting confirmed tracks are
// output at their predicted state.
//
// Throws std::invalid_argument from the constructor unless q is finite and >= 0,
// measurement_sigma > 0 with a finite non-zero square, initial_velocity_sigma >= 0 with a finite
// square, gate_chi2 is finite and > 0, 1 <= confirm_hits <= confirm_window and max_misses >= 1.
// Throws std::invalid_argument from update() when t is not finite, t is not greater than the
// previous update's t, or a measurement position is not finite. With joint_likelihood, update()
// throws std::domain_error when a cost inside the gate is not finite, as when det R underflows to
// 0. A throwing update() leaves the state as it was; an update with any t that does not throw
// starts the clock.
//
// Step record: off by default. While on, every update that does not throw sets step_record() to
// that update's StepRecord; turning it off clears it. It never changes any output.
class KalmanTracker : public Tracker {
 public:
  KalmanTracker(double q, double measurement_sigma, double initial_velocity_sigma,
                double gate_chi2 = 9.21, int confirm_hits = 3, int confirm_window = 5,
                int max_misses = 5, AssociationMode association = AssociationMode::two_stage);

  std::vector<TrackState> update(double t, const std::vector<Measurement>& measurements) override;

  void record_steps(bool on);
  const std::optional<StepRecord>& step_record() const;

 private:
  struct Tentative {};
  struct Confirmed {
    int consecutive_misses = 0;
  };

  struct Track {
    int id;
    int ego_id;
    ConstantVelocityKalman filter;
    std::variant<Tentative, Confirmed> lifecycle;
    int hits = 0;
    int misses = 0;
  };

  struct PassCosts {
    CostMatrix cost;
    double gate;
  };

  bool survives_step(Track& track, bool hit) const;
  std::vector<std::vector<std::size_t>> association_passes(const std::vector<Track>& tracks) const;
  PassCosts pass_costs(const std::vector<Track>& tracks, const std::vector<std::size_t>& rows,
                       const std::vector<Measurement>& measurements,
                       const std::vector<std::size_t>& columns) const;

  ConstantVelocityKalman new_filter_;
  double det_r_;
  double initial_velocity_sigma_;
  double gate_chi2_;
  int confirm_hits_;
  int confirm_window_;
  int max_misses_;
  AssociationMode association_;
  bool record_steps_ = false;
  std::optional<double> previous_t_;
  int next_id_ = 0;
  std::vector<Track> tracks_;
  std::optional<StepRecord> step_record_;
};

}
