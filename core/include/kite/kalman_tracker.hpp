#pragma once

#include <deque>
#include <optional>
#include <vector>

#include "kite/kalman.hpp"
#include "kite/tracker.hpp"

namespace kite {

// Multi-target tracker: one ConstantVelocityKalman per track, global nearest-neighbour
// association, and a tentative/confirmed track lifecycle.
//
// Each update at time t:
//   1. Predicts every track to t (dt = t minus the previous update's t; 0 on the first update).
//   2. Gates: a track and a measurement may pair only when the squared Mahalanobis distance of the
//      innovation is <= gate_chi2 (9.21 is the 99 % point of chi-square with 2 degrees of freedom).
//   3. Associates with solve_assignment on those squared distances: most pairs within the gate,
//      then the least total.
//   4. Updates each matched track with its measurement. Every track records a hit or a miss for
//      this step.
//   5. A tentative track becomes confirmed once it has confirm_hits hits among its last
//      confirm_window steps (the step that created it counts as a hit), and stays confirmed.
//   6. Any track with max_misses consecutive misses is deleted.
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
// previous update's t, or a measurement position is not finite. A throwing update() leaves the
// state as it was; an update with any t that does not throw starts the clock.
class KalmanTracker : public Tracker {
 public:
  KalmanTracker(double q, double measurement_sigma, double initial_velocity_sigma,
                double gate_chi2 = 9.21, int confirm_hits = 3, int confirm_window = 5,
                int max_misses = 5);

  std::vector<TrackState> update(double t, const std::vector<Measurement>& measurements) override;

 private:
  enum class Status { tentative, confirmed };

  struct Track {
    int id;
    int ego_id;
    ConstantVelocityKalman filter;
    Status status;
    std::deque<bool> recent_hits;
    int consecutive_misses;
  };

  void record(Track& track, bool hit) const;

  ConstantVelocityKalman new_filter_;
  double initial_velocity_sigma_;
  double gate_chi2_;
  int confirm_hits_;
  int confirm_window_;
  int max_misses_;
  std::optional<double> previous_t_;
  int next_id_ = 0;
  std::vector<Track> tracks_;
};

}
