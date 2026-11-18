#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "kite/kalman_tracker.hpp"
#include "kite/types.hpp"
#include "study_common.hpp"

namespace kite::drop_study {

inline constexpr double kZ95 = 1.959963984540054;

// C(n, k), exact for n up to 62. After step i the running value is C(n - k + i, i), so the
// division by i is always exact.
inline unsigned long long binomial_coefficient(int n, int k) {
  if (n < 0 || k < 0) {
    throw std::invalid_argument("binomial_coefficient needs n >= 0 and k >= 0");
  }
  if (k > n) {
    return 0;
  }
  unsigned long long result = 1;
  for (int i = 1; i <= k; ++i) {
    result =
        result * static_cast<unsigned long long>(n - k + i) / static_cast<unsigned long long>(i);
  }
  return result;
}

// The chance that a car's new track is dropped when every step detects the car independently with
// probability p and nothing else interferes: P(at most confirm_hits - 2 hits in the
// confirm_window - 1 steps after the birth), the birth being hit 1. Fewer than confirm_hits - 1
// further hits leaves the track unable to reach confirm_hits hits within confirm_window steps.
inline double drop_probability(double p, int confirm_hits, int confirm_window) {
  if (!(p >= 0.0 && p <= 1.0)) {
    throw std::invalid_argument("p must be in [0, 1]");
  }
  if (confirm_hits < 1 || confirm_window < confirm_hits) {
    throw std::invalid_argument("need 1 <= confirm_hits <= confirm_window");
  }
  const int steps = confirm_window - 1;
  double total = 0.0;
  for (int k = 0; k <= confirm_hits - 2; ++k) {
    double term = static_cast<double>(binomial_coefficient(steps, k));
    for (int i = 0; i < k; ++i) {
      term *= p;
    }
    for (int i = 0; i < steps - k; ++i) {
      term *= 1.0 - p;
    }
    total += term;
  }
  return total;
}

struct Interval {
  double low;
  double high;
};

inline Interval wilson_interval(long long successes, long long trials, double z = kZ95) {
  if (trials <= 0 || successes < 0 || successes > trials) {
    throw std::invalid_argument("wilson_interval needs 0 <= successes <= trials and trials > 0");
  }
  const double n = static_cast<double>(trials);
  const double phat = static_cast<double>(successes) / n;
  const double z2 = z * z;
  const double denominator = 1.0 + z2 / n;
  const double center = (phat + z2 / (2.0 * n)) / denominator;
  const double half = z * std::sqrt(phat * (1.0 - phat) / n + z2 / (4.0 * n * n)) / denominator;
  return {std::max(0.0, center - half), std::min(1.0, center + half)};
}

enum class Fate { pending, confirmed, dropped };

// How a track's misses broke down. A step where the track had no measurement counts in exactly
// one field.
struct MissCauses {
  int no_measurement = 0;
  int outside_gate = 0;
  int taken_by_other = 0;
};

// Follows one tentative track through the tracker's step records. Call observe() once per
// update, starting with the update after the one that started the track: that update's record
// does not list the track yet. The record lists a track only while it is alive, so a drop shows
// at the update after the track's last miss, and a track dropped by the final update stays
// pending. A confirmed track is in the update's output.
//
// A miss is "no measurement" when the step had none, "outside gate" when no measurement of the
// step was inside the track's gate, and "taken by another track" when one was inside the gate but
// the track did not get it. The gate test uses the record's predicted position and innovation
// covariance.
class TrackFollower {
 public:
  TrackFollower(int track_id, double gate_chi2) : track_id_(track_id), gate_chi2_(gate_chi2) {}

  void observe(const StepRecord& record, const std::vector<Measurement>& measurements,
               const std::vector<TrackState>& output) {
    if (fate_ != Fate::pending) {
      return;
    }
    const auto track = std::find_if(record.tracks.begin(), record.tracks.end(),
                                    [this](const TrackRecord& t) { return t.track_id == track_id_; });
    if (track == record.tracks.end()) {
      fate_ = Fate::dropped;
      return;
    }
    if (!track->measurement) {
      count_miss(*track, measurements);
    }
    if (std::any_of(output.begin(), output.end(),
                    [this](const TrackState& s) { return s.track_id == track_id_; })) {
      fate_ = Fate::confirmed;
    }
  }

  Fate fate() const { return fate_; }
  const MissCauses& misses() const { return misses_; }

 private:
  void count_miss(const TrackRecord& track, const std::vector<Measurement>& measurements) {
    if (measurements.empty()) {
      ++misses_.no_measurement;
      return;
    }
    const bool gated = std::any_of(
        measurements.begin(), measurements.end(), [&](const Measurement& z) {
          return study::mahalanobis2(track, z.position) <= gate_chi2_;
        });
    ++(gated ? misses_.taken_by_other : misses_.outside_gate);
  }

  int track_id_;
  double gate_chi2_;
  Fate fate_ = Fate::pending;
  MissCauses misses_;
};

}
