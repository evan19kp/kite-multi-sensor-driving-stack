#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
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

// A track started at most this many steps after an earlier track of the same car is a duplicate
// tentative track. Two births in the same step are 0 steps apart.
inline constexpr std::size_t kDuplicateBirthWindowSteps = 5;

// What became of every tentative track in one run, and how fast each car got a confirmed track.
// A dropped track is "clutter" when its birth measurement had no truth vehicle within the gate,
// "ambiguous" when it had one but no car label, "car already confirmed" when it was labelled and
// some confirmed track already belonged to that car after the birth step, and "real birth lost"
// otherwise. A duplicate tentative track, dropped or not, is one whose birth measurement is
// labelled to a car that an earlier track was also born from within kDuplicateBirthWindowSteps.
struct TentativeOutcome {
  int tentative = 0;
  int confirmed = 0;
  int dropped = 0;
  int unresolved = 0;
  int real_births_lost = 0;
  int car_already_confirmed = 0;
  int clutter_births = 0;
  int ambiguous_births = 0;
  int duplicate_tentative_tracks = 0;
  int cars = 0;
  int cars_never_confirmed = 0;
  long long steps_to_confirmation = 0;

  int unlabelled_births() const { return clutter_births + ambiguous_births; }

  TentativeOutcome& operator+=(const TentativeOutcome& other) {
    tentative += other.tentative;
    confirmed += other.confirmed;
    dropped += other.dropped;
    unresolved += other.unresolved;
    real_births_lost += other.real_births_lost;
    car_already_confirmed += other.car_already_confirmed;
    clutter_births += other.clutter_births;
    ambiguous_births += other.ambiguous_births;
    duplicate_tentative_tracks += other.duplicate_tentative_tracks;
    cars += other.cars;
    cars_never_confirmed += other.cars_never_confirmed;
    steps_to_confirmation += other.steps_to_confirmation;
    return *this;
  }
};

// Classifies the tentative tracks of one run. Call observe() once per update, in order, with that
// update's step record, measurements, output and one label per measurement.
//
// A track belongs to the car of its latest matched measurement that has a label. A match to an
// ambiguous measurement never changes it, so a confirmed track that takes an ambiguous
// measurement still belongs to its car. A car "has a confirmed track" at a step when some track in
// the step's output belongs to it; a coasting confirmed track is in the output. A track that
// starts on an ambiguous measurement belongs to no car until it is matched to a labelled one.
//
// Steps to confirmation, per car: from the step of the car's first labelled measurement to the
// first step whose output has a track belonging to the car. Cars are those with at least one
// labelled measurement.
class TentativeClassifier {
 public:
  explicit TentativeClassifier(double gate_chi2) : gate_chi2_(gate_chi2) {}

  // A measurement without a car label counts as ambiguous, never as clutter.
  void observe(const StepRecord& record, const std::vector<Measurement>& measurements,
               const std::vector<TrackState>& output, const std::vector<study::CarLabel>& labels) {
    std::vector<study::Label> kinds;
    for (const study::CarLabel& car : labels) {
      kinds.push_back(car ? study::Label{study::LabelKind::car, *car}
                          : study::Label{study::LabelKind::ambiguous});
    }
    observe_labelled(record, measurements, output, kinds);
  }

  void observe_labelled(const StepRecord& record, const std::vector<Measurement>& measurements,
                        const std::vector<TrackState>& output,
                        const std::vector<study::Label>& kinds) {
    if (kinds.size() != measurements.size()) {
      throw std::invalid_argument("need one label per measurement");
    }
    std::vector<study::CarLabel> labels;
    for (const study::Label& kind : kinds) {
      labels.push_back(kind.as_car());
    }
    for (Born& track : born_) {
      track.follower.observe(record, measurements, output);
    }
    for (const TrackRecord& track : record.tracks) {
      if (track.measurement && labels[*track.measurement]) {
        car_of_track_[track.track_id] = labels[*track.measurement];
      }
    }
    for (std::size_t m = 0; m < measurements.size(); ++m) {
      if (labels[m]) {
        first_measurement_step_.emplace(*labels[m], step_);
      }
      if (const std::optional<int> id = record.started_track[m]) {
        car_of_track_[*id] = labels[m];
      }
    }
    std::vector<int> cars_with_confirmed_track;
    for (const TrackState& track : output) {
      if (const study::CarLabel car = car_of_track_.at(track.track_id)) {
        cars_with_confirmed_track.push_back(*car);
        confirmation_step_.emplace(*car, step_);
      }
    }
    for (std::size_t m = 0; m < measurements.size(); ++m) {
      if (const std::optional<int> id = record.started_track[m]) {
        const bool car_has_confirmed_track =
            labels[m] && std::count(cars_with_confirmed_track.begin(),
                                    cars_with_confirmed_track.end(), *labels[m]) > 0;
        const bool duplicate =
            labels[m] && std::any_of(born_.begin(), born_.end(), [&](const Born& earlier) {
              return earlier.birth.as_car() == labels[m] &&
                     step_ - earlier.birth_step <= kDuplicateBirthWindowSteps;
            });
        born_.push_back({TrackFollower(*id, gate_chi2_), kinds[m], car_has_confirmed_track, step_,
                         duplicate});
      }
    }
    ++step_;
  }

  TentativeOutcome outcome() const {
    TentativeOutcome result;
    for (const Born& track : born_) {
      ++result.tentative;
      result.duplicate_tentative_tracks += track.duplicate ? 1 : 0;
      switch (track.follower.fate()) {
        case Fate::confirmed:
          ++result.confirmed;
          break;
        case Fate::pending:
          ++result.unresolved;
          break;
        case Fate::dropped:
          ++result.dropped;
          if (track.birth.kind == study::LabelKind::clutter) {
            ++result.clutter_births;
          } else if (track.birth.kind == study::LabelKind::ambiguous) {
            ++result.ambiguous_births;
          } else if (track.car_had_confirmed_track) {
            ++result.car_already_confirmed;
          } else {
            ++result.real_births_lost;
          }
          break;
      }
    }
    for (const auto& [car, first] : first_measurement_step_) {
      ++result.cars;
      const auto confirmed = confirmation_step_.find(car);
      if (confirmed == confirmation_step_.end()) {
        ++result.cars_never_confirmed;
      } else {
        result.steps_to_confirmation += static_cast<long long>(confirmed->second - first);
      }
    }
    return result;
  }

 private:
  struct Born {
    TrackFollower follower;
    study::Label birth;
    bool car_had_confirmed_track;
    std::size_t birth_step;
    bool duplicate;
  };

  double gate_chi2_;
  std::size_t step_ = 0;
  std::map<int, study::CarLabel> car_of_track_;
  std::vector<Born> born_;
  std::map<int, std::size_t> first_measurement_step_;
  std::map<int, std::size_t> confirmation_step_;
};

}
