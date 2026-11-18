#include <array>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "baseline_setup.hpp"
#include "drop_study.hpp"
#include "kite/generator.hpp"
#include "kite/kalman_tracker.hpp"
#include "kite/scoring.hpp"
#include "kite/types.hpp"
#include "study_common.hpp"

namespace {

using namespace kite::baseline_setup;
using namespace kite::drop_study;
using namespace kite::study;

constexpr const char* kUsage =
    "usage: kite_drop_study --monte-carlo | --tuning\n"
    "Measures how often KalmanTracker drops a tentative track, with the kite_baseline tracker\n"
    "parameters and the default two_stage association.\n"
    "  --monte-carlo  one vehicle, detection probability 0.95, 0.90, 0.80 and 0.70, 20000 seeds each\n"
    "                 from 100001: the first track born from the car is confirmed or dropped\n"
    "  --tuning       the kite_baseline scenario on seeds 101-120: every dropped tentative track,\n"
    "                 and the steps from a car's first measurement to its confirmation\n"
    "  --help         print this text\n";

constexpr int kSeedsPerP = 20000;
constexpr double kMonteCarloDurationS = 10.0;

struct Block {
  double detection_probability;
  int first_seed;
};
constexpr std::array<Block, 4> kBlocks = {
    {{0.95, 100001}, {0.90, 120001}, {0.80, 140001}, {0.70, 160001}}};

constexpr int kFirstTuningSeed = 101;
constexpr int kLastTuningSeed = 120;

std::string percent(double fraction) { return kite::format_fixed(fraction * 100.0); }

std::string seed_range(int first, int last) {
  return std::to_string(first) + "-" + std::to_string(last);
}

enum class Birth { none, confirmed, dropped, undecided };

struct MonteCarloSeed {
  Birth birth;
  MissCauses misses;
};

MonteCarloSeed run_monte_carlo_seed(int seed, double detection_probability) {
  kite::ScenarioConfig config = scenario_config(static_cast<std::uint64_t>(seed));
  config.vehicles = 1;
  config.duration_s = kMonteCarloDurationS;
  config.detection_probability = detection_probability;
  const kite::GeneratedScenario scenario = kite::generate(config);

  kite::KalmanTracker tracker = make_kalman_tracker(kite::AssociationMode::two_stage);
  tracker.record_steps(true);
  std::optional<TrackFollower> follower;
  for (const kite::ScenarioStep& step : scenario.steps) {
    const std::vector<kite::TrackState> output = tracker.update(step.t, step.measurements);
    const kite::StepRecord& record = tracker.step_record().value();
    if (follower) {
      follower->observe(record, step.measurements, output);
      if (follower->fate() != Fate::pending) {
        break;
      }
      continue;
    }
    for (const std::optional<int>& id : record.started_track) {
      if (id) {
        follower.emplace(*id, kKalmanGateChi2);
        break;
      }
    }
  }
  if (!follower) {
    return {Birth::none, {}};
  }
  switch (follower->fate()) {
    case Fate::confirmed:
      return {Birth::confirmed, {}};
    case Fate::dropped:
      return {Birth::dropped, follower->misses()};
    case Fate::pending:
      return {Birth::undecided, {}};
  }
  throw std::logic_error("unknown fate");
}

struct MonteCarloTotals {
  int confirmed = 0;
  int dropped = 0;
  int no_birth = 0;
  int undecided = 0;
  MissCauses misses;
};

MonteCarloTotals run_block(const Block& block) {
  MonteCarloTotals totals;
  for (int seed = block.first_seed; seed < block.first_seed + kSeedsPerP; ++seed) {
    const MonteCarloSeed result = run_monte_carlo_seed(seed, block.detection_probability);
    switch (result.birth) {
      case Birth::none:
        ++totals.no_birth;
        break;
      case Birth::undecided:
        ++totals.undecided;
        break;
      case Birth::confirmed:
        ++totals.confirmed;
        break;
      case Birth::dropped:
        ++totals.dropped;
        totals.misses.no_measurement += result.misses.no_measurement;
        totals.misses.outside_gate += result.misses.outside_gate;
        totals.misses.taken_by_other += result.misses.taken_by_other;
        break;
    }
  }
  return totals;
}

void print_monte_carlo() {
  const std::string caption = "Monte Carlo, 1 vehicle, two_stage, " +
                              std::to_string(kKalmanConfirmHits) + " hits in " +
                              std::to_string(kKalmanConfirmWindow) + " steps, " +
                              kite::format_fixed(kMonteCarloDurationS) + " s per seed, " +
                              count(kSeedsPerP) +
                              " seeds per detection probability. Rates and bounds are "
                              "percentages.\n\n";
  std::string rates = caption + table_header({"detection probability", "seeds", "births", "dropped",
                                              "measured rate (%)", "Wilson 95 % low (%)",
                                              "Wilson 95 % high (%)", "formula (%)",
                                              "formula inside interval"});
  std::string causes = "\n" + caption +
                       table_header({"detection probability", "confirmed", "dropped",
                                     "seeds with no measurement", "undecided",
                                     "misses: no measurement", "misses: outside gate",
                                     "misses: inside gate, taken by another track"});
  for (const Block& block : kBlocks) {
    const MonteCarloTotals totals = run_block(block);
    const int births = totals.confirmed + totals.dropped;
    const Interval interval = wilson_interval(totals.dropped, births);
    const double formula =
        drop_probability(block.detection_probability, kKalmanConfirmHits, kKalmanConfirmWindow);
    const double rate = static_cast<double>(totals.dropped) / static_cast<double>(births);
    const std::string p = kite::format_fixed(block.detection_probability);
    rates += row({p, seed_range(block.first_seed, block.first_seed + kSeedsPerP - 1), count(births),
                  count(totals.dropped), percent(rate), percent(interval.low),
                  percent(interval.high), percent(formula),
                  formula >= interval.low && formula <= interval.high ? "yes" : "no"});
    causes += row({p, count(totals.confirmed), count(totals.dropped), count(totals.no_birth),
                   count(totals.undecided), count(totals.misses.no_measurement),
                   count(totals.misses.outside_gate), count(totals.misses.taken_by_other)});
  }
  std::cout << rates << causes;
}

struct TuningSeed {
  int seed;
  int measurements = 0;
  int ambiguous = 0;
  TentativeOutcome outcome;
};

TuningSeed run_tuning_seed(int seed) {
  const kite::GeneratedScenario scenario =
      kite::generate(scenario_config(static_cast<std::uint64_t>(seed)));
  kite::KalmanTracker tracker = make_kalman_tracker(kite::AssociationMode::two_stage);
  tracker.record_steps(true);

  TuningSeed result{};
  result.seed = seed;
  TentativeClassifier classifier(kKalmanGateChi2);
  for (const kite::ScenarioStep& step : scenario.steps) {
    const std::vector<kite::TrackState> output = tracker.update(step.t, step.measurements);
    std::vector<CarLabel> labels;
    for (const kite::Measurement& z : step.measurements) {
      labels.push_back(label_of(z.position, step.truth));
      ++result.measurements;
      result.ambiguous += labels.back() ? 0 : 1;
    }
    classifier.observe(tracker.step_record().value(), step.measurements, output, labels);
  }
  result.outcome = classifier.outcome();
  return result;
}

std::string mean_steps(long long steps, int confirmed_cars) {
  if (confirmed_cars == 0) {
    return kite::format_fixed(std::nullopt);
  }
  return kite::format_fixed(static_cast<double>(steps) / static_cast<double>(confirmed_cars));
}

void print_tuning() {
  std::vector<TuningSeed> seeds;
  TuningSeed total{};
  for (int seed = kFirstTuningSeed; seed <= kLastTuningSeed; ++seed) {
    seeds.push_back(run_tuning_seed(seed));
    const TuningSeed& s = seeds.back();
    total.measurements += s.measurements;
    total.ambiguous += s.ambiguous;
    total.outcome += s.outcome;
  }
  const std::string caption = "Tentative tracks, two_stage, seeds " +
                              seed_range(kFirstTuningSeed, kLastTuningSeed) + ": " +
                              count(total.measurements) + " measurements, " +
                              count(total.ambiguous) + " ambiguous.\n\n";

  const auto track_row = [](const std::string& label, const TuningSeed& s) {
    const TentativeOutcome& o = s.outcome;
    return row({label, count(o.tentative), count(o.confirmed), count(o.dropped),
                count(o.unresolved), count(o.real_births_lost), count(o.car_already_confirmed),
                count(o.ambiguous_births)});
  };
  const auto car_row = [](const std::string& label, const TuningSeed& s) {
    const TentativeOutcome& o = s.outcome;
    return row({label, count(s.measurements), count(s.ambiguous), count(o.cars),
                count(o.cars_never_confirmed),
                mean_steps(o.steps_to_confirmation, o.cars - o.cars_never_confirmed)});
  };

  std::string out = caption + table_header({"seed", "tentative tracks", "confirmed", "dropped",
                                            "unresolved", "real births lost",
                                            "birth to a car with a confirmed track",
                                            "ambiguous birth"});
  for (const TuningSeed& s : seeds) {
    out += track_row(count(s.seed), s);
  }
  out += track_row("total", total);

  out += "\n" + caption +
         table_header({"seed", "measurements", "ambiguous", "cars", "cars never confirmed",
                       "mean steps, first measurement to confirmation"});
  for (const TuningSeed& s : seeds) {
    out += car_row(count(s.seed), s);
  }
  out += car_row("total", total);
  std::cout << out;
}

}

int main(int argc, char** argv) {
  const std::string_view flag = argc == 2 ? argv[1] : "";
  if (flag == "--help") {
    std::cout << kUsage;
    return 0;
  }
  if (flag == "--monte-carlo") {
    print_monte_carlo();
    return 0;
  }
  if (flag == "--tuning") {
    print_tuning();
    return 0;
  }
  std::cerr << kUsage;
  return 2;
}
