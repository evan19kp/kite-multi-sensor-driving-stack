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
    "usage: kite_drop_study (--monte-carlo | --tuning) [--scenario straight|hard]\n"
    "Measures how often KalmanTracker drops a tentative track, with the kite_baseline tracker\n"
    "parameters and the default two_stage association.\n"
    "  --monte-carlo  one vehicle, detection probability 0.95, 0.90, 0.80 and 0.70, 20000 seeds each\n"
    "                 from 100001: the first track born from the car is confirmed or dropped\n"
    "  --tuning       the kite_baseline scenario on seeds 101-120: every dropped tentative track,\n"
    "                 and the steps from a car's first measurement to its confirmation\n"
    "  --scenario     straight (default) or hard, as in kite_baseline. With hard, --tuning counts\n"
    "                 clutter and ambiguous measurements and births apart and adds the duplicate\n"
    "                 tentative tracks of one car; --monte-carlo takes straight only\n"
    "  --help         print this text\n";

class UsageError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

constexpr int kSeedsPerP = 20000;
constexpr double kMonteCarloDurationS = 10.0;

struct Block {
  double detection_probability;
  int first_seed;
};
constexpr std::array<Block, 4> kBlocks = {
    {{0.95, 100001}, {0.90, 120001}, {0.80, 140001}, {0.70, 160001}}};

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
  LabelCounts unlabelled;
  TentativeOutcome outcome;
};

TuningSeed run_tuning_seed(int seed, Scenario scenario_kind) {
  const kite::GeneratedScenario scenario =
      kite::generate(scenario_config(scenario_kind, static_cast<std::uint64_t>(seed)));
  kite::KalmanTracker tracker = make_kalman_tracker(kite::AssociationMode::two_stage);
  tracker.record_steps(true);

  TuningSeed result{};
  result.seed = seed;
  TentativeClassifier classifier(kKalmanGateChi2);
  for (const kite::ScenarioStep& step : scenario.steps) {
    const std::vector<kite::TrackState> output = tracker.update(step.t, step.measurements);
    std::vector<Label> labels;
    for (const kite::Measurement& z : step.measurements) {
      labels.push_back(classify_measurement(z.position, step.truth));
      ++result.measurements;
      result.unlabelled.add(labels.back());
    }
    classifier.observe_labelled(tracker.step_record().value(), step.measurements, output, labels);
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

void print_tuning(Scenario scenario) {
  const bool hard = scenario == Scenario::hard;
  std::vector<TuningSeed> seeds;
  TuningSeed total{};
  for (int seed = kFirstTuningSeed; seed <= kLastTuningSeed; ++seed) {
    seeds.push_back(run_tuning_seed(seed, scenario));
    const TuningSeed& s = seeds.back();
    total.measurements += s.measurements;
    total.unlabelled += s.unlabelled;
    total.outcome += s.outcome;
  }
  const std::string caption = "Tentative tracks, two_stage" + scenario_suffix(scenario) +
                              ", seeds " + seed_range(kFirstTuningSeed, kLastTuningSeed) + ": " +
                              label_summary(scenario, total.measurements, total.unlabelled) +
                              ".\n\n";

  const auto track_row = [hard](const std::string& label, const TuningSeed& s) {
    const TentativeOutcome& o = s.outcome;
    std::vector<std::string> cells{label,
                                   count(o.tentative),
                                   count(o.confirmed),
                                   count(o.dropped),
                                   count(o.unresolved),
                                   count(o.real_births_lost),
                                   count(o.car_already_confirmed)};
    if (hard) {
      cells.push_back(count(o.clutter_births));
      cells.push_back(count(o.ambiguous_births));
      cells.push_back(count(o.duplicate_tentative_tracks));
    } else {
      cells.push_back(count(o.unlabelled_births()));
    }
    return row(cells);
  };
  const auto car_row = [scenario](const std::string& label, const TuningSeed& s) {
    const TentativeOutcome& o = s.outcome;
    std::vector<std::string> cells{label, count(s.measurements)};
    for (std::string& cell : label_cells(scenario, s.unlabelled)) {
      cells.push_back(std::move(cell));
    }
    cells.push_back(count(o.cars));
    cells.push_back(count(o.cars_never_confirmed));
    cells.push_back(mean_steps(o.steps_to_confirmation, o.cars - o.cars_never_confirmed));
    return row(cells);
  };

  std::vector<std::string> track_names{"seed",
                                       "tentative tracks",
                                       "confirmed",
                                       "dropped",
                                       "unresolved",
                                       "real births lost",
                                       "birth to a car with a confirmed track"};
  if (hard) {
    track_names.push_back("clutter birth");
    track_names.push_back("ambiguous birth");
    track_names.push_back("duplicate tentative tracks");
  } else {
    track_names.push_back("ambiguous birth");
  }
  std::string out = caption + table_header(track_names);
  for (const TuningSeed& s : seeds) {
    out += track_row(count(s.seed), s);
  }
  out += track_row("total", total);

  std::vector<std::string> car_names{"seed", "measurements"};
  for (const std::string& name : label_headers(scenario)) {
    car_names.push_back(name);
  }
  car_names.push_back("cars");
  car_names.push_back("cars never confirmed");
  car_names.push_back("mean steps, first measurement to confirmation");
  out += "\n" + caption + table_header(car_names);
  for (const TuningSeed& s : seeds) {
    out += car_row(count(s.seed), s);
  }
  out += car_row("total", total);
  std::cout << out;
}

enum class Study { monte_carlo, tuning };

struct Options {
  Study study;
  Scenario scenario;
};

Options parse_options(int argc, char** argv) {
  std::optional<Study> study;
  std::optional<Scenario> scenario;
  for (int i = 1; i < argc; ++i) {
    const std::string_view flag = argv[i];
    if (flag == "--monte-carlo" || flag == "--tuning") {
      if (study) {
        throw UsageError("give one of --monte-carlo and --tuning, once");
      }
      study = flag == "--tuning" ? Study::tuning : Study::monte_carlo;
    } else if (flag == "--scenario") {
      if (scenario) {
        throw UsageError("duplicate argument: --scenario");
      }
      if (++i == argc) {
        throw UsageError("missing value for --scenario");
      }
      scenario = parse_scenario(argv[i]);
      if (!scenario) {
        throw UsageError("invalid value for --scenario: '" + std::string(argv[i]) + "'");
      }
    } else {
      throw UsageError("unknown argument: " + std::string(flag));
    }
  }
  if (!study) {
    throw UsageError("give --monte-carlo or --tuning");
  }
  if (*study == Study::monte_carlo && scenario == Scenario::hard) {
    throw UsageError("--monte-carlo takes --scenario straight only");
  }
  return {*study, scenario.value_or(Scenario::straight)};
}

}

int main(int argc, char** argv) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::cout << kUsage;
    return 0;
  }
  try {
    const Options options = parse_options(argc, argv);
    if (options.study == Study::monte_carlo) {
      print_monte_carlo();
    } else {
      print_tuning(options.scenario);
    }
    return 0;
  } catch (const UsageError& error) {
    std::cerr << error.what() << "\n" << kUsage;
    return 2;
  }
}
