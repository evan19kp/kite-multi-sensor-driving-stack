#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "baseline_setup.hpp"
#include "format.hpp"
#include "kite/baseline_tracker.hpp"
#include "kite/generator.hpp"
#include "kite/kalman_tracker.hpp"
#include "kite/scoring.hpp"
#include "kite/tracker.hpp"

namespace {

using kite::baseline_setup::Scenario;
using kite::tool_format::format_int;

constexpr int kDefaultSeeds = 20;

constexpr const char* kUsage =
    "usage: kite_baseline [--scenario straight|hard] [--tracker baseline|kalman]\n"
    "                     [--association MODE] [--seeds N | --seeds A-B]\n"
    "       kite_baseline [--scenario straight|hard] --grid A-B\n"
    "  --scenario     straight (default): vehicles at constant velocity, no clutter\n"
    "                 hard: the same vehicles with turning, speed changes and clutter, 2 false\n"
    "                 detections per step on average\n"
    "  --tracker      baseline (default): NearestNeighbourBaseline with gate 5 m\n"
    "                 kalman: KalmanTracker with q 0.01 m^2/s^3, measurement sigma 1.0 m,\n"
    "                 initial velocity sigma 10 m/s, chi-square gate 9.21; a new track is\n"
    "                 confirmed by 3 hits in its first 5 steps or dropped once it cannot be,\n"
    "                 and a confirmed track is deleted after 5 consecutive misses\n"
    "  --association  KalmanTracker association mode, only with --tracker kalman:\n"
    "                 two_stage (default), joint or joint_likelihood\n"
    "  --seeds        run seeds 1..N with N >= 1, or A..B with 1 <= A <= B (default 1..20)\n"
    "  --grid         tuning grid for the Kalman tracker: q in {0.01, 0.1, 1, 10} m^2/s^3 and\n"
    "                 initial velocity sigma in {5, 10, 20} m/s on seeds A..B, which must all be\n"
    "                 tuning seeds (A >= 101). Prints one mean row per setting and the setting\n"
    "                 with the lowest mean of missed + false + id_switches, ties by lower rmse_m.\n"
    "                 Cannot be combined with --tracker, --association or --seeds\n"
    "  --help         print this text\n"
    "Fixed config for every seed: 5 vehicles, 30 s, dt 0.1 s, noise 1.0 m, pd 0.95,\n"
    "half-width 100 m (plus turning, speed changes and clutter in the hard scenario).\n"
    "Scoring: match gate 3 m.\n"
    "Prints a Markdown table with one row per seed and a mean row.\n";

class UsageError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

enum class TrackerKind { baseline, kalman };

struct SeedRange {
  int first;
  int last;
};

struct Options {
  Scenario scenario = Scenario::straight;
  TrackerKind tracker = TrackerKind::baseline;
  std::optional<kite::AssociationMode> association;
  SeedRange seeds{1, kDefaultSeeds};
  std::optional<SeedRange> grid;
};

std::optional<int> parse_positive(std::string_view text) {
  int value = 0;
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(text.data(), end, value);
  if (result.ec != std::errc() || result.ptr != end || value < 1) {
    return std::nullopt;
  }
  return value;
}

SeedRange parse_seed_range(std::string_view flag, std::string_view value) {
  const std::size_t dash = value.find('-');
  const std::optional<int> first =
      dash == std::string_view::npos ? std::optional<int>(1) : parse_positive(value.substr(0, dash));
  const std::optional<int> last = parse_positive(
      dash == std::string_view::npos ? value : value.substr(dash + 1));
  if (!first || !last || *first > *last) {
    throw UsageError("invalid value for " + std::string(flag) + ": '" + std::string(value) + "'");
  }
  return {*first, *last};
}

void parse_seeds(std::string_view value, Options& options) {
  options.seeds = parse_seed_range("--seeds", value);
}

void parse_grid(std::string_view value, Options& options) {
  const SeedRange range = parse_seed_range("--grid", value);
  if (range.first < kite::baseline_setup::kFirstTuningSeed) {
    throw UsageError("--grid takes tuning seeds only (from " +
                     format_int(kite::baseline_setup::kFirstTuningSeed) + "): '" +
                     std::string(value) + "'");
  }
  options.grid = range;
}

void parse_tracker(std::string_view value, Options& options) {
  if (value == "baseline") {
    options.tracker = TrackerKind::baseline;
  } else if (value == "kalman") {
    options.tracker = TrackerKind::kalman;
  } else {
    throw UsageError("invalid value for --tracker: '" + std::string(value) + "'");
  }
}

void parse_scenario(std::string_view value, Options& options) {
  const std::optional<Scenario> scenario = kite::baseline_setup::parse_scenario(value);
  if (!scenario) {
    throw UsageError("invalid value for --scenario: '" + std::string(value) + "'");
  }
  options.scenario = *scenario;
}

void parse_association(std::string_view value, Options& options) {
  options.association = kite::baseline_setup::parse_association(value);
  if (!options.association) {
    throw UsageError("invalid value for --association: '" + std::string(value) + "'");
  }
}

using FlagParser = void (*)(std::string_view, Options&);
constexpr std::array<std::pair<std::string_view, FlagParser>, 5> kFlags = {{
    {"--scenario", parse_scenario},
    {"--tracker", parse_tracker},
    {"--association", parse_association},
    {"--seeds", parse_seeds},
    {"--grid", parse_grid},
}};

Options parse_options(int argc, char** argv) {
  Options options;
  std::set<std::string_view> seen;
  for (int i = 1; i < argc; i += 2) {
    const std::string_view flag = argv[i];
    const auto known = std::find_if(kFlags.begin(), kFlags.end(),
                                    [flag](const auto& entry) { return entry.first == flag; });
    if (known == kFlags.end()) {
      throw UsageError("unknown argument: " + std::string(flag));
    }
    if (!seen.insert(flag).second) {
      throw UsageError("duplicate argument: " + std::string(flag));
    }
    if (i + 1 == argc) {
      throw UsageError("missing value for " + std::string(flag));
    }
    known->second(argv[i + 1], options);
  }
  if (options.association && options.tracker != TrackerKind::kalman) {
    throw UsageError("--association needs --tracker kalman");
  }
  if (options.grid) {
    for (const std::string_view other : {"--tracker", "--seeds"}) {
      if (seen.count(other) != 0) {
        throw UsageError("--grid cannot be combined with " + std::string(other));
      }
    }
  }
  return options;
}

using TrackerFactory = std::function<std::unique_ptr<kite::Tracker>()>;

TrackerFactory tracker_factory(const Options& options) {
  if (options.tracker == TrackerKind::kalman) {
    return [association = options.association.value_or(kite::AssociationMode::two_stage)] {
      return std::make_unique<kite::KalmanTracker>(
          kite::baseline_setup::make_kalman_tracker(association));
    };
  }
  return [] {
    return std::make_unique<kite::NearestNeighbourBaseline>(
        kite::baseline_setup::kBaselineTrackerGateM);
  };
}

kite::ScoreResult run_seed(std::uint64_t seed, Scenario scenario, const TrackerFactory& factory) {
  const kite::GeneratedScenario generated =
      kite::generate(kite::baseline_setup::scenario_config(scenario, seed));

  const std::unique_ptr<kite::Tracker> tracker = factory();
  std::vector<kite::GroundTruthState> truth;
  std::vector<kite::TrackState> tracks;
  for (const kite::ScenarioStep& step : generated.steps) {
    const std::vector<kite::TrackState> step_tracks = tracker->update(step.t, step.measurements);
    tracks.insert(tracks.end(), step_tracks.begin(), step_tracks.end());
    truth.insert(truth.end(), step.truth.begin(), step.truth.end());
  }
  return kite::score(truth, tracks, kite::baseline_setup::kMatchGateM);
}

std::string row(const std::string& label, const std::vector<std::string>& cells) {
  std::string line = "| " + label;
  for (const std::string& cell : cells) {
    line += " | " + cell;
  }
  return line + " |\n";
}

struct Totals {
  double matched = 0.0;
  double missed = 0.0;
  double false_tracks = 0.0;
  double rmse_sum = 0.0;
  int rmse_count = 0;
  double id_switches = 0.0;
  double steps = 0.0;

  void add(const kite::ScoreResult& result) {
    matched += result.matched;
    missed += result.missed;
    false_tracks += result.false_tracks;
    if (result.rmse_m) {
      rmse_sum += *result.rmse_m;
      ++rmse_count;
    }
    id_switches += result.id_switches;
    steps += result.steps;
  }

  std::optional<double> rmse_mean() const {
    return rmse_count > 0 ? std::optional<double>(rmse_sum / rmse_count) : std::nullopt;
  }

  double errors() const { return missed + false_tracks + id_switches; }

  std::vector<std::string> mean_cells(double seeds) const {
    return {kite::format_fixed(matched / seeds),      kite::format_fixed(missed / seeds),
            kite::format_fixed(false_tracks / seeds), kite::format_fixed(rmse_mean()),
            kite::format_fixed(id_switches / seeds),  kite::format_fixed(steps / seeds)};
  }
};

double seed_count(const SeedRange& range) {
  return static_cast<double>(range.last) - range.first + 1.0;
}

Totals run_seeds(const SeedRange& range, Scenario scenario, const TrackerFactory& factory,
                 std::string* seed_rows) {
  Totals totals;
  for (std::int64_t seed = range.first; seed <= range.last; ++seed) {
    const kite::ScoreResult result = run_seed(static_cast<std::uint64_t>(seed), scenario, factory);
    if (seed_rows != nullptr) {
      *seed_rows += row(format_int(seed),
                        {format_int(result.matched), format_int(result.missed),
                         format_int(result.false_tracks), kite::format_fixed(result.rmse_m),
                         format_int(result.id_switches), format_int(result.steps)});
    }
    totals.add(result);
  }
  return totals;
}

void print_seed_table(const Options& options) {
  std::string table = "| seed | matched | missed | false | rmse_m | id_switches | steps |\n"
                      "|---|---|---|---|---|---|---|\n";
  const Totals totals =
      run_seeds(options.seeds, options.scenario, tracker_factory(options), &table);
  table += row("mean", totals.mean_cells(seed_count(options.seeds)));
  std::cout << table;
}

struct GridSetting {
  double q;
  double velocity_sigma;
  Totals totals;
};

// The R15 rule: the lowest mean of missed + false + id_switches, ties by the lower rmse_m. A
// setting with no matched seed has no rmse_m and loses a tie.
bool beats(const GridSetting& candidate, const GridSetting& best) {
  if (candidate.totals.errors() != best.totals.errors()) {
    return candidate.totals.errors() < best.totals.errors();
  }
  const double infinity = std::numeric_limits<double>::infinity();
  return candidate.totals.rmse_mean().value_or(infinity) < best.totals.rmse_mean().value_or(infinity);
}

void print_grid(const Options& options) {
  namespace setup = kite::baseline_setup;
  const SeedRange range = *options.grid;
  const double seeds = seed_count(range);

  std::vector<GridSetting> settings;
  for (const double q : setup::kTuningQs) {
    for (const double velocity_sigma : setup::kTuningVelocitySigmas) {
      const TrackerFactory factory = [q, velocity_sigma] {
        return std::make_unique<kite::KalmanTracker>(setup::make_kalman_tracker(
            q, velocity_sigma, kite::AssociationMode::two_stage));
      };
      settings.push_back({q, velocity_sigma, run_seeds(range, options.scenario, factory, nullptr)});
    }
  }

  const GridSetting* selected = &settings.front();
  for (const GridSetting& setting : settings) {
    if (beats(setting, *selected)) {
      selected = &setting;
    }
  }

  std::string text = std::string("Kalman tuning grid, ") +
                     std::string(kite::baseline_setup::scenario_name(options.scenario)) +
                     " scenario, seeds " + format_int(range.first) + "-" + format_int(range.last) +
                     ", mean per seed; cost is the mean of missed + false + id_switches\n\n"
                     "| q | velocity_sigma | matched | missed | false | rmse_m | id_switches | "
                     "steps | cost |\n"
                     "|---|---|---|---|---|---|---|---|---|\n";
  for (const GridSetting& setting : settings) {
    std::vector<std::string> cells = {kite::format_fixed(setting.velocity_sigma)};
    for (std::string& cell : setting.totals.mean_cells(seeds)) {
      cells.push_back(std::move(cell));
    }
    cells.push_back(kite::format_fixed(setting.totals.errors() / seeds));
    text += row(kite::format_fixed(setting.q), cells);
  }
  text += "\nselected: q " + kite::format_fixed(selected->q) + ", velocity_sigma " +
          kite::format_fixed(selected->velocity_sigma) + "\n";
  std::cout << text;
}

}

int main(int argc, char** argv) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::cout << kUsage;
    return 0;
  }
  Options options;
  try {
    options = parse_options(argc, argv);
  } catch (const UsageError& error) {
    std::cerr << error.what() << "\n" << kUsage;
    return 2;
  }

  if (options.grid) {
    print_grid(options);
  } else {
    print_seed_table(options);
  }
  return 0;
}
