#include <charconv>
#include <cstdint>
#include <cstddef>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "kite/baseline_tracker.hpp"
#include "kite/generator.hpp"
#include "kite/kalman_tracker.hpp"
#include "kite/scoring.hpp"
#include "kite/tracker.hpp"

namespace {

constexpr int kVehicles = 5;
constexpr double kDurationS = 30.0;
constexpr double kDtS = 0.1;
constexpr double kNoiseM = 1.0;
constexpr double kDetectionProbability = 0.95;
constexpr double kAreaHalfWidthM = 100.0;
constexpr double kTrackerGateM = 5.0;
constexpr double kKalmanQ = 0.01;
constexpr double kKalmanVelocitySigma = 10.0;
constexpr double kKalmanGateChi2 = 9.21;
constexpr int kKalmanConfirmHits = 3;
constexpr int kKalmanConfirmWindow = 5;
constexpr int kKalmanMaxMisses = 5;
constexpr double kMatchGateM = 3.0;
constexpr int kDefaultSeeds = 20;

constexpr const char* kUsage =
    "usage: kite_baseline [--tracker baseline|kalman] [--seeds N | --seeds A-B]\n"
    "  --tracker  baseline (default): NearestNeighbourBaseline with gate 5 m\n"
    "             kalman: KalmanTracker with q 0.01 m^2/s^3, measurement sigma 1.0 m,\n"
    "             initial velocity sigma 10 m/s, chi-square gate 9.21, confirmed after 3 hits\n"
    "             in 5 steps, deleted after 5 consecutive misses\n"
    "  --seeds    run seeds 1..N with N >= 1, or A..B with 1 <= A <= B (default 1..20)\n"
    "  --help     print this text\n"
    "Fixed config for every seed: 5 vehicles, 30 s, dt 0.1 s, noise 1.0 m, pd 0.95,\n"
    "half-width 100 m. Scoring: match gate 3 m.\n"
    "Prints a Markdown table with one row per seed and a mean row.\n";

class UsageError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

enum class TrackerKind { baseline, kalman };

struct Options {
  TrackerKind tracker = TrackerKind::baseline;
  int first_seed = 1;
  int last_seed = kDefaultSeeds;
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

void parse_seeds(std::string_view value, Options& options) {
  const std::size_t dash = value.find('-');
  const std::optional<int> first =
      dash == std::string_view::npos ? std::optional<int>(1) : parse_positive(value.substr(0, dash));
  const std::optional<int> last = parse_positive(
      dash == std::string_view::npos ? value : value.substr(dash + 1));
  if (!first || !last || *first > *last) {
    throw UsageError("invalid value for --seeds: '" + std::string(value) + "'");
  }
  options.first_seed = *first;
  options.last_seed = *last;
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

Options parse_options(int argc, char** argv) {
  Options options;
  bool seen_seeds = false;
  bool seen_tracker = false;
  for (int i = 1; i < argc; i += 2) {
    const std::string flag = argv[i];
    bool* seen = flag == "--seeds" ? &seen_seeds : flag == "--tracker" ? &seen_tracker : nullptr;
    if (seen == nullptr) {
      throw UsageError("unknown argument: " + flag);
    }
    if (*seen) {
      throw UsageError("duplicate argument: " + flag);
    }
    *seen = true;
    if (i + 1 == argc) {
      throw UsageError("missing value for " + flag);
    }
    if (flag == "--seeds") {
      parse_seeds(argv[i + 1], options);
    } else {
      parse_tracker(argv[i + 1], options);
    }
  }
  return options;
}

std::unique_ptr<kite::Tracker> make_tracker(TrackerKind kind) {
  if (kind == TrackerKind::kalman) {
    return std::make_unique<kite::KalmanTracker>(kKalmanQ, kNoiseM, kKalmanVelocitySigma,
                                                 kKalmanGateChi2, kKalmanConfirmHits,
                                                 kKalmanConfirmWindow, kKalmanMaxMisses);
  }
  return std::make_unique<kite::NearestNeighbourBaseline>(kTrackerGateM);
}

kite::ScoreResult run_seed(std::uint64_t seed, TrackerKind kind) {
  const kite::ScenarioConfig config{seed,    kVehicles, kDurationS, kDtS,
                                    kNoiseM, kDetectionProbability, kAreaHalfWidthM};
  const kite::GeneratedScenario scenario = kite::generate(config);

  const std::unique_ptr<kite::Tracker> tracker = make_tracker(kind);
  std::vector<kite::GroundTruthState> truth;
  std::vector<kite::TrackState> tracks;
  for (const kite::ScenarioStep& step : scenario.steps) {
    const std::vector<kite::TrackState> step_tracks = tracker->update(step.t, step.measurements);
    tracks.insert(tracks.end(), step_tracks.begin(), step_tracks.end());
    truth.insert(truth.end(), step.truth.begin(), step.truth.end());
  }
  return kite::score(truth, tracks, kMatchGateM);
}

std::string format_int(std::int64_t value) {
  char buffer[32];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
  return std::string(buffer, result.ptr);
}

std::string row(const std::string& label, const std::vector<std::string>& cells) {
  std::string line = "| " + label;
  for (const std::string& cell : cells) {
    line += " | " + cell;
  }
  return line + " |\n";
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

  std::string table = "| seed | matched | missed | false | rmse_m | id_switches | steps |\n"
                      "|---|---|---|---|---|---|---|\n";
  double matched = 0.0;
  double missed = 0.0;
  double false_tracks = 0.0;
  double rmse_sum = 0.0;
  int rmse_count = 0;
  double id_switches = 0.0;
  double steps = 0.0;
  for (std::int64_t seed = options.first_seed; seed <= options.last_seed; ++seed) {
    const kite::ScoreResult result = run_seed(static_cast<std::uint64_t>(seed), options.tracker);
    table += row(format_int(seed),
                 {format_int(result.matched), format_int(result.missed),
                  format_int(result.false_tracks), kite::format_fixed(result.rmse_m),
                  format_int(result.id_switches), format_int(result.steps)});
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

  const double seeds = static_cast<double>(options.last_seed) - options.first_seed + 1.0;
  const std::optional<double> rmse_mean =
      rmse_count > 0 ? std::optional<double>(rmse_sum / rmse_count) : std::nullopt;
  table += row("mean",
               {kite::format_fixed(matched / seeds), kite::format_fixed(missed / seeds),
                kite::format_fixed(false_tracks / seeds), kite::format_fixed(rmse_mean),
                kite::format_fixed(id_switches / seeds), kite::format_fixed(steps / seeds)});
  std::cout << table;
  return 0;
}
