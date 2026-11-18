#include <charconv>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "kite/baseline_tracker.hpp"
#include "kite/generator.hpp"
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
constexpr double kMatchGateM = 3.0;
constexpr int kDefaultSeeds = 20;

constexpr const char* kUsage =
    "usage: kite_baseline [--seeds N]\n"
    "  --seeds   run seeds 1..N, N >= 1 (default 20)\n"
    "  --help    print this text\n"
    "Fixed config for every seed: 5 vehicles, 30 s, dt 0.1 s, noise 1.0 m, pd 0.95,\n"
    "half-width 100 m. Tracker: NearestNeighbourBaseline with gate 5 m. Scoring: match gate 3 m.\n"
    "Prints a Markdown table with one row per seed and a mean row.\n";

class UsageError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

int parse_seeds(int argc, char** argv) {
  if (argc == 1) {
    return kDefaultSeeds;
  }
  const std::string_view flag = argv[1];
  if (flag != "--seeds") {
    throw UsageError("unknown argument: " + std::string(flag));
  }
  if (argc == 2) {
    throw UsageError("missing value for --seeds");
  }
  if (argc > 3) {
    const std::string_view extra = argv[3];
    throw UsageError((extra == "--seeds" ? "duplicate argument: " : "unknown argument: ") +
                     std::string(extra));
  }
  const std::string_view value = argv[2];
  int seeds = 0;
  const char* end = value.data() + value.size();
  const auto result = std::from_chars(value.data(), end, seeds);
  if (result.ec != std::errc() || result.ptr != end || seeds < 1) {
    throw UsageError("invalid value for --seeds: '" + std::string(value) + "'");
  }
  return seeds;
}

kite::ScoreResult run_seed(std::uint64_t seed) {
  const kite::ScenarioConfig config{seed,    kVehicles, kDurationS, kDtS,
                                    kNoiseM, kDetectionProbability, kAreaHalfWidthM};
  const kite::GeneratedScenario scenario = kite::generate(config);

  kite::NearestNeighbourBaseline baseline(kTrackerGateM);
  kite::Tracker& tracker = baseline;
  std::vector<kite::GroundTruthState> truth;
  std::vector<kite::TrackState> tracks;
  for (const kite::ScenarioStep& step : scenario.steps) {
    const std::vector<kite::TrackState> step_tracks = tracker.update(step.t, step.measurements);
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

std::string format_fixed(const std::optional<double>& value) {
  if (!value) {
    return "n/a";
  }
  char buffer[512];
  const auto result =
      std::to_chars(buffer, buffer + sizeof(buffer), *value, std::chars_format::fixed, 3);
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
  int seeds = 0;
  try {
    seeds = parse_seeds(argc, argv);
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
  for (int seed = 1; seed <= seeds; ++seed) {
    const kite::ScoreResult result = run_seed(static_cast<std::uint64_t>(seed));
    table += row(format_int(seed),
                 {format_int(result.matched), format_int(result.missed),
                  format_int(result.false_tracks), format_fixed(result.rmse_m),
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

  const std::optional<double> rmse_mean =
      rmse_count > 0 ? std::optional<double>(rmse_sum / rmse_count) : std::nullopt;
  table += row("mean", {format_fixed(matched / seeds), format_fixed(missed / seeds),
                        format_fixed(false_tracks / seeds), format_fixed(rmse_mean),
                        format_fixed(id_switches / seeds), format_fixed(steps / seeds)});
  std::cout << table;
  return 0;
}
