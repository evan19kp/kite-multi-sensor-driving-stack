#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>

#include "baseline_setup.hpp"
#include "kite/baseline_tracker.hpp"
#include "kite/kalman_tracker.hpp"
#include "kite/log.hpp"
#include "kite/tracker.hpp"
#include "track_log.hpp"

namespace {

constexpr const char* kUsage =
    "usage: kite_track --in <log> --out <log> [--tracker baseline|kalman]\n"
    "  --in       kite-log v0 file; only its meas records reach the tracker\n"
    "  --out      kite-log v0 file to write: the input's gt and meas records and one track\n"
    "             record per output track and step; track records already in the input are dropped\n"
    "  --tracker  kalman (default): KalmanTracker with the kite_baseline parameters\n"
    "             baseline: NearestNeighbourBaseline with the kite_baseline gate\n"
    "A step is the meas records that share one t. A step with no meas record is not run.\n";

class UsageError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct Arguments {
  std::filesystem::path in;
  std::filesystem::path out;
  bool kalman = true;
};

Arguments parse_arguments(int argc, char** argv) {
  Arguments args;
  std::set<std::string_view> seen;
  for (int i = 1; i < argc; i += 2) {
    const std::string_view flag = argv[i];
    if (flag != "--in" && flag != "--out" && flag != "--tracker") {
      throw UsageError("unknown argument: " + std::string(flag));
    }
    if (!seen.insert(flag).second) {
      throw UsageError("duplicate argument: " + std::string(flag));
    }
    if (i + 1 >= argc) {
      throw UsageError("missing value for " + std::string(flag));
    }
    const std::string_view value = argv[i + 1];
    if (flag == "--in") {
      args.in = std::string(value);
    } else if (flag == "--out") {
      args.out = std::string(value);
    } else if (value == "kalman" || value == "baseline") {
      args.kalman = value == "kalman";
    } else {
      throw UsageError("invalid value for --tracker: '" + std::string(value) + "'");
    }
  }
  for (const std::string_view flag : {"--in", "--out"}) {
    if (!seen.count(flag)) {
      throw UsageError("missing argument: " + std::string(flag));
    }
  }
  return args;
}

std::unique_ptr<kite::Tracker> make_tracker(bool kalman) {
  if (kalman) {
    return std::make_unique<kite::KalmanTracker>(
        kite::baseline_setup::make_kalman_tracker(kite::AssociationMode::two_stage));
  }
  return std::make_unique<kite::NearestNeighbourBaseline>(
      kite::baseline_setup::kBaselineTrackerGateM);
}

}

int main(int argc, char** argv) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::cout << kUsage;
    return 0;
  }
  Arguments args;
  try {
    args = parse_arguments(argc, argv);
  } catch (const UsageError& error) {
    std::cerr << error.what() << "\n" << kUsage;
    return 2;
  }

  kite::LogRecords records;
  try {
    records = kite::read_log(args.in);
  } catch (const std::exception& error) {
    std::cerr << "kite_track: cannot read " << args.in.generic_string() << ": " << error.what()
              << "\n";
    return 1;
  }

  const std::unique_ptr<kite::Tracker> tracker = make_tracker(args.kalman);
  records.tracks = kite::tool_track::run_tracker(*tracker, records.measurements);

  try {
    kite::write_log(args.out, records);
  } catch (const std::exception& error) {
    std::cerr << "kite_track: cannot write " << args.out.generic_string() << ": " << error.what()
              << "\n";
    return 1;
  }
  return 0;
}
