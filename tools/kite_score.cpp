#include <charconv>
#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include "format.hpp"
#include "kite/log.hpp"
#include "kite/scoring.hpp"

namespace {

using kite::tool_format::format_int;

constexpr const char* kUsage =
    "usage: kite_score --log <file> [--gate M]\n"
    "  --log    kite-log v0 file with gt and track records\n"
    "  --gate   match gate in meters, >= 0 (default 3)\n"
    "Prints: matched=N missed=N false=N rmse_m=X id_switches=N steps=N\n";

class UsageError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct Arguments {
  std::filesystem::path log;
  double gate_m = 3.0;
};

Arguments parse_arguments(int argc, char** argv) {
  Arguments args;
  bool seen_log = false;
  bool seen_gate = false;
  for (int i = 1; i < argc; i += 2) {
    const std::string_view flag = argv[i];
    bool* seen = flag == "--log" ? &seen_log : flag == "--gate" ? &seen_gate : nullptr;
    if (seen == nullptr) {
      throw UsageError("unknown argument: " + std::string(flag));
    }
    if (*seen) {
      throw UsageError("duplicate argument: " + std::string(flag));
    }
    if (i + 1 >= argc) {
      throw UsageError("missing value for " + std::string(flag));
    }
    *seen = true;
    const std::string_view value = argv[i + 1];
    if (flag == "--log") {
      args.log = std::string(value);
      continue;
    }
    const char* end = value.data() + value.size();
    const auto result = std::from_chars(value.data(), end, args.gate_m);
    if (result.ec != std::errc() || result.ptr != end || !std::isfinite(args.gate_m) ||
        args.gate_m < 0.0) {
      throw UsageError("invalid value for --gate: '" + std::string(value) + "'");
    }
  }
  if (!seen_log) {
    throw UsageError("missing argument: --log");
  }
  return args;
}

}

int main(int argc, char** argv) {
  Arguments args;
  try {
    args = parse_arguments(argc, argv);
  } catch (const UsageError& error) {
    std::cerr << error.what() << "\n" << kUsage;
    return 2;
  }

  kite::LogRecords records;
  try {
    records = kite::read_log(args.log);
  } catch (const std::exception& error) {
    std::cerr << "kite_score: cannot read " << args.log.generic_string() << ": " << error.what()
              << "\n";
    return 1;
  }

  const kite::ScoreResult result = kite::score(records.truth, records.tracks, args.gate_m);
  std::cout << "matched=" << format_int(result.matched) << " missed=" << format_int(result.missed)
            << " false=" << format_int(result.false_tracks)
            << " rmse_m=" << kite::format_fixed(result.rmse_m)
            << " id_switches=" << format_int(result.id_switches)
            << " steps=" << format_int(result.steps) << "\n";
  return 0;
}
