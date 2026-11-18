#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include "kite/generator.hpp"
#include "kite/log.hpp"

namespace {

constexpr double kAreaHalfWidthM = 50.0;

constexpr const char* kUsage =
    "usage: kite_gen --seed N --vehicles N --duration S --dt S --noise M --pd P --out <file>\n"
    "  --seed       unsigned integer seed\n"
    "  --vehicles   number of vehicles, >= 0\n"
    "  --duration   scenario length in seconds, >= 0\n"
    "  --dt         time step in seconds, > 0\n"
    "  --noise      position noise sigma in meters, >= 0\n"
    "  --pd         detection probability per vehicle and step, 0 to 1\n"
    "  --out        log file to write (kite-log v0)\n"
    "The area is fixed: vehicles start in a square of half-width 50 m.\n";

class UsageError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

template <typename T>
T parse_number(std::string_view flag, std::string_view text) {
  T value{};
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(text.data(), end, value);
  if (result.ec != std::errc() || result.ptr != end) {
    throw UsageError("invalid value for " + std::string(flag) + ": '" + std::string(text) + "'");
  }
  return value;
}

struct Arguments {
  kite::ScenarioConfig config{0, 0, 0.0, 0.0, 0.0, 0.0, kAreaHalfWidthM};
  std::filesystem::path out;
};

Arguments parse_arguments(int argc, char** argv) {
  constexpr std::string_view flags[] = {"--seed", "--vehicles", "--duration", "--dt",
                                        "--noise", "--pd",      "--out"};
  constexpr std::size_t kFlagCount = sizeof(flags) / sizeof(flags[0]);
  std::string_view values[kFlagCount];
  bool seen[kFlagCount] = {};

  for (int i = 1; i < argc; i += 2) {
    const std::string_view flag = argv[i];
    std::size_t index = 0;
    while (index < kFlagCount && flags[index] != flag) {
      ++index;
    }
    if (index == kFlagCount) {
      throw UsageError("unknown argument: " + std::string(flag));
    }
    if (seen[index]) {
      throw UsageError("duplicate argument: " + std::string(flag));
    }
    if (i + 1 >= argc) {
      throw UsageError("missing value for " + std::string(flag));
    }
    seen[index] = true;
    values[index] = argv[i + 1];
  }
  for (std::size_t index = 0; index < kFlagCount; ++index) {
    if (!seen[index]) {
      throw UsageError("missing argument: " + std::string(flags[index]));
    }
  }

  Arguments args;
  args.config.seed = parse_number<std::uint64_t>(flags[0], values[0]);
  args.config.vehicles = parse_number<int>(flags[1], values[1]);
  args.config.duration_s = parse_number<double>(flags[2], values[2]);
  args.config.dt_s = parse_number<double>(flags[3], values[3]);
  args.config.position_noise_m = parse_number<double>(flags[4], values[4]);
  args.config.detection_probability = parse_number<double>(flags[5], values[5]);
  args.out = std::string(values[6]);
  return args;
}

}

int main(int argc, char** argv) {
  kite::GeneratedScenario scenario;
  std::filesystem::path out;
  try {
    const Arguments args = parse_arguments(argc, argv);
    out = args.out;
    scenario = kite::generate(args.config);
  } catch (const UsageError& error) {
    std::cerr << error.what() << "\n" << kUsage;
    return 2;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid argument: " << error.what() << "\n" << kUsage;
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "kite_gen: " << error.what() << "\n";
    return 1;
  }

  try {
    const kite::LogRecords records = kite::to_log_records(scenario);
    kite::write_log(out, records);
    std::cout << "wrote " << records.truth.size() << " gt and " << records.measurements.size()
              << " meas records to " << out.generic_string() << "\n";
  } catch (const std::exception& error) {
    std::cerr << "kite_gen: cannot write " << out.generic_string() << ": " << error.what()
              << "\n";
    return 1;
  }
  return 0;
}
