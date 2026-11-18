#pragma once

#include <limits>
#include <random>

namespace kite::test_helpers {

inline constexpr double kInf = std::numeric_limits<double>::infinity();

inline double uniform01(std::mt19937_64& rng) {
  return static_cast<double>(rng() >> 11) * (1.0 / 9007199254740992.0);
}

}  // namespace kite::test_helpers
