#include "kite/angles.hpp"

#include <cmath>

namespace kite {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

}  // namespace

double wrap_angle(double radians) {
  double wrapped = std::fmod(radians, kTwoPi);
  if (wrapped >= kPi) {
    wrapped -= kTwoPi;
  } else if (wrapped < -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

}  // namespace kite
