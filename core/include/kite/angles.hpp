#pragma once

namespace kite {

// Returns the angle equivalent to `radians` in [-pi, pi); NaN and infinities give NaN.
double wrap_angle(double radians);

}  // namespace kite
