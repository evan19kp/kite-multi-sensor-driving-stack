#pragma once

#include <optional>

namespace kite {

struct PidGains {
  double kp;
  double ki;
  double kd;
};

struct PidLimits {
  double min;
  double max;
};

// PID controller with output limits, anti-windup and derivative on measurement. It receives the
// setpoint and a measured value, never ground truth.
//
// Each update(r, y, dt), with error e = r - y, computes
//   P = kp * e
//   I = ki * S, where S is the running sum of e * dt
//   D = kd * d, where d estimates -dy/dt (see below)
// and returns min(max(P + I + D, limits.min), limits.max).
//
// Anti-windup (conditional integration): S += e * dt happens before I is computed, except when
// P + ki * S_previous + D >= limits.max with e > 0, or <= limits.min with e < 0.
//
// Derivative on measurement: d uses the change in y, not in e, so a setpoint step adds nothing to
// D. With filter time constant tf, d is the backward-Euler discretisation of the first-order
// low-pass tf * d' + d = -dy/dt:
//   d = (tf * d_previous + (y_previous - y)) / (tf + dt).
// tf = 0 gives the raw difference (y_previous - y) / dt. The first update after construction or
// reset() has no y_previous, so it has D = 0 and leaves d = 0.
class Pid {
 public:
  // Throws std::invalid_argument unless every gain is finite and >= 0, both limits are finite with
  // min < max, and tf is finite and >= 0.
  Pid(PidGains gains, PidLimits limits, double tf = 0.0);

  // Throws std::invalid_argument, leaving the controller unchanged, for a non-finite setpoint or
  // measurement, or a dt that is <= 0 or not finite.
  double update(double setpoint, double measurement, double dt);

  // Clears S, d and y_previous, so the controller behaves as if newly constructed.
  void reset();

 private:
  PidGains gains_;
  PidLimits limits_;
  double tf_;
  double error_sum_ = 0.0;
  double derivative_ = 0.0;
  std::optional<double> previous_measurement_;
};

}  // namespace kite
