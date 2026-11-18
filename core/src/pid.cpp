#include "kite/pid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kite {

namespace {

bool finite_non_negative(double x) { return std::isfinite(x) && x >= 0.0; }

}  // namespace

Pid::Pid(PidGains gains, PidLimits limits, double tf) : gains_(gains), limits_(limits), tf_(tf) {
  if (!finite_non_negative(gains.kp) || !finite_non_negative(gains.ki) ||
      !finite_non_negative(gains.kd)) {
    throw std::invalid_argument("Pid: gains must be finite and >= 0");
  }
  if (!std::isfinite(limits.min) || !std::isfinite(limits.max) || limits.min >= limits.max) {
    throw std::invalid_argument("Pid: limits must be finite with min < max");
  }
  if (!finite_non_negative(tf)) {
    throw std::invalid_argument("Pid: filter time constant must be finite and >= 0");
  }
}

double Pid::update(double setpoint, double measurement, double dt) {
  if (!std::isfinite(setpoint) || !std::isfinite(measurement)) {
    throw std::invalid_argument("Pid: setpoint and measurement must be finite");
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("Pid: dt must be finite and > 0");
  }

  if (previous_measurement_) {
    derivative_ = (tf_ * derivative_ + (*previous_measurement_ - measurement)) / (tf_ + dt);
  }
  previous_measurement_ = measurement;

  const double error = setpoint - measurement;
  const double p = gains_.kp * error;
  const double d = gains_.kd * derivative_;
  const double before_integration = p + gains_.ki * error_sum_ + d;
  const bool pushes_past_max = before_integration >= limits_.max && error > 0.0;
  const bool pushes_past_min = before_integration <= limits_.min && error < 0.0;
  if (!pushes_past_max && !pushes_past_min) {
    error_sum_ += error * dt;
  }
  return std::clamp(p + gains_.ki * error_sum_ + d, limits_.min, limits_.max);
}

void Pid::reset() {
  error_sum_ = 0.0;
  derivative_ = 0.0;
  previous_measurement_.reset();
}

}  // namespace kite
