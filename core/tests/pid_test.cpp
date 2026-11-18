#include "kite/pid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace {

using kite::Pid;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr kite::PidLimits kWide{-100.0, 100.0};

void construct(double kp, double ki, double kd, double min, double max, double tf) {
  Pid pid{{kp, ki, kd}, {min, max}, tf};
  (void)pid;
}

TEST(Pid, ProportionalIsGainTimesError) {
  // kp = 2: error 5 - 3 = 2 gives 4 on every update, since P keeps no state, and error 3 - 5 = -2
  // gives -4.
  Pid pid{{2.0, 0.0, 0.0}, kWide};
  EXPECT_EQ(pid.update(5.0, 3.0, 0.1), 4.0);
  EXPECT_EQ(pid.update(5.0, 3.0, 0.1), 4.0);
  EXPECT_EQ(pid.update(3.0, 5.0, 0.1), -4.0);
}

TEST(Pid, IntegralSumsErrorTimesDt) {
  // ki = 0.5, error 2, dt 0.25: the sum of e * dt is 0.5, 1, 1.5, so I = 0.25, 0.5, 0.75.
  // Error -2 then takes the sum back to 1, so I = 0.5.
  Pid pid{{0.0, 0.5, 0.0}, kWide};
  EXPECT_EQ(pid.update(2.0, 0.0, 0.25), 0.25);
  EXPECT_EQ(pid.update(2.0, 0.0, 0.25), 0.5);
  EXPECT_EQ(pid.update(2.0, 0.0, 0.25), 0.75);
  EXPECT_EQ(pid.update(0.0, 2.0, 0.25), 0.5);
}

TEST(Pid, DerivativeIsMinusGainTimesMeasurementRate) {
  // kd = 0.5, measurements 1, 3, 3 with dt = 0.5. The first update has no previous measurement,
  // so D = 0. Then -dy/dt = (1 - 3) / 0.5 = -4, so D = -2, and a constant measurement gives D = 0.
  Pid pid{{0.0, 0.0, 0.5}, kWide};
  EXPECT_EQ(pid.update(0.0, 1.0, 0.5), 0.0);
  EXPECT_EQ(pid.update(0.0, 3.0, 0.5), -2.0);
  EXPECT_EQ(pid.update(0.0, 3.0, 0.5), 0.0);
}

TEST(Pid, DerivativeFilterIsBackwardEulerLowPass) {
  // kd = 1, tf = 0.5, dt = 0.5, so d = (0.5 d_previous + (y_previous - y)) / 1.
  // Measurements 0, 1, 1, 1: d = 0 (first update), (0 - 1) = -1, 0.5 * -1 = -0.5, then -0.25.
  // Unfiltered the same measurements give 0, -2, 0, 0.
  Pid pid{{0.0, 0.0, 1.0}, kWide, 0.5};
  EXPECT_EQ(pid.update(0.0, 0.0, 0.5), 0.0);
  EXPECT_EQ(pid.update(0.0, 1.0, 0.5), -1.0);
  EXPECT_EQ(pid.update(0.0, 1.0, 0.5), -0.5);
  EXPECT_EQ(pid.update(0.0, 1.0, 0.5), -0.25);
}

TEST(Pid, OutputIsClampedToTheLimits) {
  // kp = 10 with limits [-6, 3]: errors 1 and -1 ask for 10 and -10; error 0.25 asks for 2.5.
  Pid pid{{10.0, 0.0, 0.0}, {-6.0, 3.0}};
  EXPECT_EQ(pid.update(1.0, 0.0, 0.1), 3.0);
  EXPECT_EQ(pid.update(-1.0, 0.0, 0.1), -6.0);
  EXPECT_EQ(pid.update(0.25, 0.0, 0.1), 2.5);
}

TEST(Pid, SetpointStepGivesNoDerivativeKick) {
  // kp = 1, kd = 1, dt = 0.1, measurement held at 2. The setpoint steps from 0 to 10, so the error
  // jumps by 10; a derivative on the error would add 10 / 0.1 = 100. On the measurement it adds 0,
  // and the output is P alone, 10 - 2 = 8.
  Pid pid{{1.0, 0.0, 1.0}, {-1000.0, 1000.0}};
  EXPECT_EQ(pid.update(0.0, 2.0, 0.1), -2.0);
  EXPECT_EQ(pid.update(10.0, 2.0, 0.1), 8.0);
}

TEST(Pid, IntegralStopsWhileSaturatedAndUnwindsAtOnce) {
  // ki = 1, limits [-1, 1], error 1, dt 1. The first update integrates to 1 and outputs 1. From
  // then on the output before integration is at the max and the error is positive, so the sum
  // stays 1. When the error turns to -1 the sum drops to 0 at once and the output is 0; the next
  // update takes it to -1. A plain integrator would reach 10 after 10 updates and still output 1.
  Pid pid{{0.0, 1.0, 0.0}, {-1.0, 1.0}};
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(pid.update(1.0, 0.0, 1.0), 1.0) << "update " << i;
  }
  EXPECT_EQ(pid.update(-1.0, 0.0, 1.0), 0.0);
  EXPECT_EQ(pid.update(-1.0, 0.0, 1.0), -1.0);
}

TEST(Pid, ResetBehavesLikeANewController) {
  const auto run = [](Pid& pid) {
    std::vector<double> outputs;
    const double measurements[] = {0.0, 0.5, 1.5, 1.0, 4.0};
    for (const double y : measurements) {
      outputs.push_back(pid.update(2.0, y, 0.2));
    }
    return outputs;
  };
  Pid pid{{0.7, 0.4, 0.3}, {-1.0, 1.5}, 0.1};
  const std::vector<double> first = run(pid);
  pid.reset();
  EXPECT_EQ(run(pid), first);
}

TEST(Pid, ConstructorRejectsInvalidParameters) {
  EXPECT_NO_THROW(construct(0.0, 0.0, 0.0, -1.0, 1.0, 0.0));
  EXPECT_THROW(construct(-1.0, 0.0, 0.0, -1.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(0.0, -1.0, 0.0, -1.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(0.0, 0.0, -1.0, -1.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(kNan, 0.0, 0.0, -1.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(0.0, kInf, 0.0, -1.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(0.0, 0.0, kNan, -1.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(0.0, 0.0, 0.0, 1.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(0.0, 0.0, 0.0, 1.0, -1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(0.0, 0.0, 0.0, -kInf, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(0.0, 0.0, 0.0, -1.0, kNan, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(0.0, 0.0, 0.0, -1.0, 1.0, -0.1), std::invalid_argument);
  EXPECT_THROW(construct(0.0, 0.0, 0.0, -1.0, 1.0, kNan), std::invalid_argument);
  EXPECT_THROW(construct(0.0, 0.0, 0.0, -1.0, 1.0, kInf), std::invalid_argument);
}

TEST(Pid, InvalidUpdateThrowsAndChangesNothing) {
  Pid used{{0.7, 0.4, 0.3}, kWide, 0.1};
  Pid fresh{{0.7, 0.4, 0.3}, kWide, 0.1};
  EXPECT_EQ(used.update(1.0, 0.5, 0.1), fresh.update(1.0, 0.5, 0.1));

  EXPECT_THROW(used.update(1.0, 0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(used.update(1.0, 0.0, -0.1), std::invalid_argument);
  EXPECT_THROW(used.update(1.0, 0.0, kNan), std::invalid_argument);
  EXPECT_THROW(used.update(1.0, 0.0, kInf), std::invalid_argument);
  EXPECT_THROW(used.update(kNan, 0.0, 0.1), std::invalid_argument);
  EXPECT_THROW(used.update(-kInf, 0.0, 0.1), std::invalid_argument);
  EXPECT_THROW(used.update(1.0, kNan, 0.1), std::invalid_argument);
  EXPECT_THROW(used.update(1.0, kInf, 0.1), std::invalid_argument);

  EXPECT_EQ(used.update(1.0, 0.8, 0.1), fresh.update(1.0, 0.8, 0.1));
  EXPECT_EQ(used.update(1.0, 0.9, 0.1), fresh.update(1.0, 0.9, 0.1));
}

// Test plant: a longitudinal point-mass car. State: speed v (m/s) and acceleration a (m/s^2).
//   a_c = min(max(a_cmd, -6), 3)    actuator limits
//   a'  = (a_c - a) / tau           first-order actuator lag, tau = 0.3 s
//   v'  = a - c v                   linear drag, c = 0.05 1/s
// The controller measures v without noise or delay once per step; its command is held for the step.
// Integrated by explicit Euler with step h = 0.01 s, both derivatives taken at the old state.
// h / tau = 1/30, well inside Euler's stable range (h / tau < 2) for the lag.
// Holding 15 m/s needs a = c v = 0.75 m/s^2, so a P-only controller would settle below 15.
constexpr int kStepsPerSecond = 100;
constexpr double kStep = 1.0 / kStepsPerSecond;
constexpr double kSetpoint = 15.0;
constexpr kite::PidLimits kActuator{-6.0, 3.0};

struct SpeedPlant {
  static constexpr double kTau = 0.3;
  static constexpr double kDrag = 0.05;
  double v = 0.0;
  double a = 0.0;

  void step(double a_cmd) {
    const double a_c = std::clamp(a_cmd, kActuator.min, kActuator.max);
    const double a_dot = (a_c - a) / kTau;
    const double v_dot = a - kDrag * v;
    a += kStep * a_dot;
    v += kStep * v_dot;
  }
};

template <typename Control>
std::vector<double> closed_loop_speeds(Control control, int seconds) {
  SpeedPlant plant;
  std::vector<double> speeds{plant.v};
  for (int k = 0; k < seconds * kStepsPerSecond; ++k) {
    plant.step(control(kSetpoint, plant.v));
    speeds.push_back(plant.v);
  }
  return speeds;
}

// Step-response metrics over speeds[k] = v at t = k h:
//   rise time: from the first sample with v >= 10 % of the setpoint to the first with v >= 90 %,
//     at the sample resolution h;
//   overshoot: max(0, (max v - setpoint) / setpoint), in %;
//   steady-state error: max |setpoint - v| over the last 10 s of the run;
//   settled from: the time of the earliest sample from which every sample is within 2 % of the
//     setpoint (absent when the last sample is outside).
struct StepResponse {
  double rise_time = kNan;
  double overshoot_percent = kNan;
  double steady_state_error = kNan;
  double settled_from = kNan;
};

double first_time_at_or_above(const std::vector<double>& speeds, double threshold) {
  for (std::size_t k = 0; k < speeds.size(); ++k) {
    if (speeds[k] >= threshold) {
      return static_cast<double>(k) * kStep;
    }
  }
  return kNan;
}

StepResponse step_response(const std::vector<double>& speeds) {
  StepResponse r;
  r.rise_time = first_time_at_or_above(speeds, 0.9 * kSetpoint) -
                first_time_at_or_above(speeds, 0.1 * kSetpoint);
  const double peak = *std::max_element(speeds.begin(), speeds.end());
  r.overshoot_percent = std::max(0.0, (peak - kSetpoint) / kSetpoint * 100.0);
  const auto last_10_s = static_cast<std::size_t>(10 * kStepsPerSecond);
  r.steady_state_error = 0.0;
  for (std::size_t k = speeds.size() - last_10_s; k < speeds.size(); ++k) {
    r.steady_state_error = std::max(r.steady_state_error, std::abs(kSetpoint - speeds[k]));
  }
  for (std::size_t k = speeds.size(); k > 0; --k) {
    if (std::abs(kSetpoint - speeds[k - 1]) > 0.02 * kSetpoint) {
      if (k < speeds.size()) {
        r.settled_from = static_cast<double>(k) * kStep;
      }
      return r;
    }
  }
  r.settled_from = 0.0;
  return r;
}

// Gains for the closed loop. The output is in m/s^2 and the error in m/s, so kp is in 1/s, ki in
// 1/s^2 and kd has no unit:
//   kp = 1 1/s: the loop crosses over near 1 rad/s, a third of the actuator's 1 / tau = 3.3 rad/s,
//     so the lag costs little phase;
//   ki = 0.3 1/s^2: the integral zero at ki / kp = 0.3 rad/s sits below crossover and removes the
//     0.75 m/s^2 drag offset;
//   kd = 0.1 with tf = 0.05 s: a little damping from the filtered rate of the measured speed.
constexpr kite::PidGains kGains{1.0, 0.3, 0.1};
constexpr double kTf = 0.05;
constexpr int kRunSeconds = 40;

TEST(Pid, ClosedLoopReachesAndHoldsFifteenMetresPerSecond) {
  Pid pid{kGains, kActuator, kTf};
  const std::vector<double> speeds = closed_loop_speeds(
      [&pid](double r, double y) { return pid.update(r, y, kStep); }, kRunSeconds);
  const StepResponse response = step_response(speeds);
  std::printf("closed loop 0 -> 15 m/s: rise time %.2f s, overshoot %.3f %%, "
              "steady-state error %.2e m/s\n",
              response.rise_time, response.overshoot_percent, response.steady_state_error);

  ASSERT_FALSE(std::isnan(response.settled_from)) << "the run ends outside the 2 % band";
  EXPECT_LE(response.settled_from, 20.0);
  EXPECT_LE(response.overshoot_percent, 2.0);
  EXPECT_LE(response.steady_state_error, 0.02 * kSetpoint);
}

struct WindingPi {
  double kp;
  double ki;
  double error_sum = 0.0;

  double update(double setpoint, double measurement) {
    const double error = setpoint - measurement;
    error_sum += error * kStep;
    return std::clamp(kp * error + ki * error_sum, kActuator.min, kActuator.max);
  }
};

TEST(Pid, AntiWindupAvoidsTheOvershootOfAPlainIntegrator) {
  // From rest the command kp * 15 = 15 m/s^2 is far above the 3 m/s^2 limit, so the output starts
  // saturated, and the climb to 15 m/s takes more than 15 / 3 = 5 s. Both PI controllers (kd = 0)
  // use the same kp and ki. The plain integrator keeps summing the error during the climb,
  // overshoots by more than five times the 2 % band and settles later; with anti-windup the
  // integral starts only once the output leaves the limit, and the overshoot stays inside the band.
  Pid pid{{kGains.kp, kGains.ki, 0.0}, kActuator};
  WindingPi winding{kGains.kp, kGains.ki};
  const StepResponse with_anti_windup = step_response(closed_loop_speeds(
      [&pid](double r, double y) { return pid.update(r, y, kStep); }, kRunSeconds));
  const StepResponse without = step_response(closed_loop_speeds(
      [&winding](double r, double y) { return winding.update(r, y); }, kRunSeconds));

  EXPECT_LE(with_anti_windup.overshoot_percent, 2.0);
  EXPECT_GT(without.overshoot_percent, 10.0);
  EXPECT_GT(without.settled_from, with_anti_windup.settled_from);
}

}  // namespace
