#include "kite/generator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "kite/log.hpp"

namespace {

namespace fs = std::filesystem;

kite::ScenarioConfig make_config() {
  return {7, 5, 3.0, 0.1, 0.5, 0.8, 50.0};
}

bool same(const kite::GeneratedScenario& a, const kite::GeneratedScenario& b) {
  if (a.steps.size() != b.steps.size()) {
    return false;
  }
  for (std::size_t k = 0; k < a.steps.size(); ++k) {
    if (a.steps[k].t != b.steps[k].t || !(a.steps[k].truth == b.steps[k].truth) ||
        !(a.steps[k].measurements == b.steps[k].measurements)) {
      return false;
    }
  }
  return true;
}

std::vector<kite::Measurement> all_measurements(const kite::GeneratedScenario& scenario) {
  return kite::to_log_records(scenario).measurements;
}

class GeneratedFiles : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    dir_ = fs::temp_directory_path() /
           ("kite_sim_test_" +
            std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + "_" +
            std::to_string(ticks));
    fs::create_directories(dir_);
  }

  void TearDown() override {
    std::error_code ignored;
    fs::remove_all(dir_, ignored);
  }

  std::string write_and_read_bytes(const kite::GeneratedScenario& scenario) const {
    const fs::path path = dir_ / "generated.log";
    kite::write_log(path, kite::to_log_records(scenario));
    return read_bytes(path);
  }

  static std::string read_bytes(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
  }

  fs::path dir_;
};

TEST(Generator, SameConfigGivesIdenticalScenario) {
  const kite::ScenarioConfig config = make_config();
  EXPECT_TRUE(same(kite::generate(config), kite::generate(config)));
}

TEST(Generator, DifferentSeedsGiveDifferentMeasurements) {
  kite::ScenarioConfig a = make_config();
  kite::ScenarioConfig b = make_config();
  b.seed = a.seed + 1;
  EXPECT_NE(all_measurements(kite::generate(a)), all_measurements(kite::generate(b)));
}

TEST(Generator, StepsCoverTheDurationInclusive) {
  const kite::GeneratedScenario scenario = kite::generate({1, 2, 2.0, 0.1, 0.5, 1.0, 50.0});
  ASSERT_EQ(scenario.steps.size(), 21u);
  for (std::size_t k = 0; k < scenario.steps.size(); ++k) {
    EXPECT_EQ(scenario.steps[k].t, static_cast<double>(k) * 0.1);
  }
}

TEST(Generator, VehiclesStartInsideTheAreaWithSpeedInRange) {
  const double half_width = 50.0;
  const kite::GeneratedScenario scenario =
      kite::generate({3, 200, 0.0, 0.1, 0.5, 1.0, half_width});
  ASSERT_EQ(scenario.steps.size(), 1u);
  ASSERT_EQ(scenario.steps[0].truth.size(), 200u);
  for (const kite::GroundTruthState& state : scenario.steps[0].truth) {
    EXPECT_LE(std::abs(state.position.x), half_width);
    EXPECT_LE(std::abs(state.position.y), half_width);
    const double speed = std::hypot(state.velocity.x, state.velocity.y);
    EXPECT_GE(speed, 5.0 - 1e-9);
    EXPECT_LE(speed, 15.0 + 1e-9);
  }
}

TEST(Generator, TruthMovesAtConstantVelocity) {
  const kite::ScenarioConfig config = make_config();
  const kite::GeneratedScenario scenario = kite::generate(config);
  ASSERT_GT(scenario.steps.size(), 2u);
  for (std::size_t k = 0; k + 1 < scenario.steps.size(); ++k) {
    const auto& now = scenario.steps[k].truth;
    const auto& next = scenario.steps[k + 1].truth;
    ASSERT_EQ(now.size(), static_cast<std::size_t>(config.vehicles));
    ASSERT_EQ(next.size(), now.size());
    for (std::size_t i = 0; i < now.size(); ++i) {
      ASSERT_EQ(now[i].actor_id, static_cast<int>(i));
      ASSERT_EQ(next[i].actor_id, static_cast<int>(i));
      EXPECT_EQ(next[i].velocity, now[i].velocity);
      EXPECT_NEAR(next[i].position.x, now[i].position.x + now[i].velocity.x * config.dt_s, 1e-9);
      EXPECT_NEAR(next[i].position.y, now[i].position.y + now[i].velocity.y * config.dt_s, 1e-9);
    }
  }
}

TEST(Generator, CertainDetectionGivesOneMeasurementPerVehicleAndStep) {
  kite::ScenarioConfig config = make_config();
  config.detection_probability = 1.0;
  for (const kite::ScenarioStep& step : kite::generate(config).steps) {
    EXPECT_EQ(step.measurements.size(), static_cast<std::size_t>(config.vehicles));
  }
}

TEST(Generator, ZeroDetectionGivesNoMeasurements) {
  kite::ScenarioConfig config = make_config();
  config.detection_probability = 0.0;
  const kite::GeneratedScenario scenario = kite::generate(config);
  for (const kite::ScenarioStep& step : scenario.steps) {
    EXPECT_TRUE(step.measurements.empty());
    EXPECT_EQ(step.truth.size(), static_cast<std::size_t>(config.vehicles));
  }
}

TEST(Generator, MeasurementNoiseHasTheConfiguredMeanAndSpread) {
  const double sigma = 0.5;
  const kite::GeneratedScenario scenario = kite::generate({11, 1, 1100.0, 0.1, sigma, 1.0, 50.0});
  ASSERT_GE(scenario.steps.size(), 10000u);

  std::vector<double> noise_x;
  std::vector<double> noise_y;
  for (const kite::ScenarioStep& step : scenario.steps) {
    ASSERT_EQ(step.truth.size(), 1u);
    ASSERT_EQ(step.measurements.size(), 1u);
    noise_x.push_back(step.measurements[0].position.x - step.truth[0].position.x);
    noise_y.push_back(step.measurements[0].position.y - step.truth[0].position.y);
  }

  for (const std::vector<double>* samples : {&noise_x, &noise_y}) {
    const double n = static_cast<double>(samples->size());
    double sum = 0.0;
    for (double value : *samples) {
      sum += value;
    }
    const double mean = sum / n;
    double squares = 0.0;
    for (double value : *samples) {
      squares += (value - mean) * (value - mean);
    }
    const double stddev = std::sqrt(squares / (n - 1.0));
    EXPECT_NEAR(mean, 0.0, 0.05 * sigma);
    EXPECT_NEAR(stddev, sigma, 0.05 * sigma);
  }
}

TEST(Generator, MeasurementOrderDoesNotFollowVehicleOrder) {
  const kite::GeneratedScenario scenario = kite::generate({5, 20, 5.0, 0.1, 0.01, 1.0, 1000.0});
  bool any_step_reordered = false;
  for (const kite::ScenarioStep& step : scenario.steps) {
    ASSERT_EQ(step.measurements.size(), 20u);
    std::vector<int> behind;
    for (const kite::Measurement& m : step.measurements) {
      int nearest = -1;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (const kite::GroundTruthState& truth : step.truth) {
        const double distance =
            std::hypot(m.position.x - truth.position.x, m.position.y - truth.position.y);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest = truth.actor_id;
        }
      }
      ASSERT_LT(nearest_distance, 1.0);
      behind.push_back(nearest);
    }
    if (!std::is_sorted(behind.begin(), behind.end())) {
      any_step_reordered = true;
    }
    std::sort(behind.begin(), behind.end());
    for (std::size_t i = 0; i < behind.size(); ++i) {
      EXPECT_EQ(behind[i], static_cast<int>(i));
    }
  }
  EXPECT_TRUE(any_step_reordered);
}

TEST(Generator, RejectsInvalidConfigs) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const kite::ScenarioConfig good = make_config();
  std::vector<kite::ScenarioConfig> bad(11, good);
  bad[0].vehicles = -1;
  bad[1].dt_s = 0.0;
  bad[2].dt_s = -0.1;
  bad[3].duration_s = -1.0;
  bad[4].position_noise_m = -0.5;
  bad[5].detection_probability = 1.5;
  bad[6].detection_probability = -0.1;
  bad[7].area_half_width_m = 0.0;
  bad[8].duration_s = nan;
  bad[9].position_noise_m = inf;
  bad[10].duration_s = 1e300;
  for (const kite::ScenarioConfig& config : bad) {
    EXPECT_THROW(kite::generate(config), std::invalid_argument);
  }
}

TEST(Generator, AcceptsEmptyAndSingleStepConfigs) {
  EXPECT_EQ(kite::generate({1, 0, 1.0, 0.5, 0.5, 1.0, 50.0}).steps.size(), 3u);
  const kite::GeneratedScenario single = kite::generate({1, 3, 0.0, 0.5, 0.5, 1.0, 50.0});
  ASSERT_EQ(single.steps.size(), 1u);
  EXPECT_EQ(single.steps[0].t, 0.0);
}

kite::ScenarioConfig hard_config() {
  kite::ScenarioConfig config{7, 3, 10.0, 0.1, 1.0, 0.95, 50.0};
  config.turning = true;
  config.speed_changes = true;
  config.clutter_per_step = 2.0;
  return config;
}

double turn_between(const kite::Vec2& a, const kite::Vec2& b) {
  return std::atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);
}

struct Moments {
  double mean;
  double variance;
};

Moments moments(const std::vector<double>& samples) {
  const double n = static_cast<double>(samples.size());
  double sum = 0.0;
  for (double value : samples) {
    sum += value;
  }
  const double mean = sum / n;
  double squares = 0.0;
  for (double value : samples) {
    squares += (value - mean) * (value - mean);
  }
  return {mean, squares / (n - 1.0)};
}

TEST(Generator, ManoeuvresAndClutterAreDeterministic) {
  const kite::ScenarioConfig config = hard_config();
  EXPECT_TRUE(same(kite::generate(config), kite::generate(config)));
}

TEST(Generator, ManoeuvresStartFromTheStraightScenariosVehicles) {
  const kite::ScenarioConfig straight{7, 3, 10.0, 0.1, 1.0, 0.95, 50.0};
  const kite::GeneratedScenario plain = kite::generate(straight);
  const kite::GeneratedScenario hard = kite::generate(hard_config());
  ASSERT_EQ(hard.steps.size(), plain.steps.size());
  EXPECT_EQ(hard.steps[0].truth, plain.steps[0].truth);
  EXPECT_FALSE(hard.steps.back().truth == plain.steps.back().truth);
}

TEST(Generator, ClutterNeverMovesTheVehicles) {
  const kite::ScenarioConfig straight = make_config();
  kite::ScenarioConfig cluttered = straight;
  cluttered.clutter_per_step = 3.0;
  const kite::GeneratedScenario plain = kite::generate(straight);
  const kite::GeneratedScenario with_clutter = kite::generate(cluttered);
  ASSERT_EQ(with_clutter.steps.size(), plain.steps.size());
  for (std::size_t k = 0; k < plain.steps.size(); ++k) {
    EXPECT_EQ(with_clutter.steps[k].truth, plain.steps[k].truth);
  }
  EXPECT_GT(all_measurements(with_clutter).size(), all_measurements(plain).size());
}

// One vehicle held at a constant turn rate and speed for T = 60 s in 600 steps, against the
// closed-form circle of radius v / omega. Tolerance 1e-8 m: the per-step rotation E is a few ulp
// off and is applied 600 times, and each heading product rounds once more. That turns the heading
// by at most about 600 * 6 * 1.1e-16 = 4e-13 rad, which moves the end of a 900 m path by under
// 900 * 4e-13 = 4e-10 m. Summing 600 steps into positions under 512 m adds under
// 600 * 5.7e-14 = 4e-11 m. Truncating the series adds under 3e-22 per step, relative. The
// tolerance is about 20 times their sum. Measured with MinGW g++ 13.2, the largest error of the
// three rates is 5.5e-13 m, as the rounding errors do not all add up.
TEST(Generator, ATurningVehicleEndsWhereTheCircleSays) {
  const double dt = 0.1;
  const int steps = 600;
  const double total = steps * dt;
  const double speed = 15.0;
  const double start_angle = 0.7;
  for (double rate : {0.3, -0.3, -0.07}) {
    kite::MotionState state{{12.5, -7.25}, {std::cos(start_angle), std::sin(start_angle)}, speed};
    for (int k = 0; k < steps; ++k) {
      state = kite::advance(state, rate, speed, dt);
    }
    const double radius = speed / rate;
    const double end_angle = start_angle + rate * total;
    EXPECT_NEAR(state.position.x,
                12.5 + radius * (std::sin(end_angle) - std::sin(start_angle)), 1e-8)
        << rate;
    EXPECT_NEAR(state.position.y,
                -7.25 - radius * (std::cos(end_angle) - std::cos(start_angle)), 1e-8)
        << rate;
    EXPECT_NEAR(state.heading.x, std::cos(end_angle), 1e-12) << rate;
    EXPECT_NEAR(state.heading.y, std::sin(end_angle), 1e-12) << rate;
    EXPECT_EQ(state.speed, speed);
  }
}

// Turn rate -0.2 rad/s with speed rising linearly from 3 to 18 m/s over 60 s, against the
// closed-form integral of (v0 + a t) e^(i (phi0 + omega t)). Same error budget as the circle above.
TEST(Generator, AnAcceleratingTurningVehicleEndsWhereTheIntegralSays) {
  const double dt = 0.1;
  const int steps = 600;
  const double total = steps * dt;
  const double rate = -0.2;
  const double v0 = 3.0;
  const double acceleration = 0.25;
  const std::complex<double> start_heading = std::polar(1.0, 0.7);
  kite::MotionState state{{0.0, 0.0}, {start_heading.real(), start_heading.imag()}, v0};
  for (int k = 0; k < steps; ++k) {
    state = kite::advance(state, rate, v0 + acceleration * (k + 1) * dt, dt);
  }
  const std::complex<double> i_omega(0.0, rate);
  const std::complex<double> turn = std::exp(i_omega * total);
  const std::complex<double> end =
      start_heading * (v0 * (turn - 1.0) / i_omega +
                       acceleration * (total * turn / i_omega + (turn - 1.0) / (rate * rate)));
  EXPECT_NEAR(state.position.x, end.real(), 1e-8);
  EXPECT_NEAR(state.position.y, end.imag(), 1e-8);
  EXPECT_DOUBLE_EQ(state.speed, v0 + acceleration * total);
}

// With turn rate 0 the rotation is exactly (1, 0), so advance divides the heading h by the square
// root of |h|^2. Take h = (1 + 2^-52, 2^-26): hx^2 = 1 + 2^-51 + 2^-104 rounds to 1 + 2^-51, and
// hy^2 = 2^-52 is exact, so |h|^2 = 1 + 3 * 2^-52 exactly. Let m = 1 + 3 * 2^-53, the midpoint
// between the doubles 1 + 2^-52 and 1 + 2^-51. Since m^2 = |h|^2 + 9 * 2^-106, the square root of
// |h|^2 is m - 9 * 2^-107 to first order, and the correctly rounded root is 1 + 2^-52. MinGW-w64's
// sqrt rounds first to a 64-bit significand with the x87 fsqrt. That lands on m itself, because
// 9 * 2^-107 is under half its ulp of 2^-63, and the tie then goes to the even 1 + 2^-51. With the
// correct root, hx / root = 1 exactly and hy / root = 2^-26 / (1 + 2^-52) rounds to
// 2^-26 - 2^-78. With the double-rounded root, hx / root rounds to 1 - 2^-52.
TEST(Generator, AdvanceRenormalisesTheHeadingWithTheCorrectlyRoundedLength) {
  const double two_pow_minus_26 = 1.0 / 67108864.0;
  const double two_pow_minus_52 = two_pow_minus_26 * two_pow_minus_26;
  const kite::MotionState state{{0.0, 0.0}, {1.0 + two_pow_minus_52, two_pow_minus_26}, 10.0};
  const kite::MotionState next = kite::advance(state, 0.0, 10.0, 0.1);
  EXPECT_EQ(next.heading.x, 1.0);
  EXPECT_EQ(next.heading.y, two_pow_minus_26 - two_pow_minus_26 * two_pow_minus_52);
}

// Between two steps of one vehicle, the turn read from its two velocities is the step's turn rate
// times dt, and its displacement is the chord of that arc. Positions stay under 1024 m (a 50 m
// start plus 15 m/s for 60 s), so the rounding of the later position, half an ulp or 5.7e-14 m,
// bounds the error of the displacement; 1e-12 m is about 17 times that. The side term is
// 2 sin^2(turn / 2) / turn, not (1 - cos(turn)) / turn, which cancels for small turns.
TEST(Generator, TurningSegmentsAlternateWithStraightOnesAndFollowArcs) {
  kite::ScenarioConfig config{21, 6, 60.0, 0.1, 0.5, 1.0, 50.0};
  config.turning = true;
  const kite::GeneratedScenario scenario = kite::generate(config);
  int straight_runs = 0;
  int turning_runs = 0;
  for (std::size_t i = 0; i < 6; ++i) {
    const double speed = std::hypot(scenario.steps[0].truth[i].velocity.x,
                                    scenario.steps[0].truth[i].velocity.y);
    std::vector<int> run_lengths;
    std::vector<double> run_turns;
    for (std::size_t k = 0; k + 1 < scenario.steps.size(); ++k) {
      const kite::GroundTruthState& now = scenario.steps[k].truth[i];
      const kite::GroundTruthState& next = scenario.steps[k + 1].truth[i];
      EXPECT_NEAR(std::hypot(next.velocity.x, next.velocity.y), speed, 1e-12);
      const double turn = turn_between(now.velocity, next.velocity);
      EXPECT_LE(std::abs(turn), 0.3 * config.dt_s + 1e-15);
      const double half_sine = std::sin(turn / 2.0);
      const double along = turn == 0.0 ? 1.0 : std::sin(turn) / turn;
      const double side = turn == 0.0 ? 0.0 : 2.0 * half_sine * half_sine / turn;
      const double ux = now.velocity.x * config.dt_s;
      const double uy = now.velocity.y * config.dt_s;
      EXPECT_NEAR(next.position.x - now.position.x, ux * along - uy * side, 1e-12);
      EXPECT_NEAR(next.position.y - now.position.y, uy * along + ux * side, 1e-12);
      if (!run_turns.empty() && std::abs(turn - run_turns.back()) < 1e-13) {
        ++run_lengths.back();
      } else {
        run_turns.push_back(turn);
        run_lengths.push_back(1);
      }
    }
    for (std::size_t r = 0; r < run_turns.size(); ++r) {
      const bool straight = std::abs(run_turns[r]) < 1e-13;
      EXPECT_EQ(straight, r % 2 == 0) << "vehicle " << i << " run " << r;
      if (straight) {
        ++straight_runs;
      } else {
        ++turning_runs;
      }
      if (r + 1 < run_turns.size()) {
        EXPECT_GE(run_lengths[r], 20) << "vehicle " << i << " run " << r;
        EXPECT_LE(run_lengths[r], 60) << "vehicle " << i << " run " << r;
      }
    }
  }
  EXPECT_GT(straight_runs, 6);
  EXPECT_GT(turning_runs, 6);
}

// Without turning a vehicle keeps its heading, so its speed with sign is its velocity along its
// first heading. Each step's displacement is dt times the mean of the two speeds, as speed runs
// linearly within a step. Positions stay under 8192 m (a 50 m start plus 20 m/s for 300 s), so
// the rounding of the later position, half an ulp or 4.5e-13 m, bounds the displacement's error;
// 1e-11 m is about 22 times that.
TEST(Generator, SpeedChangesKeepTheSpeedWithinZeroToTwenty) {
  kite::ScenarioConfig config{31, 40, 300.0, 0.1, 0.5, 1.0, 50.0};
  config.speed_changes = true;
  const kite::GeneratedScenario scenario = kite::generate(config);
  int steps_at_zero = 0;
  int steps_at_twenty = 0;
  int steps_between = 0;
  for (std::size_t i = 0; i < 40; ++i) {
    const kite::Vec2 first = scenario.steps[0].truth[i].velocity;
    const double first_speed = std::hypot(first.x, first.y);
    const kite::Vec2 heading{first.x / first_speed, first.y / first_speed};
    double previous_speed = first_speed;
    for (std::size_t k = 0; k < scenario.steps.size(); ++k) {
      const kite::GroundTruthState& state = scenario.steps[k].truth[i];
      const double speed = state.velocity.x * heading.x + state.velocity.y * heading.y;
      EXPECT_GE(speed, 0.0);
      EXPECT_LE(speed, 20.0 + 1e-12);
      if (speed == 0.0) {
        ++steps_at_zero;
      } else if (std::abs(speed - 20.0) < 1e-12) {
        ++steps_at_twenty;
      } else {
        ++steps_between;
      }
      if (k > 0) {
        const kite::Vec2 before = scenario.steps[k - 1].truth[i].position;
        const double moved = (state.position.x - before.x) * heading.x +
                             (state.position.y - before.y) * heading.y;
        EXPECT_NEAR(moved, config.dt_s * (previous_speed + speed) / 2.0, 1e-11);
      }
      previous_speed = speed;
    }
  }
  EXPECT_GT(steps_at_zero, 0);
  EXPECT_GT(steps_at_twenty, 0);
  EXPECT_GT(steps_between, steps_at_zero + steps_at_twenty);
}

// 20 seeds of 1000 steps give N = 20000 per-step counts. For a Poisson count with mean L the
// sample mean has standard deviation sqrt(L / N) and the sample variance about
// sqrt((L + 2 L^2) / N). Both must be within 5 of those deviations of L. The seeds are fixed, so
// the test always gives the same answer, yet a count biased by 0.1 or a fixed count of L would
// fail it.
TEST(Generator, ClutterCountsArePoissonWithTheConfiguredMean) {
  for (double mean : {0.5, 2.0}) {
    std::vector<double> counts;
    for (std::uint64_t seed = 301; seed <= 320; ++seed) {
      kite::ScenarioConfig config{seed, 0, 99.9, 0.1, 0.5, 1.0, 50.0};
      config.clutter_per_step = mean;
      for (const kite::ScenarioStep& step : kite::generate(config).steps) {
        counts.push_back(static_cast<double>(step.measurements.size()));
      }
    }
    ASSERT_EQ(counts.size(), 20000u);
    const double n = static_cast<double>(counts.size());
    const Moments m = moments(counts);
    EXPECT_NEAR(m.mean, mean, 5.0 * std::sqrt(mean / n)) << mean;
    EXPECT_NEAR(m.variance, mean, 5.0 * std::sqrt((mean + 2.0 * mean * mean) / n)) << mean;
  }
}

TEST(Generator, ClutterIsSpreadOverTheArea) {
  const double half_width = 50.0;
  kite::ScenarioConfig config{41, 0, 99.9, 0.1, 0.5, 1.0, half_width};
  config.clutter_per_step = 2.0;
  std::vector<double> xs;
  std::vector<double> ys;
  for (const kite::Measurement& m : all_measurements(kite::generate(config))) {
    EXPECT_GE(m.position.x, -half_width);
    EXPECT_LT(m.position.x, half_width);
    EXPECT_GE(m.position.y, -half_width);
    EXPECT_LT(m.position.y, half_width);
    xs.push_back(m.position.x);
    ys.push_back(m.position.y);
  }
  ASSERT_GT(xs.size(), 1000u);
  const double uniform_sd = 2.0 * half_width / std::sqrt(12.0);
  const double n = static_cast<double>(xs.size());
  for (const std::vector<double>* samples : {&xs, &ys}) {
    const Moments m = moments(*samples);
    EXPECT_NEAR(m.mean, 0.0, 5.0 * uniform_sd / std::sqrt(n));
    EXPECT_NEAR(std::sqrt(m.variance), uniform_sd, 0.05 * uniform_sd);
  }
}

TEST(Generator, ClutterIsMixedInWithTheDetections) {
  kite::ScenarioConfig config{51, 4, 20.0, 0.1, 0.01, 1.0, 1000.0};
  config.clutter_per_step = 2.0;
  int clutter_before_a_detection = 0;
  int clutter_after_every_detection = 0;
  for (const kite::ScenarioStep& step : kite::generate(config).steps) {
    std::vector<bool> is_clutter;
    for (const kite::Measurement& m : step.measurements) {
      bool near_truth = false;
      for (const kite::GroundTruthState& truth : step.truth) {
        near_truth = near_truth || std::hypot(m.position.x - truth.position.x,
                                              m.position.y - truth.position.y) < 1.0;
      }
      is_clutter.push_back(!near_truth);
    }
    ASSERT_EQ(std::count(is_clutter.begin(), is_clutter.end(), false), 4);
    const auto last_detection = std::find(is_clutter.rbegin(), is_clutter.rend(), false);
    for (auto it = is_clutter.rbegin(); it != is_clutter.rend(); ++it) {
      if (*it && it < last_detection) {
        ++clutter_after_every_detection;
      } else if (*it) {
        ++clutter_before_a_detection;
      }
    }
  }
  EXPECT_GT(clutter_before_a_detection, 0);
  EXPECT_GT(clutter_after_every_detection, 0);
}

TEST(Generator, RejectsInvalidManoeuvreAndClutterConfigs) {
  std::vector<kite::ScenarioConfig> bad(4, make_config());
  bad[0].clutter_per_step = -0.5;
  bad[1].clutter_per_step = 1000.5;
  bad[2].clutter_per_step = std::numeric_limits<double>::quiet_NaN();
  bad[3].turning = true;
  bad[3].dt_s = 1.5;
  for (const kite::ScenarioConfig& config : bad) {
    EXPECT_THROW(kite::generate(config), std::invalid_argument);
  }
  kite::ScenarioConfig coarse = make_config();
  coarse.dt_s = 1.5;
  coarse.speed_changes = true;
  coarse.clutter_per_step = 1000.0;
  EXPECT_EQ(kite::generate(coarse).steps.size(), 3u);
  coarse.dt_s = 1.0;
  coarse.turning = true;
  EXPECT_EQ(kite::generate(coarse).steps.size(), 4u);
}

TEST_F(GeneratedFiles, ClutterLinesCarryNoIdentity) {
  kite::ScenarioConfig config{61, 0, 3.0, 0.1, 0.5, 1.0, 50.0};
  config.clutter_per_step = 2.0;
  const std::string text = write_and_read_bytes(kite::generate(config));
  std::size_t meas_lines = 0;
  std::size_t start = 0;
  while (start < text.size()) {
    const std::size_t end = text.find('\n', start);
    ASSERT_NE(end, std::string::npos);
    const std::string line = text.substr(start, end - start);
    if (line.rfind("meas,", 0) == 0) {
      ++meas_lines;
      EXPECT_EQ(std::count(line.begin(), line.end(), ','), 5) << line;
      EXPECT_NE(line.find(",0,0,"), std::string::npos) << line;
    }
    start = end + 1;
  }
  EXPECT_GT(meas_lines, 0u);
}

TEST_F(GeneratedFiles, SmallHardScenarioMatchesGoldenFileByteForByte) {
  const std::string generated = write_and_read_bytes(kite::generate(hard_config()));
  const std::string golden = read_bytes(fs::path(KITE_SIM_TEST_DATA_DIR) / "hard_seed7.log");
  ASSERT_FALSE(golden.empty());
  EXPECT_EQ(generated, golden);
}

TEST_F(GeneratedFiles, MeasurementLinesCarryNoActorId) {
  const std::string text = write_and_read_bytes(kite::generate(make_config()));
  std::size_t meas_lines = 0;
  std::size_t start = 0;
  while (start < text.size()) {
    const std::size_t end = text.find('\n', start);
    ASSERT_NE(end, std::string::npos);
    const std::string line = text.substr(start, end - start);
    if (line.rfind("meas,", 0) == 0) {
      ++meas_lines;
      EXPECT_EQ(std::count(line.begin(), line.end(), ','), 5) << line;
    }
    start = end + 1;
  }
  EXPECT_GT(meas_lines, 0u);
}

TEST_F(GeneratedFiles, Seed42MatchesGoldenFileByteForByte) {
  const kite::ScenarioConfig config{42, 3, 2.0, 0.1, 0.5, 0.9, 50.0};
  const std::string generated = write_and_read_bytes(kite::generate(config));

  const std::string golden = read_bytes(fs::path(KITE_SIM_TEST_DATA_DIR) / "seed42.log");

  ASSERT_FALSE(golden.empty());
  EXPECT_EQ(generated.find('\r'), std::string::npos);
  EXPECT_EQ(generated, golden);
}

}
