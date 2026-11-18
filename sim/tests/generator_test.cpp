#include "kite/generator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
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
