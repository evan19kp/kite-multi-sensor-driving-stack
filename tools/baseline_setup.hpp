#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "kite/generator.hpp"
#include "kite/kalman_tracker.hpp"

// The scenarios, tracker parameters and scoring gate that kite_baseline runs, shared with the
// study tools so they run exactly the same setup.
namespace kite::baseline_setup {

inline constexpr int kVehicles = 5;
inline constexpr double kDurationS = 30.0;
inline constexpr double kDtS = 0.1;
inline constexpr double kNoiseM = 1.0;
inline constexpr double kDetectionProbability = 0.95;
inline constexpr double kAreaHalfWidthM = 100.0;
inline constexpr double kBaselineTrackerGateM = 5.0;
// The sensor's noise spec, which the tracker is given rather than reading it from the scenario.
inline constexpr double kKalmanMeasurementSigmaM = 1.0;
inline constexpr double kKalmanQ = 0.01;
inline constexpr double kKalmanVelocitySigma = 10.0;
inline constexpr double kKalmanGateChi2 = 9.21;
inline constexpr int kKalmanConfirmHits = 3;
inline constexpr int kKalmanConfirmWindow = 5;
inline constexpr int kKalmanMaxMisses = 5;
inline constexpr double kMatchGateM = 3.0;

inline constexpr double kHardClutterPerStep = 2.0;

enum class Scenario { straight, hard };

inline ScenarioConfig scenario_config(std::uint64_t seed) {
  return {seed, kVehicles, kDurationS, kDtS, kNoiseM, kDetectionProbability, kAreaHalfWidthM};
}

inline ScenarioConfig scenario_config(Scenario scenario, std::uint64_t seed) {
  ScenarioConfig config = scenario_config(seed);
  if (scenario == Scenario::hard) {
    config.turning = true;
    config.speed_changes = true;
    config.clutter_per_step = kHardClutterPerStep;
  }
  return config;
}

inline KalmanTracker make_kalman_tracker(double q, double velocity_sigma,
                                         AssociationMode association) {
  return KalmanTracker(q, kKalmanMeasurementSigmaM, velocity_sigma, kKalmanGateChi2,
                       kKalmanConfirmHits, kKalmanConfirmWindow, kKalmanMaxMisses, association);
}

inline KalmanTracker make_kalman_tracker(AssociationMode association) {
  return make_kalman_tracker(kKalmanQ, kKalmanVelocitySigma, association);
}

// The tuning grid of R15, run again on the hard scenario.
inline constexpr std::array<double, 4> kTuningQs = {0.01, 0.1, 1.0, 10.0};
inline constexpr std::array<double, 3> kTuningVelocitySigmas = {5.0, 10.0, 20.0};
inline constexpr int kFirstTuningSeed = 101;

inline constexpr std::array<std::pair<Scenario, std::string_view>, 2> kScenarioNames = {{
    {Scenario::straight, "straight"},
    {Scenario::hard, "hard"},
}};

inline std::optional<Scenario> parse_scenario(std::string_view name) {
  for (const auto& [scenario, scenario_name] : kScenarioNames) {
    if (name == scenario_name) {
      return scenario;
    }
  }
  return std::nullopt;
}

inline std::string_view scenario_name(Scenario scenario) {
  for (const auto& [candidate, candidate_name] : kScenarioNames) {
    if (candidate == scenario) {
      return candidate_name;
    }
  }
  throw std::logic_error("unknown scenario");
}

inline constexpr std::array<std::pair<AssociationMode, std::string_view>, 3> kAssociationNames = {{
    {AssociationMode::two_stage, "two_stage"},
    {AssociationMode::joint, "joint"},
    {AssociationMode::joint_likelihood, "joint_likelihood"},
}};

inline std::optional<AssociationMode> parse_association(std::string_view name) {
  for (const auto& [mode, mode_name] : kAssociationNames) {
    if (name == mode_name) {
      return mode;
    }
  }
  return std::nullopt;
}

inline std::string_view association_name(AssociationMode association) {
  for (const auto& [mode, mode_name] : kAssociationNames) {
    if (mode == association) {
      return mode_name;
    }
  }
  throw std::logic_error("unknown association mode");
}

}
