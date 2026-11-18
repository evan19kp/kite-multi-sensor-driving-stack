#pragma once

#include <cstdint>

#include "kite/generator.hpp"
#include "kite/kalman_tracker.hpp"

// The scenario, tracker parameters and scoring gate that kite_baseline reports, shared with the
// study tools so they run exactly the same setup.
namespace kite::baseline_setup {

inline constexpr int kVehicles = 5;
inline constexpr double kDurationS = 30.0;
inline constexpr double kDtS = 0.1;
inline constexpr double kNoiseM = 1.0;
inline constexpr double kDetectionProbability = 0.95;
inline constexpr double kAreaHalfWidthM = 100.0;
// The sensor's noise spec, which the tracker is given rather than reading it from the scenario.
inline constexpr double kKalmanMeasurementSigmaM = 1.0;
inline constexpr double kKalmanQ = 0.01;
inline constexpr double kKalmanVelocitySigma = 10.0;
inline constexpr double kKalmanGateChi2 = 9.21;
inline constexpr int kKalmanConfirmHits = 3;
inline constexpr int kKalmanConfirmWindow = 5;
inline constexpr int kKalmanMaxMisses = 5;
inline constexpr double kMatchGateM = 3.0;

inline ScenarioConfig scenario_config(std::uint64_t seed) {
  return {seed, kVehicles, kDurationS, kDtS, kNoiseM, kDetectionProbability, kAreaHalfWidthM};
}

inline KalmanTracker make_kalman_tracker(AssociationMode association) {
  return KalmanTracker(kKalmanQ, kKalmanMeasurementSigmaM, kKalmanVelocitySigma, kKalmanGateChi2,
                       kKalmanConfirmHits, kKalmanConfirmWindow, kKalmanMaxMisses, association);
}

}
