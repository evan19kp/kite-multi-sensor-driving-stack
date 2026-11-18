#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "baseline_setup.hpp"
#include "drop_study.hpp"
#include "kite/kalman_tracker.hpp"
#include "kite/types.hpp"

namespace {

using kite::drop_study::Fate;
using kite::drop_study::TrackFollower;

TEST(BinomialCoefficient, MatchesPascalsTriangle) {
  const unsigned long long row4[] = {1, 4, 6, 4, 1};
  for (int k = 0; k <= 4; ++k) {
    EXPECT_EQ(kite::drop_study::binomial_coefficient(4, k), row4[k]) << "k=" << k;
  }
  EXPECT_EQ(kite::drop_study::binomial_coefficient(0, 0), 1u);
  EXPECT_EQ(kite::drop_study::binomial_coefficient(52, 5), 2598960u);
  EXPECT_EQ(kite::drop_study::binomial_coefficient(3, 4), 0u);
}

TEST(BinomialCoefficient, RejectsNegativeArguments) {
  EXPECT_THROW(kite::drop_study::binomial_coefficient(-1, 0), std::invalid_argument);
  EXPECT_THROW(kite::drop_study::binomial_coefficient(4, -1), std::invalid_argument);
}

TEST(DropProbability, ForTheTrackerParametersAtDetectionProbability95MatchesTheHandFormula) {
  // M = 3 hits in N = 5 steps: dropped when at most 1 hit falls in the 4 steps after the birth.
  // With p = 0.95 and q = 0.05: P = q^4 + 4 p q^3 = 6.25e-6 + 4 * 0.95 * 1.25e-4
  //                                = 6.25e-6 + 4.75e-4 = 4.8125e-4.
  const double by_hand = 0.05 * 0.05 * 0.05 * 0.05 + 4.0 * 0.95 * 0.05 * 0.05 * 0.05;
  EXPECT_NEAR(kite::drop_study::drop_probability(0.95, kite::baseline_setup::kKalmanConfirmHits,
                                                 kite::baseline_setup::kKalmanConfirmWindow),
              by_hand, 1e-15);
  EXPECT_NEAR(by_hand, 4.8125e-4, 1e-15);
}

TEST(DropProbability, FollowsTheConfirmationParameters) {
  // M = 3, N = 3: both later steps must hit, so P = 1 - p^2 = 1 - 0.25 = 0.75 at p = 0.5.
  EXPECT_NEAR(kite::drop_study::drop_probability(0.5, 3, 3), 0.75, 1e-15);
  // M = 1: the birth alone confirms, so a track is never dropped.
  EXPECT_EQ(kite::drop_study::drop_probability(0.5, 1, 5), 0.0);
  // A car that is always detected is never dropped; one that is never detected always is.
  EXPECT_EQ(kite::drop_study::drop_probability(1.0, 3, 5), 0.0);
  EXPECT_EQ(kite::drop_study::drop_probability(0.0, 3, 5), 1.0);
}

TEST(DropProbability, RejectsInvalidArguments) {
  EXPECT_THROW(kite::drop_study::drop_probability(1.5, 3, 5), std::invalid_argument);
  EXPECT_THROW(kite::drop_study::drop_probability(0.5, 0, 5), std::invalid_argument);
  EXPECT_THROW(kite::drop_study::drop_probability(0.5, 4, 3), std::invalid_argument);
}

TEST(WilsonInterval, HalfOfOneHundredMatchesTheHandComputedInterval) {
  // x = 50, n = 100, z = 1.959963984540054, z2 = 3.841458820694124.
  // phat = 0.5, denominator = 1 + z2 / 100 = 1.03841458820694.
  // center = (0.5 + z2 / 200) / denominator = 0.51920729410347 / 1.03841458820694 = 0.5.
  // half = z * sqrt(0.25 / 100 + z2 / 40000) / denominator
  //      = 1.959963984540054 * sqrt(0.0025 + 0.0000960364705) / 1.03841458820694
  //      = 1.959963984540054 * 0.05095132 / 1.03841458820694 = 0.09986274 / 1.03841459
  //      = 0.0961685.
  // The interval is 0.5 -+ 0.0961685 = [0.4038315, 0.5961685].
  const kite::drop_study::Interval interval = kite::drop_study::wilson_interval(50, 100);
  EXPECT_NEAR(interval.low, 0.4038315, 1e-6);
  EXPECT_NEAR(interval.high, 0.5961685, 1e-6);
}

TEST(WilsonInterval, ZeroSuccessesStartAtZeroAndEndAtZ2OverNPlusZ2) {
  // x = 0, n = 10: phat = 0, so the lower end is (z2 / 20 - z * sqrt(z2 / 400)) / denominator =
  // (z2 / 20 - z2 / 20) / denominator = 0. The upper end is (z2 / 10 + z2 / 10) / (1 + z2 / 10)
  // = z2 / (10 + z2) = 3.841458820694124 / 13.841458820694124 = 0.2775328.
  const kite::drop_study::Interval interval = kite::drop_study::wilson_interval(0, 10);
  EXPECT_EQ(interval.low, 0.0);
  EXPECT_NEAR(interval.high, 0.2775328, 1e-6);
}

TEST(WilsonInterval, AllSuccessesMirrorZeroSuccesses) {
  // The interval for 10 of 10 is the one for 0 of 10 reflected about 0.5: 1 - 0.2775328.
  const kite::drop_study::Interval interval = kite::drop_study::wilson_interval(10, 10);
  EXPECT_NEAR(interval.low, 0.7224672, 1e-6);
  EXPECT_NEAR(interval.high, 1.0, 1e-12);
  EXPECT_LE(interval.high, 1.0);
}

TEST(WilsonInterval, RejectsImpossibleCounts) {
  EXPECT_THROW(kite::drop_study::wilson_interval(1, 0), std::invalid_argument);
  EXPECT_THROW(kite::drop_study::wilson_interval(-1, 5), std::invalid_argument);
  EXPECT_THROW(kite::drop_study::wilson_interval(6, 5), std::invalid_argument);
}

using Step = std::vector<kite::Measurement>;

kite::Measurement z(double x, double y) { return {0.0, 0, 0, {x, y}}; }

TrackFollower follow(int track_id, const std::vector<Step>& steps) {
  kite::KalmanTracker tracker =
      kite::baseline_setup::make_kalman_tracker(kite::AssociationMode::two_stage);
  tracker.record_steps(true);
  TrackFollower follower(track_id, kite::baseline_setup::kKalmanGateChi2);
  for (std::size_t k = 0; k < steps.size(); ++k) {
    const std::vector<kite::TrackState> output =
        tracker.update(0.1 * static_cast<double>(k), steps[k]);
    if (k > 0) {
      follower.observe(tracker.step_record().value(), steps[k], output);
    }
  }
  return follower;
}

TEST(TrackFollower, ATrackWithTwoMoreHitsIsConfirmed) {
  const TrackFollower follower = follow(0, {{z(0, 0)}, {z(0.5, 0)}, {z(1.0, 0)}});
  EXPECT_EQ(follower.fate(), Fate::confirmed);
  EXPECT_EQ(follower.misses().no_measurement, 0);
  EXPECT_EQ(follower.misses().outside_gate, 0);
  EXPECT_EQ(follower.misses().taken_by_other, 0);
}

TEST(TrackFollower, ATrackStaysPendingWhileItCanStillBeConfirmed) {
  const TrackFollower follower = follow(0, {{z(0, 0)}, {z(0.5, 0)}, {}});
  EXPECT_EQ(follower.fate(), Fate::pending);
  EXPECT_EQ(follower.misses().no_measurement, 1);
}

TEST(TrackFollower, ThreeEmptyStepsDropATrackAndCountAsNoMeasurement) {
  const TrackFollower follower = follow(0, {{z(0, 0)}, {}, {}, {}, {}});
  EXPECT_EQ(follower.fate(), Fate::dropped);
  EXPECT_EQ(follower.misses().no_measurement, 3);
  EXPECT_EQ(follower.misses().outside_gate, 0);
  EXPECT_EQ(follower.misses().taken_by_other, 0);
}

TEST(TrackFollower, AMeasurementFarFromTheTrackIsAnOutsideGateMiss) {
  // Fifty metres from the track is far outside any gate; that measurement starts track 1.
  const TrackFollower follower = follow(0, {{z(0, 0)}, {z(50, 0)}, {}, {}, {}});
  EXPECT_EQ(follower.fate(), Fate::dropped);
  EXPECT_EQ(follower.misses().no_measurement, 2);
  EXPECT_EQ(follower.misses().outside_gate, 1);
  EXPECT_EQ(follower.misses().taken_by_other, 0);
}

TEST(TrackFollower, AMeasurementInsideTheGateThatAnotherTrackTookIsNotAGatingLoss) {
  // Tracks 0 and 1 start 0.5 m apart with the same covariance. The one measurement at x = 0.2 is
  // inside both gates and nearer track 0, so track 1 misses it.
  const TrackFollower follower = follow(1, {{z(0, 0), z(0.5, 0)}, {z(0.2, 0)}});
  EXPECT_EQ(follower.fate(), Fate::pending);
  EXPECT_EQ(follower.misses().no_measurement, 0);
  EXPECT_EQ(follower.misses().outside_gate, 0);
  EXPECT_EQ(follower.misses().taken_by_other, 1);
}

}
