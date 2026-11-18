#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "baseline_setup.hpp"
#include "kite/baseline_tracker.hpp"
#include "kite/generator.hpp"
#include "kite/kalman_tracker.hpp"
#include "test_helpers.hpp"

namespace kite {
namespace {

using test_helpers::kInf;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

KalmanTracker make_tracker() { return KalmanTracker{0.01, 1.0, 2.0}; }

Measurement at(double t, double x, double y, int ego_id = 0) {
  return Measurement{t, ego_id, 0, {x, y}};
}

double distance(Vec2 a, Vec2 b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

// A stationary target measured exactly: every innovation is zero, so the filter state stays at
// the first measurement with zero velocity, bit for bit.
TEST(KalmanTracker, SingleTargetIsHiddenUntilItsThirdHit) {
  KalmanTracker tracker = make_tracker();

  const auto first = tracker.update(0.0, {at(0.0, 10.0, -3.0, 4)});
  const auto second = tracker.update(1.0, {at(1.0, 10.0, -3.0, 4)});
  const auto third = tracker.update(2.0, {at(2.0, 10.0, -3.0, 4)});

  EXPECT_TRUE(first.empty());
  EXPECT_TRUE(second.empty());
  ASSERT_EQ(third.size(), 1u);
  EXPECT_EQ(third[0], (TrackState{2.0, 4, 0, {10.0, -3.0}, {0.0, 0.0}}));
}

TEST(KalmanTracker, HitsNeedNotBeConsecutiveToConfirm) {
  KalmanTracker tracker = make_tracker();

  tracker.update(0.0, {at(0.0, 0.0, 0.0)});
  tracker.update(1.0, {});
  tracker.update(2.0, {at(2.0, 0.0, 0.0)});
  const auto before = tracker.update(3.0, {});
  const auto confirmed = tracker.update(4.0, {at(4.0, 0.0, 0.0)});

  EXPECT_TRUE(before.empty());
  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(confirmed[0].track_id, 0);
}

TEST(KalmanTracker, AThirdHitAfterTheFirstFiveStepsDoesNotConfirm) {
  KalmanTracker tracker = make_tracker();

  tracker.update(0.0, {at(0.0, 0.0, 0.0)});
  tracker.update(1.0, {});
  tracker.update(2.0, {});
  tracker.update(3.0, {at(3.0, 0.0, 0.0)});
  tracker.update(4.0, {});
  const auto step5 = tracker.update(5.0, {at(5.0, 0.0, 0.0)});
  const auto step6 = tracker.update(6.0, {at(6.0, 0.0, 0.0)});
  const auto step7 = tracker.update(7.0, {at(7.0, 0.0, 0.0)});

  EXPECT_TRUE(step5.empty());
  EXPECT_TRUE(step6.empty());
  ASSERT_EQ(step7.size(), 1u);
  EXPECT_EQ(step7[0].track_id, 1);
}

TEST(KalmanTracker, TentativeTrackThatCannotConfirmIsDroppedAndItsIdIsNeverReused) {
  KalmanTracker tracker = make_tracker();
  tracker.update(0.0, {at(0.0, 2.0, 1.0)});
  for (int k = 1; k <= 3; ++k) {
    tracker.update(k, {});
  }

  const auto first = tracker.update(4.0, {at(4.0, 2.0, 1.0)});
  const auto second = tracker.update(5.0, {at(5.0, 2.0, 1.0)});
  const auto third = tracker.update(6.0, {at(6.0, 2.0, 1.0)});

  EXPECT_TRUE(first.empty());
  EXPECT_TRUE(second.empty());
  ASSERT_EQ(third.size(), 1u);
  EXPECT_EQ(third[0], (TrackState{6.0, 0, 1, {2.0, 1.0}, {0.0, 0.0}}));
}

TEST(KalmanTracker, TargetHitEveryFourthStepNeverConfirmsAndItsTracksDoNotSurvive) {
  KalmanTracker tracker = make_tracker();

  for (int k = 0; k <= 13; ++k) {
    std::vector<Measurement> z;
    if (k % 4 == 0 || k == 13) {
      z.push_back(at(k, -4.0, 6.0));
    }
    EXPECT_TRUE(tracker.update(k, z).empty()) << "step " << k;
  }
  const auto confirmed = tracker.update(14.0, {at(14.0, -4.0, 6.0)});

  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(confirmed[0].track_id, 3);
}

// A target at 10 m/s along x, measured exactly every 0.1 s for 2 s, then not measured for 4 steps
// (one fewer than the 5 misses that delete a track). The initial velocity sigma of 20 m/s keeps
// the zero-velocity prior from biasing the estimate.
TEST(KalmanTracker, TargetMissedForFewerThanKStepsCoastsAndKeepsItsId) {
  KalmanTracker tracker{0.01, 1.0, 20.0};
  for (int k = 0; k <= 20; ++k) {
    tracker.update(0.1 * k, {at(0.1 * k, 1.0 * k, 5.0)});
  }

  std::vector<double> coasting_x;
  for (int k = 21; k <= 24; ++k) {
    const auto tracks = tracker.update(0.1 * k, {});
    ASSERT_EQ(tracks.size(), 1u) << "step " << k;
    EXPECT_EQ(tracks[0].track_id, 0);
    EXPECT_NEAR(tracks[0].position.x, 1.0 * k, 0.1) << "step " << k;
    EXPECT_NEAR(tracks[0].velocity.x, 10.0, 0.5);
    coasting_x.push_back(tracks[0].position.x);
  }
  const auto back = tracker.update(2.5, {at(2.5, 25.0, 5.0)});

  ASSERT_EQ(back.size(), 1u);
  EXPECT_EQ(back[0].track_id, 0);
  EXPECT_LT(coasting_x[0], coasting_x[1]);
  EXPECT_LT(coasting_x[2], coasting_x[3]);
}

TEST(KalmanTracker, TargetMissedForKStepsIsDeletedAndItsIdIsNeverReused) {
  KalmanTracker tracker = make_tracker();
  for (int k = 0; k < 3; ++k) {
    tracker.update(k, {at(k, 0.0, 0.0)});
  }
  for (int k = 3; k < 7; ++k) {
    ASSERT_EQ(tracker.update(k, {}).size(), 1u) << "step " << k;
  }

  const auto fifth_miss = tracker.update(7.0, {});
  const auto new_first = tracker.update(8.0, {at(8.0, 0.0, 0.0)});
  const auto new_second = tracker.update(9.0, {at(9.0, 0.0, 0.0)});
  const auto new_third = tracker.update(10.0, {at(10.0, 0.0, 0.0)});

  EXPECT_TRUE(fifth_miss.empty());
  EXPECT_TRUE(new_first.empty());
  EXPECT_TRUE(new_second.empty());
  ASSERT_EQ(new_third.size(), 1u);
  EXPECT_EQ(new_third[0].track_id, 1);
}

TEST(KalmanTracker, EmptyStepsGiveEmptyOutputUntilATrackIsConfirmed) {
  KalmanTracker tracker = make_tracker();

  EXPECT_TRUE(tracker.update(0.0, {}).empty());
  EXPECT_TRUE(tracker.update(1.0, {}).empty());
  EXPECT_TRUE(tracker.update(2.0, {at(2.0, 1.0, 1.0)}).empty());
  EXPECT_TRUE(tracker.update(3.0, {at(3.0, 1.0, 1.0)}).empty());
  const auto confirmed = tracker.update(4.0, {at(4.0, 1.0, 1.0)});

  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(confirmed[0], (TrackState{4.0, 0, 0, {1.0, 1.0}, {0.0, 0.0}}));
}

TEST(KalmanTracker, EmptyStepsCoastAConfirmedTrackUntilItIsDeleted) {
  KalmanTracker tracker = make_tracker();
  for (int k = 0; k < 3; ++k) {
    tracker.update(k, {at(k, -2.0, 7.0)});
  }

  for (int k = 3; k < 7; ++k) {
    const auto tracks = tracker.update(k, {});
    ASSERT_EQ(tracks.size(), 1u) << "step " << k;
    EXPECT_EQ(tracks[0], (TrackState{static_cast<double>(k), 0, 0, {-2.0, 7.0}, {0.0, 0.0}}));
  }
  EXPECT_TRUE(tracker.update(7.0, {}).empty());
  EXPECT_TRUE(tracker.update(8.0, {}).empty());
}

// Steps 3-6 and 8-11 are misses: 8 in total, but never the 5 in a row that delete a track.
TEST(KalmanTracker, AHitResetsAConfirmedTracksMissCount) {
  KalmanTracker tracker = make_tracker();
  for (int k = 0; k < 3; ++k) {
    tracker.update(k, {at(k, 0.0, 0.0)});
  }

  for (int k = 3; k <= 11; ++k) {
    std::vector<Measurement> z;
    if (k == 7) {
      z.push_back(at(k, 0.0, 0.0));
    }
    const auto tracks = tracker.update(k, z);
    ASSERT_EQ(tracks.size(), 1u) << "step " << k;
    EXPECT_EQ(tracks[0].track_id, 0) << "step " << k;
  }
  const auto fifth_miss_in_a_row = tracker.update(12.0, {});

  EXPECT_TRUE(fifth_miss_in_a_row.empty());
}

// Target A moves +1 m/s along y = 0 and target B -1 m/s along y = 0.2, measured exactly once a
// second. They pass between steps 5 and 6: at step 5 A is at x = -0.5 and B at x = 0.5; at step 6
// A is at 0.5 and B at -0.5. Each new measurement is then 0.2 m from the other target's previous
// measurement and 1.0 m from its own.
std::vector<Measurement> crossing_step(int k) {
  const double t = k;
  return {at(t, -5.5 + k, 0.0), at(t, 5.5 - k, 0.2)};
}

TEST(KalmanTracker, CrossingTargetsKeepTheirIdsWhereNearestNeighbourSwapsThem) {
  KalmanTracker kalman = make_tracker();
  NearestNeighbourBaseline baseline{5.0};

  for (int k = 0; k <= 11; ++k) {
    const std::vector<Measurement> z = crossing_step(k);
    const Vec2 a = z[0].position;
    const Vec2 b = z[1].position;
    const auto kalman_tracks = kalman.update(k, z);
    const auto baseline_tracks = baseline.update(k, z);

    ASSERT_EQ(baseline_tracks.size(), 2u);
    const int baseline_id_of_a = baseline_tracks[0].position == a ? 0 : 1;
    EXPECT_EQ(baseline_id_of_a, k < 6 ? 0 : 1) << "step " << k;

    if (k < 2) {
      EXPECT_TRUE(kalman_tracks.empty());
      continue;
    }
    ASSERT_EQ(kalman_tracks.size(), 2u) << "step " << k;
    EXPECT_EQ(kalman_tracks[0].track_id, 0);
    EXPECT_EQ(kalman_tracks[1].track_id, 1);
    EXPECT_LT(distance(kalman_tracks[0].position, a), distance(kalman_tracks[0].position, b))
        << "step " << k;
    EXPECT_LT(distance(kalman_tracks[1].position, b), distance(kalman_tracks[1].position, a))
        << "step " << k;
  }
}

// Two stationary tracks at x = 0 and x = 2 with identical histories, so identical covariances.
// Then measurements arrive at x = 1.2 and x = 4. The measurement at 1.2 is nearer track 1 (0.8 m)
// than track 0 (1.2 m), but track 0 cannot reach x = 4 within the gate. Only the pairing
// (0, 1.2), (1, 4) updates both tracks; a greedy nearest-first match would give 1.2 to track 1.
TEST(KalmanTracker, AssociationMaximisesPairsOverAllTracks) {
  KalmanTracker tracker = make_tracker();
  for (int k = 0; k < 10; ++k) {
    tracker.update(k, {at(k, 0.0, 0.0), at(k, 2.0, 0.0)});
  }

  const auto tracks = tracker.update(10.0, {at(10.0, 1.2, 0.0), at(10.0, 4.0, 0.0)});

  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_GT(tracks[0].position.x, 0.0);
  EXPECT_LT(tracks[0].position.x, 1.2);
  EXPECT_EQ(tracks[1].track_id, 1);
  EXPECT_GT(tracks[1].position.x, 2.0);
  EXPECT_LT(tracks[1].position.x, 4.0);
}

// The measurement at x = 4 falls outside the confirmed track's gate and starts a tentative track.
// Its wide covariance puts x = 1.5 nearer to it in Mahalanobis terms than to the confirmed track.
TEST(KalmanTracker, ConfirmedTrackKeepsItsTargetsMeasurementsFromANearbyTentativeTrack) {
  KalmanTracker tracker = make_tracker();
  for (int k = 0; k < 20; ++k) {
    tracker.update(k, {at(k, 0.0, 0.0)});
  }

  const auto outlier = tracker.update(20.0, {at(20.0, 4.0, 0.0)});
  ASSERT_EQ(outlier.size(), 1u);
  EXPECT_EQ(outlier[0].position, (Vec2{0.0, 0.0}));
  for (int k = 21; k < 26; ++k) {
    const auto tracks = tracker.update(k, {at(k, 1.5, 0.0)});
    ASSERT_EQ(tracks.size(), 1u) << "step " << k;
    EXPECT_EQ(tracks[0].track_id, 0);
    EXPECT_GT(tracks[0].position.x, 0.0) << "step " << k;
  }
}

// One-off measurements 100 m apart, each far from the target and from every earlier one, start
// tentative tracks that never gather a second hit.
TEST(KalmanTracker, FarClutterNeverConfirms) {
  KalmanTracker tracker = make_tracker();

  for (int k = 0; k < 15; ++k) {
    const auto tracks =
        tracker.update(k, {at(k, 100.0 + 100.0 * k, 100.0), at(k, 0.0, 0.0)});
    if (k < 2) {
      EXPECT_TRUE(tracks.empty()) << "step " << k;
      continue;
    }
    ASSERT_EQ(tracks.size(), 1u) << "step " << k;
    EXPECT_EQ(tracks[0].track_id, 1);
    EXPECT_EQ(tracks[0].position, (Vec2{0.0, 0.0}));
  }
}

TEST(KalmanTracker, OneHitConfirmsWhenConfirmHitsIsOne) {
  KalmanTracker tracker{0.01, 1.0, 2.0, 9.21, 1, 1, 1};

  const auto first = tracker.update(0.0, {at(0.0, 3.0, 4.0)});
  const auto missed = tracker.update(1.0, {});

  ASSERT_EQ(first.size(), 1u);
  EXPECT_EQ(first[0], (TrackState{0.0, 0, 0, {3.0, 4.0}, {0.0, 0.0}}));
  EXPECT_TRUE(missed.empty());
}

TEST(KalmanTracker, InvalidParametersThrow) {
  EXPECT_THROW((KalmanTracker{-0.1, 1.0, 2.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{kNan, 1.0, 2.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{kInf, 1.0, 2.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 0.0, 2.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, -1.0, 2.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, kNan, 2.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, kInf, 2.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, -1.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, kNan}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, kInf}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, 1e200}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, 2.0, 0.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, 2.0, -1.0}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, 2.0, kNan}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, 2.0, kInf}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, 2.0, 9.21, 0, 5, 5}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, 2.0, 9.21, 3, 2, 5}), std::invalid_argument);
  EXPECT_THROW((KalmanTracker{0.01, 1.0, 2.0, 9.21, 3, 5, 0}), std::invalid_argument);
}

TEST(KalmanTracker, ZeroQAndZeroInitialVelocitySigmaAreValid) {
  KalmanTracker tracker{0.0, 1.0, 0.0};

  for (int k = 0; k < 2; ++k) {
    tracker.update(k, {at(k, 0.0, 0.0)});
  }

  EXPECT_EQ(tracker.update(2.0, {at(2.0, 0.0, 0.0)}).size(), 1u);
}

// After one hit, a rejected update with a measurement must not count as a second hit: the next
// valid update is the second hit (no output) and the one after it the third.
TEST(KalmanTracker, UpdateTimeThatDoesNotIncreaseThrowsAndLeavesStateUnchanged) {
  KalmanTracker tracker = make_tracker();
  tracker.update(1.0, {at(1.0, 0.0, 0.0)});

  EXPECT_THROW(tracker.update(1.0, {at(1.0, 0.0, 0.0)}), std::invalid_argument);
  EXPECT_THROW(tracker.update(0.5, {at(0.5, 0.0, 0.0)}), std::invalid_argument);

  EXPECT_TRUE(tracker.update(2.0, {at(2.0, 0.0, 0.0)}).empty());
  EXPECT_EQ(tracker.update(3.0, {at(3.0, 0.0, 0.0)}).size(), 1u);
}

TEST(KalmanTracker, NonFiniteUpdateTimeThrowsAndLeavesStateUnchanged) {
  KalmanTracker tracker = make_tracker();

  EXPECT_THROW(tracker.update(kNan, {at(0.0, 0.0, 0.0)}), std::invalid_argument);
  EXPECT_THROW(tracker.update(kInf, {at(0.0, 0.0, 0.0)}), std::invalid_argument);
  tracker.update(0.0, {at(0.0, 0.0, 0.0)});
  EXPECT_THROW(tracker.update(kNan, {at(0.0, 0.0, 0.0)}), std::invalid_argument);
  EXPECT_THROW(tracker.update(kInf, {at(0.0, 0.0, 0.0)}), std::invalid_argument);

  EXPECT_TRUE(tracker.update(1.0, {at(1.0, 0.0, 0.0)}).empty());
  EXPECT_EQ(tracker.update(2.0, {at(2.0, 0.0, 0.0)}).size(), 1u);
}

TEST(KalmanTracker, NonFiniteMeasurementThrowsAndLeavesStateUnchanged) {
  KalmanTracker tracker = make_tracker();
  tracker.update(0.0, {at(0.0, 0.0, 0.0)});

  EXPECT_THROW(tracker.update(1.0, {at(1.0, 0.0, 0.0), at(1.0, kNan, 0.0)}),
               std::invalid_argument);
  EXPECT_THROW(tracker.update(1.0, {at(1.0, 0.0, 0.0), at(1.0, 0.0, kInf)}),
               std::invalid_argument);

  EXPECT_TRUE(tracker.update(1.0, {at(1.0, 0.0, 0.0)}).empty());
  EXPECT_EQ(tracker.update(2.0, {at(2.0, 0.0, 0.0)}).size(), 1u);
}

TEST(KalmanTracker, EmptyFirstUpdateStillStartsTheClock) {
  KalmanTracker tracker = make_tracker();
  tracker.update(1.0, {});

  EXPECT_THROW(tracker.update(0.5, {at(0.5, 0.0, 0.0)}), std::invalid_argument);
}

TEST(KalmanTracker, StepRecordIsOffByDefaultAndClearedWhenTurnedOff) {
  KalmanTracker tracker = make_tracker();
  tracker.update(0.0, {at(0.0, 0.0, 0.0)});
  EXPECT_FALSE(tracker.step_record().has_value());

  tracker.record_steps(true);
  tracker.update(1.0, {at(1.0, 0.0, 0.0)});
  EXPECT_TRUE(tracker.step_record().has_value());

  tracker.record_steps(false);
  EXPECT_FALSE(tracker.step_record().has_value());
  tracker.update(2.0, {at(2.0, 0.0, 0.0)});
  EXPECT_FALSE(tracker.step_record().has_value());
}

// q = 0, measurement sigma 1 (R = I), initial velocity sigma 2, confirmation at 2 hits in 2 steps.
// Per axis, a new track has P = [[1, 0], [0, 4]] over (position, velocity).
//
// t = 0: (0, 0) starts track 0.
// t = 1: track 0 predicts to P = [[5, 4], [4, 4]], S = 5 + 1 = 6. (0, 0) scores 0 and (6.3, 0)
//        scores 6.3^2 / 6 = 6.615, both inside the 9.21 gate; track 0 takes (0, 0) (least cost),
//        reaches 2 hits and is confirmed. (6.3, 0) starts tentative track 1. Track 0's update has
//        zero innovation, so it stays at (0, 0) with zero velocity, and K = (5/6, 4/6) leaves
//        P = [[5 - 25/6, 4 - 20/6], [., 4 - 16/6]] = [[5/6, 4/6], [4/6, 8/6]].
// t = 2: one measurement at (3, 0).
//        Track 0 (confirmed): P_pos = 5/6 + 2 * 4/6 + 8/6 = 21/6 = 3.5, S = 4.5 I.
//          It is 3 m away and scores 3^2 / 4.5 = 2.000.
//        Track 1 (tentative): P_pos = 1 + 4 = 5, S = 6 I.
//          It is 3.3 m away and scores 3.3^2 / 6 = 10.89 / 6 = 1.815.
//        Track 0 is physically nearer, but track 1 scores lower because its S is larger.
//        Joint: track 1 takes it and moves 5/6 of -3.3 m to 3.55. Two stages: track 0 takes it
//        and moves 3.5/4.5 of 3 m to 7/3, and track 1 misses and is dropped.
std::vector<Measurement> steal_step(int k) {
  if (k == 0) {
    return {at(0.0, 0.0, 0.0)};
  }
  if (k == 1) {
    return {at(1.0, 0.0, 0.0), at(1.0, 6.3, 0.0)};
  }
  return {at(2.0, 3.0, 0.0)};
}

KalmanTracker steal_tracker(AssociationMode association) {
  return KalmanTracker{0.0, 1.0, 2.0, 9.21, 2, 2, 5, association};
}

TEST(KalmanTracker, JointAssociationLetsAWideTentativeTrackStealFromANearerConfirmedTrack) {
  KalmanTracker tracker = steal_tracker(AssociationMode::joint);
  tracker.record_steps(true);
  tracker.update(0.0, steal_step(0));
  tracker.update(1.0, steal_step(1));

  const auto tracks = tracker.update(2.0, steal_step(2));

  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0], (TrackState{2.0, 0, 0, {0.0, 0.0}, {0.0, 0.0}}));
  EXPECT_EQ(tracks[1].track_id, 1);
  EXPECT_NEAR(tracks[1].position.x, 3.55, 1e-12);
  const StepRecord& record = tracker.step_record().value();
  ASSERT_EQ(record.tracks.size(), 2u);
  EXPECT_EQ(record.tracks[0].stage, TrackStage::confirmed);
  EXPECT_FALSE(record.tracks[0].measurement.has_value());
  EXPECT_NEAR(record.tracks[0].innovation_covariance(0, 0), 4.5, 1e-12);
  EXPECT_EQ(record.tracks[1].stage, TrackStage::tentative);
  EXPECT_EQ(record.tracks[1].measurement, std::optional<std::size_t>(0));
  EXPECT_EQ(record.tracks[1].innovation_covariance(0, 0), 6.0);
}

TEST(KalmanTracker, TwoStageAssociationGivesTheSameMeasurementToTheConfirmedTrack) {
  KalmanTracker tracker = steal_tracker(AssociationMode::two_stage);
  tracker.update(0.0, steal_step(0));
  tracker.update(1.0, steal_step(1));

  const auto tracks = tracker.update(2.0, steal_step(2));

  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_NEAR(tracks[0].position.x, 7.0 / 3.0, 1e-12);
}

std::vector<TrackState> steal_outcome(AssociationMode association, double x) {
  KalmanTracker tracker = steal_tracker(association);
  tracker.update(0.0, steal_step(0));
  tracker.update(1.0, steal_step(1));
  return tracker.update(2.0, {at(2.0, x, 0.0)});
}

// joint_likelihood adds ln(det S / det R) with R = I: ln(4.5^2) = ln 20.25 = 3.0082 for track 0
// and ln(6^2) = ln 36 = 3.5835 for track 1.
// At (3, 0): track 0 scores 9 / 4.5 = 2.0000 and costs 2.0000 + 3.0082 = 5.0082; track 1 scores
// 10.89 / 6 = 1.8150 and costs 1.8150 + 3.5835 = 5.3985. joint takes the lower score (track 1);
// joint_likelihood takes the lower cost (track 0), as two_stage does.
TEST(KalmanTracker, LikelihoodCostKeepsAMeasurementThatJointGivesToAWideTentativeTrack) {
  for (const AssociationMode mode :
       {AssociationMode::two_stage, AssociationMode::joint_likelihood}) {
    const auto tracks = steal_outcome(mode, 3.0);
    ASSERT_EQ(tracks.size(), 1u);
    EXPECT_EQ(tracks[0].track_id, 0);
    EXPECT_NEAR(tracks[0].position.x, 7.0 / 3.0, 1e-12);
  }

  const auto joint = steal_outcome(AssociationMode::joint, 3.0);

  ASSERT_EQ(joint.size(), 2u);
  EXPECT_EQ(joint[0], (TrackState{2.0, 0, 0, {0.0, 0.0}, {0.0, 0.0}}));
  EXPECT_EQ(joint[1].track_id, 1);
  EXPECT_NEAR(joint[1].position.x, 3.55, 1e-12);
}

// At (3.5, 0): track 0 scores 12.25 / 4.5 = 2.7222 and costs 2.7222 + 3.0082 = 5.7304; track 1
// scores 2.8^2 / 6 = 7.84 / 6 = 1.3067 and costs 1.3067 + 3.5835 = 4.8902. Both joint modes give
// it to track 1, which moves 5/6 of -2.8 m to 6.3 - 7/3 = 3.9667 and is confirmed. two_stage gives
// it to confirmed track 0, inside its gate, which moves 3.5/4.5 of 3.5 m to 49/18.
TEST(KalmanTracker, LikelihoodCostGivesAMeasurementToAWideTentativeTrackThatTwoStageDoesNot) {
  for (const AssociationMode mode : {AssociationMode::joint, AssociationMode::joint_likelihood}) {
    const auto tracks = steal_outcome(mode, 3.5);
    ASSERT_EQ(tracks.size(), 2u);
    EXPECT_EQ(tracks[0], (TrackState{2.0, 0, 0, {0.0, 0.0}, {0.0, 0.0}}));
    EXPECT_EQ(tracks[1].track_id, 1);
    EXPECT_NEAR(tracks[1].position.x, 6.3 - 7.0 / 3.0, 1e-12);
  }

  const auto two_stage = steal_outcome(AssociationMode::two_stage, 3.5);

  ASSERT_EQ(two_stage.size(), 1u);
  EXPECT_EQ(two_stage[0].track_id, 0);
  EXPECT_NEAR(two_stage[0].position.x, 49.0 / 18.0, 1e-12);
}

// New tracks at (0, 0) and (20, 0) with the steal_tracker parameters. At t = 1 both have S = 6 I,
// so ln(det S / det R) = ln 36 = 3.5835.
// (7, 0): d^2 = 49 / 6 = 8.1667 to track 0, inside the 9.21 gate, although its joint_likelihood
//   cost 8.1667 + 3.5835 = 11.7502 is above 9.21; to track 1, 169 / 6 = 28.167, outside.
// (27.5, 0): d^2 = 56.25 / 6 = 9.375 to track 1, just outside; to track 0, 756.25 / 6 = 126.04.
// Pairing (27.5, 0) with track 1 would give two pairs instead of one, but it is outside the gate,
// so track 1 misses and is dropped. Track 0 takes (7, 0), moves 5/6 of 7 m to 35/6 and is
// confirmed.
TEST(KalmanTracker, EveryModeGatesOnTheSquaredDistanceAlone) {
  for (const AssociationMode mode : {AssociationMode::two_stage, AssociationMode::joint,
                                     AssociationMode::joint_likelihood}) {
    KalmanTracker tracker = steal_tracker(mode);
    tracker.update(0.0, {at(0.0, 0.0, 0.0), at(0.0, 20.0, 0.0)});

    const auto tracks = tracker.update(1.0, {at(1.0, 7.0, 0.0), at(1.0, 27.5, 0.0)});

    ASSERT_EQ(tracks.size(), 1u) << static_cast<int>(mode);
    EXPECT_EQ(tracks[0].track_id, 0);
    EXPECT_NEAR(tracks[0].position.x, 35.0 / 6.0, 1e-12);
  }
}

// Tracks at (0, 0) and (14.5, 0), then one measurement at (6.8, 0); both tracks have S = 6 I, so
// ln(det S / det R) = 3.5835.
// Track 0: d^2 = 6.8^2 / 6 = 7.7067, inside the 9.21 gate, and costs 7.7067 + 3.5835 = 11.2902.
// Track 1: d^2 = 7.7^2 / 6 = 9.8817, outside the gate, but below track 0's cost. A mode that let
// the raw d^2 compete with the likelihood costs would give the measurement to track 1.
TEST(KalmanTracker, AnOutsideGatePairWithTheLowerLikelihoodCostNeverBeatsAPairInsideTheGate) {
  for (const AssociationMode mode : {AssociationMode::two_stage, AssociationMode::joint,
                                     AssociationMode::joint_likelihood}) {
    KalmanTracker tracker = steal_tracker(mode);
    tracker.update(0.0, {at(0.0, 0.0, 0.0), at(0.0, 14.5, 0.0)});

    const auto tracks = tracker.update(1.0, {at(1.0, 6.8, 0.0)});

    ASSERT_EQ(tracks.size(), 1u) << static_cast<int>(mode);
    EXPECT_EQ(tracks[0].track_id, 0) << static_cast<int>(mode);
    EXPECT_NEAR(tracks[0].position.x, 6.8 * 5.0 / 6.0, 1e-12) << static_cast<int>(mode);
  }
}

// A track born at (0, 0) and predicted one second ahead has S = 6 I with these parameters, so a
// measurement at (7, 0) has d^2 = 49 / 6. With confirm_hits 2 in 2 steps, the track is confirmed
// (and output as id 0) only if it takes the measurement.
bool track_takes_the_measurement(AssociationMode mode, double gate_chi2) {
  KalmanTracker tracker{0.0, 1.0, 2.0, gate_chi2, 2, 2, 5, mode};
  tracker.update(0.0, {at(0.0, 0.0, 0.0)});
  const auto tracks = tracker.update(1.0, {at(1.0, 7.0, 0.0)});
  return tracks.size() == 1 && tracks[0].track_id == 0;
}

// The gate is set bit for bit to the d^2 the filter reports, and to the next double below it.
TEST(KalmanTracker, APairExactlyOnTheGateIsMatchableAndOneJustOutsideIsNot) {
  ConstantVelocityKalman filter{0.0, 1.0};
  filter.initialize({0.0, 0.0}, 2.0);
  filter.predict(1.0);
  const double d2 = filter.innovation_mahalanobis2({7.0, 0.0});
  ASSERT_GT(d2, 0.0);
  ASSERT_LT(d2, 9.21);
  const double just_below = std::nextafter(d2, 0.0);

  for (const AssociationMode mode : {AssociationMode::two_stage, AssociationMode::joint,
                                     AssociationMode::joint_likelihood}) {
    EXPECT_TRUE(track_takes_the_measurement(mode, d2)) << static_cast<int>(mode);
    EXPECT_FALSE(track_takes_the_measurement(mode, just_below)) << static_cast<int>(mode);
  }
}

void expect_same_record(const StepRecord& actual, const StepRecord& expected) {
  EXPECT_EQ(actual.started_track, expected.started_track);
  ASSERT_EQ(actual.tracks.size(), expected.tracks.size());
  for (std::size_t i = 0; i < actual.tracks.size(); ++i) {
    const TrackRecord& a = actual.tracks[i];
    const TrackRecord& e = expected.tracks[i];
    EXPECT_EQ(a.track_id, e.track_id);
    EXPECT_EQ(a.stage, e.stage);
    EXPECT_EQ(a.hits, e.hits);
    EXPECT_EQ(a.misses, e.misses);
    EXPECT_EQ(a.predicted, e.predicted);
    EXPECT_EQ(a.innovation_covariance.values, e.innovation_covariance.values);
    EXPECT_EQ(a.measurement, e.measurement);
  }
}

// The 1e-100 sigma makes the last update throw std::domain_error after the tracker has built that
// step's record: track 0 predicts onto (0, 0), where the measurement sits, inside its gate with an
// infinite cost.
TEST(KalmanTracker, AThrowingUpdateLeavesThePreviousStepRecordUnchanged) {
  KalmanTracker tracker{0.0, 1e-100, 10.0, 9.21, 3, 5, 5, AssociationMode::joint_likelihood};
  tracker.record_steps(true);
  tracker.update(0.0, {at(0.0, 0.0, 0.0)});
  tracker.update(0.1, {at(0.1, 1000.0, 0.0)});
  ASSERT_TRUE(tracker.step_record().has_value());
  const StepRecord before = *tracker.step_record();
  ASSERT_EQ(before.tracks.size(), 1u);

  EXPECT_THROW(tracker.update(0.1, {at(0.1, 0.0, 0.0)}), std::invalid_argument);
  ASSERT_TRUE(tracker.step_record().has_value());
  expect_same_record(*tracker.step_record(), before);
  EXPECT_THROW(tracker.update(kNan, {at(0.2, 0.0, 0.0)}), std::invalid_argument);
  ASSERT_TRUE(tracker.step_record().has_value());
  expect_same_record(*tracker.step_record(), before);
  EXPECT_THROW(tracker.update(0.2, {at(0.2, kNan, 0.0)}), std::invalid_argument);
  ASSERT_TRUE(tracker.step_record().has_value());
  expect_same_record(*tracker.step_record(), before);
  EXPECT_THROW(tracker.update(0.2, {at(0.2, 0.0, 0.0)}), std::domain_error);
  ASSERT_TRUE(tracker.step_record().has_value());
  expect_same_record(*tracker.step_record(), before);
}

// measurement_sigma 1e-100 gives R = 1e-200 I, and det R = 1e-400 underflows to 0. After 0.1 s a
// new track's S is about I (initial velocity sigma 10 m/s), so det S / det R is infinite.
TEST(KalmanTracker, JointLikelihoodThrowsWhenACostInsideTheGateIsNotFinite) {
  KalmanTracker likelihood{0.0, 1e-100, 10.0, 9.21, 3, 5, 5, AssociationMode::joint_likelihood};
  KalmanTracker joint{0.0, 1e-100, 10.0, 9.21, 3, 5, 5, AssociationMode::joint};
  likelihood.update(0.0, {at(0.0, 0.0, 0.0)});
  joint.update(0.0, {at(0.0, 0.0, 0.0)});

  EXPECT_THROW(likelihood.update(0.1, {at(0.1, 0.0, 0.0)}), std::domain_error);
  EXPECT_NO_THROW(joint.update(0.1, {at(0.1, 0.0, 0.0)}));
}

TEST(KalmanTracker, JointLikelihoodErrorNamesTheOutsideCostWhenOnlyItOverflows) {
  constexpr double gate = 1e308;
  constexpr double far_x = 2.4e154;
  ConstantVelocityKalman filter{0.0, 1.0};
  filter.initialize({0.0, 0.0}, 2.0);
  filter.predict(1.0);
  const double d2 = filter.innovation_mahalanobis2({far_x, 0.0});
  ASSERT_TRUE(std::isfinite(d2));
  ASSERT_LE(d2, gate);
  ASSERT_TRUE(std::isinf(2.0 * d2 + 1.0));

  KalmanTracker likelihood{0.0, 1.0, 2.0, gate, 2, 2, 5, AssociationMode::joint_likelihood};
  KalmanTracker joint{0.0, 1.0, 2.0, gate, 2, 2, 5, AssociationMode::joint};
  likelihood.update(0.0, {at(0.0, 0.0, 0.0)});
  joint.update(0.0, {at(0.0, 0.0, 0.0)});

  EXPECT_NO_THROW(joint.update(1.0, {at(1.0, far_x, 0.0)}));
  try {
    likelihood.update(1.0, {at(1.0, far_x, 0.0)});
    ADD_FAILURE() << "expected std::domain_error";
  } catch (const std::domain_error& error) {
    EXPECT_THAT(error.what(), testing::HasSubstr("cost outside the gate"));
    EXPECT_THAT(error.what(), testing::HasSubstr("2 * the largest cost inside the gate + 1"));
  }
}

TEST(KalmanTracker, StepRecordDescribesEveryTrackAndEveryBirth) {
  KalmanTracker tracker = steal_tracker(AssociationMode::joint);
  tracker.record_steps(true);
  tracker.update(0.0, steal_step(0));
  const StepRecord first = tracker.step_record().value();
  tracker.update(1.0, steal_step(1));
  const StepRecord second = tracker.step_record().value();
  tracker.update(2.0, steal_step(2));
  const StepRecord third = tracker.step_record().value();

  EXPECT_TRUE(first.tracks.empty());
  EXPECT_EQ(first.started_track, (std::vector<std::optional<int>>{0}));

  ASSERT_EQ(second.tracks.size(), 1u);
  const TrackRecord& young = second.tracks[0];
  EXPECT_EQ(young.track_id, 0);
  EXPECT_EQ(young.stage, TrackStage::tentative);
  EXPECT_EQ(young.hits, 1);
  EXPECT_EQ(young.misses, 0);
  EXPECT_EQ(young.age_steps(), 1);
  EXPECT_EQ(young.predicted, (Vec2{0.0, 0.0}));
  EXPECT_EQ(young.innovation_covariance.values, (std::array<double, 4>{6.0, 0.0, 0.0, 6.0}));
  EXPECT_EQ(young.measurement, std::optional<std::size_t>(0));
  EXPECT_EQ(second.started_track, (std::vector<std::optional<int>>{std::nullopt, 1}));

  ASSERT_EQ(third.tracks.size(), 2u);
  EXPECT_EQ(third.tracks[0].hits, 2);
  EXPECT_EQ(third.tracks[0].age_steps(), 2);
  EXPECT_EQ(third.tracks[1].track_id, 1);
  EXPECT_EQ(third.tracks[1].hits, 1);
  EXPECT_EQ(third.tracks[1].predicted, (Vec2{6.3, 0.0}));
  EXPECT_EQ(third.started_track, (std::vector<std::optional<int>>{std::nullopt}));
}

TEST(KalmanTracker, StepRecordNeverChangesOutputOnTheTuningSeeds) {
  for (const AssociationMode mode : {AssociationMode::two_stage, AssociationMode::joint,
                                     AssociationMode::joint_likelihood}) {
    for (std::uint64_t seed = 101; seed <= 120; ++seed) {
      const GeneratedScenario scenario = generate(baseline_setup::scenario_config(seed));
      KalmanTracker off = baseline_setup::make_kalman_tracker(mode);
      KalmanTracker on = baseline_setup::make_kalman_tracker(mode);
      on.record_steps(true);
      for (const ScenarioStep& step : scenario.steps) {
        const auto expected = off.update(step.t, step.measurements);
        ASSERT_EQ(on.update(step.t, step.measurements), expected) << "seed " << seed;
        const StepRecord& record = on.step_record().value();
        std::vector<int> uses(step.measurements.size(), 0);
        for (const TrackRecord& track : record.tracks) {
          if (track.measurement) {
            ++uses.at(*track.measurement);
          }
        }
        for (std::size_t m = 0; m < uses.size(); ++m) {
          ASSERT_EQ(uses[m] + (record.started_track[m] ? 1 : 0), 1) << "seed " << seed;
        }
      }
    }
  }
}

}
}
