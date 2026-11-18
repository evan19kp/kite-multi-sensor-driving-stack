#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "kite/baseline_tracker.hpp"
#include "kite/kalman_tracker.hpp"

namespace kite {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

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

// Hits at steps 0, 3 and 5: the window of the last 5 steps at step 5 holds steps 1-5, so only two
// hits count. The hit at step 6 makes three (steps 3, 5, 6).
TEST(KalmanTracker, HitsOlderThanTheWindowDoNotCount) {
  KalmanTracker tracker = make_tracker();

  tracker.update(0.0, {at(0.0, 0.0, 0.0)});
  tracker.update(1.0, {});
  tracker.update(2.0, {});
  tracker.update(3.0, {at(3.0, 0.0, 0.0)});
  tracker.update(4.0, {});
  const auto two_in_window = tracker.update(5.0, {at(5.0, 0.0, 0.0)});
  const auto three_in_window = tracker.update(6.0, {at(6.0, 0.0, 0.0)});

  EXPECT_TRUE(two_in_window.empty());
  ASSERT_EQ(three_in_window.size(), 1u);
  EXPECT_EQ(three_in_window[0].track_id, 0);
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

}
}
