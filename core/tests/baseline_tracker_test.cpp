#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "kite/baseline_tracker.hpp"

namespace kite {
namespace {

std::unique_ptr<Tracker> make_tracker(double gate) {
  return std::make_unique<NearestNeighbourBaseline>(gate);
}

Measurement at(double t, double x, double y, int ego_id = 0) {
  return Measurement{t, ego_id, 0, {x, y}};
}

TEST(BaselineTracker, SingleMeasurementCreatesTrackZeroWithZeroVelocity) {
  auto tracker = make_tracker(2.0);

  const auto tracks = tracker->update(0.0, {at(0.0, 3.0, 4.0, 7)});

  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0], (TrackState{0.0, 7, 0, {3.0, 4.0}, {0.0, 0.0}}));
}

TEST(BaselineTracker, MeasurementOneMetreAwayKeepsIdWithVelocityOverDt) {
  auto tracker = make_tracker(2.0);
  tracker->update(1.0, {at(1.0, 0.0, 0.0)});

  const auto tracks = tracker->update(1.5, {at(1.5, 1.0, 0.0)});

  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0], (TrackState{1.5, 0, 0, {1.0, 0.0}, {2.0, 0.0}}));
}

TEST(BaselineTracker, VelocityPointsFromPreviousToCurrentPositionInBothAxes) {
  auto tracker = make_tracker(10.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0)});

  const auto tracks = tracker->update(2.0, {at(2.0, 3.0, -4.0)});

  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_EQ(tracks[0].velocity, (Vec2{1.5, -2.0}));
}

TEST(BaselineTracker, MeasurementBeyondGateGetsNewIdAndZeroVelocity) {
  auto tracker = make_tracker(1.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0)});

  const auto tracks = tracker->update(1.0, {at(1.0, 1.5, 0.0)});

  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0], (TrackState{1.0, 0, 1, {1.5, 0.0}, {0.0, 0.0}}));
}

TEST(BaselineTracker, MeasurementExactlyAtGateMatches) {
  auto tracker = make_tracker(5.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0)});

  const auto tracks = tracker->update(1.0, {at(1.0, 3.0, 4.0)});

  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_EQ(tracks[0].velocity, (Vec2{3.0, 4.0}));
}

TEST(BaselineTracker, ZeroGateMatchesOnlyIdenticalPosition) {
  auto tracker = make_tracker(0.0);
  tracker->update(0.0, {at(0.0, 1.0, 1.0)});

  const auto same = tracker->update(1.0, {at(1.0, 1.0, 1.0)});
  const auto moved = tracker->update(2.0, {at(2.0, 1.0, 1.5)});

  ASSERT_EQ(same.size(), 1u);
  EXPECT_EQ(same[0].track_id, 0);
  ASSERT_EQ(moved.size(), 1u);
  EXPECT_EQ(moved[0].track_id, 1);
}

// Track 0 is at (0, 0) and track 1 is at (2, 0). The measurements arrive as (1.2, 0), then (0.9, 0).
// Pair distances in ascending order are 0.8 for (1.2, 0) and track 1, 0.9 for (0.9, 0) and
// track 0, 1.1 for (0.9, 0) and track 1, and 1.2 for (1.2, 0) and track 0. Greedy takes the
// closest pair first, so (1.2, 0) keeps id 1 and (0.9, 0) keeps id 0, although the measurement
// order is the reverse of the track order.
TEST(BaselineTracker, CrossingMeasurementsKeepIdOfClosestPreviousTrackGreedily) {
  auto tracker = make_tracker(5.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0), at(0.0, 2.0, 0.0)});

  const auto tracks = tracker->update(0.5, {at(0.5, 1.2, 0.0), at(0.5, 0.9, 0.0)});

  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_EQ(tracks[0].position, (Vec2{0.9, 0.0}));
  EXPECT_NEAR(tracks[0].velocity.x, 1.8, 1e-12);
  EXPECT_EQ(tracks[1].track_id, 1);
  EXPECT_EQ(tracks[1].position, (Vec2{1.2, 0.0}));
  EXPECT_NEAR(tracks[1].velocity.x, -1.6, 1e-12);
}

TEST(BaselineTracker, EqualDistanceToTwoTracksGoesToLowerTrackId) {
  auto tracker = make_tracker(5.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0), at(0.0, 2.0, 0.0)});

  const auto tracks = tracker->update(1.0, {at(1.0, 1.0, 0.0)});

  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_EQ(tracks[0].velocity, (Vec2{1.0, 0.0}));
}

TEST(BaselineTracker, EqualDistanceFromTwoMeasurementsGoesToEarlierMeasurement) {
  auto tracker = make_tracker(5.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0)});

  const auto tracks = tracker->update(1.0, {at(1.0, 1.0, 0.0), at(1.0, -1.0, 0.0)});

  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_EQ(tracks[0].position, (Vec2{1.0, 0.0}));
  EXPECT_EQ(tracks[1].track_id, 1);
  EXPECT_EQ(tracks[1].position, (Vec2{-1.0, 0.0}));
  EXPECT_EQ(tracks[1].velocity, (Vec2{0.0, 0.0}));
}

TEST(BaselineTracker, NewIdsFollowMeasurementOrderAndOutputIsSortedById) {
  auto tracker = make_tracker(1.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0)});

  const auto tracks = tracker->update(1.0, {at(1.0, 50.0, 0.0), at(1.0, 0.0, 0.5), at(1.0, 60.0, 0.0)});

  ASSERT_EQ(tracks.size(), 3u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_EQ(tracks[0].position, (Vec2{0.0, 0.5}));
  EXPECT_EQ(tracks[1].track_id, 1);
  EXPECT_EQ(tracks[1].position, (Vec2{50.0, 0.0}));
  EXPECT_EQ(tracks[2].track_id, 2);
  EXPECT_EQ(tracks[2].position, (Vec2{60.0, 0.0}));
}

TEST(BaselineTracker, EmptyInputGivesEmptyOutput) {
  auto tracker = make_tracker(1.0);

  EXPECT_TRUE(tracker->update(0.0, {}).empty());
  tracker->update(1.0, {at(1.0, 0.0, 0.0)});
  EXPECT_TRUE(tracker->update(2.0, {}).empty());
}

TEST(BaselineTracker, IdIsNotReusedAfterTrackIsDroppedByEmptyStep) {
  auto tracker = make_tracker(1.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0)});
  tracker->update(1.0, {});

  const auto tracks = tracker->update(2.0, {at(2.0, 0.0, 0.0)});

  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].track_id, 1);
  EXPECT_EQ(tracks[0].velocity, (Vec2{0.0, 0.0}));
}

TEST(BaselineTracker, IdIsNotReusedAfterTrackIsDroppedByGate) {
  auto tracker = make_tracker(1.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0)});
  const auto far = tracker->update(1.0, {at(1.0, 10.0, 0.0)});

  const auto back = tracker->update(2.0, {at(2.0, 0.0, 0.0)});

  ASSERT_EQ(far.size(), 1u);
  EXPECT_EQ(far[0].track_id, 1);
  ASSERT_EQ(back.size(), 1u);
  EXPECT_EQ(back[0].track_id, 2);
}

TEST(BaselineTracker, UnmatchedPreviousTrackIsDroppedAndOtherKeepsId) {
  auto tracker = make_tracker(1.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0), at(0.0, 10.0, 0.0)});

  const auto tracks = tracker->update(1.0, {at(1.0, 10.5, 0.0)});

  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].track_id, 1);
  EXPECT_EQ(tracks[0].velocity, (Vec2{0.5, 0.0}));
}

TEST(BaselineTracker, OneTrackIsMatchedToAtMostOneMeasurement) {
  auto tracker = make_tracker(5.0);
  tracker->update(0.0, {at(0.0, 0.0, 0.0)});

  const auto tracks = tracker->update(1.0, {at(1.0, 1.0, 0.0), at(1.0, 2.0, 0.0)});

  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_EQ(tracks[0].position, (Vec2{1.0, 0.0}));
  EXPECT_EQ(tracks[1].track_id, 1);
  EXPECT_EQ(tracks[1].velocity, (Vec2{0.0, 0.0}));
}

TEST(BaselineTracker, UpdateTimeThatDoesNotIncreaseThrowsAndLeavesStateUnchanged) {
  auto tracker = make_tracker(5.0);
  tracker->update(1.0, {at(1.0, 0.0, 0.0)});

  EXPECT_THROW(tracker->update(1.0, {at(1.0, 1.0, 0.0)}), std::invalid_argument);
  EXPECT_THROW(tracker->update(0.5, {at(0.5, 1.0, 0.0)}), std::invalid_argument);

  const auto tracks = tracker->update(2.0, {at(2.0, 1.0, 0.0)});
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].track_id, 0);
  EXPECT_EQ(tracks[0].velocity, (Vec2{1.0, 0.0}));
}

TEST(BaselineTracker, InvalidGateThrows) {
  EXPECT_THROW(NearestNeighbourBaseline(-0.1), std::invalid_argument);
  EXPECT_THROW(NearestNeighbourBaseline(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_THROW(NearestNeighbourBaseline(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

}
}
