#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "kite/scoring.hpp"

namespace kite {
namespace {

GroundTruthState actor(double t, int actor_id, double x, double y, int ego_id = 0) {
  return GroundTruthState{t, ego_id, actor_id, {x, y}, {0.0, 0.0}};
}

TrackState track(double t, int track_id, double x, double y, int ego_id = 0) {
  return TrackState{t, ego_id, track_id, {x, y}, {0.0, 0.0}};
}

TEST(Scoring, PerfectTracksHaveZeroErrorNoMissesNoFalseTracksNoSwitches) {
  const std::vector<GroundTruthState> truth = {actor(0.0, 1, 0.0, 0.0), actor(0.0, 2, 50.0, 0.0),
                                               actor(0.1, 1, 1.0, 0.0), actor(0.1, 2, 51.0, 0.0)};
  const std::vector<TrackState> tracks = {track(0.0, 10, 0.0, 0.0), track(0.0, 11, 50.0, 0.0),
                                          track(0.1, 10, 1.0, 0.0), track(0.1, 11, 51.0, 0.0)};

  const ScoreResult result = score(truth, tracks, 3.0);

  EXPECT_EQ(result.matched, 4);
  EXPECT_EQ(result.missed, 0);
  EXPECT_EQ(result.false_tracks, 0);
  ASSERT_TRUE(result.rmse_m.has_value());
  EXPECT_EQ(*result.rmse_m, 0.0);
  EXPECT_EQ(result.id_switches, 0);
  EXPECT_EQ(result.steps, 2);
}

TEST(Scoring, ConstantThreeFourOffsetGivesRmseFive) {
  const std::vector<GroundTruthState> truth = {actor(0.0, 1, 0.0, 0.0), actor(0.0, 2, 100.0, 0.0),
                                               actor(1.0, 1, 2.0, 0.0), actor(1.0, 2, 102.0, 0.0)};
  const std::vector<TrackState> tracks = {track(0.0, 0, 3.0, 4.0), track(0.0, 1, 103.0, 4.0),
                                          track(1.0, 0, 5.0, 4.0), track(1.0, 1, 105.0, 4.0)};

  const ScoreResult result = score(truth, tracks, 6.0);

  EXPECT_EQ(result.matched, 4);
  ASSERT_TRUE(result.rmse_m.has_value());
  EXPECT_EQ(*result.rmse_m, 5.0);
  EXPECT_EQ(result.id_switches, 0);
}

TEST(Scoring, TrackBeyondGateIsOneMissAndOneFalseWithNoRmse) {
  const ScoreResult result = score({actor(0.0, 1, 0.0, 0.0)}, {track(0.0, 0, 10.0, 0.0)}, 3.0);

  EXPECT_EQ(result.matched, 0);
  EXPECT_EQ(result.missed, 1);
  EXPECT_EQ(result.false_tracks, 1);
  EXPECT_FALSE(result.rmse_m.has_value());
  EXPECT_EQ(result.id_switches, 0);
  EXPECT_EQ(result.steps, 1);
}

TEST(Scoring, GateBoundaryIsInclusive) {
  const ScoreResult at_gate = score({actor(0.0, 1, 0.0, 0.0)}, {track(0.0, 0, 3.0, 4.0)}, 5.0);
  const ScoreResult inside_gate = score({actor(0.0, 1, 0.0, 0.0)}, {track(0.0, 0, 3.0, 4.0)},
                                        std::nextafter(5.0, 0.0));

  EXPECT_EQ(at_gate.matched, 1);
  ASSERT_TRUE(at_gate.rmse_m.has_value());
  EXPECT_EQ(*at_gate.rmse_m, 5.0);
  EXPECT_EQ(inside_gate.matched, 0);
  EXPECT_EQ(inside_gate.missed, 1);
  EXPECT_EQ(inside_gate.false_tracks, 1);
}

TEST(Scoring, TwoTracksSwappingIdsCountOneSwitchPerActor) {
  const std::vector<GroundTruthState> truth = {actor(0.0, 1, 0.0, 0.0), actor(0.0, 2, 20.0, 0.0),
                                               actor(1.0, 1, 0.0, 0.0), actor(1.0, 2, 20.0, 0.0)};
  const std::vector<TrackState> tracks = {track(0.0, 10, 0.0, 0.0), track(0.0, 11, 20.0, 0.0),
                                          track(1.0, 11, 0.0, 0.0), track(1.0, 10, 20.0, 0.0)};

  const ScoreResult result = score(truth, tracks, 3.0);

  EXPECT_EQ(result.matched, 4);
  EXPECT_EQ(result.id_switches, 2);
}

TEST(Scoring, UnmatchedStepKeepsTheActorsLastTrackId) {
  const std::vector<GroundTruthState> truth = {actor(0.0, 1, 0.0, 0.0), actor(1.0, 1, 0.0, 0.0),
                                               actor(2.0, 1, 0.0, 0.0), actor(3.0, 1, 0.0, 0.0)};
  const std::vector<TrackState> tracks = {track(0.0, 10, 0.0, 0.0), track(2.0, 10, 0.0, 0.0),
                                          track(3.0, 11, 0.0, 0.0)};

  const ScoreResult result = score(truth, tracks, 3.0);

  EXPECT_EQ(result.matched, 3);
  EXPECT_EQ(result.missed, 1);
  EXPECT_EQ(result.id_switches, 1);
}

TEST(Scoring, EmptyInputHasNoStepsAndNoRmse) {
  const ScoreResult result = score({}, {}, 3.0);

  EXPECT_EQ(result.matched, 0);
  EXPECT_EQ(result.missed, 0);
  EXPECT_EQ(result.false_tracks, 0);
  EXPECT_FALSE(result.rmse_m.has_value());
  EXPECT_EQ(result.id_switches, 0);
  EXPECT_EQ(result.steps, 0);
}

TEST(Scoring, StepWithoutTracksIsAllMissedAndStepWithoutTruthIsAllFalse) {
  const std::vector<GroundTruthState> truth = {actor(0.0, 1, 0.0, 0.0), actor(1.0, 1, 0.0, 0.0),
                                               actor(1.0, 2, 9.0, 0.0)};
  const std::vector<TrackState> tracks = {track(0.0, 0, 0.0, 0.0), track(2.0, 0, 0.0, 0.0),
                                          track(2.0, 1, 9.0, 0.0), track(2.0, 2, 30.0, 0.0)};

  const ScoreResult result = score(truth, tracks, 3.0);

  EXPECT_EQ(result.matched, 1);
  EXPECT_EQ(result.missed, 2);
  EXPECT_EQ(result.false_tracks, 3);
  EXPECT_EQ(result.steps, 3);
}

TEST(Scoring, SameTimeOnDifferentEgosIsTwoSteps) {
  const ScoreResult result =
      score({actor(0.0, 1, 0.0, 0.0, 0)}, {track(0.0, 0, 0.0, 0.0, 1)}, 3.0);

  EXPECT_EQ(result.matched, 0);
  EXPECT_EQ(result.missed, 1);
  EXPECT_EQ(result.false_tracks, 1);
  EXPECT_EQ(result.steps, 2);
}

// On a line: truths A = 0 and B = 4, tracks P = 3 (id 1) and Q = 8 (id 0), gate 10.
// Greedy takes the closest pair B-P (1) and then A-Q (8): total 9, squared sum 65.
// The minimum total is A-P (3) plus B-Q (4) = 7, squared sum 25.
// At t = 1 each track sits on the actor the minimum matching gave it, so that matching has no
// switches (greedy would have two). RMSE over four pairs: sqrt((25 + 0) / 4) = 2.5.
TEST(Scoring, MinimumTotalDistanceBeatsGreedyClosestPairFirst) {
  const std::vector<GroundTruthState> truth = {actor(0.0, 1, 0.0, 0.0), actor(0.0, 2, 4.0, 0.0),
                                               actor(1.0, 1, 0.0, 0.0), actor(1.0, 2, 4.0, 0.0)};
  const std::vector<TrackState> tracks = {track(0.0, 1, 3.0, 0.0), track(0.0, 0, 8.0, 0.0),
                                          track(1.0, 1, 0.0, 0.0), track(1.0, 0, 4.0, 0.0)};

  const ScoreResult result = score(truth, tracks, 10.0);

  EXPECT_EQ(result.matched, 4);
  EXPECT_EQ(result.id_switches, 0);
  ASSERT_TRUE(result.rmse_m.has_value());
  EXPECT_EQ(*result.rmse_m, 2.5);
}

// On a line: truths A = 0 and B = 4, tracks P = 3 (id 1) and Q = 7 (id 0), gate 4. Pairs in the
// gate: A-P 3, B-P 1, B-Q 3 (A-Q is 7). The closest pair B-P alone would leave A and Q unmatched.
// The largest matching is A-P and B-Q: two matches, RMSE sqrt((9 + 9) / 2) = 3.
TEST(Scoring, MostPairsWithinGateBeatsTakingTheClosestPair) {
  const std::vector<GroundTruthState> truth = {actor(0.0, 1, 0.0, 0.0), actor(0.0, 2, 4.0, 0.0)};
  const std::vector<TrackState> tracks = {track(0.0, 1, 3.0, 0.0), track(0.0, 0, 7.0, 0.0)};

  const ScoreResult result = score(truth, tracks, 4.0);

  EXPECT_EQ(result.matched, 2);
  EXPECT_EQ(result.missed, 0);
  EXPECT_EQ(result.false_tracks, 0);
  ASSERT_TRUE(result.rmse_m.has_value());
  EXPECT_EQ(*result.rmse_m, 3.0);
}

TEST(Scoring, ZeroGateMatchesOnlyIdenticalPositions) {
  const ScoreResult result = score({actor(0.0, 1, 1.0, 1.0), actor(0.0, 2, 5.0, 5.0)},
                                   {track(0.0, 0, 1.0, 1.0), track(0.0, 1, 5.0, 5.5)}, 0.0);

  EXPECT_EQ(result.matched, 1);
  EXPECT_EQ(result.missed, 1);
  EXPECT_EQ(result.false_tracks, 1);
}

TEST(Scoring, InvalidGateThrows) {
  const std::vector<GroundTruthState> truth = {actor(0.0, 1, 0.0, 0.0)};
  const std::vector<TrackState> tracks = {track(0.0, 0, 0.0, 0.0)};

  EXPECT_THROW(score(truth, tracks, -0.1), std::invalid_argument);
  EXPECT_THROW(score(truth, tracks, std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_THROW(score(truth, tracks, std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

TEST(Scoring, NonFiniteTimeThrows) {
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(score({actor(nan, 1, 0.0, 0.0)}, {}, 3.0), std::invalid_argument);
  EXPECT_THROW(score({}, {track(nan, 0, 0.0, 0.0)}, 3.0), std::invalid_argument);
}

TEST(Scoring, NonFinitePositionThrows) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<GroundTruthState> finite_truth = {actor(0.0, 1, 0.0, 0.0)};
  const std::vector<TrackState> finite_tracks = {track(0.0, 0, 0.0, 0.0)};

  EXPECT_THROW(score({actor(0.0, 1, nan, 0.0)}, finite_tracks, 3.0), std::invalid_argument);
  EXPECT_THROW(score({actor(0.0, 1, 0.0, inf)}, finite_tracks, 3.0), std::invalid_argument);
  EXPECT_THROW(score(finite_truth, {track(0.0, 0, nan, 0.0)}, 3.0), std::invalid_argument);
  EXPECT_THROW(score(finite_truth, {track(0.0, 0, 0.0, -inf)}, 3.0), std::invalid_argument);
}

}
}
