#include <gtest/gtest.h>

#include <optional>

#include "kite/kalman_tracker.hpp"
#include "kite/matrix.hpp"
#include "kite/types.hpp"
#include "steal_class.hpp"

namespace {

using kite::Matrix;
using kite::TrackRecord;
using kite::TrackStage;
using kite::Vec2;
using kite::study::classify_steal;
using kite::study::StealClass;

TrackRecord track(TrackStage stage, Vec2 predicted, Matrix<2, 2> s) {
  return TrackRecord{0, stage, 1, 0, predicted, s, std::nullopt};
}

TrackRecord tentative(Vec2 predicted, Matrix<2, 2> s) {
  return track(TrackStage::tentative, predicted, s);
}

TrackRecord confirmed(Vec2 predicted, Matrix<2, 2> s) {
  return track(TrackStage::confirmed, predicted, s);
}

Matrix<2, 2> isotropic(double s) { return Matrix<2, 2>{{s, 0.0, 0.0, s}}; }

constexpr Vec2 kOrigin{0.0, 0.0};

TEST(ClassifySteal, ATentativePredictionNearerToTheMeasurementIsNearer) {
  EXPECT_EQ(classify_steal(kOrigin, tentative({1.0, 0.0}, isotropic(1.0)),
                           confirmed({3.0, 0.0}, isotropic(1.0))),
            StealClass::nearer);
}

TEST(ClassifySteal, AFartherTentativeTrackWithALowerScoreAndALargerDetSIsH1) {
  // Distances 3 and 2. d^2: 9 / 6 = 1.5 against 4 / 1 = 4. det S: 36 against 1.
  EXPECT_EQ(classify_steal(kOrigin, tentative({3.0, 0.0}, isotropic(6.0)),
                           confirmed({2.0, 0.0}, isotropic(1.0))),
            StealClass::h1);
}

TEST(ClassifySteal, EqualDistancesAreNotNearer) {
  // Both predictions are 2 m away. d^2: 4 / 6 against 4. det S: 36 against 1.
  EXPECT_EQ(classify_steal(kOrigin, tentative({2.0, 0.0}, isotropic(6.0)),
                           confirmed({-2.0, 0.0}, isotropic(1.0))),
            StealClass::h1);
}

TEST(ClassifySteal, AFartherTentativeTrackThatDoesNotScoreLowerIsOther) {
  // Distances 3 and 2. d^2: 9 / 1.5 = 6 against 4 / 1 = 4, although det S is larger (2.25 > 1).
  EXPECT_EQ(classify_steal(kOrigin, tentative({3.0, 0.0}, isotropic(1.5)),
                           confirmed({2.0, 0.0}, isotropic(1.0))),
            StealClass::other);
}

TEST(ClassifySteal, AFartherTentativeTrackWithASmallerDetSIsOtherEvenWhenItScoresLower) {
  // Distances 3 and 2. d^2: 9 / 100 = 0.09 against 4 / 4 = 1, but det S is 100 * 0.01 = 1 against
  // 16.
  EXPECT_EQ(classify_steal(kOrigin, tentative({3.0, 0.0}, Matrix<2, 2>{{100.0, 0.0, 0.0, 0.01}}),
                           confirmed({2.0, 0.0}, isotropic(4.0))),
            StealClass::other);
}

TEST(ClassifySteal, NearerWinsOverH1WhenBothRulesHold) {
  // Distances 1 and 3, so nearer. d^2 is 1 / 6 against 9 and det S is 36 against 1, so the H1
  // conditions hold too.
  EXPECT_EQ(classify_steal(kOrigin, tentative({1.0, 0.0}, isotropic(6.0)),
                           confirmed({3.0, 0.0}, isotropic(1.0))),
            StealClass::nearer);
}

}
