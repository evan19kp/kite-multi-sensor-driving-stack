#include "kite/angles.hpp"

#include <cmath>
#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-12;

TEST(WrapAngle, ZeroStaysZero) {
  EXPECT_NEAR(kite::wrap_angle(0.0), 0.0, kTolerance);
}

TEST(WrapAngle, PiWrapsToMinusPi) {
  EXPECT_NEAR(kite::wrap_angle(kPi), -kPi, kTolerance);
}

TEST(WrapAngle, MinusPiStaysMinusPi) {
  EXPECT_NEAR(kite::wrap_angle(-kPi), -kPi, kTolerance);
}

TEST(WrapAngle, ThreePiWrapsToMinusPi) {
  EXPECT_NEAR(kite::wrap_angle(3.0 * kPi), -kPi, kTolerance);
}

TEST(WrapAngle, TwoPiPlusSmallWrapsToSmall) {
  EXPECT_NEAR(kite::wrap_angle(2.0 * kPi + 0.1), 0.1, kTolerance);
}

TEST(WrapAngle, MinusTwoPiMinusSmallWrapsToMinusSmall) {
  EXPECT_NEAR(kite::wrap_angle(-2.0 * kPi - 0.1), -0.1, kTolerance);
}

TEST(WrapAngle, LargeInputKeepsSineAndCosine) {
  const double wrapped = kite::wrap_angle(1e6);
  EXPECT_GE(wrapped, -kPi);
  EXPECT_LT(wrapped, kPi);
  EXPECT_NEAR(std::sin(wrapped), std::sin(1e6), 1e-9);
  EXPECT_NEAR(std::cos(wrapped), std::cos(1e6), 1e-9);
}

TEST(WrapAngle, NanGivesNan) {
  EXPECT_TRUE(std::isnan(kite::wrap_angle(std::numeric_limits<double>::quiet_NaN())));
}

TEST(WrapAngle, InfinityGivesNan) {
  EXPECT_TRUE(std::isnan(kite::wrap_angle(std::numeric_limits<double>::infinity())));
}

class Counter {
 public:
  virtual ~Counter() = default;
  virtual int next() = 0;
};

class MockCounter : public Counter {
 public:
  MOCK_METHOD(int, next, (), (override));
};

TEST(GMockSmoke, MockReturnsConfiguredValue) {
  MockCounter counter;
  EXPECT_CALL(counter, next()).WillOnce(testing::Return(7));
  EXPECT_EQ(counter.next(), 7);
}

}  // namespace
