#include "kite/matrix.hpp"

#include <array>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace {

using kite::Matrix;

// Every expected value below is worked out by hand from small integers or powers of two, so the
// results are exact in double precision and compared with EXPECT_EQ.

TEST(Matrix, StartsAtZeroAndIndexesRowMajor) {
  Matrix<2, 3> m;
  EXPECT_EQ(m.values, (std::array<double, 6>{0, 0, 0, 0, 0, 0}));
  m(1, 2) = 5.0;
  EXPECT_EQ(m.values[5], 5.0);
  EXPECT_EQ(m(1, 2), 5.0);
}

TEST(Matrix, IdentityHasOnesOnTheDiagonal) {
  EXPECT_EQ(kite::identity<2>().values, (std::array<double, 4>{1, 0, 0, 1}));
  EXPECT_EQ(kite::identity<4>().values,
            (std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));
}

TEST(Matrix, MultipliesTwoByFourByFourByTwo) {
  // a = [1 2 3 4; 5 6 7 8], b = [1 0; 0 1; 1 1; 2 -1].
  // Row 1: [1+3+8, 2+3-4] = [12, 1]. Row 2: [5+7+16, 6+7-8] = [28, 5].
  const Matrix<2, 4> a{{1, 2, 3, 4, 5, 6, 7, 8}};
  const Matrix<4, 2> b{{1, 0, 0, 1, 1, 1, 2, -1}};
  EXPECT_EQ((a * b).values, (std::array<double, 4>{12, 1, 28, 5}));
}

TEST(Matrix, MultipliesFourByTwoByTwoByFour) {
  // a = [1 0; 0 1; 1 1; 2 -1], b = [1 2 3 4; 5 6 7 8]. Row i of the product is
  // a(i,0) * [1 2 3 4] + a(i,1) * [5 6 7 8].
  const Matrix<4, 2> a{{1, 0, 0, 1, 1, 1, 2, -1}};
  const Matrix<2, 4> b{{1, 2, 3, 4, 5, 6, 7, 8}};
  EXPECT_EQ((a * b).values,
            (std::array<double, 16>{1, 2, 3, 4, 5, 6, 7, 8, 6, 8, 10, 12, -3, -2, -1, 0}));
}

TEST(Matrix, MultipliesFourByFourByFourByOne) {
  // A constant-velocity transition with dt = 2 moves [1, 2, 3, 4] to [1 + 2*3, 2 + 2*4, 3, 4].
  const Matrix<4, 4> f{{1, 0, 2, 0, 0, 1, 0, 2, 0, 0, 1, 0, 0, 0, 0, 1}};
  const Matrix<4, 1> x{{1, 2, 3, 4}};
  EXPECT_EQ((f * x).values, (std::array<double, 4>{7, 10, 3, 4}));
}

TEST(Matrix, MultiplyingByIdentityChangesNothing) {
  const Matrix<4, 4> m{{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}};
  EXPECT_EQ((kite::identity<4>() * m).values, m.values);
  EXPECT_EQ((m * kite::identity<4>()).values, m.values);
}

TEST(Matrix, TransposesRectangularMatrices) {
  const Matrix<2, 4> a{{1, 2, 3, 4, 5, 6, 7, 8}};
  EXPECT_EQ(kite::transpose(a).values, (std::array<double, 8>{1, 5, 2, 6, 3, 7, 4, 8}));
  EXPECT_EQ(kite::transpose(kite::transpose(a)).values, a.values);
}

TEST(Matrix, AddsAndSubtractsEntryByEntry) {
  const Matrix<2, 2> a{{1, 2, 3, 4}};
  const Matrix<2, 2> b{{10, 20, 30, 40}};
  EXPECT_EQ((a + b).values, (std::array<double, 4>{11, 22, 33, 44}));
  EXPECT_EQ((b - a).values, (std::array<double, 4>{9, 18, 27, 36}));

  const Matrix<4, 4> m{{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}};
  EXPECT_EQ((m - m).values, (Matrix<4, 4>{}.values));
  EXPECT_EQ((m + kite::identity<4>()).values,
            (std::array<double, 16>{2, 2, 3, 4, 5, 7, 7, 8, 9, 10, 12, 12, 13, 14, 15, 17}));
}

TEST(Matrix, InvertsTwoByTwo) {
  // [3 1; 2 2] has determinant 3*2 - 1*2 = 4, so its inverse is [2 -1; -2 3] / 4.
  const Matrix<2, 2> a{{3, 1, 2, 2}};
  const Matrix<2, 2> inv = kite::inverse(a);
  EXPECT_EQ(inv.values, (std::array<double, 4>{0.5, -0.25, -0.5, 0.75}));
  EXPECT_EQ((a * inv).values, kite::identity<2>().values);
}

TEST(Matrix, InverseRejectsSingularAndNonFiniteMatrices) {
  // [1 2; 2 4] has determinant 1*4 - 2*2 = 0.
  EXPECT_THROW(kite::inverse(Matrix<2, 2>{{1, 2, 2, 4}}), std::domain_error);
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(kite::inverse(Matrix<2, 2>{{inf, 0, 0, 1}}), std::domain_error);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(kite::inverse(Matrix<2, 2>{{nan, 0, 0, 1}}), std::domain_error);
}

}  // namespace
