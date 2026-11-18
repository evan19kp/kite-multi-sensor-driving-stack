#include "kite/assignment.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace {

using kite::Assignment;
using kite::CostMatrix;
using testing::ElementsAre;
using testing::IsEmpty;
using testing::Pair;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kMax = std::numeric_limits<double>::max();

std::vector<std::pair<std::size_t, std::size_t>> solve(const CostMatrix& cost, double gate) {
  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  for (const Assignment& a : kite::solve_assignment(cost, gate)) {
    pairs.emplace_back(a.row, a.column);
  }
  return pairs;
}

TEST(SolveAssignment, EmptyMatricesGiveNoPairs) {
  EXPECT_THAT(solve({0, 0, {}}, 1.0), IsEmpty());
  EXPECT_THAT(solve({0, 3, {}}, 1.0), IsEmpty());
  EXPECT_THAT(solve({3, 0, {}}, 1.0), IsEmpty());
}

TEST(SolveAssignment, GreedyByCheapestPairIsWrong) {
  EXPECT_THAT(solve({2, 2, {1, 2, 1.5, 100}}, kInf), ElementsAre(Pair(0, 1), Pair(1, 0)));
}

TEST(SolveAssignment, MorePairsBeatLowerCost) {
  EXPECT_THAT(solve({2, 2, {1, 4, 2, 9}}, 5.0), ElementsAre(Pair(0, 1), Pair(1, 0)));
}

TEST(SolveAssignment, OnlyPairsWithinTheGateAreReturned) {
  EXPECT_THAT(solve({2, 2, {1, 9, 9, 9}}, 5.0), ElementsAre(Pair(0, 0)));
  EXPECT_THAT(solve({2, 2, {6, 9, 9, 7}}, 5.0), IsEmpty());
  EXPECT_THAT(solve({1, 1, {5.0}}, 5.0), ElementsAre(Pair(0, 0)));
}

TEST(SolveAssignment, WideMatrixLeavesColumnsUnused) {
  EXPECT_THAT(solve({2, 3, {5, 1, 9, 5, 9, 9}}, 6.0), ElementsAre(Pair(0, 1), Pair(1, 0)));
}

TEST(SolveAssignment, TallMatrixLeavesRowsUnused) {
  EXPECT_THAT(solve({3, 2, {5, 5, 1, 9, 9, 9}}, 6.0), ElementsAre(Pair(0, 1), Pair(1, 0)));
}

TEST(SolveAssignment, NegativeGateAdmitsOnlyNegativeCosts) {
  const CostMatrix cost{2, 2, {-2, 1, 3, -5}};
  EXPECT_THAT(solve(cost, -1.0), ElementsAre(Pair(0, 0), Pair(1, 1)));
  EXPECT_THAT(solve(cost, -3.0), ElementsAre(Pair(1, 1)));
  EXPECT_THAT(solve(cost, -kInf), IsEmpty());
  EXPECT_THAT(solve({2, 2, {0, 1, 2, 3}}, -1.0), IsEmpty());
}

TEST(SolveAssignment, NegativeCostsMinimiseTotal) {
  EXPECT_THAT(solve({2, 2, {-1, -4, -3, -1}}, kInf), ElementsAre(Pair(0, 1), Pair(1, 0)));
}

TEST(SolveAssignment, InfiniteCostIsNeverAdmissible) {
  EXPECT_THAT(solve({2, 2, {kInf, 1, kInf, kInf}}, kInf), ElementsAre(Pair(0, 1)));
  EXPECT_THAT(solve({1, 1, {kInf}}, kMax), IsEmpty());
}

TEST(SolveAssignment, GateOfOneE308DoesNotOverflow) {
  EXPECT_THAT(solve({2, 2, {1e308, 1e307, 1e307, 1e308}}, 1e308),
              ElementsAre(Pair(0, 1), Pair(1, 0)));
  EXPECT_THAT(solve({2, 2, {1e308, 5.0, kInf, kInf}}, 1e308), ElementsAre(Pair(0, 1)));
}

TEST(SolveAssignment, InfiniteGateDoesNotOverflow) {
  EXPECT_THAT(solve({2, 2, {1e308, 1e307, 1e307, 1e308}}, kInf),
              ElementsAre(Pair(0, 1), Pair(1, 0)));
  EXPECT_THAT(solve({2, 2, {1e308, 2.0, 1.0, 1e308}}, kInf), ElementsAre(Pair(0, 1), Pair(1, 0)));
}

TEST(SolveAssignment, CostsNearTheLargestDoubleDoNotOverflow) {
  EXPECT_EQ(solve({3, 3, {kMax, kMax, kMax, kMax, kMax, kMax, kMax, kMax, kMax}}, kInf).size(), 3u);
  EXPECT_THAT(solve({2, 2, {kMax, 1.0, 2.0, kMax}}, kMax), ElementsAre(Pair(0, 1), Pair(1, 0)));
  EXPECT_THAT(solve({2, 2, {-kMax, kMax, kMax, -kMax}}, kInf), ElementsAre(Pair(0, 0), Pair(1, 1)));
}

TEST(SolveAssignment, HugeCostsStillRankByTotal) {
  EXPECT_THAT(solve({2, 2, {9e307, 1e307, 2e307, 9e307}}, kInf), ElementsAre(Pair(0, 1), Pair(1, 0)));
  EXPECT_THAT(solve({2, 2, {1e307, 9e307, 9e307, 2e307}}, kInf), ElementsAre(Pair(0, 0), Pair(1, 1)));
}

TEST(SolveAssignment, OppositeSignCostsNearTheLargestDoubleStillGiveTheMostPairs) {
  const CostMatrix cost{2, 2, {9e307, -9e307, 9e307, -1.1e308}};
  EXPECT_EQ(solve(cost, kInf).size(), 2u);
  EXPECT_EQ(solve(cost, kMax).size(), 2u);
}

CostMatrix five_by_five(double x, double b) {
  return {5, 5, {x, 0, b, 0, x, 0, 0, b, x, b, b, x, b, b, b, x, 0, b, b, b, b, 0, b, x, 0}};
}

TEST(SolveAssignment, NonNegativeCostsNearTheLargestDoubleStillGiveTheMostPairs) {
  EXPECT_EQ(solve(five_by_five(kMax / 2, kMax), kMax / 2).size(), 4u);
}

TEST(SolveAssignment, CostsUpToOneE300GiveTheLeastTotal) {
  EXPECT_THAT(solve(five_by_five(1e300, kMax), 1e300),
              ElementsAre(Pair(0, 3), Pair(1, 0), Pair(3, 1), Pair(4, 4)));
  EXPECT_THAT(solve(five_by_five(1e300, 2e300), 1e300),
              ElementsAre(Pair(0, 3), Pair(1, 0), Pair(3, 1), Pair(4, 4)));
}

TEST(SolveAssignment, TiedSolutionsGiveTheSameAnswerOnEveryCall) {
  const CostMatrix cost{3, 3, {1, 1, 1, 1, 1, 1, 1, 1, 1}};
  const auto first = solve(cost, kInf);
  for (int call = 0; call < 5; ++call) {
    EXPECT_EQ(solve(cost, kInf), first);
  }
}

TEST(SolveAssignment, SmallGateWithHugeOutOfGateCosts) {
  EXPECT_THAT(solve({2, 2, {kMax, 5.0, 1.0, kMax}}, 10.0), ElementsAre(Pair(0, 1), Pair(1, 0)));
}

TEST(SolveAssignment, NaNGateThrows) {
  EXPECT_THROW(kite::solve_assignment({1, 1, {1.0}}, kNaN), std::invalid_argument);
  EXPECT_THROW(kite::solve_assignment({0, 0, {}}, kNaN), std::invalid_argument);
}

TEST(SolveAssignment, NaNCostThrows) {
  EXPECT_THROW(kite::solve_assignment({2, 2, {1.0, 2.0, kNaN, 4.0}}, 10.0), std::invalid_argument);
  EXPECT_THROW(kite::solve_assignment({1, 1, {kNaN}}, kInf), std::invalid_argument);
}

TEST(SolveAssignment, WrongValueCountThrows) {
  EXPECT_THROW(kite::solve_assignment({2, 2, {1.0, 2.0, 3.0}}, 10.0), std::invalid_argument);
  EXPECT_THROW(kite::solve_assignment({0, 3, {1.0}}, 10.0), std::invalid_argument);
}

}  // namespace
