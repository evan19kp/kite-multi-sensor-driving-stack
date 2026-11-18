#include "kite/assignment.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "test_helpers.hpp"

namespace {

using kite::Assignment;
using kite::CostMatrix;
using kite::test_helpers::kInf;
using kite::test_helpers::uniform01;

constexpr double kTolerance = 1e-9;
constexpr std::uint64_t kBaseSeed = 20260000;
constexpr int kCases = 5000;

struct Best {
  std::size_t pairs = 0;
  double cost = 0.0;
};

bool admissible(double value, double gate) { return std::isfinite(value) && value <= gate; }

void search(const CostMatrix& matrix, double gate, std::size_t row, std::vector<bool>& column_used,
            std::size_t pairs, double cost, Best& best) {
  if (row == matrix.rows) {
    if (pairs > best.pairs || (pairs == best.pairs && cost < best.cost)) {
      best = {pairs, cost};
    }
    return;
  }
  search(matrix, gate, row + 1, column_used, pairs, cost, best);
  for (std::size_t column = 0; column < matrix.columns; ++column) {
    const double value = matrix.values[row * matrix.columns + column];
    if (column_used[column] || !admissible(value, gate)) {
      continue;
    }
    column_used[column] = true;
    search(matrix, gate, row + 1, column_used, pairs + 1, cost + value, best);
    column_used[column] = false;
  }
}

Best brute_force(const CostMatrix& matrix, double gate) {
  Best best;
  std::vector<bool> column_used(matrix.columns, false);
  search(matrix, gate, 0, column_used, 0, 0.0, best);
  return best;
}

void expect_matches_brute_force(const CostMatrix& matrix, double gate) {
  const std::vector<Assignment> pairs = kite::solve_assignment(matrix, gate);

  std::set<std::size_t> rows;
  std::set<std::size_t> columns;
  double total = 0.0;
  for (const Assignment& pair : pairs) {
    ASSERT_LT(pair.row, matrix.rows);
    ASSERT_LT(pair.column, matrix.columns);
    EXPECT_TRUE(rows.insert(pair.row).second) << "row " << pair.row << " used twice";
    EXPECT_TRUE(columns.insert(pair.column).second) << "column " << pair.column << " used twice";
    const double value = matrix.values[pair.row * matrix.columns + pair.column];
    EXPECT_TRUE(admissible(value, gate)) << "pair outside the gate, cost " << value;
    total += value;
  }

  const Best expected = brute_force(matrix, gate);
  EXPECT_EQ(pairs.size(), expected.pairs);
  EXPECT_NEAR(total, expected.cost, kTolerance);
}

TEST(AssignmentBruteForce, FiveThousandSeededRandomCases) {
  for (int i = 0; i < kCases; ++i) {
    const std::uint64_t seed = kBaseSeed + static_cast<std::uint64_t>(i);
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    std::mt19937_64 rng(seed);

    const std::size_t rows = rng() % 7;
    const std::size_t columns = rng() % 7;
    const bool whole_numbers = rng() % 2 == 0;
    CostMatrix matrix{rows, columns, std::vector<double>(rows * columns, 0.0)};
    for (double& value : matrix.values) {
      value = whole_numbers ? static_cast<double>(rng() % 6) : 10.0 * uniform01(rng);
      if (rng() % 25 == 0) {
        value = kInf;
      }
    }

    double gate = 0.0;
    switch (rng() % 6) {
      case 0:
        gate = -1.0;
        break;
      case 1:
        gate = 0.0;
        break;
      case 2:
        gate = 10.0 * uniform01(rng);
        break;
      case 3:
        gate = 3.0 * uniform01(rng);
        break;
      case 4:
        gate = 10.0;
        break;
      default:
        gate = kInf;
        break;
    }

    expect_matches_brute_force(matrix, gate);
    if (testing::Test::HasFatalFailure() || testing::Test::HasNonfatalFailure()) {
      return;
    }
  }
}

TEST(AssignmentBruteForce, EmptyMatricesHaveNoPairs) {
  expect_matches_brute_force({0, 0, {}}, 1.0);
  expect_matches_brute_force({0, 4, {}}, kInf);
  expect_matches_brute_force({4, 0, {}}, kInf);
}

TEST(AssignmentBruteForce, WideMatrix) {
  expect_matches_brute_force({2, 5, {4, 1, 7, 3, 9, 2, 8, 1, 6, 5}}, 8.0);
}

TEST(AssignmentBruteForce, TallMatrix) {
  expect_matches_brute_force({5, 2, {4, 2, 1, 8, 7, 1, 3, 6, 9, 5}}, 8.0);
}

TEST(AssignmentBruteForce, SingleCell) {
  expect_matches_brute_force({1, 1, {3.0}}, 3.0);
  expect_matches_brute_force({1, 1, {3.0}}, 2.9);
}

}  // namespace
