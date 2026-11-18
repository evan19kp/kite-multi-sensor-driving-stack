#pragma once

#include <cstddef>
#include <vector>

namespace kite {

// A rows x columns table of costs, stored row-major: values[row * columns + column].
// Either dimension may be zero.
struct CostMatrix {
  std::size_t rows;
  std::size_t columns;
  std::vector<double> values;
};

struct Assignment {
  std::size_t row;
  std::size_t column;
};

// Pairs rows with distinct columns. A pair is admissible when its cost is finite and <= gate; only
// admissible pairs are returned, ordered by row.
//
// The result has as many pairs as possible, then the least total cost among those. Costs may be
// negative. Among equal-cost solutions the choice is deterministic but not specified.
//
// An infinite gate admits every finite cost. A negative gate is legal and admits only costs <= gate,
// which are negative. Infinite costs are never admissible. Gates and costs as large as DBL_MAX are
// admitted and ranked like any others.
//
// Throws std::invalid_argument for a NaN gate, a NaN cost, or values.size() != rows * columns.
std::vector<Assignment> solve_assignment(const CostMatrix& cost, double gate);

}  // namespace kite
