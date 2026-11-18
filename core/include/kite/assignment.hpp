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
// The result has as many pairs as possible for any gate, including infinity and DBL_MAX. Among
// those it has the least total cost on the inputs the tests check: a brute-force search with costs
// in [0, 10), and hand-made cases with costs up to 1e300. With costs as large as DBL_MAX / 2 the
// total may not be the least, for example costs of 0 and DBL_MAX / 2 under a gate of DBL_MAX / 2.
// Among equal-cost solutions the choice is deterministic but not specified.
//
// An infinite gate admits every finite cost. A negative gate admits nothing. An infinite cost is
// never admissible.
//
// Throws std::invalid_argument for a NaN gate, a negative or NaN cost, or
// values.size() != rows * columns.
std::vector<Assignment> solve_assignment(const CostMatrix& cost, double gate);

}  // namespace kite
