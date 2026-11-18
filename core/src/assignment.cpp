#include "kite/assignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace kite {

namespace {

struct GatedCost {
  double outside_gate;
  double total;
};

GatedCost operator-(const GatedCost& a, const GatedCost& b) {
  return {a.outside_gate - b.outside_gate, a.total - b.total};
}

GatedCost& operator+=(GatedCost& a, const GatedCost& b) {
  a.outside_gate += b.outside_gate;
  a.total += b.total;
  return a;
}

GatedCost& operator-=(GatedCost& a, const GatedCost& b) {
  a.outside_gate -= b.outside_gate;
  a.total -= b.total;
  return a;
}

bool operator<(const GatedCost& a, const GatedCost& b) {
  return a.outside_gate < b.outside_gate || (a.outside_gate == b.outside_gate && a.total < b.total);
}

bool within_gate(double value, double gate) { return std::isfinite(value) && value <= gate; }

std::vector<std::size_t> min_cost_assignment(const std::vector<GatedCost>& cost, std::size_t n) {
  const double inf = std::numeric_limits<double>::infinity();
  const GatedCost unreached{inf, inf};
  std::vector<GatedCost> u(n + 1, GatedCost{0.0, 0.0});
  std::vector<GatedCost> v(n + 1, GatedCost{0.0, 0.0});
  std::vector<std::size_t> row_of_column(n + 1, 0);
  std::vector<std::size_t> way(n + 1, 0);
  for (std::size_t row = 1; row <= n; ++row) {
    row_of_column[0] = row;
    std::size_t column = 0;
    std::vector<GatedCost> min_slack(n + 1, unreached);
    std::vector<bool> used(n + 1, false);
    do {
      used[column] = true;
      const std::size_t r = row_of_column[column];
      GatedCost delta = unreached;
      std::size_t next = 0;
      for (std::size_t c = 1; c <= n; ++c) {
        if (used[c]) {
          continue;
        }
        const GatedCost slack = cost[(r - 1) * n + (c - 1)] - u[r] - v[c];
        if (slack < min_slack[c]) {
          min_slack[c] = slack;
          way[c] = column;
        }
        if (min_slack[c] < delta) {
          delta = min_slack[c];
          next = c;
        }
      }
      for (std::size_t c = 0; c <= n; ++c) {
        if (used[c]) {
          u[row_of_column[c]] += delta;
          v[c] -= delta;
        } else {
          min_slack[c] -= delta;
        }
      }
      column = next;
    } while (row_of_column[column] != 0);
    do {
      const std::size_t previous = way[column];
      row_of_column[column] = row_of_column[previous];
      column = previous;
    } while (column != 0);
  }

  std::vector<std::size_t> column_of_row(n, 0);
  for (std::size_t c = 1; c <= n; ++c) {
    column_of_row[row_of_column[c] - 1] = c - 1;
  }
  return column_of_row;
}

}  // namespace

std::vector<Assignment> solve_assignment(const CostMatrix& cost, double gate) {
  if (std::isnan(gate)) {
    throw std::invalid_argument("gate must not be NaN");
  }
  if (cost.values.size() != cost.rows * cost.columns) {
    throw std::invalid_argument("cost matrix size does not match rows * columns");
  }
  for (const double value : cost.values) {
    if (std::isnan(value)) {
      throw std::invalid_argument("cost must not be NaN");
    }
  }
  if (cost.rows == 0 || cost.columns == 0) {
    return {};
  }

  const std::size_t n = std::max(cost.rows, cost.columns);
  std::vector<GatedCost> padded(n * n, GatedCost{0.0, 0.0});
  for (std::size_t r = 0; r < cost.rows; ++r) {
    for (std::size_t c = 0; c < cost.columns; ++c) {
      const double value = cost.values[r * cost.columns + c];
      padded[r * n + c] = within_gate(value, gate) ? GatedCost{0.0, value} : GatedCost{1.0, 0.0};
    }
  }

  const std::vector<std::size_t> column_of_row = min_cost_assignment(padded, n);
  std::vector<Assignment> pairs;
  for (std::size_t r = 0; r < cost.rows; ++r) {
    const std::size_t c = column_of_row[r];
    if (c < cost.columns && within_gate(cost.values[r * cost.columns + c], gate)) {
      pairs.push_back({r, c});
    }
  }
  return pairs;
}

}  // namespace kite
