#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace kite {

// A fixed-size Rows x Columns matrix of doubles, stored row-major and zero-initialised.
template <std::size_t Rows, std::size_t Columns>
struct Matrix {
  std::array<double, Rows * Columns> values{};

  double& operator()(std::size_t row, std::size_t column) { return values[row * Columns + column]; }
  double operator()(std::size_t row, std::size_t column) const {
    return values[row * Columns + column];
  }
};

template <std::size_t N>
Matrix<N, N> identity() {
  Matrix<N, N> result;
  for (std::size_t i = 0; i < N; ++i) {
    result(i, i) = 1.0;
  }
  return result;
}

// Each entry is summed in index order k = 0, 1, ....
template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
Matrix<Rows, Columns> operator*(const Matrix<Rows, Inner>& a, const Matrix<Inner, Columns>& b) {
  Matrix<Rows, Columns> result;
  for (std::size_t i = 0; i < Rows; ++i) {
    for (std::size_t j = 0; j < Columns; ++j) {
      double sum = 0.0;
      for (std::size_t k = 0; k < Inner; ++k) {
        sum += a(i, k) * b(k, j);
      }
      result(i, j) = sum;
    }
  }
  return result;
}

template <std::size_t Rows, std::size_t Columns>
Matrix<Rows, Columns> operator+(const Matrix<Rows, Columns>& a, const Matrix<Rows, Columns>& b) {
  Matrix<Rows, Columns> result;
  for (std::size_t i = 0; i < Rows * Columns; ++i) {
    result.values[i] = a.values[i] + b.values[i];
  }
  return result;
}

template <std::size_t Rows, std::size_t Columns>
Matrix<Rows, Columns> operator-(const Matrix<Rows, Columns>& a, const Matrix<Rows, Columns>& b) {
  Matrix<Rows, Columns> result;
  for (std::size_t i = 0; i < Rows * Columns; ++i) {
    result.values[i] = a.values[i] - b.values[i];
  }
  return result;
}

template <std::size_t Rows, std::size_t Columns>
Matrix<Columns, Rows> transpose(const Matrix<Rows, Columns>& a) {
  Matrix<Columns, Rows> result;
  for (std::size_t i = 0; i < Rows; ++i) {
    for (std::size_t j = 0; j < Columns; ++j) {
      result(j, i) = a(i, j);
    }
  }
  return result;
}

// Throws std::domain_error when the determinant is zero or not finite.
inline Matrix<2, 2> inverse(const Matrix<2, 2>& a) {
  const double det = a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0);
  if (det == 0.0 || !std::isfinite(det)) {
    throw std::domain_error("inverse: matrix is singular or not finite");
  }
  Matrix<2, 2> result;
  result(0, 0) = a(1, 1) / det;
  result(0, 1) = -a(0, 1) / det;
  result(1, 0) = -a(1, 0) / det;
  result(1, 1) = a(0, 0) / det;
  return result;
}

}  // namespace kite
