#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
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

// Lower-triangular L with a = L L^T, read from the lower triangle of a; nullopt when a is not
// positive definite.
template <std::size_t N>
std::optional<Matrix<N, N>> cholesky(const Matrix<N, N>& a) {
  Matrix<N, N> l;
  for (std::size_t j = 0; j < N; ++j) {
    double pivot = a(j, j);
    for (std::size_t k = 0; k < j; ++k) {
      pivot -= l(j, k) * l(j, k);
    }
    if (!(pivot > 0.0)) {
      return std::nullopt;
    }
    l(j, j) = std::sqrt(pivot);
    for (std::size_t i = j + 1; i < N; ++i) {
      double sum = a(i, j);
      for (std::size_t k = 0; k < j; ++k) {
        sum -= l(i, k) * l(j, k);
      }
      l(i, j) = sum / l(j, j);
    }
  }
  return l;
}

// Solves L w = b for lower-triangular L with a non-zero diagonal.
template <std::size_t N>
Matrix<N, 1> forward_substitute(const Matrix<N, N>& lower, const Matrix<N, 1>& b) {
  Matrix<N, 1> w;
  for (std::size_t i = 0; i < N; ++i) {
    double sum = b(i, 0);
    for (std::size_t k = 0; k < i; ++k) {
      sum -= lower(i, k) * w(k, 0);
    }
    w(i, 0) = sum / lower(i, i);
  }
  return w;
}

}  // namespace kite
