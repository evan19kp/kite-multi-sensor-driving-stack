#include "kite/kalman.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace kite {

namespace {

void require_finite(Vec2 z, const char* where) {
  if (!std::isfinite(z.x) || !std::isfinite(z.y)) {
    throw std::invalid_argument(std::string(where) + ": measurement is not finite");
  }
}

template <typename OptionalEstimate>
auto& require_initialized(OptionalEstimate& estimate) {
  if (!estimate) {
    throw std::logic_error("ConstantVelocityKalman: not initialized");
  }
  return *estimate;
}

Matrix<2, 4> position_rows() {
  Matrix<2, 4> h;
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  return h;
}

Matrix<2, 2> measurement_noise(double r) {
  Matrix<2, 2> noise;
  noise(0, 0) = r;
  noise(1, 1) = r;
  return noise;
}

struct Innovation {
  Matrix<2, 1> y;
  Matrix<2, 2> s;
};

Innovation innovation(const Matrix<4, 1>& x, const Matrix<4, 4>& p, Vec2 z, double r) {
  const Matrix<2, 4> h = position_rows();
  const Matrix<2, 1> measured{{z.x, z.y}};
  return {measured - h * x, h * p * transpose(h) + measurement_noise(r)};
}

}  // namespace

ConstantVelocityKalman::ConstantVelocityKalman(double q, double measurement_sigma)
    : q_(q), r_(measurement_sigma * measurement_sigma) {
  if (!std::isfinite(q) || q < 0.0) {
    throw std::invalid_argument("ConstantVelocityKalman: q must be finite and >= 0");
  }
  if (!(measurement_sigma > 0.0) || !std::isfinite(r_) || r_ <= 0.0) {
    throw std::invalid_argument(
        "ConstantVelocityKalman: measurement_sigma must be > 0 with a finite, non-zero square");
  }
}

void ConstantVelocityKalman::initialize(Vec2 z, double velocity_sigma) {
  require_finite(z, "initialize");
  const double velocity_variance = velocity_sigma * velocity_sigma;
  if (!std::isfinite(velocity_variance) || velocity_sigma < 0.0) {
    throw std::invalid_argument("initialize: velocity_sigma must be >= 0 with a finite square");
  }
  Estimate estimate;
  estimate.x(0, 0) = z.x;
  estimate.x(1, 0) = z.y;
  estimate.p(0, 0) = r_;
  estimate.p(1, 1) = r_;
  estimate.p(2, 2) = velocity_variance;
  estimate.p(3, 3) = velocity_variance;
  estimate_ = estimate;
}

void ConstantVelocityKalman::predict(double dt) {
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument("predict: dt must be finite and >= 0");
  }
  Estimate& estimate = require_initialized(estimate_);
  if (dt == 0.0) {
    return;
  }

  Matrix<4, 4> f = identity<4>();
  f(0, 2) = dt;
  f(1, 3) = dt;

  const double dt2 = dt * dt;
  Matrix<4, 4> noise;
  for (std::size_t pos = 0; pos < 2; ++pos) {
    const std::size_t vel = pos + 2;
    noise(pos, pos) = q_ * dt2 * dt / 3.0;
    noise(pos, vel) = q_ * dt2 / 2.0;
    noise(vel, pos) = q_ * dt2 / 2.0;
    noise(vel, vel) = q_ * dt;
  }

  estimate.x = f * estimate.x;
  estimate.p = f * estimate.p * transpose(f) + noise;
}

void ConstantVelocityKalman::update(Vec2 z) {
  require_finite(z, "update");
  Estimate& estimate = require_initialized(estimate_);

  const Innovation inn = innovation(estimate.x, estimate.p, z, r_);
  const Matrix<2, 4> h = position_rows();
  const Matrix<4, 2> gain = estimate.p * transpose(h) * inverse(inn.s);
  const Matrix<4, 4> a = identity<4>() - gain * h;

  estimate.x = estimate.x + gain * inn.y;
  estimate.p = a * estimate.p * transpose(a) + gain * measurement_noise(r_) * transpose(gain);
}

double ConstantVelocityKalman::innovation_mahalanobis2(Vec2 z) const {
  require_finite(z, "innovation_mahalanobis2");
  const Estimate& estimate = require_initialized(estimate_);
  const Innovation inn = innovation(estimate.x, estimate.p, z, r_);
  return (transpose(inn.y) * inverse(inn.s) * inn.y)(0, 0);
}

const Matrix<4, 1>& ConstantVelocityKalman::state() const {
  return require_initialized(estimate_).x;
}

const Matrix<4, 4>& ConstantVelocityKalman::covariance() const {
  return require_initialized(estimate_).p;
}

}  // namespace kite
