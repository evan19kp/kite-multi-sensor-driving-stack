#pragma once

#include <optional>

#include "kite/matrix.hpp"
#include "kite/types.hpp"

namespace kite {

// Kalman filter for one target moving at constant velocity in the plane. The state is
// [x, y, vx, vy] in m and m/s; a measurement is a position [x, y] in m.
//
// Motion: over dt seconds x' = F x with F = [[I, dt I], [0, I]], plus white-noise acceleration of
// spectral density q (m^2/s^3) on each axis, which adds Q = q * [[dt^3/3, dt^2/2], [dt^2/2, dt]]
// to that axis's (position, velocity) covariance block.
// Measurement: z = H x + noise with H = [I 0] and noise covariance R = sigma^2 I.
//
// The filter receives measurements only. It starts without an estimate: given valid arguments,
// predict, update, innovation_mahalanobis2, innovation_covariance, state and covariance throw
// std::logic_error until initialize is called. Invalid arguments throw std::invalid_argument first.
// update and innovation_mahalanobis2 throw std::domain_error when S = H P H^T + R has a zero or
// non-finite determinant.
class ConstantVelocityKalman {
 public:
  // Throws std::invalid_argument unless q is finite and >= 0, and measurement_sigma > 0 with
  // sigma^2 finite and non-zero.
  ConstantVelocityKalman(double q, double measurement_sigma);

  // Starts (or restarts) the estimate at position z with variance sigma^2 per axis, zero velocity
  // with variance velocity_sigma^2 per axis, and zero cross terms.
  // Throws std::invalid_argument for a non-finite z, or a velocity_sigma that is negative or whose
  // square is not finite.
  void initialize(Vec2 z, double velocity_sigma);

  // Advances the estimate by dt seconds. dt = 0 leaves state and covariance bit-identical.
  // Throws std::invalid_argument for a negative or non-finite dt.
  void predict(double dt);

  // Corrects the estimate with measurement z. The covariance uses the Joseph form
  // P = (I - K H) P (I - K H)^T + K R K^T. Throws std::invalid_argument for a non-finite z.
  void update(Vec2 z);

  // y^T S^-1 y with innovation y = z - H x and S = H P H^T + R; under the model it is chi-square
  // with 2 degrees of freedom. Throws std::invalid_argument for a non-finite z.
  double innovation_mahalanobis2(Vec2 z) const;

  // S = H P H^T + R, the innovation covariance that innovation_mahalanobis2 uses.
  Matrix<2, 2> innovation_covariance() const;

  const Matrix<4, 1>& state() const;
  const Matrix<4, 4>& covariance() const;

 private:
  struct Estimate {
    Matrix<4, 1> x;
    Matrix<4, 4> p;
  };

  double q_;
  double r_;
  std::optional<Estimate> estimate_;
};

}  // namespace kite
