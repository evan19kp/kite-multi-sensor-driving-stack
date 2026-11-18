#include "kite/kalman.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>

#include <gtest/gtest.h>

#include "kite/generator.hpp"

namespace {

using kite::ConstantVelocityKalman;
using kite::Matrix;
using kite::Vec2;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// The raw-bits uniform documented in kite/generator.hpp: (raw >> 11) * 2^-53, in [0, 1).
double uniform01(std::mt19937_64& rng) {
  return static_cast<double>(rng() >> 11) * (1.0 / 9007199254740992.0);
}

// e^T P^-1 e = |w|^2 where P = L L^T and L w = e, since P^-1 = L^-T L^-1.
double normalised_error_squared(const Matrix<4, 1>& e, const Matrix<4, 4>& p) {
  const std::optional<Matrix<4, 4>> l = kite::cholesky(p);
  if (!l) {
    ADD_FAILURE() << "covariance is not positive definite";
    return kNan;
  }
  const Matrix<4, 1> w = kite::forward_substitute(*l, e);
  return (kite::transpose(w) * w)(0, 0);
}

double trace(const Matrix<4, 4>& p) { return p(0, 0) + p(1, 1) + p(2, 2) + p(3, 3); }

void construct(double q, double sigma) {
  ConstantVelocityKalman filter{q, sigma};
  (void)filter;
}

TEST(ConstantVelocityKalman, NoiselessPredictMovesPositionByVelocityTimesDt) {
  // Give the filter a non-zero velocity estimate from two measurements, then predict with q = 0.
  // F = [[I, dt I], [0, I]] maps (x, y, vx, vy) to (x + vx dt, y + vy dt, vx, vy). dt = 2 is a
  // power of two, so vx * dt is exact and x + vx * dt has a single rounding however it is computed.
  ConstantVelocityKalman filter{0.0, 1.0};
  filter.initialize({0.0, 0.0}, 5.0);
  filter.predict(1.0);
  filter.update({3.0, -2.0});
  const Matrix<4, 1> before = filter.state();
  ASSERT_NE(before(2, 0), 0.0);
  ASSERT_NE(before(3, 0), 0.0);

  filter.predict(2.0);
  const Matrix<4, 1>& after = filter.state();
  EXPECT_EQ(after(0, 0), before(0, 0) + before(2, 0) * 2.0);
  EXPECT_EQ(after(1, 0), before(1, 0) + before(3, 0) * 2.0);
  EXPECT_EQ(after(2, 0), before(2, 0));
  EXPECT_EQ(after(3, 0), before(3, 0));
}

TEST(ConstantVelocityKalman, PredictAddsWhiteNoiseAccelerationCovariance) {
  // sigma = 1 and velocity_sigma = 0 give P = diag(1, 1, 0, 0). With zero velocity variance,
  // F P F^T = P, so the predicted covariance is P + Q. Per axis, with q = 3 and dt = 2:
  //   Q = q [[dt^3/3, dt^2/2], [dt^2/2, dt]] = 3 [[8/3, 2], [2, 2]] = [[8, 6], [6, 6]].
  // In state order (x, y, vx, vy) the x block sits on indices 0 and 2, the y block on 1 and 3:
  //   P' = [[9, 0, 6, 0], [0, 9, 0, 6], [6, 0, 6, 0], [0, 6, 0, 6]].
  // Every value is a small integer, so the comparison is exact.
  ConstantVelocityKalman filter{3.0, 1.0};
  filter.initialize({0.0, 0.0}, 0.0);
  filter.predict(2.0);
  EXPECT_EQ(filter.covariance().values,
            (std::array<double, 16>{9, 0, 6, 0, 0, 9, 0, 6, 6, 0, 6, 0, 0, 6, 0, 6}));
}

TEST(ConstantVelocityKalman, PredictGrowsCovarianceAndUpdateShrinksIt) {
  // "Grows" and "shrinks" mean every diagonal entry (each variance) and the trace.
  //
  // Predict: P' = F P F^T + Q. Right after initialize the cross terms are zero, so per axis
  //   P'_pp = P_pp + dt^2 P_vv + q dt^3/3 and P'_vv = P_vv + q dt,
  // both strictly larger for dt > 0 and q > 0.
  //
  // Update: P+ = P - K S K^T with S positive definite, so no variance can grow. The position
  // variance falls by P_pp^2 / (P_pp + sigma^2) > 0. The velocity variance falls by
  // P_pv^2 / (P_pp + sigma^2), which is > 0 because the predict made P_pv = dt P_vv + q dt^2/2 > 0.
  ConstantVelocityKalman filter{0.5, 1.0};
  filter.initialize({0.0, 0.0}, 5.0);
  const Matrix<4, 4> initial = filter.covariance();

  filter.predict(1.0);
  const Matrix<4, 4> predicted = filter.covariance();
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_GT(predicted(i, i), initial(i, i)) << "diagonal " << i;
  }
  EXPECT_GT(trace(predicted), trace(initial));

  filter.update({1.0, 1.0});
  const Matrix<4, 4>& updated = filter.covariance();
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_LT(updated(i, i), predicted(i, i)) << "diagonal " << i;
  }
  EXPECT_LT(trace(updated), trace(predicted));
}

TEST(ConstantVelocityKalman, PredictWithZeroDtChangesNothing) {
  ConstantVelocityKalman filter{2.0, 0.7};
  filter.initialize({1.25, -3.5}, 4.0);
  filter.predict(0.3);
  filter.update({1.5, -3.25});
  const Matrix<4, 1> state = filter.state();
  const Matrix<4, 4> covariance = filter.covariance();

  filter.predict(0.0);
  EXPECT_EQ(std::memcmp(state.values.data(), filter.state().values.data(), sizeof(state.values)),
            0);
  EXPECT_EQ(std::memcmp(covariance.values.data(), filter.covariance().values.data(),
                        sizeof(covariance.values)),
            0);
}

TEST(ConstantVelocityKalman, UpdateWithEqualWeightsAveragesPriorAndMeasurement) {
  // Prior position (0, 0) with variance 1 and a measurement (2, -4) with variance 1. Per axis
  // K = 1 / (1 + 1) = 1/2, so the estimate is the average (1, -2), and the Joseph form gives
  // (1 - K)^2 * 1 + K^2 * 1 = 1/2. With zero velocity variance and zero cross terms the velocity
  // and its variance stay 0. All values are exact in binary.
  ConstantVelocityKalman filter{0.0, 1.0};
  filter.initialize({0.0, 0.0}, 0.0);
  filter.update({2.0, -4.0});
  EXPECT_EQ(filter.state().values, (std::array<double, 4>{1, -2, 0, 0}));
  EXPECT_EQ(filter.covariance().values,
            (std::array<double, 16>{0.5, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

TEST(ConstantVelocityKalman, InnovationMahalanobisDistance) {
  // sigma = 2: P_pos = 4 I and R = 4 I, so S = 8 I. A measurement 4 m from the estimate along x
  // has y = (4, 0) and y^T S^-1 y = 16 / 8 = 2. One 1 m off along each axis gives (1 + 1) / 8.
  ConstantVelocityKalman filter{1.0, 2.0};
  filter.initialize({1.0, 1.0}, 3.0);
  EXPECT_EQ(filter.innovation_mahalanobis2({5.0, 1.0}), 2.0);
  EXPECT_EQ(filter.innovation_mahalanobis2({2.0, 0.0}), 0.25);
  EXPECT_EQ(filter.innovation_mahalanobis2({1.0, 1.0}), 0.0);
}

// Runs 1,000 random steps (each a predict with dt in [0, 2) s or an update with a position in
// [-100, 100) m, with equal odds) and checks after every step that P is symmetric and positive
// definite.
//
// The Joseph form is symmetric in exact arithmetic but not bit for bit: A P A^T sums entry (i, j)
// and entry (j, i) in different orders. So symmetry is checked to a tolerance:
//   |P_ij - P_ji| <= 1e-12 * sqrt(P_ii P_jj),
// where sqrt(P_ii P_jj) bounds |P_ij| for any covariance (Cauchy-Schwarz). Rounding alone leaves
// asymmetries near 1e-16 (both tests print the largest), so 1e-12 has four orders of margin, yet
// the simple form P - K H P reaches about 6e-11 with the precise sensor and fails.
double largest_asymmetry_over_random_steps(double q, double sigma, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  ConstantVelocityKalman filter{q, sigma};
  filter.initialize({0.0, 0.0}, 7.36);
  double worst = 0.0;
  for (int step = 0; step < 1000; ++step) {
    if (uniform01(rng) < 0.5) {
      filter.predict(2.0 * uniform01(rng));
    } else {
      const double x = 200.0 * uniform01(rng) - 100.0;
      const double y = 200.0 * uniform01(rng) - 100.0;
      filter.update({x, y});
    }
    const Matrix<4, 4>& p = filter.covariance();
    for (std::size_t i = 0; i < 4; ++i) {
      for (std::size_t j = i + 1; j < 4; ++j) {
        const double scale = std::sqrt(p(i, i) * p(j, j));
        const double asymmetry = std::abs(p(i, j) - p(j, i)) / scale;
        worst = asymmetry > worst ? asymmetry : worst;
        EXPECT_LE(asymmetry, 1e-12) << "step " << step << " entry " << i << "," << j;
      }
    }
    EXPECT_TRUE(kite::cholesky(p).has_value()) << "not positive definite after step " << step;
    if (testing::Test::HasFailure()) {
      break;
    }
  }
  return worst;
}

TEST(ConstantVelocityKalman, StaysSymmetricPositiveDefiniteOverRandomSteps) {
  const double worst = largest_asymmetry_over_random_steps(1.0, 1.0, 20261401);
  std::cout << "largest relative asymmetry: " << worst << "\n";
}

TEST(ConstantVelocityKalman, StaysSymmetricPositiveDefiniteWithAPreciseSensor) {
  // A sensor far more precise than the prediction (sigma = 1e-4 m against a predicted position
  // spread of metres) is where P - K H P loses symmetry to rounding: it subtracts two nearly equal
  // numbers. The Joseph form adds two positive semi-definite terms instead.
  const double worst = largest_asymmetry_over_random_steps(100.0, 1e-4, 20261402);
  std::cout << "largest relative asymmetry: " << worst << "\n";
}

TEST(ConstantVelocityKalman, StationaryTargetConvergesToTruePosition) {
  // A target standing at (12.5, -7.25) is measured N = 1000 times, dt = 0.1 s apart, with noise
  // uniform in [-sqrt(3), sqrt(3)) m per axis: mean 0, variance sigma^2 = 1 (a uniform of
  // half-width a has variance a^2/3).
  //
  // With q = 0 the filter is the least-squares fit of a straight line x(t) = x_N + v (t - t_N) to
  // the N measurements (initialize uses the first measurement with its own variance, so it counts
  // as one of them; the velocity prior, variance 100 against data information
  // sum (t_i - t_mean)^2 / sigma^2 = 0.01 * N (N^2 - 1) / 12 = 833,332.5, changes the result by
  // about 1e-8 relative). For equally spaced points the line-fit variances are
  //   Var(x_N) = sigma^2 (4N - 2) / (N (N + 1)) = 0.0039940 m^2  (standard deviation 0.0632 m),
  //   Var(v)   = sigma^2 / 833,332.5           = 1.2000e-6 (m/s)^2  (standard deviation 0.0011 m/s).
  // The filter's covariance must match both to 0.1 %. The estimate must be within 0.25 m of the
  // true position (about 4 standard deviations) and each velocity component within 0.005 m/s
  // (about 4.6).
  constexpr int kN = 1000;
  constexpr double kDt = 0.1;
  const Vec2 truth{12.5, -7.25};
  const double half_width = std::sqrt(3.0);
  std::mt19937_64 rng(20261403);
  const auto measure = [&]() {
    const double nx = (2.0 * uniform01(rng) - 1.0) * half_width;
    const double ny = (2.0 * uniform01(rng) - 1.0) * half_width;
    return Vec2{truth.x + nx, truth.y + ny};
  };

  ConstantVelocityKalman filter{0.0, 1.0};
  filter.initialize(measure(), 10.0);
  for (int i = 1; i < kN; ++i) {
    filter.predict(kDt);
    filter.update(measure());
  }

  const double n = kN;
  const double position_variance = (4.0 * n - 2.0) / (n * (n + 1.0));
  const double velocity_variance = 1.0 / (kDt * kDt * n * (n * n - 1.0) / 12.0);
  const Matrix<4, 4>& p = filter.covariance();
  EXPECT_NEAR(p(0, 0), position_variance, 1e-3 * position_variance);
  EXPECT_NEAR(p(1, 1), position_variance, 1e-3 * position_variance);
  EXPECT_NEAR(p(2, 2), velocity_variance, 1e-3 * velocity_variance);
  EXPECT_NEAR(p(3, 3), velocity_variance, 1e-3 * velocity_variance);

  const Matrix<4, 1>& x = filter.state();
  EXPECT_NEAR(x(0, 0), truth.x, 0.25);
  EXPECT_NEAR(x(1, 0), truth.y, 0.25);
  EXPECT_NEAR(x(2, 0), 0.0, 0.005);
  EXPECT_NEAR(x(3, 0), 0.0, 0.005);
}

// Chi-square quantile by the Wilson-Hilferty approximation: (X / k)^(1/3) is close to normal with
// mean 1 - 2/(9k) and variance 2/(9k), so the p-quantile is k (1 - 2/(9k) + z_p sqrt(2/(9k)))^3.
double wilson_hilferty(double k, double z) {
  const double c = 1.0 - 2.0 / (9.0 * k) + z * std::sqrt(2.0 / (9.0 * k));
  return k * c * c * c;
}

// Exact chi-square CDF P(k/2, x/2), the regularised lower incomplete gamma function, by its power
// series P(a, x) = x^a e^-x / Gamma(a) * sum_n x^n / (a (a + 1) ... (a + n)).
// For x far above a the sum overflows while the prefactor underflows.
double chi_square_cdf(double k, double x) {
  const double a = k / 2.0;
  const double half = x / 2.0;
  double term = 1.0 / a;
  double sum = term;
  for (int n = 1; term > 1e-17 * sum; ++n) {
    term *= half / (a + n);
    sum += term;
  }
  const double cdf = std::exp(a * std::log(half) - half - std::lgamma(a)) * sum;
  if (!std::isfinite(cdf)) {
    throw std::domain_error("chi_square_cdf: series is not finite");
  }
  return cdf;
}

double chi_square_quantile(double k, double p) {
  double lo = 0.0;
  double hi = 2.0 * k;
  if (chi_square_cdf(k, hi) < p) {
    throw std::domain_error("chi_square_quantile: 2k does not bracket the quantile");
  }
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (chi_square_cdf(k, mid) < p) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

constexpr int kRuns = 200;
constexpr double kDegrees = 4.0 * kRuns;
constexpr double kStandardNormalQuantile975 = 1.959963984540054;

TEST(ConstantVelocityKalman, WilsonHilfertyBoundsMatchExactChiSquareQuantiles) {
  const double lower = wilson_hilferty(kDegrees, -kStandardNormalQuantile975);
  const double upper = wilson_hilferty(kDegrees, kStandardNormalQuantile975);
  const double exact_lower = chi_square_quantile(kDegrees, 0.025);
  const double exact_upper = chi_square_quantile(kDegrees, 0.975);
  std::cout << "chi-square(" << kDegrees << ") 2.5 % / 97.5 %: Wilson-Hilferty " << lower << " / "
            << upper << ", exact " << exact_lower << " / " << exact_upper << "\n";
  EXPECT_NEAR(lower, exact_lower, 1e-4 * exact_lower);
  EXPECT_NEAR(upper, exact_upper, 1e-4 * exact_upper);
}

constexpr double kGeneratedSigma = 1.0;

// The generator's vehicles move at exactly constant velocity, so the true process noise is zero.
// Any q > 0 would make P larger than the real error covariance and push NEES below 4.
constexpr double kGeneratedQ = 0.0;

// Start speeds s are uniform in [5, 15) m/s and headings uniform, so per axis
//   E[vx] = 0, E[vx vy] = 0, and E[vx^2] = E[s^2] E[cos^2] = (Var s + (E s)^2) / 2
//   = (100/12 + 100) / 2 = 325/6 = 54.17 (m/s)^2, so velocity_sigma = sqrt(325/6) = 7.36 m/s.
// The zero-velocity prior then has the right mean and covariance, and the start position is the
// first measurement, whose error is the measurement noise.
constexpr double kGeneratedVelocityVariance = 325.0 / 6.0;

// Truth: kite::generate with one vehicle, detection probability 1 (one measurement per step),
// noise sigma = 1 m, dt = 0.1 s, 5 s (steps 0..50).
kite::GeneratedScenario generated_scenario(std::uint64_t seed) {
  return kite::generate({seed, 1, 5.0, 0.1, kGeneratedSigma, 1.0, 100.0});
}

// Initializes the filter at step 0's measurement, then predicts and updates through step 50.
void filter_generated_scenario(const kite::GeneratedScenario& scenario,
                               ConstantVelocityKalman& filter) {
  ASSERT_EQ(scenario.steps.size(), 51u);
  ASSERT_EQ(scenario.steps[0].measurements.size(), 1u);
  const kite::Measurement* previous = &scenario.steps[0].measurements[0];
  filter.initialize(previous->position, std::sqrt(kGeneratedVelocityVariance));
  for (std::size_t k = 1; k < scenario.steps.size(); ++k) {
    ASSERT_EQ(scenario.steps[k].measurements.size(), 1u);
    const kite::Measurement& m = scenario.steps[k].measurements[0];
    filter.predict(m.t - previous->t);
    filter.update(m.position);
    previous = &m;
  }
}

// Bit-exact pins, checked on every CI platform (Linux g++ and MSVC). The generator uses only
// IEEE-exact operations (see kite/generator.hpp), the filter only +, -, * and /, and the NEES adds
// one sqrt per Cholesky pivot. Each is correctly rounded and their order is fixed, so with
// floating-point contraction off in kite_core and in this test target the bits cannot differ
// between platforms. The values were printed with %a by a MinGW g++ 13.2 build.
TEST(ConstantVelocityKalman, GeneratedRunMatchesPinnedBits) {
  const kite::GeneratedScenario scenario = generated_scenario(20261500);
  ConstantVelocityKalman filter{kGeneratedQ, kGeneratedSigma};
  ASSERT_NO_FATAL_FAILURE(filter_generated_scenario(scenario, filter));
  EXPECT_EQ(filter.state().values,
            (std::array<double, 4>{-0x1.5b6412d037e57p+4, -0x1.ffd0061377759p+5,
                                   0x1.5bc153c260c75p+3, 0x1.30839625bffd7p+2}));
  constexpr double kPosition = 0x1.37f300fa0e64ap-4;
  constexpr double kPositionVelocity = 0x1.729de775fced5p-6;
  constexpr double kVelocityPosition = 0x1.729de775fced6p-6;
  constexpr double kVelocity = 0x1.287e52c4ca579p-7;
  EXPECT_EQ(filter.covariance().values,
            (std::array<double, 16>{kPosition, 0, kPositionVelocity, 0,
                                    0, kPosition, 0, kPositionVelocity,
                                    kVelocityPosition, 0, kVelocity, 0,
                                    0, kVelocityPosition, 0, kVelocity}));
}

TEST(ConstantVelocityKalman, AverageNeesOnGeneratedScenariosIsInsideChiSquareBounds) {
  // Why evaluate at step 50: the Kalman filter is the best linear estimator, so with a prior of
  // the right mean and covariance P equals the true error covariance, Gaussian or not, and
  // E[NEES] = 4 at every step. The chi-square bounds also need the error to be Gaussian. The
  // velocity prior is not (speeds lie on a ring), but its weight in the estimate shrinks as
  // measurements arrive. After 51 measurements the data alone pins velocity to variance
  //   sigma^2 / sum (t_i - t_mean)^2 = 1 / (0.01 * 51 * (51^2 - 1) / 12) = 1 / 110.5
  //   = 0.0090 (m/s)^2,
  // about 1/6000 of the prior's 54.17, so the error is the Gaussian measurement noise's to within
  // that fraction.
  //
  // Bounds: the N = 200 runs use independent seeds, so N * (average NEES) ~ chi-square(4N = 800).
  // The 95 %
  // two-sided interval for the average is [chi2(0.025; 4N), chi2(0.975; 4N)] / N, computed by
  // Wilson-Hilferty; WilsonHilfertyBoundsMatchExactChiSquareQuantiles checks it agrees with the
  // exact quantiles to 1e-4 relative.
  const double lower = wilson_hilferty(kDegrees, -kStandardNormalQuantile975) / kRuns;
  const double upper = wilson_hilferty(kDegrees, kStandardNormalQuantile975) / kRuns;

  double total = 0.0;
  for (int run = 0; run < kRuns; ++run) {
    const std::uint64_t seed = 20261500 + static_cast<std::uint64_t>(run);
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    const kite::GeneratedScenario scenario = generated_scenario(seed);
    ConstantVelocityKalman filter{kGeneratedQ, kGeneratedSigma};
    ASSERT_NO_FATAL_FAILURE(filter_generated_scenario(scenario, filter));

    const kite::GroundTruthState& truth = scenario.steps.back().truth.at(0);
    const Matrix<4, 1>& x = filter.state();
    const Matrix<4, 1> error{{x(0, 0) - truth.position.x, x(1, 0) - truth.position.y,
                              x(2, 0) - truth.velocity.x, x(3, 0) - truth.velocity.y}};
    total += normalised_error_squared(error, filter.covariance());
  }

  const double average = total / kRuns;
  std::cout << "average NEES over " << kRuns << " runs: " << average << ", 95 % bounds [" << lower
            << ", " << upper << "]\n";
  EXPECT_GE(average, lower);
  EXPECT_LE(average, upper);
  // Pinned like GeneratedRunMatchesPinnedBits.
  EXPECT_EQ(average, 0x1.00b60e3d61dcp+2);
}

TEST(ConstantVelocityKalman, ConstructorRejectsInvalidParameters) {
  EXPECT_NO_THROW(construct(0.0, 1.0));
  EXPECT_THROW(construct(-1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(construct(kNan, 1.0), std::invalid_argument);
  EXPECT_THROW(construct(kInf, 1.0), std::invalid_argument);
  EXPECT_THROW(construct(1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(construct(1.0, -1.0), std::invalid_argument);
  EXPECT_THROW(construct(1.0, kNan), std::invalid_argument);
  EXPECT_THROW(construct(1.0, kInf), std::invalid_argument);
  EXPECT_THROW(construct(1.0, 1e200), std::invalid_argument);
  EXPECT_THROW(construct(1.0, 1e-200), std::invalid_argument);
}

TEST(ConstantVelocityKalman, EverythingButInitializeThrowsBeforeInitialize) {
  ConstantVelocityKalman filter{1.0, 1.0};
  EXPECT_THROW(filter.predict(1.0), std::logic_error);
  EXPECT_THROW(filter.update({0.0, 0.0}), std::logic_error);
  EXPECT_THROW(filter.innovation_mahalanobis2({0.0, 0.0}), std::logic_error);
  EXPECT_THROW(filter.state(), std::logic_error);
  EXPECT_THROW(filter.covariance(), std::logic_error);
  filter.initialize({0.0, 0.0}, 1.0);
  EXPECT_NO_THROW(filter.predict(1.0));
}

TEST(ConstantVelocityKalman, InitializeAgainRestartsTheEstimate) {
  // A second initialize with sigma = 2 and velocity_sigma = 0.5 must give state (-4, 6, 0, 0) and
  // P = diag(4, 4, 0.25, 0.25), as on a new filter.
  ConstantVelocityKalman filter{1.0, 2.0};
  filter.initialize({1.0, 1.0}, 3.0);
  filter.predict(0.5);
  filter.update({2.0, 0.0});
  ASSERT_NE(filter.state()(2, 0), 0.0);
  ASSERT_NE(filter.covariance()(0, 2), 0.0);

  filter.initialize({-4.0, 6.0}, 0.5);
  EXPECT_EQ(filter.state().values, (std::array<double, 4>{-4, 6, 0, 0}));
  EXPECT_EQ(filter.covariance().values,
            (std::array<double, 16>{4, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0.25}));
}

TEST(ConstantVelocityKalman, FailedFirstInitializeLeavesTheFilterUninitialized) {
  ConstantVelocityKalman filter{1.0, 1.0};
  EXPECT_THROW(filter.initialize({kNan, 0.0}, 1.0), std::invalid_argument);
  EXPECT_THROW(filter.initialize({0.0, 0.0}, -1.0), std::invalid_argument);
  EXPECT_THROW(filter.state(), std::logic_error);
  EXPECT_THROW(filter.covariance(), std::logic_error);
}

TEST(ConstantVelocityKalman, FailedInitializeLeavesTheEstimateUnchanged) {
  ConstantVelocityKalman filter{1.0, 2.0};
  filter.initialize({1.0, 1.0}, 3.0);
  filter.predict(0.5);
  filter.update({2.0, 0.0});
  const Matrix<4, 1> state = filter.state();
  const Matrix<4, 4> covariance = filter.covariance();

  EXPECT_THROW(filter.initialize({0.0, kInf}, 1.0), std::invalid_argument);
  EXPECT_THROW(filter.initialize({-4.0, 6.0}, kNan), std::invalid_argument);
  EXPECT_EQ(filter.state().values, state.values);
  EXPECT_EQ(filter.covariance().values, covariance.values);
}

TEST(ConstantVelocityKalman, NonFiniteOrNegativeInputThrows) {
  ConstantVelocityKalman filter{1.0, 1.0};
  EXPECT_THROW(filter.initialize({kNan, 0.0}, 1.0), std::invalid_argument);
  EXPECT_THROW(filter.initialize({0.0, kInf}, 1.0), std::invalid_argument);
  EXPECT_THROW(filter.initialize({0.0, 0.0}, -1.0), std::invalid_argument);
  EXPECT_THROW(filter.initialize({0.0, 0.0}, kNan), std::invalid_argument);
  EXPECT_THROW(filter.initialize({0.0, 0.0}, kInf), std::invalid_argument);
  EXPECT_THROW(filter.initialize({0.0, 0.0}, 1e200), std::invalid_argument);

  filter.initialize({0.0, 0.0}, 1.0);
  const Matrix<4, 1> state = filter.state();
  const Matrix<4, 4> covariance = filter.covariance();
  EXPECT_THROW(filter.predict(-0.1), std::invalid_argument);
  EXPECT_THROW(filter.predict(kNan), std::invalid_argument);
  EXPECT_THROW(filter.predict(kInf), std::invalid_argument);
  EXPECT_THROW(filter.update({kNan, 0.0}), std::invalid_argument);
  EXPECT_THROW(filter.update({0.0, -kInf}), std::invalid_argument);
  EXPECT_THROW(filter.innovation_mahalanobis2({kInf, 0.0}), std::invalid_argument);
  EXPECT_THROW(filter.innovation_mahalanobis2({0.0, kNan}), std::invalid_argument);
  EXPECT_EQ(filter.state().values, state.values);
  EXPECT_EQ(filter.covariance().values, covariance.values);
}

TEST(ConstantVelocityKalman, OverflowedCovarianceThrowsDomainError) {
  // dt = 1e200 is finite, but dt^2 = 1e400 overflows, so P and then det S are not finite.
  ConstantVelocityKalman filter{1.0, 1.0};
  filter.initialize({0.0, 0.0}, 1.0);
  filter.predict(1e200);
  EXPECT_THROW(filter.update({0.0, 0.0}), std::domain_error);
  EXPECT_THROW(filter.innovation_mahalanobis2({0.0, 0.0}), std::domain_error);
}

}  // namespace
