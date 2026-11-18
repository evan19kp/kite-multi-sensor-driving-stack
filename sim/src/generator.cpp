#include "kite/generator.hpp"

#include <cmath>
#include <cstddef>
#include <random>
#include <stdexcept>
#include <utility>

namespace kite {

namespace {

constexpr double kTwoPowMinus53 = 1.0 / 9007199254740992.0;
constexpr double kLn2 = 0.693147180559945309417232121458;
constexpr double kSqrtHalf = 0.70710678118654752440;
constexpr double kMinDiscRadiusSquared = 1e-12;
constexpr int kLnTerms = 14;
constexpr double kMinSpeed = 5.0;
constexpr double kMaxSpeed = 15.0;
constexpr double kMaxSteps = 1e9;

class Rng {
 public:
  explicit Rng(std::uint64_t seed) : engine_(seed) {}

  double uniform01() {
    return static_cast<double>(engine_() >> 11) * kTwoPowMinus53;
  }

  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform01(); }

  std::uint64_t below(std::uint64_t n) {
    const std::uint64_t reject_below = (std::uint64_t{0} - n) % n;
    std::uint64_t raw = engine_();
    while (raw < reject_below) {
      raw = engine_();
    }
    return raw % n;
  }

 private:
  std::mt19937_64 engine_;
};

struct DiscPoint {
  double u;
  double v;
  double radius_squared;
};

DiscPoint disc_point(Rng& rng) {
  for (;;) {
    const double u = 2.0 * rng.uniform01() - 1.0;
    const double v = 2.0 * rng.uniform01() - 1.0;
    const double s = u * u + v * v;
    if (s < 1.0 && s >= kMinDiscRadiusSquared) {
      return {u, v, s};
    }
  }
}

double natural_log(double x) {
  int exponent = 0;
  double mantissa = std::frexp(x, &exponent);
  if (mantissa < kSqrtHalf) {
    mantissa *= 2.0;
    --exponent;
  }
  const double z = (mantissa - 1.0) / (mantissa + 1.0);
  const double z_squared = z * z;
  double series = 0.0;
  for (int k = kLnTerms - 1; k >= 0; --k) {
    series = series * z_squared + 1.0 / static_cast<double>(2 * k + 1);
  }
  return 2.0 * z * series + static_cast<double>(exponent) * kLn2;
}

Vec2 standard_normal_pair(Rng& rng) {
  const DiscPoint p = disc_point(rng);
  const double factor = std::sqrt(-2.0 * natural_log(p.radius_squared) / p.radius_squared);
  return {p.u * factor, p.v * factor};
}

void shuffle(std::vector<Measurement>& items, Rng& rng) {
  for (std::size_t i = items.size(); i > 1; --i) {
    const auto j = static_cast<std::size_t>(rng.below(i));
    std::swap(items[i - 1], items[j]);
  }
}

void validate(const ScenarioConfig& config) {
  const bool finite = std::isfinite(config.duration_s) && std::isfinite(config.dt_s) &&
                      std::isfinite(config.position_noise_m) &&
                      std::isfinite(config.detection_probability) &&
                      std::isfinite(config.area_half_width_m);
  if (!finite) {
    throw std::invalid_argument("scenario config has a non-finite value");
  }
  if (config.vehicles < 0) {
    throw std::invalid_argument("vehicles must be >= 0");
  }
  if (config.dt_s <= 0.0) {
    throw std::invalid_argument("dt_s must be > 0");
  }
  if (config.duration_s < 0.0) {
    throw std::invalid_argument("duration_s must be >= 0");
  }
  if (config.position_noise_m < 0.0) {
    throw std::invalid_argument("position_noise_m must be >= 0");
  }
  if (config.detection_probability < 0.0 || config.detection_probability > 1.0) {
    throw std::invalid_argument("detection_probability must be in [0, 1]");
  }
  if (config.area_half_width_m <= 0.0) {
    throw std::invalid_argument("area_half_width_m must be > 0");
  }
  if (config.duration_s / config.dt_s > kMaxSteps) {
    throw std::invalid_argument("duration_s / dt_s is too large");
  }
}

struct Vehicle {
  Vec2 start;
  Vec2 velocity;
};

}

GeneratedScenario generate(const ScenarioConfig& config) {
  validate(config);
  Rng rng(config.seed);

  std::vector<Vehicle> vehicles;
  vehicles.reserve(static_cast<std::size_t>(config.vehicles));
  for (int i = 0; i < config.vehicles; ++i) {
    Vehicle vehicle{};
    vehicle.start.x = rng.uniform(-config.area_half_width_m, config.area_half_width_m);
    vehicle.start.y = rng.uniform(-config.area_half_width_m, config.area_half_width_m);
    const DiscPoint heading = disc_point(rng);
    const double length = std::sqrt(heading.radius_squared);
    const double speed = rng.uniform(kMinSpeed, kMaxSpeed);
    vehicle.velocity = {speed * (heading.u / length), speed * (heading.v / length)};
    vehicles.push_back(vehicle);
  }

  const long long last_step = std::llround(config.duration_s / config.dt_s);
  GeneratedScenario scenario;
  scenario.steps.reserve(static_cast<std::size_t>(last_step) + 1);
  for (long long k = 0; k <= last_step; ++k) {
    ScenarioStep step;
    step.t = static_cast<double>(k) * config.dt_s;
    for (int i = 0; i < config.vehicles; ++i) {
      const Vehicle& vehicle = vehicles[static_cast<std::size_t>(i)];
      const Vec2 position{vehicle.start.x + vehicle.velocity.x * step.t,
                          vehicle.start.y + vehicle.velocity.y * step.t};
      step.truth.push_back({step.t, 0, i, position, vehicle.velocity});
      if (rng.uniform01() < config.detection_probability) {
        const Vec2 noise = standard_normal_pair(rng);
        step.measurements.push_back(
            {step.t, 0, 0,
             {position.x + config.position_noise_m * noise.x,
              position.y + config.position_noise_m * noise.y}});
      }
    }
    shuffle(step.measurements, rng);
    scenario.steps.push_back(std::move(step));
  }
  return scenario;
}

LogRecords to_log_records(const GeneratedScenario& scenario) {
  LogRecords records;
  for (const ScenarioStep& step : scenario.steps) {
    records.truth.insert(records.truth.end(), step.truth.begin(), step.truth.end());
    records.measurements.insert(records.measurements.end(), step.measurements.begin(),
                                step.measurements.end());
  }
  return records;
}

}
