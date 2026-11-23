#include "kite/generator.hpp"

#include <algorithm>
#include <array>
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
constexpr int kSeriesTerms = 16;
constexpr double kMaxTurnRate = 0.3;
constexpr double kMaxTurningDt = 1.0;
constexpr double kMinAcceleration = -3.0;
constexpr double kMaxAcceleration = 2.0;
constexpr double kMaxManoeuvreSpeed = 20.0;
constexpr double kMinSegmentS = 2.0;
constexpr double kMaxSegmentS = 6.0;
constexpr double kMaxClutterPerStep = 1000.0;

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

long long poisson(Rng& rng, double mean) {
  long long count = 0;
  double arrival = -natural_log(1.0 - rng.uniform01());
  while (arrival <= mean) {
    ++count;
    arrival -= natural_log(1.0 - rng.uniform01());
  }
  return count;
}

using Series = std::array<double, kSeriesTerms>;

struct SeriesCoefficients {
  Series rotation;
  Series mean_rotation;
  Series weighted_rotation;
};

const SeriesCoefficients& series_coefficients() {
  static const SeriesCoefficients coefficients = [] {
    SeriesCoefficients c{};
    double factorial = 1.0;
    for (int n = 0; n < kSeriesTerms; ++n) {
      if (n > 0) {
        factorial *= static_cast<double>(n);
      }
      c.rotation[static_cast<std::size_t>(n)] = 1.0 / factorial;
      c.mean_rotation[static_cast<std::size_t>(n)] =
          1.0 / (factorial * static_cast<double>(n + 1));
      c.weighted_rotation[static_cast<std::size_t>(n)] =
          1.0 / (factorial * static_cast<double>(n + 2));
    }
    return c;
  }();
  return coefficients;
}

Vec2 sum_series(const Series& coefficients, double theta) {
  Vec2 sum{0.0, 0.0};
  for (int n = kSeriesTerms - 1; n >= 0; --n) {
    sum = {coefficients[static_cast<std::size_t>(n)] - sum.y * theta, sum.x * theta};
  }
  return sum;
}

Vec2 times(const Vec2& a, const Vec2& b) {
  return {a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x};
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
                      std::isfinite(config.area_half_width_m) &&
                      std::isfinite(config.clutter_per_step);
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
  if (config.clutter_per_step < 0.0 || config.clutter_per_step > kMaxClutterPerStep) {
    throw std::invalid_argument("clutter_per_step must be in [0, 1000]");
  }
  if (config.turning && config.dt_s > kMaxTurningDt) {
    throw std::invalid_argument("dt_s must be <= 1 when turning is on");
  }
}

struct Segment {
  long long steps_left = 0;
  double rate = 0.0;
};

long long segment_steps(Rng& rng, double dt_s) {
  return std::max(std::llround(rng.uniform(kMinSegmentS, kMaxSegmentS) / dt_s), 1LL);
}

Segment straight_segment(Rng& rng, double dt_s) {
  return {segment_steps(rng, dt_s), 0.0};
}

Segment turning_segment(Rng& rng, double dt_s) {
  Segment segment{segment_steps(rng, dt_s), 0.0};
  segment.rate = rng.uniform(-kMaxTurnRate, kMaxTurnRate);
  return segment;
}

Segment acceleration_segment(Rng& rng, double dt_s) {
  Segment segment{segment_steps(rng, dt_s), 0.0};
  segment.rate = rng.uniform(kMinAcceleration, kMaxAcceleration);
  return segment;
}

struct Vehicle {
  MotionState motion;
  bool in_turning_segment = false;
  Segment turn;
  Segment acceleration;

  Vec2 velocity() const { return {motion.speed * motion.heading.x, motion.speed * motion.heading.y}; }
};

void start_manoeuvres(Vehicle& vehicle, const ScenarioConfig& config, Rng& rng) {
  if (config.turning) {
    vehicle.turn = straight_segment(rng, config.dt_s);
  }
  if (config.speed_changes) {
    vehicle.acceleration = acceleration_segment(rng, config.dt_s);
  }
}

void drive_one_step(Vehicle& vehicle, const ScenarioConfig& config, Rng& rng) {
  const double end_speed =
      std::min(std::max(vehicle.motion.speed + vehicle.acceleration.rate * config.dt_s, 0.0),
               kMaxManoeuvreSpeed);
  vehicle.motion = advance(vehicle.motion, vehicle.turn.rate, end_speed, config.dt_s);
  if (config.turning && --vehicle.turn.steps_left == 0) {
    vehicle.in_turning_segment = !vehicle.in_turning_segment;
    vehicle.turn = vehicle.in_turning_segment ? turning_segment(rng, config.dt_s)
                                              : straight_segment(rng, config.dt_s);
  }
  if (config.speed_changes && --vehicle.acceleration.steps_left == 0) {
    vehicle.acceleration = acceleration_segment(rng, config.dt_s);
  }
}

void add_clutter(ScenarioStep& step, const ScenarioConfig& config, Rng& rng) {
  if (config.clutter_per_step == 0.0) {
    return;
  }
  const long long count = poisson(rng, config.clutter_per_step);
  for (long long c = 0; c < count; ++c) {
    const double x = rng.uniform(-config.area_half_width_m, config.area_half_width_m);
    const double y = rng.uniform(-config.area_half_width_m, config.area_half_width_m);
    step.measurements.push_back({step.t, 0, 0, {x, y}});
  }
}

}

MotionState advance(const MotionState& state, double turn_rate_rad_s, double end_speed,
                    double dt_s) {
  const double theta = turn_rate_rad_s * dt_s;
  const SeriesCoefficients& c = series_coefficients();
  const Vec2 rotation = sum_series(c.rotation, theta);
  const Vec2 mean_rotation = sum_series(c.mean_rotation, theta);
  const Vec2 weighted_rotation = sum_series(c.weighted_rotation, theta);
  const double gain = end_speed - state.speed;
  const Vec2 travel = times(state.heading, {state.speed * mean_rotation.x + gain * weighted_rotation.x,
                                            state.speed * mean_rotation.y + gain * weighted_rotation.y});
  const Vec2 turned = times(state.heading, rotation);
  const double length = std::sqrt(turned.x * turned.x + turned.y * turned.y);
  return {{state.position.x + dt_s * travel.x, state.position.y + dt_s * travel.y},
          {turned.x / length, turned.y / length},
          end_speed};
}

GeneratedScenario generate(const ScenarioConfig& config) {
  validate(config);
  Rng rng(config.seed);

  std::vector<Vehicle> vehicles;
  vehicles.reserve(static_cast<std::size_t>(config.vehicles));
  for (int i = 0; i < config.vehicles; ++i) {
    Vehicle vehicle{};
    vehicle.motion.position.x = rng.uniform(-config.area_half_width_m, config.area_half_width_m);
    vehicle.motion.position.y = rng.uniform(-config.area_half_width_m, config.area_half_width_m);
    const DiscPoint heading = disc_point(rng);
    const double length = std::sqrt(heading.radius_squared);
    vehicle.motion.heading = {heading.u / length, heading.v / length};
    vehicle.motion.speed = rng.uniform(kMinSpeed, kMaxSpeed);
    vehicles.push_back(vehicle);
  }
  const bool manoeuvring = config.turning || config.speed_changes;
  for (Vehicle& vehicle : vehicles) {
    start_manoeuvres(vehicle, config, rng);
  }

  const long long last_step = std::llround(config.duration_s / config.dt_s);
  GeneratedScenario scenario;
  scenario.steps.reserve(static_cast<std::size_t>(last_step) + 1);
  for (long long k = 0; k <= last_step; ++k) {
    ScenarioStep step;
    step.t = static_cast<double>(k) * config.dt_s;
    for (int i = 0; i < config.vehicles; ++i) {
      const Vehicle& vehicle = vehicles[static_cast<std::size_t>(i)];
      const Vec2 velocity = vehicle.velocity();
      const Vec2 position =
          manoeuvring ? vehicle.motion.position
                      : Vec2{vehicle.motion.position.x + velocity.x * step.t,
                             vehicle.motion.position.y + velocity.y * step.t};
      step.truth.push_back({step.t, 0, i, position, velocity});
      if (rng.uniform01() < config.detection_probability) {
        const Vec2 noise = standard_normal_pair(rng);
        step.measurements.push_back(
            {step.t, 0, 0,
             {position.x + config.position_noise_m * noise.x,
              position.y + config.position_noise_m * noise.y}});
      }
    }
    add_clutter(step, config, rng);
    shuffle(step.measurements, rng);
    scenario.steps.push_back(std::move(step));
    if (manoeuvring && k < last_step) {
      for (Vehicle& vehicle : vehicles) {
        drive_one_step(vehicle, config, rng);
      }
    }
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
