#pragma once

#include <cstdint>
#include <vector>

#include "kite/log.hpp"
#include "kite/types.hpp"

namespace kite {

struct ScenarioConfig {
  std::uint64_t seed;
  int vehicles;
  double duration_s;
  double dt_s;
  double position_noise_m;
  double detection_probability;
  double area_half_width_m;
};

struct ScenarioStep {
  double t;
  std::vector<GroundTruthState> truth;
  std::vector<Measurement> measurements;
};

struct GeneratedScenario {
  std::vector<ScenarioStep> steps;
};

// Generates a scenario of constant-velocity vehicles and their noisy detections.
//
// Throws std::invalid_argument unless: vehicles >= 0, duration_s >= 0, dt_s > 0,
// position_noise_m >= 0, 0 <= detection_probability <= 1, area_half_width_m > 0, every double is
// finite, and duration_s / dt_s is at most 1e9.
//
// Output. Steps k = 0..K with K = llround(duration_s / dt_s) and t = k * dt_s (a product, never a
// running sum). Truth holds one state per vehicle in index order: ego_id 0, actor_id = vehicle
// index, position = start + velocity * t. Measurements have ego_id 0 and sensor_id 0, and no
// identity. Truth and measurements stay in separate vectors.
//
// Exact algorithm, so the same config gives the same bytes on every compiler and platform. Every
// value comes from IEEE-exact operations (+, -, *, /, sqrt, frexp) applied to the raw 64-bit
// output of one std::mt19937_64 seeded with `seed`. No std::*_distribution, std::shuffle, or
// platform transcendental function is used.
//
//   raw      one call of the engine.
//   uniform  u = (raw >> 11) * 2^-53, in [0, 1). A value in [lo, hi) is lo + (hi - lo) * u.
//   disc     draw u = 2 * uniform - 1, then v = 2 * uniform - 1. Reject the pair and draw again if
//            s = u*u + v*v is >= 1 or < 1e-12. The accepted (u, v) is uniform in the unit disc.
//   heading  disc point (u, v) divided by sqrt(s): a uniform unit direction, without trig.
//   normal   Marsaglia's polar form of Box-Muller. Take a disc point, then
//            f = sqrt(-2 * ln(s) / s). The pair (u * f, v * f) is two independent standard
//            normals. Both are used: the first is the x noise and the second the y noise of one
//            detection, so no value is cached between draws.
//   ln       our own: frexp splits s = m * 2^e, with m moved into [sqrt(0.5), sqrt(2)); with
//            z = (m - 1) / (m + 1), ln(m) = 2 * (z + z^3/3 + z^5/5 + ...) summed over exactly 14
//            terms by Horner's rule from the last term; the result is that sum plus e * ln(2).
//   below    an integer in [0, n): draw raw until raw >= 2^64 mod n, then raw mod n. This has no
//            modulo bias.
//   shuffle  Fisher-Yates: for i = size - 1 down to 1, j = below(i + 1), swap elements i and j.
//            A list of fewer than 2 elements draws nothing.
//
// Draw order:
//   1. For each vehicle in index order: start x = uniform in [-half_width, half_width), start y
//      likewise, heading (a disc draw), then speed = uniform in [5, 15) m/s. Velocity is speed
//      times the heading.
//   2. For each step in time order, for each vehicle in index order: one uniform for detection.
//      The vehicle is detected when that uniform is < detection_probability, so probability 1
//      always detects and 0 never does. A detected vehicle then draws one normal pair and the
//      measurement is its true position plus noise * (x, y) of the pair. The detection draw is
//      made even when the probability is 0 or 1, and the normal draw even when the noise is 0.
//   3. After the last vehicle of a step, shuffle that step's measurements.
GeneratedScenario generate(const ScenarioConfig& config);

// Flattens a scenario into log records: truth and measurements in step order, no tracks.
LogRecords to_log_records(const GeneratedScenario& scenario);

}
