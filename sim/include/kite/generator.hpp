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
  bool turning = false;
  bool speed_changes = false;
  double clutter_per_step = 0.0;
};

struct MotionState {
  Vec2 position;
  Vec2 heading;
  double speed;
};

// Moves a vehicle over one step of dt_s. Its heading turns at the constant rate turn_rate_rad_s
// and its speed changes linearly from state.speed to end_speed. With theta = turn_rate_rad_s *
// dt_s and complex numbers written as (re, im), multiplied as (a, b)(c, d) = (ac - bd, ad + bc):
//
//   E  = sum over n = 0..15 of (i theta)^n / n!             cos(theta) + i sin(theta)
//   F0 = sum over n = 0..15 of (i theta)^n / (n + 1)!       integral over u in [0, 1] of e^(i theta u)
//   F1 = sum over n = 0..15 of (i theta)^n / (n! (n + 2))   integral over u in [0, 1] of u e^(i theta u)
//
//   position' = position + dt_s * heading (state.speed * F0 + (end_speed - state.speed) * F1)
//   heading'  = heading E, divided by its length sqrt(re^2 + im^2)
//   speed'    = end_speed
//
// This is the exact integral of that motion, up to the series' truncation. For |theta| <= 0.3 the
// truncation is below 0.3^16 / 16! < 3e-22, relative. Each series is summed by Horner's rule from
// the last term. Each coefficient is 1 divided by an integer that is exact in a double, so it is
// correctly rounded. Only +, -, *, / and the generator's own sqrt (see generate) are used.
MotionState advance(const MotionState& state, double turn_rate_rad_s, double end_speed,
                    double dt_s);

struct ScenarioStep {
  double t;
  std::vector<GroundTruthState> truth;
  std::vector<Measurement> measurements;
};

struct GeneratedScenario {
  std::vector<ScenarioStep> steps;
};

// Generates a scenario of vehicles and their noisy detections. By default the vehicles move at
// constant velocity and there is no clutter.
//
// Throws std::invalid_argument unless: vehicles >= 0, duration_s >= 0, dt_s > 0,
// position_noise_m >= 0, 0 <= detection_probability <= 1, area_half_width_m > 0,
// 0 <= clutter_per_step <= 1000, dt_s <= 1 when turning is on, every double is finite, and
// duration_s / dt_s is at most 1e9.
//
// Output. Steps k = 0..K with K = llround(duration_s / dt_s) and t = k * dt_s (a product, never a
// running sum). Truth holds one state per vehicle in index order: ego_id 0, actor_id = vehicle
// index. Measurements have ego_id 0 and sensor_id 0, and no identity. Truth and measurements stay
// in separate vectors.
//
// Motion. With turning and speed_changes both off, position = start + velocity * t. With either
// on, each vehicle carries a MotionState (position, heading, speed), first its start, and moves from
// step k to k + 1 by advance(state, turn rate, end speed, dt_s). Truth velocity is speed times heading.
//   turning        segments alternate straight (turn rate 0) and turning, starting straight. A
//                  turning segment has a turn rate uniform in [-0.3, 0.3) rad/s. Off: the rate
//                  is always 0.
//   speed_changes  segments of constant acceleration uniform in [-3, 2) m/s^2. Each step's end
//                  speed is min(max(speed + acceleration * dt_s, 0), 20), so within the step the
//                  speed runs linearly between two values in [0, 20] m/s. Off: the end speed is
//                  the speed.
// A segment lasts n = max(1, llround(d / dt_s)) steps, where d is uniform in [2, 6) s. The turning
// and speed segments run on separate clocks. Vehicles may drive out of the area; nothing keeps them
// in.
//
// Clutter. Each step adds false detections, as many as a Poisson draw with mean clutter_per_step,
// each uniform over the area and shaped like any other measurement.
//
// Exact algorithm, so the same config gives the same bytes on every compiler and platform. Every
// value comes from IEEE-exact operations (+, -, *, /, frexp, and the exact llround, min and max)
// applied to the raw 64-bit output of one std::mt19937_64 seeded with `seed`. No
// std::*_distribution, std::shuffle, platform transcendental function or std::sqrt is used.
// MinGW-w64's sqrt runs the x87 fsqrt at 64-bit precision and then rounds again to a double, so
// some of its results are one ulp off the correctly rounded root that glibc and MSVC return.
// Floating-point contraction (fused multiply-add) is disabled for GCC and Clang, so a * b + c
// rounds twice.
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
//   sqrt     our own, correctly rounded, for x > 0. frexp splits x = m * 2^e, with m moved into
//            [0.5, 2) so that e is even. Start at r = (1 + m) / 2 and take 4 Heron steps
//            r = (r + m / r) / 2. Each step maps the relative error e to e^2 / (2 (1 + e)), so it
//            falls from at most 0.061 to under 1e-24, and r ends within a few ulp of sqrt(m), off
//            only by the rounding of the steps. Let r+ and r- be the doubles next to r. Then
//            sqrt(m) rounds to r exactly when r * r- < m <= r * r+. The
//            reason is that the upper midpoint squared is r * r+ + u^2/4, with u = r+ - r and
//            u^2/4 <= 2^-106. Both m and r * r+ are multiples of 2^-106, and m is never the square
//            of a midpoint. The lower midpoint works the same way. Step r up while m > r * r+,
//            then down while m <= r * r-. Each comparison is exact: Dekker's product, with
//            Veltkamp's split by 2^27 + 1 and no fused multiply-add, gives r * r+ as a rounded
//            product plus its exact error, and m minus the rounded product is exact by Sterbenz's
//            lemma. The result is r * 2^(e/2), with the power of two built by repeated doubling or
//            halving.
//   below    an integer in [0, n): draw raw until raw >= 2^64 mod n, then raw mod n. This has no
//            modulo bias.
//   shuffle  Fisher-Yates: for i = size - 1 down to 1, j = below(i + 1), swap elements i and j.
//            A list of fewer than 2 elements draws nothing.
//   poisson  a count with mean lambda, the number of arrivals of a unit-rate Poisson process in
//            [0, lambda]. Start with count 0 and sum 0. Repeat: draw u = uniform and add the
//            exponential gap -ln(1 - u) to the sum. Stop if the sum > lambda, else add 1 to the
//            count. 1 - u is exact and in (0, 1], so ln is finite. A count c costs c + 1 uniforms.
//   segment  one uniform d in [2, 6) s, then n = max(1, llround(d / dt_s)) steps.
//
// Draw order:
//   1. For each vehicle in index order: start x = uniform in [-half_width, half_width), start y
//      likewise, heading (a disc draw), then speed = uniform in [5, 15) m/s. Velocity is speed
//      times the heading.
//   2. After the last vehicle, for each vehicle in index order: if turning, its first segment
//      (straight). Then, if speed_changes, its first speed segment, then its acceleration.
//   3. For each step in time order, for each vehicle in index order: one uniform for detection.
//      The vehicle is detected when that uniform is < detection_probability, so probability 1
//      always detects and 0 never does. A detected vehicle then draws one normal pair and the
//      measurement is its true position plus noise * (x, y) of the pair. The detection draw is
//      made even when the probability is 0 or 1, and the normal draw even when the noise is 0.
//   4. After the last vehicle of a step, if clutter_per_step > 0: one poisson count c, then c
//      clutter positions, each x = uniform in [-half_width, half_width) then y likewise,
//      appended in draw order.
//   5. Shuffle that step's measurements, detections and clutter together.
//   6. If turning or speed_changes, before the next step (none after the last), for each vehicle
//      in index order: advance it by one step. Then, if turning, count one step off its turning
//      segment; if that ended, draw the next one, which switches between straight and turning,
//      and a turning one then draws its rate. Then, if speed_changes, count one step off its
//      speed segment; if that ended, draw the next one, then its acceleration.
GeneratedScenario generate(const ScenarioConfig& config);

// Flattens a scenario into log records: truth and measurements in step order, no tracks.
LogRecords to_log_records(const GeneratedScenario& scenario);

}
