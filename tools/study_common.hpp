#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "baseline_setup.hpp"
#include "kite/kalman_tracker.hpp"
#include "kite/matrix.hpp"
#include "kite/types.hpp"

// Helpers the study tools share. Ground truth enters only through label_of, which labels a
// measurement; no tracker ever sees it.
namespace kite::study {

inline constexpr double kLabelMarginM = 1.0;

using CarLabel = std::optional<int>;

inline double distance(Vec2 a, Vec2 b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double mahalanobis2(const TrackRecord& track, Vec2 z) {
  const Matrix<2, 1> y{{z.x - track.predicted.x, z.y - track.predicted.y}};
  return (transpose(y) * inverse(track.innovation_covariance) * y)(0, 0);
}

inline CarLabel label_of(Vec2 z, const std::vector<GroundTruthState>& truth) {
  double nearest = std::numeric_limits<double>::infinity();
  double second = nearest;
  int car = -1;
  for (const GroundTruthState& vehicle : truth) {
    const double d = distance(z, vehicle.position);
    if (d < nearest) {
      second = nearest;
      nearest = d;
      car = vehicle.actor_id;
    } else if (d < second) {
      second = d;
    }
  }
  if (nearest <= baseline_setup::kMatchGateM && second - nearest >= kLabelMarginM) {
    return car;
  }
  return std::nullopt;
}

}
