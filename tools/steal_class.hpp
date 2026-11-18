#pragma once

#include "kite/kalman_tracker.hpp"
#include "kite/matrix.hpp"
#include "kite/types.hpp"
#include "study_common.hpp"

namespace kite::study {

enum class StealClass { h1, nearer, other };

inline double det(const Matrix<2, 2>& s) { return s(0, 0) * s(1, 1) - s(0, 1) * s(1, 0); }

// Why a tentative track took measurement z from a confirmed track of the same car. nearer: the
// tentative track's prediction is strictly closer to z. Otherwise h1: it scores a lower d^2 and has
// the larger det S. Otherwise other.
inline StealClass classify_steal(Vec2 z, const TrackRecord& tentative,
                                 const TrackRecord& confirmed) {
  if (distance(z, tentative.predicted) < distance(z, confirmed.predicted)) {
    return StealClass::nearer;
  }
  if (mahalanobis2(tentative, z) < mahalanobis2(confirmed, z) &&
      det(tentative.innovation_covariance) > det(confirmed.innovation_covariance)) {
    return StealClass::h1;
  }
  return StealClass::other;
}

}
