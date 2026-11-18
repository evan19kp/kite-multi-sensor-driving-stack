#pragma once

#include <vector>

#include "kite/types.hpp"

namespace kite {

// Trackers receive measurements only, never ground truth.
class Tracker {
 public:
  virtual ~Tracker();

  virtual std::vector<TrackState> update(double t, const std::vector<Measurement>& measurements) = 0;
};

}
