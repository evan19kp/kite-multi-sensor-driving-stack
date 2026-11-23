#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "baseline_setup.hpp"
#include "kite/kalman_tracker.hpp"
#include "kite/matrix.hpp"
#include "kite/types.hpp"

// Helpers the study tools share. Ground truth enters only through classify_measurement, which
// labels a measurement; no tracker ever sees it.
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

// car: the nearest truth vehicle is within the 3 m gate and at least 1 m nearer than the second
// nearest. clutter: no truth vehicle is within the gate. ambiguous: the rest.
enum class LabelKind { car, ambiguous, clutter };

struct Label {
  LabelKind kind;
  int car = -1;

  CarLabel as_car() const { return kind == LabelKind::car ? CarLabel(car) : std::nullopt; }
};

inline Label classify_measurement(Vec2 z, const std::vector<GroundTruthState>& truth) {
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
  if (nearest > baseline_setup::kMatchGateM) {
    return {LabelKind::clutter};
  }
  if (second - nearest >= kLabelMarginM) {
    return {LabelKind::car, car};
  }
  return {LabelKind::ambiguous};
}

struct LabelCounts {
  int clutter = 0;
  int ambiguous = 0;

  void add(const Label& label) {
    clutter += label.kind == LabelKind::clutter ? 1 : 0;
    ambiguous += label.kind == LabelKind::ambiguous ? 1 : 0;
  }
  int unlabelled() const { return clutter + ambiguous; }
  LabelCounts& operator+=(const LabelCounts& other) {
    clutter += other.clutter;
    ambiguous += other.ambiguous;
    return *this;
  }
};

// The straight scenario prints one "ambiguous" column that counts every measurement without a car
// label, as it did before the hard scenario existed. The hard scenario splits it into clutter and
// ambiguous. A column heading wraps each name in the prefix and suffix given.
inline std::vector<std::string> label_headers(baseline_setup::Scenario scenario,
                                              const std::string& prefix = "",
                                              const std::string& suffix = "") {
  std::vector<std::string> names{"ambiguous"};
  if (scenario == baseline_setup::Scenario::hard) {
    names = {"clutter", "ambiguous"};
  }
  for (std::string& name : names) {
    name = prefix + name + suffix;
  }
  return names;
}

inline void append(std::vector<std::string>& cells, const std::vector<std::string>& more) {
  cells.insert(cells.end(), more.begin(), more.end());
}

inline std::string row(const std::vector<std::string>& cells) {
  std::string line = "|";
  for (const std::string& cell : cells) {
    line += " " + cell + " |";
  }
  return line + "\n";
}

inline std::string divider(std::size_t columns) {
  std::string line;
  for (std::size_t i = 0; i < columns; ++i) {
    line += "|---";
  }
  return line + "|\n";
}

inline std::string table_header(const std::vector<std::string>& names) {
  return row(names) + divider(names.size());
}

inline std::string count(int value) { return std::to_string(value); }

inline std::vector<std::string> label_cells(baseline_setup::Scenario scenario,
                                            const LabelCounts& counts) {
  if (scenario == baseline_setup::Scenario::hard) {
    return {count(counts.clutter), count(counts.ambiguous)};
  }
  return {count(counts.unlabelled())};
}

inline std::string label_summary(baseline_setup::Scenario scenario, int measurements,
                                 const LabelCounts& counts) {
  std::string text = count(measurements) + " measurements, ";
  if (scenario == baseline_setup::Scenario::hard) {
    return text + count(counts.clutter) + " clutter, " + count(counts.ambiguous) + " ambiguous";
  }
  return text + count(counts.unlabelled()) + " ambiguous";
}

inline std::string scenario_suffix(baseline_setup::Scenario scenario) {
  return scenario == baseline_setup::Scenario::hard ? ", hard scenario" : "";
}

}
