#include "kite/scoring.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace kite {

namespace {

struct Step {
  std::vector<const GroundTruthState*> truth;
  std::vector<const TrackState*> tracks;
};

std::vector<std::size_t> min_cost_assignment(const std::vector<double>& cost, std::size_t n) {
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0.0);
  std::vector<double> v(n + 1, 0.0);
  std::vector<std::size_t> row_of_column(n + 1, 0);
  std::vector<std::size_t> way(n + 1, 0);
  for (std::size_t row = 1; row <= n; ++row) {
    row_of_column[0] = row;
    std::size_t column = 0;
    std::vector<double> min_slack(n + 1, inf);
    std::vector<bool> used(n + 1, false);
    do {
      used[column] = true;
      const std::size_t r = row_of_column[column];
      double delta = inf;
      std::size_t next = 0;
      for (std::size_t c = 1; c <= n; ++c) {
        if (used[c]) {
          continue;
        }
        const double slack = cost[(r - 1) * n + (c - 1)] - u[r] - v[c];
        if (slack < min_slack[c]) {
          min_slack[c] = slack;
          way[c] = column;
        }
        if (min_slack[c] < delta) {
          delta = min_slack[c];
          next = c;
        }
      }
      for (std::size_t c = 0; c <= n; ++c) {
        if (used[c]) {
          u[row_of_column[c]] += delta;
          v[c] -= delta;
        } else {
          min_slack[c] -= delta;
        }
      }
      column = next;
    } while (row_of_column[column] != 0);
    do {
      const std::size_t previous = way[column];
      row_of_column[column] = row_of_column[previous];
      column = previous;
    } while (column != 0);
  }

  std::vector<std::size_t> column_of_row(n, 0);
  for (std::size_t c = 1; c <= n; ++c) {
    column_of_row[row_of_column[c] - 1] = c - 1;
  }
  return column_of_row;
}

}

ScoreResult score(const std::vector<GroundTruthState>& truth, const std::vector<TrackState>& tracks,
                  double match_gate_m) {
  if (!std::isfinite(match_gate_m) || match_gate_m < 0.0) {
    throw std::invalid_argument("match gate must be finite and non-negative");
  }

  std::map<std::pair<int, double>, Step> steps;
  for (const GroundTruthState& state : truth) {
    if (!std::isfinite(state.t)) {
      throw std::invalid_argument("ground truth time must be finite");
    }
    steps[{state.ego_id, state.t}].truth.push_back(&state);
  }
  for (const TrackState& track : tracks) {
    if (!std::isfinite(track.t)) {
      throw std::invalid_argument("track time must be finite");
    }
    steps[{track.ego_id, track.t}].tracks.push_back(&track);
  }

  ScoreResult result{0, 0, 0, std::nullopt, 0, static_cast<int>(steps.size())};
  double squared_distance_sum = 0.0;
  std::map<std::pair<int, int>, int> last_track_of_actor;

  for (auto& entry : steps) {
    Step& step = entry.second;
    std::stable_sort(step.truth.begin(), step.truth.end(),
                     [](const GroundTruthState* a, const GroundTruthState* b) {
                       return a->actor_id < b->actor_id;
                     });
    std::stable_sort(step.tracks.begin(), step.tracks.end(),
                     [](const TrackState* a, const TrackState* b) { return a->track_id < b->track_id; });

    const std::size_t truths = step.truth.size();
    const std::size_t track_count = step.tracks.size();
    const std::size_t n = std::max(truths, track_count);
    const double costlier_than_any_in_gate_total = match_gate_m * static_cast<double>(n + 1) + 1.0;

    std::vector<double> distance(n * n, 0.0);
    std::vector<double> cost(n * n, 0.0);
    for (std::size_t i = 0; i < truths; ++i) {
      for (std::size_t j = 0; j < track_count; ++j) {
        const double d = std::hypot(step.truth[i]->position.x - step.tracks[j]->position.x,
                                    step.truth[i]->position.y - step.tracks[j]->position.y);
        distance[i * n + j] = d;
        cost[i * n + j] = d <= match_gate_m ? d : costlier_than_any_in_gate_total;
      }
    }

    const std::vector<std::size_t> column_of_row = min_cost_assignment(cost, n);
    int matched_here = 0;
    for (std::size_t i = 0; i < truths; ++i) {
      const std::size_t j = column_of_row[i];
      if (j >= track_count || distance[i * n + j] > match_gate_m) {
        continue;
      }
      const double d = distance[i * n + j];
      ++matched_here;
      squared_distance_sum += d * d;

      const GroundTruthState& actor = *step.truth[i];
      const int track_id = step.tracks[j]->track_id;
      const auto [last, first_match] =
          last_track_of_actor.try_emplace({actor.ego_id, actor.actor_id}, track_id);
      if (!first_match && last->second != track_id) {
        ++result.id_switches;
        last->second = track_id;
      }
    }

    result.matched += matched_here;
    result.missed += static_cast<int>(truths) - matched_here;
    result.false_tracks += static_cast<int>(track_count) - matched_here;
  }

  if (result.matched > 0) {
    result.rmse_m = std::sqrt(squared_distance_sum / result.matched);
  }
  return result;
}

}
