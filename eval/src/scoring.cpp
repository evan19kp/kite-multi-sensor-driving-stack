#include "kite/scoring.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <utility>

#include "kite/assignment.hpp"

namespace kite {

namespace {

struct Step {
  std::vector<const GroundTruthState*> truth;
  std::vector<const TrackState*> tracks;
};

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
    if (!std::isfinite(state.position.x) || !std::isfinite(state.position.y)) {
      throw std::invalid_argument("ground truth position must be finite");
    }
    steps[{state.ego_id, state.t}].truth.push_back(&state);
  }
  for (const TrackState& track : tracks) {
    if (!std::isfinite(track.t)) {
      throw std::invalid_argument("track time must be finite");
    }
    if (!std::isfinite(track.position.x) || !std::isfinite(track.position.y)) {
      throw std::invalid_argument("track position must be finite");
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
    CostMatrix distance{truths, track_count, std::vector<double>(truths * track_count, 0.0)};
    for (std::size_t i = 0; i < truths; ++i) {
      for (std::size_t j = 0; j < track_count; ++j) {
        distance.values[i * track_count + j] =
            std::hypot(step.truth[i]->position.x - step.tracks[j]->position.x,
                       step.truth[i]->position.y - step.tracks[j]->position.y);
      }
    }

    int matched_here = 0;
    for (const Assignment& pair : solve_assignment(distance, match_gate_m)) {
      const std::size_t i = pair.row;
      const std::size_t j = pair.column;
      const double d = distance.values[i * track_count + j];
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

std::string format_fixed(const std::optional<double>& value) {
  if (!value) {
    return "n/a";
  }
  char buffer[512];
  const auto result =
      std::to_chars(buffer, buffer + sizeof(buffer), *value, std::chars_format::fixed, 3);
  return std::string(buffer, result.ptr);
}

}
