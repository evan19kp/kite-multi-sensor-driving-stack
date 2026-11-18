#pragma once

#include <optional>
#include <vector>

#include "kite/types.hpp"

namespace kite {

struct ScoreResult {
  int matched;
  int missed;
  int false_tracks;
  std::optional<double> rmse_m;
  int id_switches;
  int steps;
};

// Scores tracks against ground truth.
//
// Steps. Records are grouped by exact (ego_id, t). `steps` counts the distinct keys found in truth
// or tracks. A step with tracks but no truth makes every track false; a step with truth but no
// tracks makes every truth missed.
//
// Matching, per step. A truth and a track may pair only within the gate (distance <= gate,
// metres). The matching first has as many pairs as possible, then the minimum total distance among
// those. Matchings of equal cost are decided by actor and track ids, not by input order, as long as
// those ids are unique within a step.
//
// Results. Unmatched truths are missed and unmatched tracks are false. rmse_m is the root mean
// square distance over all matched pairs, or nullopt when nothing matched. An identity switch is
// a match of an (ego_id, actor_id) to a track id other than the one at its last matched step;
// steps where the actor is unmatched do not reset that memory.
//
// Throws std::invalid_argument for a negative or non-finite gate, or a non-finite t.
ScoreResult score(const std::vector<GroundTruthState>& truth, const std::vector<TrackState>& tracks,
                  double match_gate_m);

}
