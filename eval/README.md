# eval

Scores tracker output against ground truth.

- `kite_eval` (`include/kite/scoring.hpp`): `score` matches tracks to ground truth at each time step
  with the Hungarian algorithm within a gate and returns matched, missed and false counts, position
  RMSE, identity switches and the number of steps. The header states the exact rules.
- `tools/kite_score` scores a log that has `gt` and `track` records.
- `tools/kite_baseline` runs the no-filter baseline tracker on generated scenarios and prints the
  table in `docs/BASELINE.md`.

Ground truth is used for scoring and training labels only. It is never an input to the driving stack.
