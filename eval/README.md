# eval

Scores tracker output against ground truth.

- `kite_eval` (`include/kite/scoring.hpp`): `score` matches tracks to ground truth at each time step
  with the Hungarian algorithm within a gate and returns matched, missed and false counts, position
  RMSE, identity switches and the number of steps. The header states the exact rules.
- `tools/kite_score` scores a log that has `gt` and `track` records.
- `tools/kite_baseline` runs a tracker on generated scenarios and prints a Markdown table with one
  row per seed and a mean row. Without options it runs the baseline tracker and prints the table in
  `docs/BASELINE.md`.
  - `--tracker baseline|kalman`: `baseline` (default) is `NearestNeighbourBaseline`; `kalman` is
    `KalmanTracker`.
  - `--association two_stage|joint|joint_likelihood`: `KalmanTracker` association mode, only with
    `--tracker kalman`; `two_stage` is the default.
  - `--seeds N | A-B`: run seeds 1..N with N >= 1, or A..B with 1 <= A <= B (default 1..20).
- `tools/kite_association_study` runs `KalmanTracker` on seeds 101-120 and prints tables of the
  measurements that tentative tracks took from confirmed tracks of the same vehicle.
  `tools/kite_drop_study` measures how often `KalmanTracker` drops a tentative track. Both are run
  through CTest. `docs/INVESTIGATION-ASSOCIATION.md` has their output and what it shows.

Ground truth is used for scoring and training labels only. It is never an input to the driving stack.
