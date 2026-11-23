# eval

Scores tracker output against ground truth.

- `kite_eval` (`include/kite/scoring.hpp`): `score` matches tracks to ground truth at each time step
  with the Hungarian algorithm within a gate and returns matched, missed and false counts, position
  RMSE, identity switches and the number of steps. The header states the exact rules.
- `tools/kite_score` scores a log that has `gt` and `track` records.
- `tools/kite_baseline` runs a tracker on generated scenarios and prints a Markdown table with one
  row per seed and a mean row. Without options it runs the baseline tracker and prints the table in
  `docs/BASELINE.md`.
  - `--scenario straight|hard`: `straight` (default) is the scenario above, with output unchanged;
    `hard` adds turning, speed changes and clutter (2 false detections per step on average). The
    `hard` tables are in the "Hard scenario" section of `docs/BASELINE.md`.
  - `--tracker baseline|kalman`: `baseline` (default) is `NearestNeighbourBaseline`; `kalman` is
    `KalmanTracker`.
  - `--association two_stage|joint|joint_likelihood`: `KalmanTracker` association mode, only with
    `--tracker kalman`; `two_stage` is the default.
  - `--seeds N | A-B`: run seeds 1..N with N >= 1, or A..B with 1 <= A <= B (default 1..20).
  - `--grid A-B`: the Kalman tuning grid (q in {0.01, 0.1, 1, 10}, initial velocity sigma in {5, 10,
    20}) on seeds A..B. It prints one mean row per setting and the setting with the lowest mean of
    missed + false + id_switches, ties by lower rmse_m. Every seed must be a tuning seed (A >= 101);
    a range that includes a reporting seed exits 2. It cannot be combined with `--tracker`,
    `--association` or `--seeds`.
- `tools/kite_track --in <log> --out <log> [--tracker baseline|kalman]` runs a tracker over a log
  and writes a log with the input's `gt` and `meas` records plus one `track` record per output
  track and step. `kalman` is the default; both trackers use the `kite_baseline` settings from
  `tools/baseline_setup.hpp`. The tracker gets the `meas` records only, one time step at a time
  (the records that share a `t`, in time order), so a step with no `meas` record is not run. Track
  records already in the input are dropped. Score the output with `kite_score --log <out>`. A usage
  error exits 2; an unreadable input or unwritable output exits 1.
- `tools/kite_association_study` runs `KalmanTracker` on seeds 101-120 and prints tables of the
  measurements that tentative tracks took from confirmed tracks of the same vehicle.
  `tools/kite_drop_study` measures how often `KalmanTracker` drops a tentative track. Both are run
  through CTest. `docs/INVESTIGATION-ASSOCIATION.md` has their output and what it shows.
  - `--scenario straight|hard` (both tools): `straight` (default) prints exactly what the tools
    printed before the option; `hard` runs the hard scenario of `kite_baseline` on seeds 101-120
    and labels a measurement "clutter" when no truth vehicle is within 3 m. The hard tables state
    the clutter and ambiguous counts apart. `kite_drop_study --scenario hard` takes `--tuning` only
    and adds the count of duplicate tentative tracks of one car; `--monte-carlo` with `hard` exits
    2. The hard tables are in the "Hard scenario (seeds 101-120)" section of
    `docs/INVESTIGATION-ASSOCIATION.md`.

Ground truth is used for scoring and training labels only. It is never an input to the driving stack.
