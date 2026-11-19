# Roadmap

Items are `R<n>`, numbered continuously across the project. Each sprint's first PR
(`chore: seed sprint <name> roadmap`) adds its items here unticked; each item's branch ticks only
its own line when it merges. Agents never add items that a brief didn't list.

## Sprint 0

- [x] **R2 C++ build skeleton with vendored GoogleTest** (build): `scripts/vendor_googletest.py`
  (stdlib only; downloads the pinned GoogleTest release tag from GitHub, prints its SHA-256,
  extracts only `CMakeLists.txt`, `LICENSE`, `googletest/`, `googlemock/`, refuses absolute or
  `..` members and non-empty destinations without `--force`, writes `VENDORED.md`) with offline
  tests via `--archive`; GoogleTest vendored in `third_party/googletest`; root `CMakeLists.txt`
  (CMake 3.20+, C++17, warnings as errors for our targets only), `CMakePresets.json` (`dev`);
  `core/` library `kite_core` with `kite::wrap_angle` and GoogleTest/gMock tests; README build and
  test commands. Tests: `cmake --preset dev`, build and `ctest` pass; Python script tests and
  forbidden-files check pass; CI green.
- [x] **R3 CI builds and tests C++ on Linux and Windows** (ci): `app` job builds and tests with
  g++ and Ninja on Linux; new `app-windows` job with MSVC; `e2e` job removed; `scripts/ci_paths.json`
  and its test updated; `actions/checkout`, `actions/setup-python` and `gitleaks/gitleaks-action`
  bumped to their newest major tags. No self-hosted runner; `gitleaks` job name unchanged. Tests:
  PR CI shows `app` and `app-windows` green with the R2 tests in their logs; Python script tests
  pass.
- [x] **R4 Folder stubs and project overview** (docs): `bridge/`, `perception/`, `scenarios/`,
  `eval/` and `viewer/` READMEs saying what is planned there; `docs/PROJECT.md` (project
  paragraph, planned stages, folder map, standing principles). Only .md files. Tests:
  forbidden-files check passes; reviewer checks every stub says planned.
- [x] **R5 AGENTS.md filled in and protocol fixes** (docs): project, stack, public-repo line,
  machine table, check commands, human-owned files, CI cost rules and the three new protocol rules
  (merging after `gh pr ready`, repo scripts by relative path, PR bodies with `--body-file`). Only
  AGENTS.md, ROADMAP.md and `docs/BUILD-LOG.md`. Tests: no `FILL` left; reviewer compares old and
  new AGENTS.md line by line.

## Sprint 0-smoke

- [x] **R1 Honest README, build log and parking list** (docs): new `README.md` (title, short
  description, planned architecture, build and test, simulation only, credits, licence),
  `docs/BUILD-LOG.md` (who built what, with which model, in which PR) and `docs/LATER.md` (ideas
  parked for later); "Standing rules (R1)" in AGENTS.md. Only .md files; no results, no other
  product names, credit line `BeamNG.tech by BeamNG GmbH`. Tests: CI `gitleaks` job green
  (including the forbidden-files check); reviewer runs `python scripts/check_forbidden_files.py`
  and the forbidden-word searches.

## Sprint 0A.1

- [x] **R6 AGENTS.md and CLAUDE.md fixes** (docs): AGENTS.md machine table RAM "64 GB
  DDR5"; protocol step 5 PR body in `.run/pr-body-<item>.md` with `--body-file` (the body is the
  evidence record); step 8 drops the `scratch/` evidence copy and runs
  `python .worktrees/<item>/scripts/stop_all.py`; `scratch/` stays an optional work area only; new
  rule "Repo scripts always run by path relative to the repo root, never by absolute path."; step 2
  kept. CLAUDE.md graphify section replaced by "Graphify: not set up yet (the director says when).
  Until then, read files directly." with no absolute path. Tests: CLAUDE.md has no `C:/` and no
  drive letter; reviewer diffs AGENTS.md line by line; forbidden-files check passes.
- [x] **R7 CI path rules for the real folders** (ci): `scripts/ci_paths.json` drops the
  `frontend/`, `backend/`, `src/` rules and npm/maven `seed_paths` (keeps `.github/workflows/`);
  adds `sim/`, `eval/`, `tools/` to `app` and `app-windows`; `bridge/`, `perception/`,
  `scenarios/`, `viewer/` to `python`; `docs/` stays `["app"]`. `scripts/ci_changes.py` docstring
  drops `e2e`. `scripts/test_ci_changes.py` tests each new and removed prefix and finds the repo
  root by walking up to `AGENTS.md`. Tests: Python script tests pass; CI green.
- [x] **R8 Shared types and log format v0** (feat): `core/include/kite/types.hpp` (`Vec2`,
  `GroundTruthState`, `Measurement` with no true identity, `TrackState`); `core/include/kite/log.hpp`
  and `core/src/log.cpp` (`write_log`, `read_log`; header `# kite-log v0`; `gt`, `meas`, `track`
  records; `std::to_chars` shortest form; `\n` line endings; loud errors naming the line number);
  `docs/LOG-FORMAT.md`. Tests: round trip of every record type, exact bytes of a tiny log, each
  error case.
- [x] **R9 Synthetic scenario generator** (feat, needs R8): library `kite_sim` in `sim/`
  (`ScenarioConfig`, `generate`), constant-velocity vehicles, Bernoulli detections with Gaussian
  noise, seeded shuffle; one `std::mt19937_64`, own uniform and Box-Muller sampling, no
  `std::*_distribution`; CLI `kite_gen`. Tests: determinism, seed sensitivity, constant velocity,
  pd 1 and 0, noise statistics, no actor id in measurements, golden file
  `sim/tests/data/seed42.log` byte for byte on every platform.
- [x] **R10 Tracker interface and the no-filter baseline tracker** (feat, needs R8): abstract
  `Tracker` in `core/include/kite/tracker.hpp` (measurements only, never ground truth);
  `NearestNeighbourBaseline` (greedy nearest-neighbour within a gate, velocity by difference, ids
  never reused). Tests: new track, kept id and velocity, beyond gate, crossing case, empty input,
  no id reuse.
- [x] **R11 Scoring harness and the measured baseline** (feat, Opus builder; needs R8, R9, R10):
  library `kite_eval` in `eval/` (`score`, Hungarian matching within a gate; matched, missed, false,
  position RMSE, identity switches, steps); CLIs `kite_score` and `kite_baseline`;
  `docs/BASELINE.md` with the command and its real output. Tests: hand-made scoring cases including
  one where greedy matching is wrong.

## Sprint 0A.2

- [x] **R12 Docs, licence and protocol fixes** (docs): `LICENSE` (all rights reserved, viewing
  only; third-party components keep their own licences); README "Licence" section points to it;
  AGENTS.md step 8 gets the sprint merge order, step 10 puts the PR body in
  `.run/pr-body-<item>.md`, new rules for fresh foreground agents and the progress-file end marker,
  `stop_all.py` copies agree; `docs/PROJECT.md` folder map gains `sim/` and `tools/` and its line 3
  matches README; R6's roadmap title reworded. With `.gitattributes` on main, the golden test drops
  its `\r` stripping. Tests: forbidden-files check passes; reviewer diffs AGENTS.md line by line;
  LICENSE text matches exactly; golden test passes on both CI jobs.
- [x] **R13 Shared assignment solver with a brute-force cross-check** (refactor + test): Hungarian
  solver moves from `eval/src/scoring.cpp` to `core/include/kite/assignment.hpp` +
  `core/src/assignment.cpp` (`solve_assignment(cost, gate)`: most pairs within the gate, then least
  total cost); the scorer calls it with no output change; big-M overflow fixed for huge or infinite
  gates. Tests: `core/tests/assignment_bruteforce_test.cpp` compares 5,000 seeded random cases
  (0-6 rows, 0-6 columns, rectangular and empty) with a brute-force solver; every scoring test and
  `kite_baseline_doc_matches` pass unchanged.
- [x] **R14 Constant-velocity Kalman filter** (feat, Opus builder): `ConstantVelocityKalman` in
  `core/include/kite/kalman.hpp` + `core/src/kalman.cpp` (state `[x, y, vx, vy]`, white-noise
  acceleration `predict(dt)`, Joseph-form `update`, `innovation_mahalanobis2`, `initialize`); small
  fixed-size matrix helpers in `core/`, each tested. Tests (with derivations): exact noiseless
  predict, covariance grows and shrinks, stays symmetric positive definite over 1,000 steps,
  stationary convergence, NEES inside its 95 % chi-square bounds on generator data, `dt = 0` no-op,
  non-finite input throws.
- [x] **R15 Kalman multi-vehicle tracker with data association, scored** (feat, Opus builder; needs
  R13, R14): `KalmanTracker : Tracker` (predict, chi-square gate 9.21, global nearest neighbour via
  `solve_assignment`, tentative tracks confirmed after M of N hits, deleted after K misses, confirmed
  tracks only, ids never reused); `kite_baseline --tracker baseline|kalman --seeds A-B`;
  `docs/BASELINE.md` gains a Kalman table on seeds 1-20 (tuned on seeds 101-120 only), a second
  doc-matches test and a short comparison. Tests: confirmation, coasting and deletion, crossing
  targets keep ids where the baseline swaps, far clutter never confirms, empty steps.
- [x] **R16 PID controller with a speed-tracking test plant** (feat, Opus builder): `Pid` in
  `core/include/kite/pid.hpp` + `core/src/pid.cpp` (output limits, anti-windup, derivative on
  measurement with optional low-pass, `reset()`, throws on bad `dt` or non-finite input); test plant
  with first-order actuator lag and acceleration limits [-6, 3] m/s². Tests: hand-computed P/I/D
  cases, clamping, anti-windup against a plain integrator, no derivative kick, closed loop 0 to
  15 m/s within 2 % with printed rise time, overshoot and steady-state error.

## Sprint 0A.2.1

Investigation sprint: measure the Kalman tracker's association and tentative-track rules; no
default or parameter changes. Seeds 1-20 never appear in any output.

- [x] **R21 Association diagnostics: why do tentative tracks steal measurements?** (feat, Opus
  builder): optional per-step observer record in `KalmanTracker` (off by default; per track id,
  tentative/confirmed, age, hits, misses, predicted position, innovation covariance S, matched
  measurement; per unmatched measurement the tentative track it started); association mode
  constructor parameter (`two_stage` default, byte-identical; `joint`: one `solve_assignment` over
  all tracks, same costs and gate, shared lifecycle code); study tool `tools/kite_association_study`
  (CTest only) on seeds 101-120 that labels measurements with truth (3 m gate, 1 m margin, else
  ambiguous), finds steals, classifies them H1 / nearer / other, counts duplicates and ambiguous
  labels; `docs/INVESTIGATION-ASSOCIATION.md` section "R21: why measurements get stolen" with
  doc-matches tests. Tests: observer on/off identical on seeds 101-120; default outputs unchanged;
  hand-made two-track steal with both scores' arithmetic in comments.
- [x] **R22 Compare association methods on the tuning seeds** (feat, Opus builder; needs R21):
  third mode `joint_likelihood` (cost d² + ln(det S / det R), derivation in the header; gate stays
  d² <= gate_chi2 in every mode); `kite_baseline --association two_stage|joint|joint_likelihood`
  (default `two_stage`, only with `--tracker kalman`); `docs/INVESTIGATION-ASSOCIATION.md` section
  "R22: comparison on seeds 101-120" with three doc-matched tables (`two_stage` mean row equals
  `kite_baseline_kalman_tuning`'s) and steals, duplicates and reverse steals per mode. No winner
  chosen, no default changed. Tests: hand-made cases where the modes pair differently; gate test in
  all three modes; mode parsing and usage errors.
- [x] **R23 Measure how often a real car's new track is dropped** (test, Sonnet; needs R21): Monte
  Carlo with the 0A.1 generator, 1 vehicle, default tracker, p in {0.95, 0.90, 0.80, 0.70}, 20,000
  seeds per p from 100001 (exact ranges stated): births, dropped, rate with 95 % Wilson interval,
  the M-of-N formula value, and miss causes (no measurement vs outside gate); on seeds 101-120 the
  dropped tentative tracks split into real births lost vs the rest, and mean steps to confirmation;
  `docs/INVESTIGATION-ASSOCIATION.md` section "R23: tentative tracks dropped" with doc-matches
  tests. No parameter or drop-rule change.

## Sprint 0A.2.2

Safe cleanup sprint: docs, missing tests and duplicate helpers. No behaviour or printed-output
change; every existing test passes unchanged.

- [x] **R24 Docs refresh** (docs, Sonnet): README.md line 3 and `docs/PROJECT.md` line 3 status
  line updated to the Phase 0A status (nothing else in README.md); `eval/README.md` describes
  `kite_baseline`'s `--tracker`, `--seeds` and `--association` options from its usage text and adds
  `tools/kite_association_study` and `tools/kite_drop_study` with a pointer to
  `docs/INVESTIGATION-ASSOCIATION.md`; the R21 sentence about the 314 ambiguous measurements
  reworded (prose only, no table or number change). Tests: every doc-matches test passes
  unchanged; forbidden-files check passes; reviewer checks each new sentence against the code.
- [x] **R25 Missing tests the reviewers flagged** (test, Sonnet): unit tests only, no production
  change: `KalmanTracker` gate boundary at `gate_chi2`; a `joint_likelihood`-only gate case where
  the outside-gate pair has the lower cost; a throwing `update()` leaves the previous step record
  unchanged; R21 steal classification (nearer / H1 / other) on hand-made records; R23 first
  confirmation step kept and a track switching labels; `cholesky` result is lower triangular.
  Tests: each new test mutation-checked; total test count before and after; existing tests
  unchanged.
- [x] **R26 Remove duplicate helpers** (refactor, Sonnet): one shared test helper header for
  `uniform01` and `kInf` (the generator's own `uniform01` untouched); one shared `format_int` in
  `tools/` with byte-identical output; unused `<cmath>` removed from
  `tools/kite_association_study.cpp`; the misleading `core/src/kalman_tracker.cpp` error message
  near a 1e308 gate states the real condition, with a message test. Tests: full `ctest` and Python
  tests pass with the same count plus any message test; no printed output, table, golden file or
  random sequence changes.

## Sprint 0B

BeamNG.tech spike scripts, tested against a fake simulator, for Evan to run on the real one.
Agents never run the real simulator; every test uses the fake.

- [x] **R17 Simulator adapter, fake simulator and the connect/sensor/multi-car spike scripts**
  (feat, Sonnet): package `bridge/kite_bridge/` with `sim_api.py` (small simulator interface),
  `beamng_adapter.py` (the only module that imports `beamngpy`, lazily), `fake_sim.py` (seeded
  synthetic readings with configurable rates, dropouts and latency) and `measure.py` (pure
  functions: effective update rate, poll latency mean/p50/p95/max, empty and repeated readings,
  steps per second; documented input domain); scripts `bridge/spike_connect.py`,
  `bridge/spike_sensors.py`, `bridge/spike_multicar.py` with `--home` (required unless `--fake`),
  `--headless`, `--seconds`, `--traffic`, `--out`, `--fake`; `bridge/requirements.txt`
  (`beamngpy==1.36`, `rerun-sdk`); `.gitignore` adds `recordings/`. Tests: each script end to end
  against the fake writes valid JSON; hand-computed rates and percentiles; injected dropouts and
  repeats counted exactly; missing `--home` without `--fake` is a usage error; `beamngpy` never
  imported by tests.
- [ ] **R18 CI runs the bridge tests** (ci, Sonnet; needs R17): the `python` job also runs
  `python -m unittest discover -s bridge -v` (no installs); a test confirms `bridge/` changes select
  `python`; `scripts/ci_paths.json` maps `docs/INVESTIGATION-ASSOCIATION.md` to `app` and
  `app-windows` in `md_read_by_tests`, with a test. Tests: CI's `python` job log shows the bridge
  tests running and passing.
- [ ] **R19 Live lidar into Rerun for Demo 1** (feat, Sonnet; needs R17):
  `bridge/kite_bridge/rerun_view.py` (the only module that imports `rerun`, lazily; one simulator
  timeline; lidar coloured by distance near blue to far red, our car's position, the camera image,
  other cars as a separate ground-truth layer); `bridge/spike_lidar_rerun.py` (traffic mode with AI
  traffic, live viewer, `--save <file.rrd>` inside `--out`). Tests: with `--fake` and a fake Rerun
  logger: log calls per step, colours monotonic in distance, ground truth only under its own entity
  path, recording path inside `--out`; `rerun` never imported by tests.
- [ ] **R20 How Evan runs the spike** (docs, Sonnet; needs R17, R19): `docs/SPIKE-0B.md` (setup
  with PowerShell commands, each script's exact command with the `--home` placeholder, expected
  output, run time, what to send back, a headless run, the Demo 1 run with and without `--save`,
  what to do when a BeamNGpy call fails, recordings stay out of the repo); one README.md line under
  "Build and test". Tests: forbidden-files check passes; every command matches the scripts' real
  `--help`.

## Later

Planned, not scheduled. Do not pick these up as roadmap items.

-
