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

- [x] **R6 AGENTS.md, CLAUDE.md and README fixes** (docs): AGENTS.md machine table RAM "64 GB
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
- [ ] **R8 Shared types and log format v0** (feat): `core/include/kite/types.hpp` (`Vec2`,
  `GroundTruthState`, `Measurement` with no true identity, `TrackState`); `core/include/kite/log.hpp`
  and `core/src/log.cpp` (`write_log`, `read_log`; header `# kite-log v0`; `gt`, `meas`, `track`
  records; `std::to_chars` shortest form; `\n` line endings; loud errors naming the line number);
  `docs/LOG-FORMAT.md`. Tests: round trip of every record type, exact bytes of a tiny log, each
  error case.
- [ ] **R9 Synthetic scenario generator** (feat, needs R8): library `kite_sim` in `sim/`
  (`ScenarioConfig`, `generate`), constant-velocity vehicles, Bernoulli detections with Gaussian
  noise, seeded shuffle; one `std::mt19937_64`, own uniform and Box-Muller sampling, no
  `std::*_distribution`; CLI `kite_gen`. Tests: determinism, seed sensitivity, constant velocity,
  pd 1 and 0, noise statistics, no actor id in measurements, golden file
  `sim/tests/data/seed42.log` byte for byte on every platform.
- [ ] **R10 Tracker interface and the no-filter baseline tracker** (feat, needs R8): abstract
  `Tracker` in `core/include/kite/tracker.hpp` (measurements only, never ground truth);
  `NearestNeighbourBaseline` (greedy nearest-neighbour within a gate, velocity by difference, ids
  never reused). Tests: new track, kept id and velocity, beyond gate, crossing case, empty input,
  no id reuse.
- [ ] **R11 Scoring harness and the measured baseline** (feat, Opus builder; needs R8, R9, R10):
  library `kite_eval` in `eval/` (`score`, Hungarian matching within a gate; matched, missed, false,
  position RMSE, identity switches, steps); CLIs `kite_score` and `kite_baseline`;
  `docs/BASELINE.md` with the command and its real output. Tests: hand-made scoring cases including
  one where greedy matching is wrong.

## Later

Planned, not scheduled. Do not pick these up as roadmap items.

-
