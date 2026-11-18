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
- [ ] **R4 Folder stubs and project overview** (docs): `bridge/`, `perception/`, `scenarios/`,
  `eval/` and `viewer/` READMEs saying what is planned there; `docs/PROJECT.md` (project
  paragraph, planned stages, folder map, standing principles). Only .md files. Tests:
  forbidden-files check passes; reviewer checks every stub says planned.
- [ ] **R5 AGENTS.md filled in and protocol fixes** (docs): project, stack, public-repo line,
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

## Later

Planned, not scheduled. Do not pick these up as roadmap items.

-
