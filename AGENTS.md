# AGENTS.md

Instructions for coding agents working in this repo. Where this file and ROADMAP.md exist, they
override any earlier handoff notes. A sprint brief overrides this file for that sprint.

## Project

Driving Stack: a modular autonomous-driving software stack (perception, tracking, prediction,
planning, control), developed and tested in the BeamNG.tech simulator. It is an independent,
non-commercial student project. Out of scope: real vehicles, real-world data and commercial use.

Stack: `core/` (C++17 library and tests), `sim/`, `tools/`, `third_party/` (vendored GoogleTest, never edited by
hand), `bridge/`, `perception/`, `scenarios/`, `eval/`, `viewer/`, `scripts/` (repo tooling),
`docs/`.

Public repository: everything committed, including PR text, is visible to anyone.

## Hardware & local environment

Single developer machine; all agents run here.

| Component | Spec |
|---|---|
| OS / shell | Windows 11 Home, PowerShell, Git Bash |
| CPU / RAM / GPU | Intel Core i7-13700K / 64 GB DDR5 / NVIDIA RTX 4070 SUPER 12 GB |
| Languages | C++17 (MinGW g++ 13.2 locally, MSVC and g++ in CI), CMake 4.4.4 (minimum 3.20), Ninja 1.13.2, Python 3.13 locally (3.12 in CI) |

- Paths: use forward slashes or Path APIs in code; scripts must work from PowerShell and Git Bash.

## Orientation (start of every item)

1. Read this file and ROADMAP.md (and the sprint brief).
2. `git checkout main`, `git pull`, then `gh pr list` to see open PRs.
3. Pick the first unchecked roadmap item whose dependencies are merged and that has no open PR.
4. If every item is done or blocked, stop and summarize. Do not invent new roadmap items.

## Check commands

The repo root is the directory containing `AGENTS.md`. Code and tests find the root by walking up
to `AGENTS.md`, never by counting `../`.

- C++ with the `dev` preset, run from the repo root against a worktree (the build stays in
  `.worktrees/<item>/build/dev`):
  - `cmake -S .worktrees/<item> --preset dev`
  - `cmake --build .worktrees/<item>/build/dev`
  - `ctest --test-dir .worktrees/<item>/build/dev --output-on-failure`
- C++ without the preset (same steps as the README's "Without the preset" block):
  - `cmake -S .worktrees/<item> -B .worktrees/<item>/build/dev -G Ninja -DCMAKE_BUILD_TYPE=Debug`
  - `cmake --build .worktrees/<item>/build/dev`
  - `ctest --test-dir .worktrees/<item>/build/dev --output-on-failure`
- Python script tests (from repo root; stdlib only): `python -m unittest discover -s scripts -v`.
  Against a worktree: `python -m unittest discover -s .worktrees/<item>/scripts -v`.
- Forbidden-files check: `python scripts/check_forbidden_files.py`. Against a worktree:
  `python .worktrees/<item>/scripts/check_forbidden_files.py`.

## Checking CI

The token cannot read check results, so never use `gh pr checks`. Use workflow runs:

1. `gh run list --branch <pr-branch> --limit 3` to get the latest run ID (wait ~20 s after a push).
2. `gh run watch <run-id> --exit-status` in the foreground; a non-zero exit means it failed.
3. `gh run view <run-id> --log-failed` to see the failures.
4. A known flaky test may be rerun once: `gh run rerun <run-id> --failed`, both runs logged. A second
   failure is real. Never cancel runs.
5. **Merging:** `gh pr ready <n>` starts a new CI run. After marking ready: wait about 20 s,
   `gh run list --branch <branch> --limit 3`, then `gh run watch <run-id> --exit-status` on that new
   run; merge only after it is green. If `gh pr merge` is refused with "base branch policy prohibits
   the merge", re-check that run and the branch's freshness (update from main if needed) once; a
   second refusal is NEEDS HUMAN with the exact message.

## Protocol (per item)

1. Branch `feat/<item-id>-<short-name>` in its own worktree
   (`python scripts/worktree.py create <item-id> --branch feat/<item-id>-<name>`).
2. Invoke poteto-mode and follow the matching playbook and its evidence rules.
3. Make the item's check commands pass locally. Capture real command output as evidence.
4. Tick the item's checkbox in ROADMAP.md in the same branch, never in a separate commit to main.
5. Push once per build round, when that round's local checks pass. The first push opens the PR as
   a **draft**. The PR body lives in `.run/pr-body-<item>.md` and is passed with
   `gh pr create --draft --body-file .run/pr-body-<item>.md`. It has what changed, how to verify,
   Evidence, NEEDS HUMAN. The PR body is the evidence record.
6. Review by a separate fresh reviewer (not the author): the diff against the item, the brief and
   these rules. Reviewers list the checks they ran and reply PASS or FAIL with reasons.
7. On PASS the conductor marks the PR ready (`gh pr ready <n>`), updates it from main, and waits
   for the workflow run on the exact head commit (see "Checking CI"); it must be green.
8. Merge order: update the branch from latest main, and wait for CI green on the exact commit. Then
   (1) `python .worktrees/<item>/scripts/stop_all.py`, (2) `gh pr merge --squash --delete-branch`,
   (3) `python scripts/worktree.py remove <item-id>` only if the folder is still there, then
   `git worktree prune` and check the folder is gone. Then `git checkout main`, `git pull`.
9. **Repo scripts:** run them by path **relative to the repo root**, never by absolute path:
   `python scripts/<x>.py` for the main checkout's copy, `python .worktrees/<item>/scripts/<x>.py`
   for a worktree's copy. Absolute paths are denied.
10. **PR bodies:** the body is in `.run/pr-body-<item>.md` (see step 5), passed with
    `gh pr create --draft --body-file .run/pr-body-<item>.md`, never in the worktree (inline bodies
    with `#` lines are refused). The body file is not committed.

## Working rules

- Exactly one builder at a time, then a separate fresh reviewer, then merge.
- Builders and reviewers invoke poteto-mode and follow the matching playbook.
- Reviewers use the strongest allowed model; builders a mid-tier model unless the item says otherwise.
- PR bodies have an "Evidence" section with real command output.
- Endpoint changes are verified with real HTTP requests against a running server in a test.
- Never edit `.claude/`, `.gitattributes`, or human-owned scripts listed here: `scripts/check_forbidden_files.py`, `scripts/forbidden_files.json`, `third_party/` (only its vendoring script changes it).
- Scratch work, if any, goes only in `scratch/<item>/` (gitignored). It is an optional work area,
  not an evidence store. Evidence goes in the PR body. During a sprint a guard blocks edits and
  test runs in the main checkout except in `.run/` and `scratch/` (the hook's `--allow scratch`);
  if the project's scratch folder has another name, change both here and in the hook entry.
- Fail loudly: missing inputs or unknown ids are errors with a non-zero exit, never silent skips.
- Check content, not just format.
- AGENTS.md is append-only for agents. Never remove or weaken an existing rule unless the brief
  explicitly says so. Reviewers FAIL any PR that drops a rule line without brief authorization.
- Tests never call paid services (fake providers or a local fake server) and never read the
  developer's real data folders or environment variables; they create temp folders and pass
  settings explicitly.
- Anything that costs money is off by default and enforced in the backend (see the brief).

## If stuck

After 3 failed attempts at the same problem, or if a decision is genuinely ambiguous: pick the
simplest reversible option and note it in the PR as an ASSUMPTION. If you truly cannot proceed,
leave the PR as a draft titled `BLOCKED: ...` with a clear question and move on to an independent
item. Never loop.

## Hard rules

- Never push to main. Never force-push. Never rewrite main's history. (A hook enforces this.)
- Never read, create, or commit .env files, keys, or tokens. Never print secrets.
- Never delete, skip, or weaken a test to make checks pass.
- Only edit `.github/workflows` in items that say so.
- Stage files explicitly by path; never `git add -A` or `git add .`.
- Run shell commands one at a time (no chaining with `;` or `&&`). Never start a command with
  `VAR=value`; pass settings as flags or config.
- Only download from package registries and GitHub.
- Stop every dev server or background process you start before you finish the item.
- Keep changes small and focused on the current item. Prefer less code.
- PR titles follow Conventional Commits: `type(R#): short description`, lowercase, imperative.
  Types: feat, fix, refactor, test, docs, ci, perf, chore. `chore: ...` for anything that is not a
  roadmap item; Dependabot's `chore(deps): ...` is allowed. Enforced by `.github/workflows/pr-title.yml`.
- No `Co-Authored-By` trailers and no "Generated with" lines in commits or PRs.
- Dependabot PRs are handled only as an item the sprint brief names (update the branch, rerun the
  checks, fix what the update breaks, fresh reviewer PASS with changelogs checked). Never merge one
  on your own initiative, and never a major update.
- Cross-platform: new code works on Windows, macOS and Linux. Every `.ps1` has a `.sh` twin.

## Processes and retry budget

- Every background process an agent or script starts is recorded:
  `python scripts/proc_registry.py record --pid <n> --cmd <argv...>`. Run
  `python .worktrees/<item>/scripts/stop_all.py` at the start and end of every item; it stops only
  recorded processes.
- The same failing command runs at most twice. After the first failure, diagnose and make one
  targeted fix. If it fails again, the item is BLOCKED.
- Denied commands are never retried or worked around. Add them to the NEEDS HUMAN list (exact
  command and why) in the final summary and the PR body.

## Worktrees and merge order

- Each item works in `.worktrees/<item>/` (gitignored) via `scripts/worktree.py`. A builder never
  edits the main checkout or another worktree.
- Merge one PR at a time: update the branch from main (`git merge origin/main`, no force-push),
  wait for green CI on it, then merge.

## CI cost rules

- Builders open draft PRs and push once per build round; never push work in progress to see CI.
- Pushes to main run only the light checks.
- No self-hosted runner (public repo); `HEAVY_RUNNER` stays unset, so heavy jobs run on GitHub-hosted runners.
- `scripts/ci_paths.json` maps changed paths to jobs; update it when you add a top-level folder.

## Standing rules

Each item that introduces a lasting rule adds a "## Standing rules (R<n>)" section below.

## Standing rules (R1)

- README stays honest: planned work says planned, no results without a named test, credit line kept.
- `docs/BUILD-LOG.md` gets a row in every item's PR.

## Standing rules (R2)

- `third_party/googletest` changes only through `scripts/vendor_googletest.py`; never edit it by hand.
- Warnings are errors on our own targets only (`kite_enable_warnings`), never on GoogleTest.
- C++ builds and tests stay in `<worktree>/build/` (gitignored); never commit build output.

## Standing rules (R3)

- C++ builds and tests run in CI on Linux g++ (`app`) and Windows MSVC (`app-windows`).
- A new top-level folder gets a rule in `scripts/ci_paths.json`; C++ folders map to both `app` and `app-windows`.
- No self-hosted runner: `HEAVY_RUNNER` stays unset and every job runs on a GitHub-hosted runner.

## Standing rules (R4)

- Planned folders say "planned" in their README until code lands there.
- The principles in `docs/PROJECT.md` are standing: ground truth scores, never drives; every number names the test that produced it.

## Standing rules (R5)

- The merging, repo-script and PR-body rules under "Checking CI" and "Protocol (per item)" are standing.
- This repository is public (see Project): committed files, PR titles, bodies and comments never contain secrets or the simulator's install paths, drive letters or environment variable names.

## Standing rules (R6)

- Repo scripts always run by path relative to the repo root, never by absolute path.

## Standing rules (R8)

- Measurement records and the `Measurement` type never carry a true identity.
- Any change to the log format bumps its version line and updates `docs/LOG-FORMAT.md` in the same PR.

## Standing rules (R9)

- Generated data uses one `std::mt19937_64` and our own sampling built only from exact operations
  (+, -, *, /, `sqrt`, `frexp`). Never use `std::*_distribution`, `std::shuffle` or a platform
  transcendental function (`log`, `sin`, `cos`, `exp`) for generated data.
- `sim/tests/data/seed42.log` changes only together with a deliberate generator change. Regenerate
  it with `kite_gen`; never edit it by hand.

## Standing rules (R10)

- Trackers implement `kite::Tracker` and receive only measurements: never ground truth, actor ids
  or the scenario config.

## Standing rules (R11)

- Ground truth reaches only the scorer (`kite_eval`); it is never passed to a tracker.
- The table in `docs/BASELINE.md` changes only by pasting `kite_baseline` output. The CTest test
  `kite_baseline_doc_matches` enforces this.
- The baseline is never tuned to score better.

## Standing rules (R12)

- Every builder and reviewer is a new agent started in the foreground; never run in the background,
  never resumed.
- The sprint progress file `.run/sprint-<name>-progress.md` is append-only and its last line is always
  `<!-- progress end -->`; each new line is added by editing that marker into "new line + marker".

## Standing rules (R13)

- Optimal matching within a gate (most pairs, then least total cost) uses `kite::solve_assignment`;
  no item adds a second optimal solver. The greedy `NearestNeighbourBaseline` is deliberate.
- `kite_core` builds with `-ffp-contract=off`, like `kite_sim`, so its floating-point results do not
  depend on the compiler's fused multiply-add choices.

## Standing rules (R14)

- Filters receive measurements only; only tests and the scorer compare filter output against
  ground truth.
- Fixed-size matrix code lives in `core/include/kite/matrix.hpp`; no item adds a second matrix
  library.

## Standing rules (R15)

- Tracker and filter parameters are tuned only on seeds 101-120; seeds 1-20 are the reporting set.
- The Kalman table in `docs/BASELINE.md` changes only by pasting `kite_baseline --tracker kalman`
  output; `kite_baseline_kalman_doc_matches` enforces it.
- `kite_baseline` without `--tracker` stays the baseline tracker, so its table and
  `kite_baseline_doc_matches` do not change.

## Standing rules (R16)

- Controllers receive the setpoint and measured state only, never ground truth from a scenario.
- Controller performance numbers (rise time, overshoot, steady-state error) in docs or PRs are
  copied from a test's printed output, never computed or estimated by hand.

## Standing rules (R21)

- The `KalmanTracker` step record is off by default and never changes any output; the test
  `KalmanTracker.StepRecordNeverChangesOutputOnTheTuningSeeds` enforces it.
- `AssociationMode::two_stage` stays the default; other association modes are for studies.
- Study tools may use ground truth only to label and classify; they never pass truth, actor ids or
  the scenario config to a tracker.
- `kite_baseline` and the study tools take the scenario and Kalman parameters from
  `tools/baseline_setup.hpp`; no tool keeps its own copy.
- Tables in `docs/INVESTIGATION-ASSOCIATION.md` change only by pasting the tool's output; a
  doc-matches CTest test enforces each one.

## Standing rules (R22)

- Every `KalmanTracker` association mode gates on d² <= gate_chi2 alone; a mode may change the
  cost of a pair inside the gate, never which pairs are inside. The test
  `KalmanTracker.EveryModeGatesOnTheSquaredDistanceAlone` covers every mode.
- Every cost passed to `kite::solve_assignment` is finite and >= 0.

## Standing rules (R23)

- The measurement labelling rule, a 3 m gate and a 1 m margin, lives only in
  `tools/study_common.hpp`. Study tools include it and keep no copy.
- Monte Carlo studies draw seeds from 100001 up and state the exact ranges in their output; seeds
  1-20 stay the reporting set.

## Standing rules (R26)

- Test helpers shared by several core test files live in `core/tests/test_helpers.hpp`. A helper shared by several tools lives in one header in `tools/` (`tools/format.hpp` for `format_int`, `tools/study_common.hpp` for the study tools' table helpers). Add to the matching header instead of copying a helper into a second file.

## Standing rules (R17)

- Only `bridge/kite_bridge/beamng_adapter.py` imports `beamngpy`, and only lazily inside methods.
- Bridge tests use only the fake simulator (`kite_bridge/fake_sim.py`); they never import the
  adapter and never start, connect to or locate the real simulator.
- The simulator's folder comes only from `--home`. Recorded data goes only to `--out`
  (`recordings/` by default, gitignored) and is never committed.
- Ground truth recorded from the simulator goes in its own file, apart from sensor data, and is
  never fed to anything that drives.
- The bridge never fills in missing sensor data silently. A reading without a sensor time keeps
  the poll time and is labelled `poll` in the table and JSON; a reading in an unexpected format is
  reported as unexpected, not counted as an empty poll.

## Standing rules (R18)

- CI's `python` job runs the bridge tests (`python -m unittest discover -s bridge -v`) with no
  installs, so bridge tests stay stdlib-only.
- A Markdown file that a CTest test reads is listed in `md_read_by_tests` in
  `scripts/ci_paths.json`, so changing it runs the C++ jobs.

## Standing rules (R19)

- Only `bridge/kite_bridge/rerun_view.py` imports `rerun`, and only lazily inside a method. Its
  Rerun calls are unverified until a real run and carry `# ASSUMPTION:` comments.
- Ground truth (our car's and the other cars' positions, anything from simulator state) is logged
  only under `ground_truth/`, never under a sensor path (`world/`, `camera/`), and is never fed
  to anything that drives. Inside a Rerun recording, the separate `ground_truth/` entity root is
  the separation between ground truth and sensor data.
- `.rrd` recordings are written only under `--out` (`--save` takes a name inside it) and are
  never committed.

## Standing rules (R28)

- Every sensor setting the adapter passes lives in `bridge/kite_bridge/sensor_settings.py` and
  nowhere else; scripts print it as `sensor:` lines, so never print or document a setting the
  adapter does not pass.
- `readings.py` never truth-tests a raw poll (use `is None` and `len()`); a poll that raises or is
  outside its stated input domain counts under `unexp` and never stops the run.

## Standing rules (R29)

- Bridge scripts connect with a socket timeout (`--timeout`) and end the simulator only through
  `common.session`'s one shutdown path; no script adds its own close or kill.
- A timeout from a simulator call is never caught as an ordinary failure. It stops the run, even
  where other errors are counted under `unexp` or skipped.
- The bridge ends only the process BeamNGpy launched, through its own handle; never by name or PID.

## Standing rules (R30)

- Bridge output goes only under `recordings/`, `scratch/` or outside the repo. The scripts refuse
  an `--out` that resolves inside the repo anywhere else (exit 2). Every JSON file carries the
  top-level `"provenance"` field and every Rerun recording logs it once, both from
  `PROVENANCE` in `bridge/kite_bridge/output_guard.py`; the text is defined nowhere else.
- Images in the repo live only under `docs/media/`, chosen by Evan, and are never simulator asset
  files.
- Never paste simulator error output into a public PR or issue text.

## Standing rules (R31)

- Under `bridge/` and `docs/`, the provenance marker text is spelled only in
  `bridge/kite_bridge/output_guard.py` and `bridge/tests/test_output_guard.py`. Docs describe
  the `provenance` field and point to the `PROVENANCE` constant;
  `bridge/tests/test_marker_spelling.py` fails if any other file there spells it.

## Standing rules (R32)

- Raw-poll byte counts and fingerprints come only from `bridge/kite_bridge/raw_data.py`; scripts
  and docs say what it documents and keep no second counting rule.
- `common.measure_sensors` takes its clock as a parameter. Tests use `FakeClock` with costs on
  `FakeSim` and never wait for real. A real-time factor with 0 wall seconds is `None` (`null` in
  JSON, `n/a` printed), never infinite.

## Standing rules (R33)

- `--rates` parsing and the choice of which steps a sensor is polled on live only in
  `bridge/kite_bridge/rates.py`; scripts and tests keep no second schedule. `--rates all=60` polls
  every sensor after every step, as the scripts did before `--rates`.
- A `possible loss` note is decided only from a requested update rate that `sensor_settings.py`
  passes; GPS and IMU have none, so their loss is checked by comparing `new` between two runs.

## Standing rules (R34)

- Every streamed reading goes through `bridge/kite_bridge/stream_check.py` before anything uses
  it; no script or module keeps a second check. What the check drops is counted (`suspect`
  readings, `sus_rows` rows) in the printed summary and the JSON, never dropped or passed on
  silently.
- Streamed reads are timed apart from polls (`stream_reads_s`, `in stream reads`). Without
  `--stream` the scripts poll every sensor and print what they printed before.
- No doc or PR claims streaming works on the real simulator until Evan's run shows it.

## Simulator files (hard rule)

- Never read, list, search, copy, launch or write about the simulator's install folder, its user
  folder, or the drive it is on, and never put any of its files (vehicles, maps, assets, scripts,
  configs, license files) in the repo. A guard hook blocks these actions and CI fails any PR that
  adds such files. Tests use the fake simulator; only Evan runs code against the real one.
- Code that needs the simulator's folder takes it as a command-line argument. Never hard-code a
  path to it and never read it from an environment variable.
- Never weaken scripts/check_forbidden_files.py or scripts/forbidden_files.json unless the brief
  says so and Evan approved it.
