# AGENTS.md

Instructions for coding agents working in this repo. Where this file and ROADMAP.md exist, they
override any earlier handoff notes. A sprint brief overrides this file for that sprint.

## Project

Driving Stack: FILL: one paragraph on what it is, who uses it, and what is out of scope.

Stack: FILL (folders and what lives in each).

Personal use / private repo unless PRODUCT.md says otherwise.

## Hardware & local environment

Single developer machine; all agents run here.

| Component | Spec |
|---|---|
| OS / shell | FILL (Windows 11, PowerShell; Git Bash available) |
| CPU / RAM / GPU | FILL |
| Languages | FILL (versions) |

- Paths: use forward slashes or Path APIs in code; scripts must work from PowerShell and Git Bash.

## Orientation (start of every item)

1. Read this file and ROADMAP.md (and the sprint brief).
2. `git checkout main`, `git pull`, then `gh pr list` to see open PRs.
3. Pick the first unchecked roadmap item whose dependencies are merged and that has no open PR.
4. If every item is done or blocked, stop and summarize. Do not invent new roadmap items.

## Check commands

The repo root is the directory containing `AGENTS.md`. Code and tests find the root by walking up
to `AGENTS.md`, never by counting `../`.

- FILL: the exact install, lint, test and build commands per area.
- Python script tests (from repo root; stdlib only): `python -m unittest discover -s scripts -v`.

## Checking CI

The token cannot read check results, so never use `gh pr checks`. Use workflow runs:

1. `gh run list --branch <pr-branch> --limit 3` to get the latest run ID (wait ~20 s after a push).
2. `gh run watch <run-id> --exit-status` in the foreground; a non-zero exit means it failed.
3. `gh run view <run-id> --log-failed` to see the failures.
4. A known flaky test may be rerun once: `gh run rerun <run-id> --failed`, both runs logged. A second
   failure is real. Never cancel runs.

## Protocol (per item)

1. Branch `feat/<item-id>-<short-name>` in its own worktree
   (`python scripts/worktree.py create <item-id> --branch feat/<item-id>-<name>`).
2. Invoke poteto-mode and follow the matching playbook and its evidence rules.
3. Make the item's check commands pass locally. Capture real command output as evidence.
4. Tick the item's checkbox in ROADMAP.md in the same branch, never in a separate commit to main.
5. Push once per build round, when that round's local checks pass. The first push opens the PR as
   a **draft** (`gh pr create --draft`): what changed, how to verify, Evidence, NEEDS HUMAN.
6. Review by a separate fresh reviewer (not the author): the diff against the item, the brief and
   these rules. Reviewers list the checks they ran and reply PASS or FAIL with reasons.
7. On PASS the conductor marks the PR ready (`gh pr ready <n>`), updates it from main, and waits
   for the workflow run on the exact head commit (see "Checking CI"); it must be green.
8. Copy evidence from the worktree into the main checkout's `scratch/<item>/` (file by file).
   Merge: `gh pr merge --squash --delete-branch`, then `python scripts/stop_all.py` in the worktree,
   `python scripts/worktree.py remove <item-id>`, `git checkout main`, `git pull`.

## Working rules

- Exactly one builder at a time, then a separate fresh reviewer, then merge.
- Builders and reviewers invoke poteto-mode and follow the matching playbook.
- Reviewers use the strongest allowed model; builders a mid-tier model unless the item says otherwise.
- PR bodies have an "Evidence" section with real command output.
- Endpoint changes are verified with real HTTP requests against a running server in a test.
- Never edit `.claude/`, `.gitattributes`, or human-owned scripts listed here: FILL.
- Scratch work goes only in `scratch/<item>/` (gitignored). During a sprint a guard blocks edits and
  test runs in the main checkout except in `.run/` and `scratch/` (the hook's `--allow scratch`); if
  the project's scratch folder has another name, change both here and in the hook entry.
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
  `python scripts/stop_all.py` at the start and end of every item; it stops only recorded processes.
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
- e2e runs only on ready PRs. Pushes to main run only the light checks.
- Heavy jobs use the `HEAVY_RUNNER` repository variable (self-hosted), with the fork guard in
  `runs-on`. Light jobs stay on GitHub-hosted runners.
- `scripts/ci_paths.json` maps changed paths to jobs; update it when you add a top-level folder.

## Standing rules

Each item that introduces a lasting rule adds a "## Standing rules (R<n>)" section below.
