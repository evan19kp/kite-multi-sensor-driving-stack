# Roadmap

Items are `R<n>`, numbered continuously across the project. Each sprint's first PR
(`chore: seed sprint <name> roadmap`) adds its items here unticked; each item's branch ticks only
its own line when it merges. Agents never add items that a brief didn't list.

## Sprint 0

- [ ] **R1 ...** (type): description, limits, tests.

## Sprint 0-smoke

- [ ] **R1 Honest README, build log and parking list** (docs): new `README.md` (title, short
  description, planned architecture, build and test, simulation only, credits, licence),
  `docs/BUILD-LOG.md` (who built what, with which model, in which PR) and `docs/LATER.md` (ideas
  parked for later); "Standing rules (R1)" in AGENTS.md. Only .md files; no results, no other
  product names, credit line `BeamNG.tech by BeamNG GmbH`. Tests: CI `gitleaks` job green
  (including the forbidden-files check); reviewer runs `python scripts/check_forbidden_files.py`
  and the forbidden-word searches.

## Later

Planned, not scheduled. Do not pick these up as roadmap items.

-
