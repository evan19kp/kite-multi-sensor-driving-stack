#!/usr/bin/env python3
"""Decide which CI jobs a pull request needs from the files it changes.

CI's light `gitleaks` job runs this on the PR's merge commit (changed files = `git diff --name-only HEAD^1
HEAD`) and writes `<job>=true|false` lines to $GITHUB_OUTPUT, plus `seed=true|false`. Each heavy job in
ci.yml keeps its name and skips itself with a job-level `if:` (a skipped job counts as passing for required
checks). An empty output (this script crashed) is not 'false', so every job runs.

The mapping lives in scripts/ci_paths.json:
  {
    "jobs": ["app", "app-windows", "python"],            every gated job name in ci.yml
    "always_all": [".github/workflows/", "scripts/ci_changes.py", "scripts/ci_paths.json"],
    "rules": [{"prefix": "core/", "jobs": ["app", "app-windows"]}, ...],   first matching prefix wins
    "md_read_by_tests": {"DESIGN.md": ["app"]},          Markdown files that tests read
    "seed_paths": [".github/workflows/"]                 inputs of the dependency caches
  }
Rules, in order: always_all → every job; md_read_by_tests → its jobs; any other *.md → nothing; the first
rule whose prefix matches → its jobs; an unknown path → every job, so nothing slips through. No files at all
→ every job and seed=true.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

CONFIG = Path(__file__).resolve().with_name("ci_paths.json")


def load(path: Path = CONFIG) -> dict:
    cfg = json.loads(path.read_text(encoding="utf-8"))
    jobs = cfg.get("jobs") or []
    if not jobs:
        raise ValueError("ci_paths.json: 'jobs' is empty")
    for rule in cfg.get("rules", []):
        unknown = set(rule.get("jobs", [])) - set(jobs)
        if unknown:
            raise ValueError(f"ci_paths.json: rule {rule.get('prefix')!r} names unknown jobs {sorted(unknown)}")
    return cfg


def jobs_for(path: str, cfg: dict) -> frozenset[str]:
    path = path.replace("\\", "/")
    every = frozenset(cfg["jobs"])
    if path.startswith(tuple(cfg.get("always_all", []))):
        return every
    md = cfg.get("md_read_by_tests", {})
    if path in md:
        return frozenset(md[path])
    if path.endswith(".md"):
        return frozenset()
    for rule in cfg.get("rules", []):
        if path.startswith(rule["prefix"]):
            return frozenset(rule["jobs"])
    return every


def classify(paths: list[str], cfg: dict) -> dict[str, bool]:
    if not paths:
        return {job: True for job in cfg["jobs"]}
    needed = frozenset().union(*(jobs_for(p, cfg) for p in paths))
    return {job: job in needed for job in cfg["jobs"]}


def needs_seed(paths: list[str], cfg: dict) -> bool:
    if not paths:
        return True
    seeds = tuple(cfg.get("seed_paths", []))
    return bool(seeds) and any(p.replace("\\", "/").startswith(seeds) for p in paths)


def changed_files() -> list[str]:
    out = subprocess.run(["git", "diff", "--name-only", "--no-renames", "HEAD^1", "HEAD"],
                         check=True, capture_output=True, text=True).stdout
    return [line for line in out.splitlines() if line]


def main() -> int:
    cfg = load()
    paths = changed_files()
    lines = [f"{job}={'true' if on else 'false'}" for job, on in classify(paths, cfg).items()]
    lines.append(f"seed={'true' if needs_seed(paths, cfg) else 'false'}")
    print(f"{len(paths)} changed file(s)")
    for path in paths:
        print(f"  {path}")
    print("\n".join(lines))
    target = os.environ.get("GITHUB_OUTPUT")
    if target:
        with open(target, "a", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
