"""CI check: no forbidden files in the repo (another program's files, assets, binaries, keys).

Checks every tracked file (`git ls-files`, or the paths given on the command line) against the
rules in scripts/forbidden_files.json:
  - "extensions": file endings that may never be committed (e.g. ".jbeam", ".dll", ".key");
  - "name_patterns": file names that may never be committed (glob style, e.g. "*.materials.json");
  - "path_markers": text that may not appear in a file's path (e.g. "BeamNG/", "Bin64/");
  - "content_markers": text that may not appear inside a file (case-insensitive regular
    expressions, e.g. "bCDDL" or "copyright.*beamng gmbh");
  - "max_file_kb": the largest file allowed;
  - "content_exempt": files whose *content* is not scanned (this script, its test and its rules,
    which must name the markers). Their extension, name, path and size are still checked.
Exit code 1 with one line per problem, 0 when clean. Runs in CI on every pull request and push.
Run locally: python scripts/check_forbidden_files.py
"""
from __future__ import annotations

import fnmatch
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RULES = os.path.join(HERE, "forbidden_files.json")


def load_rules(path: str = RULES) -> dict:
    with open(path, encoding="utf-8") as f:
        r = json.load(f)
    r.setdefault("extensions", [])
    r.setdefault("name_patterns", [])
    r.setdefault("path_markers", [])
    r.setdefault("content_markers", [])
    r.setdefault("max_file_kb", 1024)
    r.setdefault("content_exempt", [])
    return r


def tracked_files(root: str) -> list[str]:
    out = subprocess.run(["git", "ls-files", "-z"], cwd=root, capture_output=True, check=True)
    return [p for p in out.stdout.decode("utf-8", "replace").split("\0") if p]


def problems_for(rel: str, root: str, rules: dict) -> list[str]:
    found = []
    path = rel.replace("\\", "/")
    low = path.lower()
    name = os.path.basename(low)
    for ext in rules["extensions"]:
        if low.endswith(ext.lower()):
            found.append(f"{path}: forbidden file type {ext}")
    for pat in rules["name_patterns"]:
        if fnmatch.fnmatch(name, pat.lower()):
            found.append(f"{path}: forbidden file name pattern {pat}")
    for marker in rules["path_markers"]:
        if marker.lower() in low:
            found.append(f"{path}: forbidden path ({marker})")
    full = os.path.join(root, rel)
    if not os.path.isfile(full):
        return found
    size_kb = os.path.getsize(full) / 1024
    if size_kb > rules["max_file_kb"]:
        found.append(f"{path}: {size_kb:.0f} KB is over the {rules['max_file_kb']} KB limit")
    if path in rules["content_exempt"]:
        return found
    try:
        with open(full, "rb") as f:
            text = f.read(5 * 1024 * 1024).decode("utf-8", "replace")
    except OSError as e:
        return found + [f"{path}: could not be read ({e})"]
    for marker in rules["content_markers"]:
        if re.search(marker, text, re.IGNORECASE):
            found.append(f"{path}: contains forbidden text matching {marker!r}")
    return found


def check(root: str, files: list[str], rules: dict) -> list[str]:
    return [p for rel in files for p in problems_for(rel, root, rules)]


def main(argv: list[str]) -> int:
    root = os.path.dirname(HERE)
    rules = load_rules()
    files = argv or tracked_files(root)
    found = check(root, files, rules)
    for line in found:
        print(f"FORBIDDEN: {line}")
    if found:
        print(f"{len(found)} problem(s). These files must not be in the repo: remove them (and from "
              "the branch history) before merging. See scripts/forbidden_files.json.")
        return 1
    print(f"Forbidden-files check passed ({len(files)} files).")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
