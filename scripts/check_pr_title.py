"""Check a PR title against Conventional Commits `type(R#): description`.

Usage: PR_TITLE="..." python scripts/check_pr_title.py   (or pass the title as argv[1])
Exits 0 when valid, 1 when invalid.
"""

import os
import re
import sys

TYPES = "feat|fix|refactor|test|docs|ci|perf|chore"
# Scope is required (R followed by digits), except for `chore:` and Dependabot's
# `chore(deps)` / `chore(deps-dev)`.
# The description must start with a lowercase letter; the rest is free text.
PATTERN = re.compile(rf"^(?:(?:{TYPES})\(R\d+\)|chore(?:\((?:deps|deps-dev)\))?): [a-z]\S*(?: \S+)*$")


def is_valid(title: str) -> bool:
    return PATTERN.fullmatch(title) is not None


def main() -> int:
    title = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("PR_TITLE", "")
    if is_valid(title):
        print(f"OK: {title}")
        return 0
    print(f"INVALID PR title: {title!r}", file=sys.stderr)
    print("Expected: type(R#): lowercase description (`chore: ...` and `chore(deps): ...` allowed)", file=sys.stderr)
    print(f"Types: {TYPES.replace('|', ', ')}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
