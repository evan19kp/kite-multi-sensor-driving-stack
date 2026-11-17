"""Git worktrees for parallel roadmap items, one folder per item under the main checkout's `.worktrees/<item>/`.

    python scripts/worktree.py create <item> [--branch feat/<item>-<name>] [--base main]
    python scripts/worktree.py remove <item> [--force]
    python scripts/worktree.py list

`create` makes the branch from the up-to-date `origin/<base>` when the branch does not exist (fetching first when
an `origin` remote exists), else from the local `<base>`. It is safe to run again for an existing worktree.
`remove` refuses a worktree with uncommitted or untracked changes unless --force; it keeps the branch. Nothing
outside `.worktrees/` is ever touched. Works from the main checkout or from inside any worktree.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

import proc_registry

ITEM_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9-]{0,39}$")
WORKTREES = ".worktrees"


class WorktreeError(Exception):
    pass


def git(root: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess:
    result = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True)
    if check and result.returncode != 0:
        raise WorktreeError(f"git {' '.join(args)} failed: {(result.stderr or result.stdout).strip()}")
    return result


def worktree_path(root: Path, item: str) -> Path:
    if not ITEM_RE.match(item):
        raise WorktreeError(f"invalid item id {item!r}: use letters, digits and '-' (for example R40)")
    return root / WORKTREES / item


def registered(root: Path) -> dict:
    """{resolved worktree path: branch or None} for every git worktree of the repo."""
    found, current = {}, None
    for line in git(root, "worktree", "list", "--porcelain").stdout.splitlines() + [""]:
        if line.startswith("worktree "):
            current = Path(line[len("worktree "):]).resolve()
            found[current] = None
        elif line.startswith("branch ") and current is not None:
            found[current] = line[len("branch refs/heads/"):]
        elif not line:
            current = None
    return found


def has_ref(root: Path, ref: str) -> bool:
    return git(root, "rev-parse", "--verify", "--quiet", ref, check=False).returncode == 0


def create(root: Path, item: str, branch: str = None, base: str = "main") -> Path:
    path = worktree_path(root, item)
    branch = branch or f"feat/{item}"
    if git(root, "check-ref-format", "--branch", branch, check=False).returncode != 0:
        raise WorktreeError(f"invalid branch name {branch!r}")
    known = registered(root)
    if path.resolve() in known:
        if known[path.resolve()] != branch:
            raise WorktreeError(f"{path} already exists on branch {known[path.resolve()]}, not {branch}")
        return path
    if path.exists():
        raise WorktreeError(f"{path} exists but is not a git worktree; remove it by hand")
    if has_ref(root, f"refs/heads/{branch}"):
        git(root, "worktree", "add", str(path), branch)
        return path
    if git(root, "remote", "get-url", "origin", check=False).returncode == 0:
        git(root, "fetch", "origin", base)
        start = f"origin/{base}"
    else:
        start = base
    if not has_ref(root, start):
        raise WorktreeError(f"base {start} does not exist")
    git(root, "worktree", "add", "-b", branch, str(path), start)
    return path


def remove(root: Path, item: str, force: bool = False) -> Path:
    path = worktree_path(root, item)
    if path.resolve() not in registered(root):
        raise WorktreeError(f"{path} is not a worktree of this repo")
    if not force:
        dirty = git(path, "status", "--porcelain").stdout.strip()
        if dirty:
            raise WorktreeError(f"{path} has uncommitted changes (use --force to discard them):\n{dirty}")
    git(root, "worktree", "remove", *(["--force"] if force else []), str(path))
    return path


def listing(root: Path) -> list:
    base = (root / WORKTREES).resolve()
    return [(p.name, branch, p) for p, branch in sorted(registered(root).items()) if p.parent == base]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="action", required=True)
    mk = sub.add_parser("create", help="create .worktrees/<item> on the item's branch")
    mk.add_argument("item")
    mk.add_argument("--branch")
    mk.add_argument("--base", default="main")
    rm = sub.add_parser("remove", help="remove .worktrees/<item>")
    rm.add_argument("item")
    rm.add_argument("--force", action="store_true")
    sub.add_parser("list", help="list worktrees under .worktrees/")
    args = parser.parse_args(argv)
    root = proc_registry.main_checkout()
    try:
        if args.action == "create":
            print(f"worktree ready: {create(root, args.item, args.branch, args.base)}")
        elif args.action == "remove":
            print(f"removed worktree {remove(root, args.item, args.force)} (branch kept)")
        else:
            for item, branch, path in listing(root):
                print(f"{item}\t{branch}\t{path}")
    except WorktreeError as error:
        print(f"worktree.py: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
