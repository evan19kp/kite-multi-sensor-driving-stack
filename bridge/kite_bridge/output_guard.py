"""Keeps what the bridge writes out of the repo. Everything the scripts write comes from the
simulator and must never be committed: `PROVENANCE` marks it, and `out_refusal` rejects an output
folder inside the repo unless it is `recordings/` or `scratch/`, both git-ignored."""

from pathlib import Path

PROVENANCE = "BeamNG.tech-generated data - never commit"
ALLOWED_FOLDERS = ("recordings", "scratch")
WORKTREES_FOLDER = ".worktrees"
ROOT_MARKER = "AGENTS.md"


def repo_roots(start=None):
    """The repo root, found by walking up from `start` (default: this file) to the first folder
    with AGENTS.md. A worktree (`<main>/.worktrees/<item>/`) has its own AGENTS.md and sits
    inside the main checkout, so its main checkout follows it in the list. The last entry is the
    outermost root: the whole repo. Raises RuntimeError when there is no AGENTS.md above."""
    here = Path(start or __file__).resolve()
    nearest = next((p for p in here.parents if (p / ROOT_MARKER).is_file()), None)
    if nearest is None:
        raise RuntimeError(f"{ROOT_MARKER} not found above {here}")
    main = nearest.parent.parent
    if nearest.parent.name == WORKTREES_FOLDER and (main / ROOT_MARKER).is_file():
        return [nearest, main]
    return [nearest]


def out_refusal(out, roots, cwd=None):
    """None when `out` may be written, else the reason it may not. `out` is resolved (relative
    to `cwd`, default the current folder; `..` and symlinks followed) first. A folder outside the
    outermost of `roots` is allowed. Inside it, only `recordings/` and `scratch/` under any of
    `roots` are. Domain: `roots` is a non-empty list of folders, outermost last."""
    roots = [Path(root).resolve() for root in roots]
    folder = (Path(cwd) if cwd else Path.cwd()).joinpath(out).resolve()
    if not folder.is_relative_to(roots[-1]):
        return None
    if any(folder.is_relative_to(root / name) for root in roots for name in ALLOWED_FOLDERS):
        return None
    allowed = ", ".join(f"{name}/" for name in ALLOWED_FOLDERS)
    return f"{out!r} is inside the repo; use {allowed} or a folder outside the repo"
