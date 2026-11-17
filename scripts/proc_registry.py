"""Registry of background processes this repo's agents and scripts start.

One JSON file per process in <run dir>/procs/<pid>.json. The run dir is `.run/` at the repo root, or the
folder in env AGENT_RUN_DIR (tests use a temp folder). Record format:

    {"pid": 123, "cmd": ["npx", "vite"], "started": "2026-01-01T12:00:00+00:00",
     "created": 133500000000000000, "image": "node.exe"}

`created` is the OS process creation time (Windows FILETIME ticks, Linux clock ticks since boot) and `image` is
the executable name. Together they detect PID reuse: a live PID whose identity differs from the record is a
different process. Either may be null where the OS does not give it. PID-reuse protection covers Windows
and Linux only: on macOS, which has no /proc, identity is not checked and a recorded PID that is alive is
treated as the recorded process.

CLI (stdlib only, so PowerShell scripts can call it):
    python scripts/proc_registry.py record --pid N --cmd <argv...>   (--cmd goes last)
    python scripts/proc_registry.py remove --pid N
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

IS_WINDOWS = os.name == "nt"
RUN_DIR_ENV = "AGENT_RUN_DIR"


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "AGENTS.md").is_file():
            return parent
    raise RuntimeError("AGENTS.md not found above " + __file__)


def main_checkout(start=None) -> Path:
    """The main working tree of the repo containing `start` (default: this repo), even from a linked worktree.

    Falls back to that folder's own git top level, or to `start` itself, when git cannot say (no git, bare repo).
    """
    start = Path(start) if start else repo_root()
    try:
        out = subprocess.run(
            ["git", "-C", str(start), "rev-parse", "--path-format=absolute", "--git-common-dir"],
            capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return start
    common = Path(out)
    return common.parent if common.name == ".git" else start


def procs_dir() -> Path:
    base = os.environ.get(RUN_DIR_ENV)
    return (Path(base) if base else repo_root() / ".run") / "procs"


def _windows_identity(pid: int):
    import ctypes
    from ctypes import wintypes

    k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    k32.OpenProcess.restype = wintypes.HANDLE
    k32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    k32.CloseHandle.argtypes = [wintypes.HANDLE]
    k32.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    k32.GetProcessTimes.argtypes = [wintypes.HANDLE] + [ctypes.POINTER(wintypes.FILETIME)] * 4
    k32.QueryFullProcessImageNameW.argtypes = [
        wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR, ctypes.POINTER(wintypes.DWORD)]
    handle = k32.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
    if not handle:
        return None
    try:
        code = wintypes.DWORD()
        if not k32.GetExitCodeProcess(handle, ctypes.byref(code)) or code.value != 259:  # STILL_ACTIVE
            return None
        created, exited, kernel, user = (wintypes.FILETIME() for _ in range(4))
        ticks = None
        if k32.GetProcessTimes(handle, *(ctypes.byref(t) for t in (created, exited, kernel, user))):
            ticks = (created.dwHighDateTime << 32) | created.dwLowDateTime
        buf = ctypes.create_unicode_buffer(32768)
        size = wintypes.DWORD(len(buf))
        image = None
        if k32.QueryFullProcessImageNameW(handle, 0, buf, ctypes.byref(size)):
            image = os.path.basename(buf.value)
        return {"created": ticks, "image": image}
    finally:
        k32.CloseHandle(handle)


def _posix_identity(pid: int):
    try:
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8", errors="replace")
    except OSError:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return None
        except PermissionError:
            pass
        return {"created": None, "image": None}
    # comm may contain spaces and parentheses; the fields after the last ')' are fixed.
    name = stat[stat.index("(") + 1:stat.rindex(")")]
    fields = stat[stat.rindex(")") + 2:].split()
    if fields[0] in ("Z", "X"):  # zombie or dead: no longer running
        return None
    return {"created": int(fields[19]), "image": name}


def identity(pid: int):
    """Return {"created", "image"} for a running process, or None when it is not running."""
    return _windows_identity(pid) if IS_WINDOWS else _posix_identity(pid)


def same_process(recorded: dict, current: dict) -> bool:
    """False when the live process cannot be the one recorded (PID reuse)."""
    for key in ("created", "image"):
        a, b = recorded.get(key), current.get(key)
        if a is not None and b is not None and str(a).lower() != str(b).lower():
            return False
    return True


def record(pid: int, cmd: list) -> Path:
    current = identity(pid)
    if current is None:
        raise ValueError(f"pid {pid} is not running; nothing to record")
    entry = {
        "pid": pid,
        "cmd": list(cmd),
        "started": datetime.now(timezone.utc).isoformat(),
        **current,
    }
    directory = procs_dir()
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / f"{pid}.json"
    fd, tmp = tempfile.mkstemp(dir=directory, suffix=".tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        json.dump(entry, handle)
    os.replace(tmp, target)
    return target


def remove(pid: int) -> bool:
    try:
        (procs_dir() / f"{pid}.json").unlink()
        return True
    except FileNotFoundError:
        return False


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="action", required=True)
    rec = sub.add_parser("record", help="record a running process")
    rec.add_argument("--pid", type=int, required=True)
    rec.add_argument("--cmd", nargs=argparse.REMAINDER, required=True, help="the command line, as separate arguments")
    rem = sub.add_parser("remove", help="forget a process (no error if it was not recorded)")
    rem.add_argument("--pid", type=int, required=True)
    args = parser.parse_args(argv)
    if args.action == "record":
        if not args.cmd:
            parser.error("--cmd needs at least one argument")
        try:
            print(f"recorded pid {args.pid} in {record(args.pid, args.cmd)}")
        except ValueError as error:
            print(f"proc_registry.py: {error}", file=sys.stderr)
            return 1
    else:
        print(f"removed record for pid {args.pid}" if remove(args.pid) else f"no record for pid {args.pid}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
