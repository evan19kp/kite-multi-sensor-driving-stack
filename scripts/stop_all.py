"""Stop every process recorded in the run registry (scripts/proc_registry.py), and nothing else.

Run at the start and end of every item:  python scripts/stop_all.py

Recorded and still running: stopped (Windows: whole tree via taskkill; POSIX: SIGTERM, then SIGKILL).
Recorded but dead, or the PID now belongs to a different process: the record is cleared, the process is left alone.
Unrecorded processes are never looked at. Exit 1 if a recorded live process could not be stopped.
"""

import json
import os
import signal
import subprocess
import sys
import time

import proc_registry

STOP_WAIT_SECONDS = 10


def _wait_gone(pid: int, seconds: float) -> bool:
    deadline = time.monotonic() + seconds
    while proc_registry.identity(pid) is not None:
        if time.monotonic() > deadline:
            return False
        time.sleep(0.1)
    return True


def kill(pid: int) -> None:
    if proc_registry.IS_WINDOWS:
        subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True, check=False)
        return
    try:
        _signal(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    if not _wait_gone(pid, 3):
        try:
            _signal(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def _signal(pid: int, sig) -> None:
    """POSIX: signal the whole process group when `pid` leads its own (a backend started in a new session, whose
    forked JVM must not outlive Maven); otherwise only the process."""
    try:
        leads_group = os.getpgid(pid) == pid
    except OSError:
        leads_group = False
    if leads_group:
        os.killpg(pid, sig)
    else:
        os.kill(pid, sig)


def stop_all() -> int:
    failures = 0
    directory = proc_registry.procs_dir()
    for path in sorted(directory.glob("*.json")) if directory.is_dir() else []:
        try:
            entry = json.loads(path.read_text(encoding="utf-8"))
            pid = int(entry["pid"])
        except (OSError, ValueError, KeyError, TypeError):
            path.unlink(missing_ok=True)
            print(f"cleared unreadable record {path.name}")
            continue
        label = f"pid {pid} ({' '.join(map(str, entry.get('cmd', [])))})"
        current = proc_registry.identity(pid)
        if current is None:
            path.unlink(missing_ok=True)
            print(f"cleared stale record: {label} is not running")
        elif not proc_registry.same_process(entry, current):
            path.unlink(missing_ok=True)
            print(f"cleared stale record: {label} was reused by another process, left running")
        else:
            kill(pid)
            if _wait_gone(pid, STOP_WAIT_SECONDS):
                path.unlink(missing_ok=True)
                print(f"stopped {label}")
            else:
                failures += 1
                print(f"FAILED to stop {label}", file=sys.stderr)
    if not failures:
        print("stop_all: done")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(stop_all())
