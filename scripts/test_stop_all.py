import json
import os
import subprocess
import sys
import tempfile
import types
import unittest
import unittest.mock
from pathlib import Path

import proc_registry
import stop_all

SLEEPER = [sys.executable, "-c", "import time; time.sleep(60)"]
SCRIPTS = Path(__file__).resolve().parent


class StopAllTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        patch = unittest.mock.patch.dict(os.environ, {proc_registry.RUN_DIR_ENV: tmp.name})
        patch.start()
        self.addCleanup(patch.stop)
        self.children = []
        self.addCleanup(self.reap)

    def spawn(self):
        child = subprocess.Popen(SLEEPER)
        self.children.append(child)
        return child

    def reap(self):
        for child in self.children:
            if child.poll() is None:
                child.kill()
            child.wait(timeout=10)

    def record_file(self, pid):
        return proc_registry.procs_dir() / f"{pid}.json"

    def test_stops_recorded_live_process_and_clears_record(self):
        child = self.spawn()
        proc_registry.record(child.pid, SLEEPER)
        self.assertEqual(stop_all.stop_all(), 0)
        child.wait(timeout=10)
        self.assertIsNotNone(child.poll())
        self.assertFalse(self.record_file(child.pid).exists())

    def test_clears_stale_record_of_dead_pid(self):
        child = self.spawn()
        proc_registry.record(child.pid, SLEEPER)
        child.kill()
        child.wait(timeout=10)
        self.assertEqual(stop_all.stop_all(), 0)
        self.assertFalse(self.record_file(child.pid).exists())

    def test_never_touches_unrecorded_process(self):
        recorded = self.spawn()
        bystander = self.spawn()
        proc_registry.record(recorded.pid, SLEEPER)
        self.assertEqual(stop_all.stop_all(), 0)
        recorded.wait(timeout=10)
        self.assertIsNone(bystander.poll())
        self.assertIsNotNone(proc_registry.identity(bystander.pid))

    def test_reused_pid_is_cleared_but_not_killed(self):
        child = self.spawn()
        path = proc_registry.record(child.pid, SLEEPER)
        entry = json.loads(path.read_text(encoding="utf-8"))
        if entry["created"] is None:
            self.skipTest("no process creation time on this platform")
        entry["created"] += 1
        path.write_text(json.dumps(entry), encoding="utf-8")
        self.assertEqual(stop_all.stop_all(), 0)
        self.assertIsNone(child.poll())
        self.assertFalse(path.exists())

    def test_reused_pid_with_other_image_is_not_killed(self):
        child = self.spawn()
        path = proc_registry.record(child.pid, SLEEPER)
        entry = json.loads(path.read_text(encoding="utf-8"))
        entry.update(created=None, image="definitely-not-python")
        path.write_text(json.dumps(entry), encoding="utf-8")
        self.assertEqual(stop_all.stop_all(), 0)
        self.assertIsNone(child.poll())
        self.assertFalse(path.exists())

    def test_unreadable_record_is_cleared(self):
        directory = proc_registry.procs_dir()
        directory.mkdir(parents=True)
        (directory / "999.json").write_text("not json", encoding="utf-8")
        self.assertEqual(stop_all.stop_all(), 0)
        self.assertFalse((directory / "999.json").exists())

    def test_empty_registry_is_fine(self):
        self.assertEqual(stop_all.stop_all(), 0)

    def test_failure_to_stop_exits_non_zero_and_keeps_record(self):
        child = self.spawn()
        path = proc_registry.record(child.pid, SLEEPER)
        with unittest.mock.patch.object(stop_all, "kill"), \
                unittest.mock.patch.object(stop_all, "STOP_WAIT_SECONDS", 0.3):
            self.assertEqual(stop_all.stop_all(), 1)
        self.assertTrue(path.exists())
        self.assertIsNone(child.poll())


class PosixKillTest(unittest.TestCase):
    """stop_all.kill on POSIX with the OS calls faked (they do not exist on Windows)."""

    SIGNALS = types.SimpleNamespace(SIGTERM=15, SIGKILL=9)

    def run_kill(self, pgid, gone=True):
        calls = []
        patches = [
            unittest.mock.patch.object(proc_registry, "IS_WINDOWS", False),
            unittest.mock.patch.object(stop_all, "signal", self.SIGNALS),
            unittest.mock.patch.object(stop_all, "_wait_gone", return_value=gone),
            unittest.mock.patch.object(os, "getpgid", create=True,
                                       side_effect=pgid if isinstance(pgid, Exception) else (lambda pid: pgid)),
            unittest.mock.patch.object(os, "killpg", create=True, side_effect=lambda g, s: calls.append(("killpg", g, s))),
            unittest.mock.patch.object(os, "kill", side_effect=lambda p, s: calls.append(("kill", p, s))),
        ]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        stop_all.kill(4242)
        return calls

    def test_a_process_leading_its_own_group_is_stopped_with_its_group(self):
        self.assertEqual(self.run_kill(pgid=4242), [("killpg", 4242, 15)])

    def test_a_process_in_another_group_is_signalled_alone(self):
        self.assertEqual(self.run_kill(pgid=1), [("kill", 4242, 15)])

    def test_an_unreadable_group_falls_back_to_the_process(self):
        self.assertEqual(self.run_kill(pgid=OSError("no such process")), [("kill", 4242, 15)])

    def test_a_group_that_ignores_sigterm_gets_sigkill(self):
        self.assertEqual(self.run_kill(pgid=4242, gone=False), [("killpg", 4242, 15), ("killpg", 4242, 9)])


class RegistryCliTest(unittest.TestCase):
    def run_cli(self, run_dir, *args):
        env = {**os.environ, proc_registry.RUN_DIR_ENV: run_dir}
        return subprocess.run([sys.executable, str(SCRIPTS / "proc_registry.py"), *args],
                              env=env, capture_output=True, text=True)

    def test_record_writes_pid_cmd_start_and_identity_then_remove(self):
        with tempfile.TemporaryDirectory() as run_dir:
            child = subprocess.Popen(SLEEPER)
            try:
                result = self.run_cli(run_dir, "record", "--pid", str(child.pid), "--cmd", *SLEEPER)
                self.assertEqual(result.returncode, 0, result.stderr)
                entry = json.loads((Path(run_dir) / "procs" / f"{child.pid}.json").read_text(encoding="utf-8"))
                self.assertEqual(entry["pid"], child.pid)
                self.assertEqual(entry["cmd"], SLEEPER)
                self.assertTrue(entry["started"])
                self.assertTrue(entry["created"] is not None or entry["image"] is not None)
                self.assertEqual(self.run_cli(run_dir, "remove", "--pid", str(child.pid)).returncode, 0)
                self.assertFalse((Path(run_dir) / "procs" / f"{child.pid}.json").exists())
            finally:
                child.kill()
                child.wait(timeout=10)

    def test_record_of_dead_pid_fails(self):
        with tempfile.TemporaryDirectory() as run_dir:
            child = subprocess.Popen(SLEEPER)
            child.kill()
            child.wait(timeout=10)
            result = self.run_cli(run_dir, "record", "--pid", str(child.pid), "--cmd", "x")
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((Path(run_dir) / "procs").exists())


if __name__ == "__main__":
    unittest.main()
