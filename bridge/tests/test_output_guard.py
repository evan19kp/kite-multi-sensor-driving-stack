import contextlib
import importlib
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tests import BRIDGE_DIR
from tests.test_rerun_view import fake_rerun, fake_session
from kite_bridge import common, output_guard
from kite_bridge.output_guard import out_refusal, repo_roots

MARKER = "BeamNG.tech-generated data - never commit"
JSON_SCRIPTS = {"spike_connect": "connect.json", "spike_sensors": "sensors.json",
                "spike_multicar": "multicar.json"}
ALL_SCRIPTS = (*JSON_SCRIPTS, "spike_lidar_rerun")


def make_repo(parent, name="repo"):
    root = Path(parent) / name
    for folder in ("docs", "recordings", "scratch"):
        (root / folder).mkdir(parents=True)
    (root / "AGENTS.md").write_text("x\n", encoding="utf-8")
    return root


class FakeRepoCase(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = Path(tmp.name).resolve()
        self.repo = make_repo(self.tmp)
        self.outside = self.tmp / "datasets"
        self.outside.mkdir()


class OutRefusalTest(FakeRepoCase):
    def refusal(self, out, cwd=None):
        return out_refusal(out, [self.repo], cwd=cwd or self.repo)

    def test_a_folder_inside_the_repo_but_not_under_recordings_or_scratch_is_refused(self):
        for out in ("docs/x", "docs", ".", "bridge", "recordings_old", "scratchpad/a", str(self.repo / "docs")):
            self.assertIn("inside the repo", self.refusal(out) or "", out)

    def test_dot_dot_back_into_the_repo_is_refused(self):
        for out in (f"../{self.repo.name}/docs", "recordings/../docs", "scratch/a/../../docs/y"):
            self.assertIn("inside the repo", self.refusal(out) or "", out)
        self.assertIn("inside the repo", self.refusal(f"../{self.repo.name}/docs", cwd=self.outside) or "")

    def test_recordings_and_scratch_inside_the_repo_are_accepted(self):
        for out in ("recordings", "recordings/a", "scratch/b", "scratch/R30/out/deep",
                    str(self.repo / "recordings" / "a")):
            self.assertIsNone(self.refusal(out), out)

    def test_a_folder_outside_the_repo_is_accepted_even_if_it_does_not_exist_yet(self):
        for out in (str(self.outside), str(self.outside / "new" / "deep"), f"../{self.outside.name}/x"):
            self.assertIsNone(self.refusal(out), out)

    def test_a_relative_folder_is_taken_from_the_given_folder_not_from_the_repo(self):
        self.assertIsNone(self.refusal("docs", cwd=self.outside))
        self.assertIsNone(self.refusal("recordings", cwd=self.outside))
        self.assertIn("inside the repo", self.refusal("docs", cwd=self.repo) or "")

    def test_a_symlink_is_followed_before_deciding(self):
        into_docs = self.repo / "scratch" / "link"
        from_outside = self.outside / "link"
        try:
            into_docs.symlink_to(self.repo / "docs", target_is_directory=True)
            from_outside.symlink_to(self.repo / "docs", target_is_directory=True)
        except (OSError, NotImplementedError) as error:
            self.skipTest(f"this system cannot create symlinks here: {error}")
        self.assertIn("inside the repo", self.refusal("scratch/link/x") or "")
        self.assertIn("inside the repo", self.refusal(str(from_outside / "x")) or "")
        out_link = self.repo / "scratch" / "away"
        out_link.symlink_to(self.outside, target_is_directory=True)
        self.assertIsNone(self.refusal("scratch/away/x"))

    def test_the_refusal_names_what_is_allowed(self):
        message = self.refusal("docs/x")
        self.assertIn("recordings/", message)
        self.assertIn("scratch/", message)
        self.assertIn("outside the repo", message)


class WorktreeRootsTest(FakeRepoCase):
    def setUp(self):
        super().setUp()
        self.worktree = make_repo(self.repo / ".worktrees", "R1")
        self.roots = [self.worktree, self.repo]

    def refusal(self, out):
        return out_refusal(out, self.roots, cwd=self.worktree)

    def test_the_roots_of_a_worktree_are_the_worktree_then_the_main_checkout(self):
        start = self.worktree / "bridge" / "kite_bridge" / "output_guard.py"
        self.assertEqual(repo_roots(start), self.roots)
        self.assertEqual(repo_roots(self.repo / "bridge" / "kite_bridge" / "output_guard.py"), [self.repo])

    def test_a_folder_with_its_own_agents_md_outside_a_worktrees_folder_is_its_own_repo(self):
        stray = make_repo(self.tmp / "other", "R2")
        start = stray / "bridge" / "kite_bridge" / "output_guard.py"
        self.assertEqual(repo_roots(start), [stray])

    def test_no_agents_md_above_is_an_error(self):
        with self.assertRaisesRegex(RuntimeError, "AGENTS.md not found above"):
            repo_roots(self.outside / "bridge" / "x.py")

    def test_the_main_checkout_counts_as_the_repo_from_inside_a_worktree(self):
        for out in ("../../docs", "../../.worktrees/R1/docs", "docs", str(self.repo / "docs" / "x")):
            self.assertIn("inside the repo", self.refusal(out) or "", out)

    def test_recordings_and_scratch_of_either_root_are_accepted(self):
        for out in ("recordings", "scratch/x", "../../recordings/a", "../../scratch/b"):
            self.assertIsNone(self.refusal(out), out)


class ScriptOutFolderTest(FakeRepoCase):
    def run_script(self, script, out, *argv):
        stdout, stderr = io.StringIO(), io.StringIO()
        module = importlib.import_module(script)
        with (mock.patch.object(common, "repo_roots", lambda: [self.repo]),
              contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr)):
            try:
                code = module.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", str(out), *argv])
            except SystemExit as stop:
                code = stop.code
        return code, stdout.getvalue(), stderr.getvalue()

    def test_every_script_refuses_an_out_inside_the_repo_and_writes_nothing(self):
        for script in ALL_SCRIPTS:
            for out in (self.repo / "docs", self.repo / "docs" / "x", self.repo / "recordings" / ".." / "docs"):
                code, stdout, stderr = self.run_script(script, out)
                self.assertEqual(code, 2, (script, out))
                self.assertIn("--out", stderr)
                self.assertIn("inside the repo", stderr)
                self.assertEqual(stdout, "")
        self.assertEqual(list((self.repo / "docs").iterdir()), [])

    def test_recordings_scratch_and_a_folder_outside_the_repo_are_accepted_and_written(self):
        for out in (self.repo / "recordings" / "a", self.repo / "scratch" / "b", self.outside / "c"):
            for script, name in JSON_SCRIPTS.items():
                code, _, stderr = self.run_script(script, out)
                self.assertEqual(code, 0, (script, out, stderr))
                self.assertTrue((out / name).is_file(), (script, out))
            code, _, stderr = self.run_script("spike_lidar_rerun", out)
            self.assertEqual(code, 0, stderr)


class MarkerInJsonTest(FakeRepoCase):
    def test_every_json_the_scripts_write_carries_the_marker_as_a_top_level_field(self):
        for script, name in JSON_SCRIPTS.items():
            out = self.outside / script
            stdout = io.StringIO()
            module = importlib.import_module(script)
            with contextlib.redirect_stdout(stdout):
                code = module.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", str(out)])
            self.assertEqual(code, 0, script)
            data = json.loads((out / name).read_text(encoding="utf-8"))
            self.assertEqual(data["provenance"], MARKER, script)

    def test_the_marker_is_the_one_constant(self):
        self.assertEqual(output_guard.PROVENANCE, MARKER)

    def test_a_json_written_after_a_failed_close_still_carries_the_marker(self):
        from kite_bridge.fake_sim import FakeSim

        class BrokenClose(FakeSim):
            def close(self):
                raise RuntimeError("close failed")

        import spike_connect

        with (mock.patch.object(common, "FakeSim", BrokenClose), contextlib.redirect_stdout(io.StringIO()),
              contextlib.redirect_stderr(io.StringIO())):
            spike_connect.main(["--fake", "--seconds", "1", "--out", str(self.outside)])
        saved = json.loads((self.outside / "connect.json").read_text(encoding="utf-8"))
        self.assertEqual(saved["provenance"], MARKER)


class MarkerInRecordingTest(unittest.TestCase):
    def make(self, save_path):
        import spike_lidar_rerun

        fake = fake_rerun()
        self.order = []
        for name in ("init", "save", "log"):
            getattr(fake, name).side_effect = lambda *a, _name=name, **k: self.order.append(_name)
        with mock.patch.dict(sys.modules, {"rerun": fake}):
            spike_lidar_rerun.make_recorder(save_path, False)
        return fake

    def test_a_saved_recording_starts_with_one_provenance_text_entry(self):
        with tempfile.TemporaryDirectory() as tmp:
            fake = self.make(Path(tmp) / "demo.rrd")
        fake.TextLog.assert_called_once_with(MARKER)
        self.assertEqual([call.args for call in fake.log.call_args_list],
                         [("meta/provenance", fake.TextLog.return_value)])

    def test_the_entry_goes_to_the_file_not_before_it_is_opened(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.make(Path(tmp) / "demo.rrd")
        self.assertEqual(self.order, ["init", "save", "log"])

    def test_a_whole_script_run_logs_the_provenance_once_before_any_step(self):
        import spike_lidar_rerun

        fake = fake_rerun()
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.dict(sys.modules, {"rerun": fake}))
            stack.enter_context(mock.patch.object(common, "session", fake_session))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            code = spike_lidar_rerun.main(["--home", tmp, "--out", str(Path(tmp) / "out"),
                                           "--seconds", "0.05", "--traffic", "0", "--save", "demo.rrd"])
        self.assertEqual(code, 0)
        fake.TextLog.assert_called_once_with(MARKER)
        first = fake.log.call_args_list[0]
        self.assertEqual(first.args, ("meta/provenance", fake.TextLog.return_value))
        self.assertGreater(len(fake.log.call_args_list), 1)
        self.assertEqual(sum(call.args[0] == "meta/provenance" for call in fake.log.call_args_list), 1)


class CommandLineRefusalTest(unittest.TestCase):
    def run_script(self, *argv):
        root = BRIDGE_DIR.parent
        done = subprocess.run([sys.executable, str(BRIDGE_DIR / "spike_connect.py"), "--fake", *argv],
                              capture_output=True, text=True, timeout=120, cwd=root)
        return done.returncode, done.stdout, done.stderr

    def test_an_out_inside_the_repo_is_a_usage_error_with_exit_2(self):
        for out in ("docs/x", f"..{os.sep}{BRIDGE_DIR.parent.name}{os.sep}docs"):
            code, stdout, stderr = self.run_script("--out", out)
            self.assertEqual(code, 2, out)
            self.assertIn("usage", stderr)
            self.assertIn("--out", stderr)
            self.assertIn("inside the repo", stderr)
            self.assertEqual(stdout, "")

    def test_an_out_outside_the_repo_is_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, stdout, stderr = self.run_script("--seconds", "1", "--out", tmp)
            self.assertEqual(code, 0, stderr)
            self.assertTrue((Path(tmp) / "connect.json").is_file())


if __name__ == "__main__":
    unittest.main()
