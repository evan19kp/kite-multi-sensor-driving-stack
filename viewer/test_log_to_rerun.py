import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path

import log_to_rerun
import scene

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").is_file())
GOLDEN = ROOT / "sim" / "tests" / "data" / "seed42_kalman_tracks.log"


class FakeSink:
    def __init__(self):
        self.save_path = "not made"
        self.times = []
        self.paths = []
        self.closed = False

    def factory(self, save_path):
        self.save_path = save_path
        return self

    def set_time(self, seconds):
        self.times.append(seconds)

    def log_points(self, path, positions, colors, labels):
        self.paths.append(path)

    def close(self):
        self.closed = True


def run(*argv, sink=None):
    sink = sink or FakeSink()
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            code = log_to_rerun.main([str(a) for a in argv], sink_factory=sink.factory)
        except SystemExit as stop:
            code = stop.code
    return code, out.getvalue(), err.getvalue(), sink


class ShowsALog(unittest.TestCase):
    def test_live_run_has_no_save_path_and_logs_the_whole_log(self):
        code, out, _, sink = run(GOLDEN)
        self.assertEqual(code, 0)
        self.assertIsNone(sink.save_path)
        self.assertEqual(len(sink.times), 21)
        self.assertEqual(sink.paths.count(scene.TRACKS_PATH), 21)
        self.assertTrue(sink.closed)
        self.assertIn("21 time steps, 63 ground truth, 57 measurement and 56 track records", out)

    def test_only_ground_truth_entities_are_under_ground_truth(self):
        _, _, _, sink = run(GOLDEN)
        self.assertEqual({p for p in sink.paths if p.startswith("ground_truth")}, {scene.GROUND_TRUTH_PATH})

    def test_neither_rerun_nor_numpy_is_imported(self):
        self.assertNotIn("rerun", sys.modules)
        self.assertNotIn("numpy", sys.modules)


class SavePath(unittest.TestCase):
    def test_save_goes_to_a_file_named_inside_out(self):
        with tempfile.TemporaryDirectory() as folder:
            code, out, _, sink = run(GOLDEN, "--save", "clip.rrd", "--out", folder)
            self.assertEqual(code, 0)
            self.assertEqual(sink.save_path, Path(folder) / "clip.rrd")
            self.assertIn(str(Path(folder) / "clip.rrd"), out)

    def test_default_out_is_recordings(self):
        with contextlib.chdir(ROOT):
            code, _, _, sink = run(GOLDEN, "--save", "clip.rrd")
        self.assertEqual(code, 0)
        self.assertEqual(sink.save_path, Path("recordings") / "clip.rrd")

    def test_save_that_leaves_out_is_refused_with_exit_2(self):
        with tempfile.TemporaryDirectory() as folder:
            for name in ("../clip.rrd", "sub/../../clip.rrd", "/tmp/clip.rrd", "C:/clip.rrd", "\\clip.rrd",
                         "clip.txt", ".rrd", ""):
                with self.subTest(name=name):
                    code, _, err, sink = run(GOLDEN, "--save", name, "--out", folder)
                    self.assertEqual(code, 2)
                    self.assertIn("--save", err)
                    self.assertEqual(sink.save_path, "not made")

    def test_out_inside_the_repo_is_refused_except_recordings_and_scratch(self):
        for folder in (ROOT, ROOT / "docs", ROOT / "viewer", ROOT / "recordings" / ".." / "docs"):
            with self.subTest(folder=folder):
                code, _, err, sink = run(GOLDEN, "--save", "clip.rrd", "--out", folder)
                self.assertEqual(code, 2)
                self.assertIn("inside the repo", err)
                self.assertEqual(sink.save_path, "not made")
        for folder in (ROOT / "recordings", ROOT / "scratch" / "R48"):
            with self.subTest(folder=folder):
                code, _, _, sink = run(GOLDEN, "--save", "clip.rrd", "--out", folder)
                self.assertEqual(code, 0)
                self.assertEqual(sink.save_path, folder / "clip.rrd")


class Failures(unittest.TestCase):
    def test_a_bad_log_exits_1_naming_the_file_and_line(self):
        with tempfile.TemporaryDirectory() as folder:
            bad = Path(folder) / "bad.log"
            bad.write_bytes(b"# kite-log v0\nmeas,0,0,0,1.50,2\n")
            code, _, err, sink = run(bad)
            self.assertEqual(code, 1)
            self.assertIn(f"{bad}: line 2:", err)
            self.assertEqual(sink.save_path, "not made")

    def test_a_missing_log_exits_1(self):
        with tempfile.TemporaryDirectory() as folder:
            code, _, err, sink = run(Path(folder) / "none.log")
            self.assertEqual(code, 1)
            self.assertIn("none.log", err)
            self.assertEqual(sink.save_path, "not made")

    def test_no_arguments_exit_2(self):
        self.assertEqual(run()[0], 2)

    def test_a_missing_rerun_exits_1_and_says_how_to_install_it(self):
        def no_rerun(save_path):
            raise ModuleNotFoundError("No module named 'rerun'")

        sink = FakeSink()
        sink.factory = no_rerun
        code, _, err, _ = run(GOLDEN, sink=sink)
        self.assertEqual(code, 1)
        self.assertIn("pip install -r bridge/requirements.txt", err)

    def test_the_sink_is_closed_when_logging_fails(self):
        sink = FakeSink()
        sink.log_points = lambda *args: (_ for _ in ()).throw(RuntimeError("viewer gone"))
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):
                log_to_rerun.main([str(GOLDEN)], sink_factory=sink.factory)
        self.assertTrue(sink.closed)


if __name__ == "__main__":
    unittest.main()
