import contextlib
import io
import math
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tests import BRIDGE_DIR
from kite_bridge import common, rerun_view
from kite_bridge.fake_sim import FakeSim
from kite_bridge.readings import image_from_raw, points_from_raw
from kite_bridge.rerun_view import (CAMERA_PATH, EGO_PATH, GROUND_TRUTH_PATH, LIDAR_PATH,
                                    distance_colour, log_step)

IMAGE = [[(0, 0, 0)]]


class RecordingRecorder:
    def __init__(self):
        self.calls = []
        self.closed = 0

    def set_time(self, seconds):
        self.calls.append(("time", seconds))

    def log_points(self, path, positions, colors):
        self.calls.append(("points", path, list(positions), list(colors)))

    def log_image(self, path, image):
        self.calls.append(("image", path, image))

    def close(self):
        self.closed += 1

    def points_at(self, path):
        return [c for c in self.calls if c[0] == "points" and c[1] == path]

    def paths(self):
        return [c[1] for c in self.calls if c[0] != "time"]


def run_script(*args):
    done = subprocess.run([sys.executable, str(BRIDGE_DIR / "spike_lidar_rerun.py"), *args],
                          capture_output=True, text=True, timeout=120)
    return done.returncode, done.stdout, done.stderr


class DistanceColourTest(unittest.TestCase):
    def test_near_is_blue_far_is_red_and_the_middle_is_half_way(self):
        self.assertEqual(distance_colour(0), (0, 0, 255))
        self.assertEqual(distance_colour(25), (128, 0, 127))
        self.assertEqual(distance_colour(50), (255, 0, 0))

    def test_beyond_the_far_distance_stays_red(self):
        self.assertEqual(distance_colour(500), (255, 0, 0))

    def test_red_never_falls_and_blue_never_rises_with_distance(self):
        colours = [distance_colour(d / 2) for d in range(0, 160)]
        for near, far in zip(colours, colours[1:]):
            self.assertLessEqual(near[0], far[0])
            self.assertGreaterEqual(near[2], far[2])
        self.assertNotEqual(colours[0], colours[-1])

    def test_a_distance_outside_the_domain_is_an_error(self):
        for bad in (-0.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                distance_colour(bad)


class LogStepTest(unittest.TestCase):
    def test_one_step_logs_the_time_then_lidar_position_and_image(self):
        rec = RecordingRecorder()
        log_step(rec, 1.5, (1.0, 2.0, 3.0), [(1.0, 2.0, 13.0)], IMAGE)
        self.assertEqual(rec.calls, [
            ("time", 1.5),
            ("points", LIDAR_PATH, [(1.0, 2.0, 13.0)], [(51, 0, 204)]),
            ("points", EGO_PATH, [(1.0, 2.0, 3.0)], [rerun_view.EGO_COLOUR]),
            ("image", CAMERA_PATH, IMAGE)])

    def test_lidar_colours_follow_the_distance_from_our_car(self):
        rec = RecordingRecorder()
        points = [(30.0, 0.0, 0.0), (2.0, 0.0, 0.0), (10.0, 0.0, 0.0), (45.0, 0.0, 0.0)]
        log_step(rec, 0.0, (0.0, 0.0, 0.0), points, IMAGE)
        _, _, logged, colours = rec.points_at(LIDAR_PATH)[0]
        by_distance = sorted(zip(logged, colours), key=lambda pair: pair[0][0])
        reds = [colour[0] for _, colour in by_distance]
        blues = [colour[2] for _, colour in by_distance]
        self.assertEqual(reds, sorted(reds))
        self.assertEqual(blues, sorted(blues, reverse=True))
        self.assertLess(reds[0], reds[-1])

    def test_other_cars_are_logged_only_under_their_own_path(self):
        rec = RecordingRecorder()
        others = [(40.0, 3.5, 0.0), (55.0, -3.5, 0.0)]
        log_step(rec, 0.0, (0.0, 0.0, 0.0), [(5.0, 0.0, 0.0)], IMAGE, others)
        self.assertEqual(sorted(rec.paths()), sorted([LIDAR_PATH, EGO_PATH, CAMERA_PATH, GROUND_TRUTH_PATH]))
        self.assertEqual(rec.points_at(GROUND_TRUTH_PATH)[0][2], others)
        for path in (LIDAR_PATH, EGO_PATH):
            for other in others:
                self.assertNotIn(other, rec.points_at(path)[0][2])
        self.assertFalse(GROUND_TRUTH_PATH.startswith(("world", "camera")))

    def test_no_traffic_information_logs_no_ground_truth(self):
        rec = RecordingRecorder()
        log_step(rec, 0.0, (0.0, 0.0, 0.0), [(5.0, 0.0, 0.0)], IMAGE, None)
        self.assertEqual(rec.points_at(GROUND_TRUTH_PATH), [])
        self.assertEqual(len(rec.calls), 4)


class RecordingNameTest(unittest.TestCase):
    def test_a_name_inside_out_is_accepted(self):
        self.assertEqual(rerun_view.recording_name("demo.rrd"), "demo.rrd")
        self.assertEqual(rerun_view.recording_name("day1/demo.RRD"), "day1/demo.RRD")
        self.assertEqual(rerun_view.recording_path("out", "demo.rrd"), Path("out") / "demo.rrd")

    def test_a_name_that_leaves_out_or_is_not_rrd_is_rejected(self):
        for bad in ("../demo.rrd", "a/../../demo.rrd", "a\\..\\demo.rrd", "/tmp/demo.rrd",
                    "C:\\demo.rrd", "C:demo.rrd", "\\demo.rrd", "demo.txt", "demo", ".rrd", ""):
            with self.assertRaises(ValueError, msg=bad):
                rerun_view.recording_name(bad)


class FakeSimSceneTest(unittest.TestCase):
    def make(self, seed=0, traffic=2):
        sim = FakeSim(seed=seed)
        car = sim.load_scenario("fake_sedan", 1)[0]
        sim.spawn_traffic(traffic)
        sim.attach_sensors(car)
        return sim, car

    def test_lidar_points_are_seeded_and_surround_our_car(self):
        sim, car = self.make()
        sim.step(30)
        points = sim.lidar_points(car)
        again, _ = self.make()
        again.step(30)
        self.assertEqual(points, again.lidar_points(car))
        self.assertEqual(len(points), 200)
        x, y, z = sim.position(car)
        self.assertTrue(all(1.0 <= ((px - x) ** 2 + (py - y) ** 2) ** 0.5 <= 60.0 + 1e-9 for px, py, _ in points))
        self.assertNotEqual(points, self.make(seed=1)[0].lidar_points(car))

    def test_camera_image_is_rows_of_rgb_pixels(self):
        sim, car = self.make()
        image = sim.camera_image(car)
        self.assertEqual((len(image), len(image[0])), (4, 8))
        self.assertTrue(all(len(p) == 3 and all(0 <= v <= 255 for v in p) for row in image for p in row))
        sim.step(1)
        self.assertNotEqual(sim.camera_image(car), image)

    def test_traffic_positions_match_the_request_and_move(self):
        sim, _ = self.make(traffic=3)
        first = sim.other_positions()
        sim.step(60)
        self.assertEqual(len(first), 3)
        self.assertAlmostEqual(sim.other_positions()[0][0] - first[0][0], 10.0)

    def test_no_traffic_requested_gives_no_other_cars(self):
        self.assertEqual(FakeSim().other_positions(), [])

    def test_the_sensors_must_be_attached_first(self):
        sim = FakeSim()
        car = sim.load_scenario("fake_sedan", 1)[0]
        with self.assertRaises(KeyError):
            sim.lidar_points(car)
        with self.assertRaises(KeyError):
            sim.camera_image(car)


class Image:
    size = (2, 2)

    def convert(self, mode):
        assert mode == "RGB"
        return self

    def getdata(self):
        return [(1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12)]


class ReadingsTest(unittest.TestCase):
    def test_points_come_from_the_point_cloud_and_ignore_extra_columns(self):
        raw = {"pointCloud": [[1, 2, 3, 99], [4.5, 5, 6, 99]], "colours": []}
        self.assertEqual(points_from_raw(raw), [(1.0, 2.0, 3.0), (4.5, 5.0, 6.0)])

    def test_an_empty_poll_has_no_points_and_an_unexpected_format_is_an_error(self):
        self.assertEqual(points_from_raw({}), [])
        with self.assertRaisesRegex(ValueError, "no 'pointCloud' in \\['other'\\]"):
            points_from_raw({"other": 1})

    def test_the_camera_image_becomes_rows_of_pixels(self):
        self.assertEqual(image_from_raw({"colour": Image()}),
                         [[(1, 2, 3), (4, 5, 6)], [(7, 8, 9), (10, 11, 12)]])

    def test_a_camera_reading_without_colour_is_an_error(self):
        with self.assertRaisesRegex(ValueError, "no 'colour'"):
            image_from_raw({"depth": 1})
        with self.assertRaisesRegex(ValueError, "no 'colour' in list"):
            image_from_raw([])


class ScriptTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = str(Path(tmp.name) / "out")
        self.recorder = RecordingRecorder()
        self.factory_calls = []

    def run_main(self, *argv, sim_class=None):
        import spike_lidar_rerun

        def factory(path, fake):
            self.factory_calls.append((path, fake))
            return self.recorder

        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(spike_lidar_rerun, "make_recorder", factory))
            if sim_class:
                stack.enter_context(mock.patch.object(common, "FakeSim", sim_class))
            stack.enter_context(contextlib.redirect_stdout(stdout))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            code = spike_lidar_rerun.main(["--fake", "--out", self.out, *argv])
        return code, stdout.getvalue(), stderr.getvalue()

    def test_two_seconds_log_five_calls_per_step_on_one_increasing_timeline(self):
        code, stdout, stderr = self.run_main("--seconds", "2", "--traffic", "3")
        self.assertEqual(code, 0, stderr)
        self.assertEqual(len(self.recorder.calls), 120 * 5)
        times = [c[1] for c in self.recorder.calls if c[0] == "time"]
        self.assertEqual(len(times), 120)
        self.assertEqual(times, sorted(set(times)))
        self.assertAlmostEqual(times[0], 1 / 60)
        self.assertAlmostEqual(times[-1], 2.0)
        self.assertIn("logged 120 steps", stdout)
        self.assertEqual(self.recorder.closed, 1)

    def test_every_step_logs_traffic_ground_truth_only_under_its_own_path(self):
        self.run_main("--seconds", "1", "--traffic", "3")
        gt = self.recorder.points_at(GROUND_TRUTH_PATH)
        self.assertEqual(len(gt), 60)
        self.assertTrue(all(len(call[2]) == 3 for call in gt))
        ego = self.recorder.points_at(EGO_PATH)
        self.assertEqual(len(ego), 60)
        (x, y, z), = ego[-1][2]
        self.assertAlmostEqual(x, 10.0)
        self.assertEqual((y, z), (0.0, 0.0))
        sensor_points = [p for c in self.recorder.points_at(LIDAR_PATH) + ego for p in c[2]]
        traffic_points = {p for c in gt for p in c[2]}
        self.assertFalse(traffic_points & set(sensor_points))

    def test_the_lidar_colours_are_distance_colours_of_the_logged_points(self):
        self.run_main("--seconds", "1", "--traffic", "0")
        _, _, points, colours = self.recorder.points_at(LIDAR_PATH)[-1]
        ego = self.recorder.points_at(EGO_PATH)[-1][2][0]
        self.assertEqual(len(points), 200)
        self.assertEqual(colours, [distance_colour(math.dist(p, ego)) for p in points])

    def test_without_traffic_no_ground_truth_is_logged(self):
        self.run_main("--seconds", "1", "--traffic", "0")
        self.assertEqual(self.recorder.points_at(GROUND_TRUTH_PATH), [])
        self.assertEqual(len(self.recorder.calls), 60 * 4)

    def test_traffic_positions_that_cannot_be_read_are_reported_once_and_skipped(self):
        class NoTraffic(FakeSim):
            def other_positions(self):
                raise RuntimeError("no traffic list")

        code, stdout, _ = self.run_main("--seconds", "1", "--traffic", "2", sim_class=NoTraffic)
        self.assertEqual(code, 0)
        self.assertEqual(stdout.count("other cars not logged (RuntimeError: no traffic list)"), 1)
        self.assertEqual(self.recorder.points_at(GROUND_TRUTH_PATH), [])
        self.assertEqual(len(self.recorder.points_at(LIDAR_PATH)), 60)

    def test_the_recording_path_is_inside_out_and_the_recorder_is_told_it_is_fake(self):
        code, stdout, _ = self.run_main("--seconds", "1", "--save", "day1/demo.rrd")
        self.assertEqual(code, 0)
        path = Path(self.out) / "day1" / "demo.rrd"
        self.assertEqual(self.factory_calls, [(path, True)])
        self.assertIn(f"--fake keeps nothing; the recording would have gone to {path}", stdout)

    def test_without_save_there_is_no_recording_path(self):
        self.run_main("--seconds", "1")
        self.assertEqual(self.factory_calls, [(None, True)])

    def test_the_recorder_is_closed_when_the_simulator_fails(self):
        class BrokenStep(FakeSim):
            def step(self, n=1):
                raise RuntimeError("boom")

        code, _, stderr = self.run_main("--seconds", "1", sim_class=BrokenStep)
        self.assertEqual(code, 1)
        self.assertEqual(stderr.splitlines()[-1], "error: RuntimeError: boom")
        self.assertEqual(self.recorder.closed, 1)


class CommandLineTest(unittest.TestCase):
    def test_missing_home_without_fake_is_a_usage_error(self):
        code, stdout, stderr = run_script()
        self.assertEqual(code, 2)
        self.assertIn("--home", stderr)
        self.assertEqual(stdout, "")

    def test_a_save_name_outside_out_or_not_rrd_is_a_usage_error(self):
        for bad in ("../x.rrd", "/tmp/x.rrd", "C:\\x.rrd", "x.txt"):
            code, stdout, stderr = run_script("--fake", "--save", bad)
            self.assertEqual(code, 2, bad)
            self.assertIn("usage", stderr)
            self.assertIn("--save", stderr)
            self.assertEqual(stdout, "")

    def test_help_lists_the_common_options_and_says_what_save_and_fake_do(self):
        code, stdout, _ = run_script("--help")
        self.assertEqual(code, 0)
        for option in ("--home", "--headless", "--seconds", "--traffic", "--out", "--fake", "--model", "--save"):
            self.assertIn(option, stdout)
        flat = re.sub(r"\s+", " ", stdout)
        self.assertIn("does not open a viewer", flat)
        self.assertIn("--fake uses the fake simulator and a recorder that keeps nothing", flat)

    def test_a_fake_run_writes_no_file_even_with_save(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "out"
            code, stdout, stderr = run_script("--fake", "--seconds", "1", "--out", str(out), "--save", "demo.rrd")
            self.assertEqual(code, 0, stderr)
            self.assertIn("logged 60 steps", stdout)
            self.assertIn(str(out / "demo.rrd"), stdout)
            self.assertFalse(out.exists())


class NoRerunTest(unittest.TestCase):
    def test_running_the_script_against_the_fake_imports_neither_rerun_nor_the_adapter(self):
        import spike_lidar_rerun

        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(spike_lidar_rerun.main(["--fake", "--seconds", "1", "--out", tmp]), 0)
        for name in ("rerun", "beamngpy", "kite_bridge.beamng_adapter"):
            self.assertNotIn(name, sys.modules)

    def test_only_rerun_view_imports_rerun(self):
        sources = [*BRIDGE_DIR.glob("*.py"), *(BRIDGE_DIR / "kite_bridge").glob("*.py")]
        self.assertGreater(len(sources), 5)
        for path in sources:
            imports = re.search(r"(?m)^\s*(import|from)\s+rerun\b", path.read_text(encoding="utf-8"))
            self.assertEqual(bool(imports), path.name == "rerun_view.py", path.name)

    def test_rerun_view_imports_rerun_only_inside_a_method(self):
        text = (BRIDGE_DIR / "kite_bridge" / "rerun_view.py").read_text(encoding="utf-8")
        self.assertIsNone(re.search(r"(?m)^(import|from)\s+(rerun|numpy)\b", text))


if __name__ == "__main__":
    unittest.main()
