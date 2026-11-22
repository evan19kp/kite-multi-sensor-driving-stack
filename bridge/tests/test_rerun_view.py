import contextlib
import io
import math
import re
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

from tests import BRIDGE_DIR
from kite_bridge import common, rerun_view
from kite_bridge.fake_sim import DEFAULT_RATES, FakeClock, FakeSim
from kite_bridge.readings import image_from_raw, points_from_raw, reading_from_raw
from kite_bridge.rerun_view import (CAMERA_PATH, EGO_FRAME_PATH, EGO_PATH, GROUND_TRUTH_PATH, LIDAR_PATH,
                                    distance_colour, log_step)
from kite_bridge.sim_api import Reading

IMAGE = [[(0, 0, 0)]]
LISTS = rerun_view.ListArrays()


class RecordingRecorder:
    """Records what a RerunRecorder on the plain-list path would hand to Rerun: a lidar scan as
    its x, y, z points and their distance colours, like a ("points", ...) call."""

    def __init__(self):
        self.calls = []
        self.closed = 0

    def all_finite(self, cloud):
        return LISTS.all_finite(LISTS.points(cloud))

    def set_time(self, seconds):
        self.calls.append(("time", seconds))

    def log_lidar(self, path, cloud, centre):
        points = LISTS.points(cloud)
        self.calls.append(("points", path, points, LISTS.distance_colours(points, centre)))

    def log_points(self, path, positions, colors):
        self.calls.append(("points", path, list(positions), list(colors)))

    def log_translation(self, path, translation):
        self.calls.append(("translation", path, tuple(translation)))

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

    def test_ground_truth_is_logged_only_under_the_ground_truth_root(self):
        rec = RecordingRecorder()
        ego = (0.0, 0.0, 0.0)
        lidar = [(5.0, 0.0, 0.0)]
        others = [(40.0, 3.5, 0.0), (55.0, -3.5, 0.0)]
        log_step(rec, 0.0, ego, lidar, IMAGE, others)
        self.assertEqual(sorted(rec.paths()), sorted([LIDAR_PATH, EGO_PATH, CAMERA_PATH, GROUND_TRUTH_PATH]))
        self.assertEqual(rec.points_at(GROUND_TRUTH_PATH)[0][2], others)
        self.assertEqual(rec.points_at(EGO_PATH)[0][2], [ego])
        ground_truth_paths = [p for p in rec.paths() if p.startswith("ground_truth/")]
        self.assertEqual(sorted(ground_truth_paths), sorted([EGO_PATH, GROUND_TRUTH_PATH]))
        sensor_paths = [p for p in rec.paths() if p not in ground_truth_paths]
        self.assertEqual(sorted(sensor_paths), sorted([LIDAR_PATH, CAMERA_PATH]))
        sensor_points = rec.points_at(LIDAR_PATH)[0][2]
        for position in [ego, *others]:
            self.assertNotIn(position, sensor_points)
        for path in ground_truth_paths:
            self.assertNotIn(lidar[0], rec.points_at(path)[0][2])

    def test_a_missing_lidar_reading_logs_no_lidar_and_an_empty_scan_logs_an_empty_layer(self):
        missing, empty = RecordingRecorder(), RecordingRecorder()
        log_step(missing, 0.0, (0.0, 0.0, 0.0), None, IMAGE)
        log_step(empty, 0.0, (0.0, 0.0, 0.0), [], IMAGE)
        self.assertEqual(missing.points_at(LIDAR_PATH), [])
        self.assertEqual(missing.paths(), [EGO_PATH, CAMERA_PATH])
        self.assertEqual(empty.points_at(LIDAR_PATH), [("points", LIDAR_PATH, [], [])])

    def test_a_missing_camera_reading_logs_no_image(self):
        rec = RecordingRecorder()
        log_step(rec, 0.0, (0.0, 0.0, 0.0), [(5.0, 0.0, 0.0)], None)
        self.assertEqual(rec.paths(), [LIDAR_PATH, EGO_PATH])

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

    def test_an_empty_poll_or_a_scan_with_no_points_is_no_reading_for_both_converters(self):
        for raw in ({}, None, [], {"time": 4.0, "pointCloud": []}):
            self.assertIsNone(points_from_raw(raw), raw)
            reading = reading_from_raw("lidar", raw, 7.5)
            self.assertIsNone(reading.timestamp, raw)
            self.assertIsNone(reading.unexpected, raw)

    def test_an_unexpected_format_is_an_error(self):
        with self.assertRaisesRegex(ValueError, "no 'pointCloud'; got \\['other'\\]"):
            points_from_raw({"other": 1})

    def test_the_camera_image_becomes_rows_of_pixels(self):
        self.assertEqual(image_from_raw({"colour": Image()}),
                         [[(1, 2, 3), (4, 5, 6)], [(7, 8, 9), (10, 11, 12)]])

    def test_an_empty_camera_poll_is_no_reading(self):
        for raw in ({}, None, []):
            self.assertIsNone(image_from_raw(raw), raw)

    def test_a_camera_reading_without_colour_is_an_error(self):
        with self.assertRaisesRegex(ValueError, "no 'colour'"):
            image_from_raw({"depth": 1})
        with self.assertRaisesRegex(ValueError, "no 'colour' in list"):
            image_from_raw([{"depth": 1}])


class ScriptTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = str(Path(tmp.name) / "out")
        self.recorder = RecordingRecorder()
        self.factory_calls = []

    def run_main(self, *argv, sim_class=None):
        import spike_lidar_rerun

        def factory(path, fake, live_stride):
            self.factory_calls.append((path, fake, live_stride))
            return self.recorder

        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(spike_lidar_rerun, "make_recorder", factory))
            if sim_class:
                stack.enter_context(mock.patch.object(common, "FakeSim", sim_class))
            stack.enter_context(contextlib.redirect_stdout(stdout))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            code = spike_lidar_rerun.main(["--fake", "--mode", "lockstep", "--out", self.out, *argv])
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

    def test_every_step_logs_ground_truth_only_under_the_ground_truth_root(self):
        self.run_main("--seconds", "1", "--traffic", "3")
        gt = self.recorder.points_at(GROUND_TRUTH_PATH)
        self.assertEqual(len(gt), 60)
        self.assertTrue(all(len(call[2]) == 3 for call in gt))
        ego = self.recorder.points_at(EGO_PATH)
        self.assertEqual(len(ego), 60)
        (x, y, z), = ego[-1][2]
        self.assertAlmostEqual(x, 10.0)
        self.assertEqual((y, z), (0.0, 0.0))
        self.assertEqual(sorted(set(self.recorder.paths())),
                         sorted([CAMERA_PATH, EGO_PATH, GROUND_TRUTH_PATH, LIDAR_PATH]))
        sensor_points = {p for c in self.recorder.points_at(LIDAR_PATH) for p in c[2]}
        ground_truth_points = {p for c in gt + ego for p in c[2]}
        self.assertFalse(ground_truth_points & sensor_points)

    def test_steps_without_a_lidar_reading_log_no_lidar_and_the_run_says_how_many(self):
        class FlakyLidar(FakeSim):
            def lidar_points(self, car):
                points = super().lidar_points(car)
                return None if self._steps % 4 == 0 else points

        code, stdout, _ = self.run_main("--seconds", "1", "--traffic", "0", sim_class=FlakyLidar)
        self.assertEqual(code, 0)
        self.assertEqual(len(self.recorder.points_at(LIDAR_PATH)), 45)
        self.assertEqual(len(self.recorder.points_at(EGO_PATH)), 60)
        self.assertEqual(stdout.count("lidar gave no reading on 15 of 60 steps"), 1)

    def test_steps_without_a_camera_reading_log_no_image_and_the_run_says_how_many(self):
        class FlakyCamera(FakeSim):
            def camera_image(self, car):
                image = super().camera_image(car)
                return None if self._steps % 3 == 0 else image

        code, stdout, _ = self.run_main("--seconds", "1", "--traffic", "0", sim_class=FlakyCamera)
        self.assertEqual(code, 0)
        self.assertEqual(len([c for c in self.recorder.calls if c[0] == "image"]), 40)
        self.assertEqual(len(self.recorder.points_at(LIDAR_PATH)), 60)
        self.assertEqual(stdout.count("camera gave no reading on 20 of 60 steps"), 1)

    def test_a_run_with_every_reading_prints_no_missing_reading_warning(self):
        _, stdout, _ = self.run_main("--seconds", "1", "--traffic", "0")
        self.assertNotIn("no reading", stdout)

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
        self.assertEqual(self.factory_calls, [(path, True, 1)])
        self.assertIn(f"--fake keeps nothing; the recording would have gone to {path}", stdout)

    def test_without_save_there_is_no_recording_path(self):
        self.run_main("--seconds", "1")
        self.assertEqual(self.factory_calls, [(None, True, 1)])

    def test_live_stride_reaches_the_recorder_and_the_run_says_so(self):
        code, stdout, _ = self.run_main("--seconds", "1", "--live-stride", "3")
        self.assertEqual(code, 0)
        self.assertEqual(self.factory_calls, [(None, True, 3)])
        self.assertIn("the live viewer showed 1 of every 3 lidar points", stdout)

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

    def test_a_live_stride_below_1_or_not_a_whole_number_is_a_usage_error(self):
        for bad in ("0", "-2", "1.5", "x"):
            code, stdout, stderr = run_script("--fake", "--live-stride", bad)
            self.assertEqual((code, stdout), (2, ""), bad)
            self.assertIn("--live-stride", stderr)

    def test_live_stride_with_save_is_a_usage_error_because_a_recording_keeps_every_point(self):
        code, stdout, stderr = run_script("--fake", "--save", "demo.rrd", "--live-stride", "2")
        self.assertEqual((code, stdout), (2, ""))
        self.assertIn("a --save recording keeps every lidar point", stderr)

    def test_a_fake_run_writes_no_file_even_with_save(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "out"
            code, stdout, stderr = run_script("--fake", "--mode", "lockstep", "--seconds", "1", "--out", str(out),
                                              "--save", "demo.rrd")
            self.assertEqual(code, 0, stderr)
            self.assertIn("logged 60 steps", stdout)
            self.assertIn(str(out / "demo.rrd"), stdout)
            self.assertFalse(out.exists())


COMMON_RERUN = ("init", "save", "log", "Points3D", "Image", "EncodedImage", "Transform3D", "TextLog",
                "send_blueprint", "disconnect")
NEW_RERUN = (*COMMON_RERUN, "set_time")
OLD_RERUN = (*COMMON_RERUN, "set_time_seconds")
BLUEPRINT = ("Blueprint", "Horizontal", "Vertical", "Spatial3DView", "Spatial2DView", "TextLogView")
ENCODE_S = 0.01


def fake_rerun(api=NEW_RERUN, writes=None):
    """A stand-in for the rerun module and its blueprint module. With `writes`, save(path) writes
    that many bytes to `path`, as Rerun would write a recording there."""
    fake = types.ModuleType("rerun")
    for name in api:
        setattr(fake, name, mock.Mock(name=name))
    fake.blueprint = types.ModuleType("rerun.blueprint")
    for name in BLUEPRINT:
        setattr(fake.blueprint, name, mock.Mock(name=name))
    if writes is not None:
        fake.save.side_effect = lambda path: Path(path).write_bytes(bytes(writes))
    return fake


def installed(fake, blueprint=True):
    """Makes `import rerun` give `fake`, and `import rerun.blueprint` its blueprint module, or fail
    without `blueprint`."""
    return mock.patch.dict(sys.modules, {"rerun": fake, **({"rerun.blueprint": fake.blueprint} if blueprint else {})})


class FakePng:
    """A lossless encoder stand-in: records each image and returns bytes naming it. With a clock,
    each image takes ENCODE_S fake seconds."""

    def __init__(self, clock=None):
        self.images = []
        self.clock = clock

    def __call__(self, pixels):
        self.images.append(pixels)
        if self.clock is not None:
            self.clock.advance(ENCODE_S)
        return f"png {len(self.images)}".encode()


@contextlib.contextmanager
def plain_lists(encoder):
    """Makes a RerunRecorder built with its defaults take the plain-list path and `encoder`, so
    neither numpy nor Pillow is imported."""
    with (mock.patch.object(rerun_view, "NumpyArrays", rerun_view.ListArrays),
          mock.patch.object(rerun_view, "pillow_png", encoder)):
        yield


@contextlib.contextmanager
def fake_session(args):
    sim = FakeSim()
    sim.open()
    try:
        yield sim
    finally:
        sim.close()


class RerunRecorderTest(unittest.TestCase):
    def recorder(self, save_path=None, api=NEW_RERUN, live_stride=1, encoder=None, clock=None, blueprint=True):
        fake = fake_rerun(api)
        with installed(fake, blueprint):
            recorder = rerun_view.RerunRecorder(save_path, live_stride, arrays=rerun_view.ListArrays(),
                                                encode_png=encoder, clock=clock or FakeClock().now)
        self.assertNotIn("rerun", sys.modules)
        if save_path is not None:
            self.addCleanup(recorder.close)
        return recorder, fake

    def test_without_save_it_spawns_a_viewer_and_writes_no_file(self):
        _, fake = self.recorder()
        fake.init.assert_called_once_with("kite_lidar", spawn=True)
        fake.save.assert_not_called()

    def test_with_save_it_records_to_the_file_and_spawns_no_viewer(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "out" / "day1" / "demo.rrd"
            _, fake = self.recorder(path)
            self.assertTrue(path.parent.is_dir())
            self.assertFalse(path.exists())
        fake.init.assert_called_once_with("kite_lidar")
        fake.save.assert_called_once_with(str(path))

    def test_time_uses_set_time_when_rerun_has_it_else_set_time_seconds(self):
        new, new_fake = self.recorder()
        new.set_time(1.5)
        new_fake.set_time.assert_called_once_with("sim_time", duration=1.5)
        old, old_fake = self.recorder(api=OLD_RERUN)
        old.set_time(1.5)
        old_fake.set_time_seconds.assert_called_once_with("sim_time", 1.5)

    def test_a_step_reaches_rerun_as_plain_lists_on_the_entity_paths(self):
        recorder, fake = self.recorder()
        fake.log.reset_mock()
        ego, lidar, others = (0.0, 0.0, 0.0), [(5.0, 0.0, 0.0)], [(40.0, 3.5, 0.0)]
        log_step(recorder, 1.5, ego, lidar, IMAGE, others)
        self.assertEqual(fake.Points3D.call_args_list, [
            mock.call(lidar, colors=[distance_colour(5.0)]),
            mock.call([ego], colors=[rerun_view.EGO_COLOUR]),
            mock.call(others, colors=[rerun_view.GROUND_TRUTH_COLOUR])])
        for call in fake.Points3D.call_args_list:
            self.assertIs(type(call.args[0]), list)
            self.assertIs(type(call.kwargs["colors"]), list)
        fake.Image.assert_called_once_with(IMAGE)
        self.assertEqual([call.args for call in fake.log.call_args_list], [
            (LIDAR_PATH, fake.Points3D.return_value),
            (EGO_PATH, fake.Points3D.return_value),
            (CAMERA_PATH, fake.Image.return_value),
            (GROUND_TRUTH_PATH, fake.Points3D.return_value)])

    def test_close_disconnects(self):
        recorder, fake = self.recorder()
        recorder.close()
        fake.disconnect.assert_called_once_with()

    def test_a_scan_is_one_points_call_with_its_xyz_and_distance_colours_and_never_beamng_colours(self):
        recorder, fake = self.recorder()
        fake.log.reset_mock()
        cloud = [(3.0, 4.0, 0.0, 7.0), (30.0, 40.0, 9.0, 7.0), (6.0, 8.0, 0.0, 7.0)]
        raw = {"pointCloud": cloud, "colours": [(9, 9, 9)] * 3}
        self.assertTrue(rerun_view.log_reading(recorder, "lidar", Reading(2.0, 3, raw=raw)))
        # Without an ego the distance is from the origin in x and y, the height left out: 5 m, 50 m
        # and 10 m. Blue 255 at 0 m and red 255 at 50 m, linear between: 5 m is red 25.5, which
        # rounds to the even 26, and 10 m red 51.
        fake.Points3D.assert_called_once_with([(3.0, 4.0, 0.0), (30.0, 40.0, 9.0), (6.0, 8.0, 0.0)],
                                              colors=[(26, 0, 229), (255, 0, 0), (51, 0, 204)])
        self.assertEqual([call.args for call in fake.log.call_args_list], [(LIDAR_PATH, fake.Points3D.return_value)])
        fake.Transform3D.assert_not_called()

    def test_with_an_ego_world_ego_moves_to_it_and_the_lidar_points_stay_where_they_were_seen(self):
        recorder, fake = self.recorder()
        fake.log.reset_mock()
        ego = (100.0, 200.0)
        cloud = [(103.0, 204.0, 40.0)]
        rerun_view.log_reading(recorder, "lidar", Reading(2.0, 1, raw={"pointCloud": cloud}), ego)
        # A GPS reading gives no height, so world/ego sits at height 0.
        self.assertEqual(fake.Transform3D.call_args_list,
                         [mock.call(translation=(100.0, 200.0, 0.0)), mock.call(translation=(-100.0, -200.0, 0.0))])
        self.assertEqual([call.args[0] for call in fake.log.call_args_list], [EGO_FRAME_PATH, LIDAR_PATH, LIDAR_PATH])
        # 3 m and 4 m from the ego in x and y: 5 m. Its 40 m of height is left out.
        fake.Points3D.assert_called_once_with(cloud, colors=[distance_colour(5.0)])

    def test_a_point_whose_distance_is_not_finite_is_an_error_not_a_colour(self):
        recorder, fake = self.recorder()
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                recorder.log_lidar(LIDAR_PATH, [(1.0, 0.0, 0.0), (bad, 0.0, 0.0)], rerun_view.GROUND_ORIGIN)
        fake.Points3D.assert_not_called()

    def test_a_free_mode_scan_with_a_non_finite_x_y_or_z_logs_nothing_and_says_so(self):
        recorder, fake = self.recorder(live_stride=2)
        fake.log.reset_mock()
        # The stride shows points 0 and 2 only; the bad value is in point 1, which it skips.
        for bad in ((float("nan"), 0.0, 0.0), (0.0, float("inf"), 0.0), (0.0, 0.0, float("nan"))):
            cloud = [(1.0, 0.0, 0.0), bad, (2.0, 0.0, 0.0)]
            logged = rerun_view.log_reading(recorder, "lidar", Reading(2.0, 1, raw={"pointCloud": cloud}), (1.0, 2.0))
            self.assertFalse(logged, bad)
        fake.log.assert_not_called()
        fake.Points3D.assert_not_called()
        fake.Transform3D.assert_not_called()

    def test_live_stride_thins_the_live_view_and_a_recording_keeps_every_point(self):
        cloud = [(float(i), 0.0, 0.0) for i in range(10)]
        live, live_fake = self.recorder(live_stride=3)
        live.log_lidar(LIDAR_PATH, cloud, rerun_view.GROUND_ORIGIN)
        self.assertEqual(live_fake.Points3D.call_args.args[0], [cloud[0], cloud[3], cloud[6], cloud[9]])
        with tempfile.TemporaryDirectory() as tmp:
            saved, saved_fake = self.recorder(Path(tmp) / "demo.rrd", live_stride=3, encoder=FakePng())
        saved.log_lidar(LIDAR_PATH, cloud, rerun_view.GROUND_ORIGIN)
        self.assertEqual(saved_fake.Points3D.call_args.args[0], cloud)

    def test_a_recording_stores_each_image_through_the_lossless_encoder_never_raw_or_lossy(self):
        clock = FakeClock()
        encoder = FakePng(clock)
        with tempfile.TemporaryDirectory() as tmp:
            recorder, fake = self.recorder(Path(tmp) / "demo.rrd", encoder=encoder, clock=clock.now)
        recorder.log_image(CAMERA_PATH, IMAGE)
        recorder.log_image(CAMERA_PATH, IMAGE)
        recorder.close()
        self.assertEqual(encoder.images, [IMAGE, IMAGE])
        self.assertEqual(fake.EncodedImage.call_args_list, [mock.call(contents=b"png 1", media_type="image/png"),
                                                            mock.call(contents=b"png 2", media_type="image/png")])
        fake.Image.assert_not_called()
        self.assertEqual(recorder.stats.encoded, 2)
        self.assertAlmostEqual(recorder.stats.encode_s, 2 * ENCODE_S)
        sent = " ".join(str(getattr(fake, name).mock_calls) for name in NEW_RERUN).lower()
        for lossy in ("jpeg", "jpg", "webp", "compress"):
            self.assertNotIn(lossy, sent)

    def test_the_live_view_passes_images_as_they_are_and_encodes_nothing(self):
        encoder = FakePng()
        recorder, fake = self.recorder(encoder=encoder)
        recorder.log_image(CAMERA_PATH, IMAGE)
        fake.Image.assert_called_once_with(IMAGE)
        fake.EncodedImage.assert_not_called()
        self.assertEqual((encoder.images, recorder.stats.encoded), ([], 0))

    def test_the_default_layout_has_a_3d_view_that_follows_world_ego_and_keeps_ground_truth_separate(self):
        _, fake = self.recorder()
        view = fake.blueprint.Spatial3DView
        view.assert_called_once()
        self.assertEqual(view.call_args.kwargs["origin"], "/world/ego")
        self.assertIn("/ground_truth/**", view.call_args.kwargs["contents"])
        fake.send_blueprint.assert_called_once_with(fake.blueprint.Blueprint.return_value)

    def test_without_a_usable_layout_module_the_recorder_still_works_and_says_how_to_follow(self):
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            recorder, fake = self.recorder(blueprint=False)
        fake.send_blueprint.assert_not_called()
        self.assertIn("default layout not sent", stdout.getvalue())
        self.assertIn("set the 3D view's origin to /world/ego", stdout.getvalue())
        recorder.log_lidar(LIDAR_PATH, [(1.0, 0.0, 0.0)], rerun_view.GROUND_ORIGIN)
        fake.Points3D.assert_called_once()


class GpsPositionTest(unittest.TestCase):
    def test_the_latest_reading_gives_the_position_in_metres(self):
        poll = [{"time": 0.1, "x": 1, "y": 2}, {"time": 0.2, "x": 4.5, "y": -5.0, "lat": 0.1, "lon": 0.2}]
        self.assertEqual(rerun_view.gps_position(poll), (4.5, -5.0))
        self.assertEqual(rerun_view.gps_position({"time": 0.1, "x": 1, "y": 2}), (1.0, 2.0))

    def test_latitude_and_longitude_alone_give_no_position(self):
        self.assertIsNone(rerun_view.gps_position([{"time": 0.1, "lat": 1.5, "lon": 2.5}]))

    def test_a_reading_without_finite_numbers_x_and_y_gives_no_position(self):
        for x, y in ((1.0, None), (float("nan"), 2.0), (1.0, float("inf")), ("1", 2.0), (True, 2.0), ((1.0,), 2.0)):
            self.assertIsNone(rerun_view.gps_position([{"time": 0.1, "x": x, "y": y}]), (x, y))
        self.assertIsNone(rerun_view.gps_position(["not a dict"]))


class RealRecorderTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = Path(tmp.name)
        self.fake = fake_rerun()

    def make(self, save_path):
        import spike_lidar_rerun

        with installed(self.fake):
            recorder = spike_lidar_rerun.make_recorder(save_path, False, 1)
        self.addCleanup(recorder.close)
        self.assertNotIn("rerun", sys.modules)

    def test_with_a_save_path_the_recording_goes_to_that_file_and_no_viewer_opens(self):
        path = self.tmp / "out" / "demo.rrd"
        self.make(path)
        self.fake.save.assert_called_once_with(str(path))
        self.assertNotIn(mock.call("kite_lidar", spawn=True), self.fake.init.call_args_list)

    def test_without_a_save_path_a_viewer_opens_and_nothing_is_saved(self):
        self.make(None)
        self.fake.init.assert_called_once_with("kite_lidar", spawn=True)
        self.fake.save.assert_not_called()

    def run_saved(self, *argv, session=fake_session, encoder=None, clock=None):
        import spike_lidar_rerun

        out = self.tmp / "out"
        stdout, stderr = io.StringIO(), io.StringIO()
        clock = clock or FakeClock()
        with contextlib.ExitStack() as stack:
            stack.enter_context(installed(self.fake))
            stack.enter_context(plain_lists(encoder or FakePng()))
            stack.enter_context(mock.patch.object(common, "session", session))
            stack.enter_context(mock.patch.object(common, "wall_clock", clock.now))
            stack.enter_context(mock.patch.object(common, "wall_sleep", clock.sleep))
            stack.enter_context(contextlib.redirect_stdout(stdout))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            code = spike_lidar_rerun.main(["--home", str(self.tmp), "--out", str(out), "--traffic", "0", *argv])
        self.assertEqual(code, 0, stderr.getvalue())
        self.assertNotIn("rerun", sys.modules)
        return out, stdout.getvalue()

    def test_a_run_with_save_records_to_the_file_inside_out_and_says_so(self):
        self.fake = fake_rerun(writes=1)
        out, stdout = self.run_saved("--mode", "lockstep", "--seconds", "0.05", "--save", "day1/demo.rrd")
        path = out / "day1" / "demo.rrd"
        self.fake.save.assert_called_once_with(str(path))
        self.assertIn(f"wrote {path}", stdout)
        self.fake.disconnect.assert_called_once_with()

    def test_a_saved_run_prints_its_size_on_disk_per_simulated_second_and_the_encoding_time(self):
        self.fake = fake_rerun(writes=2_500_000)
        clock = FakeClock()
        _, stdout = self.run_saved("--mode", "lockstep", "--seconds", "1", "--save", "demo.rrd",
                                   encoder=FakePng(clock), clock=clock)
        # 1 s of lock-step is 60 steps, each with one camera image, and the fake encoder takes
        # 0.01 s per image. 2,500,000 bytes over 1 simulated s is 2.50 MB per simulated s.
        self.assertIn("recording: 2500000 bytes on disk after close, 2.50 MB (1 MB = 10^6 bytes) over 1.000 "
                      "simulated s: 2.50 MB per simulated s", stdout)
        self.assertIn("PNG encoding: 60 camera images in 0.600 s", stdout)

    def test_a_saved_free_run_divides_by_the_simulated_seconds_the_timer_gave(self):
        self.fake = fake_rerun(writes=3_000_000)
        clock = FakeClock()

        @contextlib.contextmanager
        def clocked_session(args):
            sim = FakeSim(clock=clock)
            sim.open()
            try:
                yield sim
            finally:
                sim.close()

        _, stdout = self.run_saved("--seconds", "2", "--save", "demo.rrd", session=clocked_session, clock=clock)
        # The fake Timer runs with the fake clock and the run stops at 2 s of Timer time:
        # 3,000,000 bytes over 2 simulated s is 1.50 MB per simulated s.
        self.assertIn("3.00 MB (1 MB = 10^6 bytes) over 2.000 simulated s: 1.50 MB per simulated s", stdout)

    def test_a_recording_over_0_simulated_seconds_has_no_rate(self):
        import spike_lidar_rerun

        # R32: a rate over 0 seconds is n/a, never a division by zero or a made-up number.
        lines = spike_lidar_rerun.recording_lines(1_000_000, 0.0, 0, 0.0)
        self.assertEqual(lines[0], "recording: 1000000 bytes on disk after close, 1.00 MB (1 MB = 10^6 bytes) "
                                   "over 0.000 simulated s: n/a MB per simulated s")

    def test_a_recording_missing_after_close_is_an_error(self):
        import spike_lidar_rerun

        stderr = io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(installed(self.fake))
            stack.enter_context(plain_lists(FakePng()))
            stack.enter_context(mock.patch.object(common, "session", fake_session))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            code = spike_lidar_rerun.main(["--home", str(self.tmp), "--out", str(self.tmp / "out"), "--mode",
                                           "lockstep", "--seconds", "0.05", "--traffic", "0", "--save", "demo.rrd"])
        self.assertEqual(code, 1)
        self.assertIn("error: no recording at", stderr.getvalue())


class FreeModeViewTest(unittest.TestCase):
    """The default free run against the fake, with a recorder that keeps what reaches Rerun."""

    def run_free(self, sim_class=FakeSim, *argv):
        import spike_lidar_rerun

        clock = FakeClock()
        sims, recorder = [], RecordingRecorder()

        def make():
            sims.append(sim_class(clock=clock))
            return sims[-1]

        stdout = io.StringIO()
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(common, "FakeSim", make))
            stack.enter_context(mock.patch.object(common, "wall_clock", clock.now))
            stack.enter_context(mock.patch.object(common, "wall_sleep", clock.sleep))
            stack.enter_context(mock.patch.object(spike_lidar_rerun, "make_recorder", lambda *a: recorder))
            stack.enter_context(contextlib.redirect_stdout(stdout))
            code = spike_lidar_rerun.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", tmp, *argv])
        self.assertEqual(code, 0)
        return stdout.getvalue(), sims[0], recorder

    def test_world_ego_follows_the_gps_position_and_nothing_comes_from_ground_truth(self):
        stdout, sim, recorder = self.run_free(GpsWithPosition)
        moves = [c for c in recorder.calls if c[0] == "translation"]
        ego_moves = [c[2] for c in moves if c[1] == EGO_FRAME_PATH]
        self.assertGreater(len(ego_moves), 0)
        for position in ego_moves:
            self.assertEqual(position[1:], (200.0, 0.0))
            self.assertIn(position[0] - 100.0, sim.gps_ticks)
            self.assertNotEqual(position, sim.position("kite1"))
        self.assertEqual([c[2] for c in moves if c[1] == LIDAR_PATH],
                         [(-x, -y, 0.0) for x, y, _ in ego_moves])
        self.assertEqual({c[1] for c in moves}, {EGO_FRAME_PATH, LIDAR_PATH})
        logged = [c[1] for c in recorder.calls if c[0] != "time"]
        self.assertFalse([path for path in logged if path.startswith("ground_truth/")])
        self.assertNotIn("without a position", stdout)

    def test_an_imu_position_is_never_used_for_world_ego(self):
        _, _, recorder = self.run_free(ImuWithPosition)
        self.assertEqual([c for c in recorder.calls if c[0] == "translation"], [])

    def test_a_gps_without_a_position_leaves_world_ego_at_the_origin_and_the_run_says_so(self):
        stdout, sim, recorder = self.run_free()
        self.assertEqual([c for c in recorder.calls if c[0] == "translation"], [])
        # The fake GPS makes 10 samples per simulated second (DEFAULT_RATES) and is polled 5
        # times per wall second, staggered to 50 ms: due at 0.05 + k / 5 for k = 0 to 4, before
        # the Timer reading at 1 s ends the run. The polls cost nothing, so each lands on its due
        # time and sees a new sample: 5 new polls, none with a position.
        self.assertIn("gps gave 5 new polls without a position in their latest reading; world/ego did not move "
                      "for them", stdout)

    def test_the_gps_line_counts_polls_even_when_a_poll_carries_several_readings(self):
        stdout, sim, _ = self.run_free(GpsAt20Hz)
        # The same 5 polls as above, due at 0.05 + k / 5 for k = 0 to 4. At 20 samples per simulated
        # second every poll after the first carries the samples made since the one before it.
        self.assertGreater(max(sim.gps_readings_per_poll), 1)
        self.assertEqual(len(sim.gps_readings_per_poll), 5)
        self.assertIn("gps gave 5 new polls without a position in their latest reading", stdout)

    def test_a_polled_scan_with_a_non_finite_point_is_counted_and_not_logged_and_the_run_goes_on(self):
        stdout, sim, recorder = self.run_free(LidarWithNonFinitePoints, "--no-stream")
        scans = [c[2] for c in recorder.points_at(LIDAR_PATH)]
        for scan in scans:
            self.assertTrue(all(math.isfinite(value) for point in scan for value in point), scan)
        logged_times = {scan[0][2] for scan in scans}
        self.assertEqual(logged_times & set(LidarWithNonFinitePoints.BAD), set())
        # The fake lidar updates 10 times per simulated second; the scans of 0.1 s and 0.3 s each
        # hold one point with a NaN, in x and in z, and every other new scan is logged.
        self.assertEqual(sorted(logged_times | set(LidarWithNonFinitePoints.BAD)), sorted(sim.lidar_times))
        self.assertIn(f"logged {len(scans)} lidar scans", stdout)
        self.assertIn("lidar gave 2 new scans with a non-finite point; not logged", stdout)

    def test_only_readings_with_new_content_are_logged(self):
        stdout, sim, recorder = self.run_free()
        scans = [c[2] for c in recorder.points_at(LIDAR_PATH)]
        self.assertEqual(len(scans), len({tuple(scan) for scan in scans}))
        # The fake lidar updates 10 times per simulated second (DEFAULT_RATES) and is read 20
        # times per wall second at k / 20, k = 0 to 20, so each update is read twice and logged
        # once: the updates at 0.0, 0.1, ..., 1.0 s.
        self.assertEqual(len(scans), 11)
        self.assertIn("logged 11 lidar scans and 21 camera images at Timer times, each only when its content "
                      "was new", stdout)


class GpsWithPosition(FakeSim):
    """Its GPS readings also carry "x": 100 + sample tick and "y": 200, in metres."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.gps_ticks = set()

    def poll(self, car, kind):
        reading = super().poll(car, kind)
        if kind != "gps" or reading.raw is None:
            return reading
        self.gps_ticks.update(float(sample["tick"]) for sample in reading.raw)
        raw = [dict(sample, x=100.0 + sample["tick"], y=200.0) for sample in reading.raw]
        return reading_from_raw("gps", raw, self.sim_time)


class GpsAt20Hz(FakeSim):
    """Its GPS makes 20 samples per simulated second; records how many readings each GPS poll holds."""

    def __init__(self, **kwargs):
        super().__init__(rates={**DEFAULT_RATES, "gps": 20.0}, **kwargs)
        self.gps_readings_per_poll = []

    def poll(self, car, kind):
        reading = super().poll(car, kind)
        if kind == "gps" and reading.raw is not None:
            self.gps_readings_per_poll.append(len(reading.raw))
        return reading


class LidarWithNonFinitePoints(FakeSim):
    """Its polled lidar scan of time t is a plain list of rows: (5, 0, t), plus the point in BAD
    for the scans of 0.1 s and 0.3 s. Records the time of every scan it gives."""

    BAD = {0.1: (float("nan"), 1.0, 2.0), 0.3: (1.0, 2.0, float("nan"))}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.lidar_times = set()

    def poll(self, car, kind):
        reading = super().poll(car, kind)
        if kind != "lidar" or reading.raw is None:
            return reading
        time = reading.raw["time"]
        self.lidar_times.add(time)
        cloud = [(5.0, 0.0, time), *([self.BAD[time]] if time in self.BAD else [])]
        return reading_from_raw("lidar", {"time": time, "pointCloud": cloud}, self.sim_time)


class ImuWithPosition(FakeSim):
    """Its IMU readings carry a position under "pos" and under "x" and "y". Only a GPS reading
    may place world/ego, so these must not move it."""

    def poll(self, car, kind):
        reading = super().poll(car, kind)
        if kind != "imu" or reading.raw is None:
            return reading
        raw = [dict(sample, pos=(1.0, 2.0, 3.0), x=1.0, y=2.0) for sample in reading.raw]
        return reading_from_raw("imu", raw, self.sim_time)


class NoRerunTest(unittest.TestCase):
    def test_running_the_script_against_the_fake_imports_neither_rerun_nor_the_adapter(self):
        import spike_lidar_rerun

        clock = FakeClock()
        for mode in ("lockstep", "free"):
            with (tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()),
                  mock.patch.object(common, "FakeSim", lambda: FakeSim(clock=clock)),
                  mock.patch.object(common, "wall_clock", clock.now), mock.patch.object(common, "wall_sleep", clock.sleep)):
                self.assertEqual(spike_lidar_rerun.main(["--fake", "--mode", mode, "--seconds", "1", "--out", tmp]), 0)
            for name in ("rerun", "numpy", "PIL", "beamngpy", "kite_bridge.beamng_adapter"):
                self.assertNotIn(name, sys.modules, mode)

    def test_only_rerun_view_imports_rerun(self):
        sources = [*BRIDGE_DIR.glob("*.py"), *(BRIDGE_DIR / "kite_bridge").glob("*.py")]
        self.assertGreater(len(sources), 5)
        for path in sources:
            imports = re.search(r"(?m)^\s*(import|from)\s+rerun\b", path.read_text(encoding="utf-8"))
            self.assertEqual(bool(imports), path.name == "rerun_view.py", path.name)

    def test_rerun_view_imports_rerun_only_inside_a_method(self):
        text = (BRIDGE_DIR / "kite_bridge" / "rerun_view.py").read_text(encoding="utf-8")
        self.assertIsNone(re.search(r"(?m)^(import|from)\s+rerun\b", text))

    def test_only_rerun_view_imports_numpy_or_pillow_and_only_inside_a_method(self):
        sources = [*BRIDGE_DIR.glob("*.py"), *(BRIDGE_DIR / "kite_bridge").glob("*.py"), *(BRIDGE_DIR / "tests").glob("*.py")]
        for path in sources:
            text = path.read_text(encoding="utf-8")
            for package in ("numpy", "PIL"):
                imports = re.search(rf"(?m)^\s*(import|from)\s+{package}\b", text)
                self.assertEqual(bool(imports), path.name == "rerun_view.py", (path.name, package))
                self.assertIsNone(re.search(rf"(?m)^(import|from)\s+{package}\b", text), (path.name, package))

    def test_no_bridge_code_names_a_lossy_image_format(self):
        for path in [*BRIDGE_DIR.glob("*.py"), *(BRIDGE_DIR / "kite_bridge").glob("*.py")]:
            self.assertIsNone(re.search(r"(?i)jpe?g|webp|\.compress\(", path.read_text(encoding="utf-8")), path.name)


if __name__ == "__main__":
    unittest.main()
