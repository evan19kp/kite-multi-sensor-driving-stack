import contextlib
import importlib
import io
import itertools
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tests import BRIDGE_DIR
from tests.fake_array import FakeArray
from tests.test_free_running import SENSOR_RATES, BoundedClock
from tests.test_slow_polls import RawRecorder
from kite_bridge import common
from kite_bridge.fake_sim import FakeSim
from kite_bridge.output_guard import PROVENANCE
from kite_bridge.readings import reading_from_raw
from kite_bridge.rerun_view import LIDAR_PATH

SCRIPTS = ("spike_connect", "spike_sensors", "spike_multicar", "spike_lidar_rerun")
EVERY_THIRD = 3


def run_main(module, argv, sim_class=FakeSim):
    """Runs a script's main on a fake simulator and a fake clock, with a recorder that keeps its
    calls and prints nothing. Returns (code, stdout, stderr, simulators, recorder)."""
    clock = BoundedClock()
    sims, recorder = [], RawRecorder()

    def make():
        sims.append(sim_class(clock=clock, rates=SENSOR_RATES))
        return sims[-1]

    stdout, stderr = io.StringIO(), io.StringIO()
    script = importlib.import_module(module)
    with contextlib.ExitStack() as stack:
        stack.enter_context(mock.patch.object(common, "FakeSim", make))
        stack.enter_context(mock.patch.object(common, "wall_clock", clock.now))
        stack.enter_context(mock.patch.object(common, "wall_sleep", clock.sleep))
        if hasattr(script, "make_recorder"):
            stack.enter_context(mock.patch.object(script, "make_recorder", lambda *args: recorder))
        stack.enter_context(contextlib.redirect_stdout(stdout))
        stack.enter_context(contextlib.redirect_stderr(stderr))
        code = script.main(["--fake", "--traffic", "0", *argv])
    return code, stdout.getvalue(), stderr.getvalue(), sims, recorder


class FailingReads(FakeSim):
    """Lock-step lidar and camera reads that raise `error` on every third call, and free-mode
    streamed or polled reads of those two sensors that raise it on every call."""

    error = RuntimeError("lidar boom")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._calls = itertools.count(1)

    def _fail_sometimes(self):
        if next(self._calls) % EVERY_THIRD == 0:
            raise self.error

    def lidar_points(self, car):
        self._fail_sometimes()
        return super().lidar_points(car)

    def camera_image(self, car):
        self._fail_sometimes()
        return super().camera_image(car)

    def stream(self, car, kind):
        raise self.error

    def poll(self, car, kind):
        if kind in ("lidar", "camera"):
            raise self.error
        return super().poll(car, kind)


class TimingOut(FailingReads):
    error = TimeoutError("timed out")


class RaisingReadsInLockstepTest(unittest.TestCase):
    def run_demo(self, sim_class, tmp):
        return run_main("spike_lidar_rerun", ["--mode", "lockstep", "--seconds", "1", "--out", tmp], sim_class)

    def test_a_raising_lidar_or_camera_read_is_counted_under_unexp_and_the_run_goes_on(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, stdout, stderr, _, recorder = self.run_demo(FailingReads, tmp)
        self.assertEqual((code, stderr), (0, ""))
        # Each step makes one lidar read then one camera read, so the shared counter fails the
        # 3rd, 6th, ... call: 120 calls, 40 failures, split 20 and 20 between lidar and camera.
        self.assertIn("unexp: lidar read raised on 20 of 60 steps, first: RuntimeError: lidar boom;", stdout)
        self.assertIn("unexp: camera read raised on 20 of 60 steps, first: RuntimeError: lidar boom;", stdout)
        self.assertIn("logged 60 steps", stdout)
        lidar_logged = [c for c in recorder.calls if c[0] == "points" and c[1] == LIDAR_PATH]
        images_logged = [c for c in recorder.calls if c[0] == "image"]
        self.assertEqual((len(lidar_logged), len(images_logged)), (40, 40))
        self.assertEqual(recorder.closed, 1)

    def test_a_raised_read_is_not_also_counted_as_no_reading(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, stdout, _, _, _ = self.run_demo(FailingReads, tmp)
        self.assertEqual(code, 0)
        self.assertIn("unexp: lidar read raised", stdout)
        self.assertNotIn("gave no reading", stdout)

    def test_a_step_with_no_reading_is_still_counted_as_no_reading(self):
        class Silent(FakeSim):
            def lidar_points(self, car):
                return None

        with tempfile.TemporaryDirectory() as tmp:
            _, stdout, _, _, _ = self.run_demo(Silent, tmp)
        self.assertIn("lidar gave no reading on 60 of 60 steps", stdout)
        self.assertNotIn("unexp", stdout)

    def test_a_run_whose_reads_do_not_raise_prints_no_unexp_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, stdout, _, _, _ = self.run_demo(FakeSim, tmp)
        self.assertNotIn("unexp", stdout)

    def test_a_timeout_stops_the_run_and_shuts_the_simulator_down(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, stdout, stderr, (sim,), _ = self.run_demo(TimingOut, tmp)
        self.assertEqual(code, 1)
        self.assertIn("timed out after", stderr)
        self.assertNotIn("unexp", stdout)
        self.assertTrue(sim.closed)


class RaisingReadsInFreeModeTest(unittest.TestCase):
    def run_demo(self, sim_class, tmp, *extra):
        return run_main("spike_lidar_rerun", ["--seconds", "1", "--out", tmp, *extra], sim_class)

    def test_a_raising_streamed_read_is_counted_under_unexp_and_the_run_goes_on(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, stdout, _, _, recorder = self.run_demo(FailingReads, tmp)
        self.assertEqual(code, 0)
        # The free defaults read the lidar 20 and the camera 40 times per wall second: 21 and 41
        # reads in a 1 s run, all of which raise.
        self.assertIn("unexpected format: kite1 lidar 21 readings, first: stream raised RuntimeError: lidar boom",
                      stdout)
        self.assertIn("unexpected format: kite1 camera 41 readings, first: stream raised RuntimeError: lidar boom",
                      stdout)
        self.assertRegex(stdout, r"(?m)^kite1\s+lidar\s+0\s.*\s+21\s+n/a\s+n/a\s+stream\s+0\s+0$")
        self.assertIn("logged 0 lidar scans and 0 camera images", stdout)
        self.assertEqual(recorder.closed, 1)

    def test_a_raising_polled_read_is_counted_under_unexp_and_the_run_goes_on(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, stdout, _, _, _ = self.run_demo(FailingReads, tmp, "--no-stream")
        self.assertEqual(code, 0)
        # Due at k / 20 and k / 40 for k = 0 to 20 and 0 to 40, but a pass makes at most one poll
        # and at 1 s the Timer wins the tie for it and ends the run: 20 and 40 polls.
        self.assertIn("unexpected format: kite1 lidar 20 readings, first: poll raised RuntimeError: lidar boom", stdout)
        self.assertIn("unexpected format: kite1 camera 40 readings, first: poll raised RuntimeError: lidar boom", stdout)

    def test_a_timeout_stops_the_run_and_shuts_the_simulator_down(self):
        for extra in ((), ("--no-stream",)):
            with self.subTest(extra=extra), tempfile.TemporaryDirectory() as tmp:
                code, stdout, stderr, (sim,), _ = self.run_demo(TimingOut, tmp, *extra)
            self.assertEqual(code, 1)
            self.assertIn("timed out after", stderr)
            self.assertNotIn("unexpected format", stdout)
            self.assertTrue(sim.closed)


class MulticarJsonTest(unittest.TestCase):
    def saved(self, tmp, script, name, *extra):
        code, _, stderr, _, _ = run_main(script, ["--seconds", "1", "--out", tmp, *extra])
        self.assertEqual(code, 0, stderr)
        return json.loads((Path(tmp) / name).read_text(encoding="utf-8"))

    def test_multicar_json_has_the_settings_the_run_used_and_they_match_sensors_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            multicar = self.saved(tmp, "spike_multicar", "multicar.json", "--stream", "--lidar-pos", "1,2,3")
            sensors = self.saved(tmp, "spike_sensors", "sensors.json", "--stream", "--lidar-pos", "1,2,3")
        settings = multicar["sensor_settings"]
        self.assertEqual(set(settings), {"lidar", "camera", "radar", "gps", "imu"})
        self.assertEqual(settings["lidar"]["pos"], [1, 2, 3])
        self.assertIs(settings["lidar"]["is_streaming"], True)
        self.assertEqual(settings, sensors["sensor_settings"])

    def test_multicar_json_settings_follow_the_defaults_without_options(self):
        with tempfile.TemporaryDirectory() as tmp:
            settings = self.saved(tmp, "spike_multicar", "multicar.json")["sensor_settings"]
        self.assertEqual(settings["lidar"]["pos"], [0, 0, 1.7])
        self.assertNotIn("is_streaming", settings["lidar"])


class DetectColumnTest(unittest.TestCase):
    def test_the_detect_column_prints_one_decimal(self):
        rows = itertools.cycle((2, 3))

        class AlternatingRadar(FakeSim):
            def poll(self, car, kind):
                if kind == "radar":
                    return reading_from_raw(kind, FakeArray(next(rows), 7), self.sim_time)
                return super().poll(car, kind)

        sim = AlternatingRadar()
        cars = sim.load_scenario("fake_cherrier_tograc", 1)
        result = common.measure_sensors(sim, cars, 1.0)
        # 60 polls alternate 2 and 3 detections: the mean is 2.5, which no whole number shows.
        self.assertRegex(common.format_result(result), r"(?m)^kite1\s+radar\s.*\s2\.5$")


class ListModelsOutTest(unittest.TestCase):
    INSIDE_THE_REPO = BRIDGE_DIR.parent / "docs" / "not_written_by_a_test"

    def run_script(self, script, *argv):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            try:
                code = importlib.import_module(script).main(["--fake", "--out", str(self.INSIDE_THE_REPO), *argv])
            except SystemExit as stop:
                code = stop.code
        return code, stdout.getvalue(), stderr.getvalue()

    def test_list_models_does_not_check_an_out_that_it_never_writes(self):
        for script in SCRIPTS:
            with self.subTest(script=script):
                code, stdout, stderr = self.run_script(script, "--list-models")
                self.assertEqual((code, stderr), (0, ""))
                self.assertIn("vivace", stdout.splitlines())
                self.assertFalse(self.INSIDE_THE_REPO.exists())

    def test_the_same_out_is_still_refused_when_the_run_writes(self):
        for script in SCRIPTS:
            with self.subTest(script=script):
                code, stdout, stderr = self.run_script(script)
                self.assertEqual((code, stdout), (2, ""))
                self.assertIn("inside the repo", stderr)


class WriteJsonProvenanceTest(unittest.TestCase):
    def test_a_payload_with_its_own_provenance_is_refused_and_nothing_is_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                common.write_json(tmp, "x.json", {"provenance": "mine", "value": 1})
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_a_payload_without_one_is_written_with_the_constant_first(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            common.write_json(tmp, "x.json", {"value": 1})
            saved = json.loads((Path(tmp) / "x.json").read_text(encoding="utf-8"))
        self.assertEqual(saved, {"provenance": PROVENANCE, "value": 1})


if __name__ == "__main__":
    unittest.main()
