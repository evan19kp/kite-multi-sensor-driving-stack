import array
import contextlib
import io
import json
import math
import tempfile
import unittest
from fractions import Fraction
from pathlib import Path
from unittest import mock

from tests.test_free_running import SENSOR_RATES, BoundedClock
from tests.test_scripts import run_script
from kite_bridge import common, rates
from kite_bridge.fake_sim import FakeSim
from kite_bridge.rerun_view import CAMERA_PATH, EGO_PATH, GROUND_ORIGIN, GROUND_TRUTH_PATH, LIDAR_PATH
from kite_bridge.stream_check import reading_from_stream

# Evan's real runs: a poll costs about 20 ms, a streamed read about 1 ms.
SLOW_S = 0.020
STREAM_S = 0.001
POLLED = ("radar", "gps", "imu")
FREE_DEFAULTS = rates.parse_rates(None, rates.FREE_DEFAULT_RATES)
# The fake lidar updates 10 times per simulated second (SENSOR_RATES). 3 steps before the run put
# its updates at 0.05, 0.15, 0.25, ... wall s, half way between the Timer's due times, while the
# polls that start at 0.1 k are still running.
STEPS_BEFORE = 3
OFFSET_S = STEPS_BEFORE / 60


class Timed(FakeSim):
    """Records the fake-clock time each request or streamed read starts, and which lidar update
    each streamed lidar read returned (row 0 of update t is t + 1, see FakeSim)."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.starts = {}
        self.lidar_reads = []

    def poll(self, car, kind):
        self.starts.setdefault(kind, []).append(self._clock.now())
        return super().poll(car, kind)

    def timer_seconds(self):
        self.starts.setdefault("timer", []).append(self._clock.now())
        return super().timer_seconds()

    def stream(self, car, kind):
        started = self._clock.now()
        reading = super().stream(car, kind)
        if kind == "lidar":
            first_x = array.array("f", reading.raw["pointCloud"].tobytes()[:4])[0]
            self.lidar_reads.append((started, round(first_x) - 1))
        return reading


def free_stream_run(slow_s, seconds=2.0):
    clock = BoundedClock()
    sim = Timed(clock=clock, rates=SENSOR_RATES, request_cost=dict.fromkeys((*POLLED, "timer"), slow_s),
                stream_cost=STREAM_S)
    cars = sim.load_scenario("fake_cherrier_tograc", 1)
    sim.step(STEPS_BEFORE)
    result = common.measure_sensors(sim, cars, seconds, clock=clock.now, sleep=clock.sleep, poll_rates=FREE_DEFAULTS,
                                    stream=True, mode=common.FREE)
    return result, {r["sensor"]: r for r in result["sensors"]}, sim


def first_reads(sim):
    """{lidar update: wall time of the first read that returned it}"""
    first = {}
    for started, update in sim.lidar_reads:
        first.setdefault(update, started)
    return first


def appeared(update):
    return update / 10 - OFFSET_S


class LidarWaitsForOnePollAtMostTest(unittest.TestCase):
    def test_every_lidar_update_is_read_within_one_slow_poll_of_appearing(self):
        result, rows, sim = free_stream_run(SLOW_S)
        first = first_reads(sim)
        # Updates 1 to 20 appear at 0.05 to 1.95 s, inside the 2 s run. Each appears on a lidar
        # due time (multiples of 0.05 s). A pass reads the due streamed sensors before its one
        # poll, the lidar first, so the read waits only for the poll already running: 20 ms.
        waits = [first[update] - appeared(update) for update in range(1, 21)]
        self.assertLessEqual(max(waits), SLOW_S + 1e-9)
        # The bound is reached: the Timer, radar, GPS and IMU are all due at 0.1 k s, so their four
        # polls run back to back from 0.1 k, and each update at 0.1 k + 0.05 waits for one of them.
        self.assertGreater(max(waits), STREAM_S)
        self.assertEqual(rows["lidar"]["schedule"]["skipped"], 0)
        self.assertLessEqual(rows["lidar"]["schedule"]["worst_late_ms"], SLOW_S * 1000 + 1e-6)
        # The Timer wins the tie at 0.1 k, so radar, GPS and IMU each wait for at least its 20 ms
        # poll every time: every one of their polls is late, none by less than 20 ms.
        for kind in POLLED:
            schedule = rows[kind]["schedule"]
            self.assertEqual(schedule["late"], rows[kind]["polls"], kind)
            self.assertGreaterEqual(schedule["worst_late_ms"], SLOW_S * 1000, kind)


class StarvedSensorTest(unittest.TestCase):
    def test_a_poll_that_cannot_keep_up_reports_every_due_time_it_missed(self):
        # Four polls due 10 times per wall second at 30 ms each need 1.2 s per wall second, so
        # the polls cannot all keep up and some due times pass with no read.
        result, rows, sim = free_stream_run(0.030)
        schedules = {kind: rows[kind]["schedule"] for kind in POLLED}
        schedules["timer"] = result["timer"]["schedule"]
        reads = {kind: rows[kind]["polls"] for kind in POLLED}
        reads["timer"] = result["timer"]["polls"]
        self.assertGreater(sum(s["skipped"] for s in schedules.values()), 0)
        run_end = Fraction(sim.starts["timer"][-1])
        for kind, schedule in schedules.items():
            # Every due time k / 10 up to the sensor's last read was read or counted as skipped,
            # and no due time after the run's last Timer poll was counted.
            accounted = reads[kind] + schedule["skipped"]
            self.assertGreaterEqual(accounted, math.floor(Fraction(sim.starts[kind][-1]) * 10) + 1, kind)
            self.assertLessEqual(accounted, math.floor(run_end * 10) + 1, kind)
        text = common.format_result(result)
        for kind in POLLED:
            line = f"skipped: kite1 {kind} missed {schedules[kind]['skipped']} due reads"
            self.assertEqual(line in text, schedules[kind]["skipped"] > 0, kind)
        self.assertEqual(rows["lidar"]["schedule"]["skipped"], 0)
        first = first_reads(sim)
        self.assertLessEqual(max(first[u] - appeared(u) for u in range(1, 20)), 0.030 + 1e-9)

    def test_the_schedule_block_lists_every_sensor_and_the_timer(self):
        result, _, _ = free_stream_run(SLOW_S)
        text = common.format_result(result)
        self.assertIn("schedule: late counts reads that started 2 ms or more after their due time", text)
        self.assertRegex(text, r"(?m)^car\s+sensor\s+late\s+worst_ms\s+skipped$")
        for kind in ("lidar", "camera", *POLLED, "timer"):
            self.assertRegex(text, rf"(?m)^kite1\s+{kind}\s+\d+\s+\d+\.\d\d\s+\d+$")


class LockstepUnchangedTest(unittest.TestCase):
    def test_lockstep_reports_no_schedule_and_keeps_its_defaults(self):
        clock = BoundedClock()
        sim = FakeSim(clock=clock, rates=SENSOR_RATES, request_cost=SLOW_S)
        cars = sim.load_scenario("fake_cherrier_tograc", 1)
        result = common.measure_sensors(sim, cars, 1.0, clock=clock.now, poll_rates=rates.parse_rates(None))
        self.assertTrue(all("schedule" not in row for row in result["sensors"]))
        self.assertNotIn("schedule", common.format_result(result))
        self.assertEqual({r["sensor"]: r["configured_rate_hz"] for r in result["sensors"]},
                         {"lidar": 20.0, "camera": 20.0, "radar": 20.0, "gps": 10.0, "imu": 10.0})


class DefaultsTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name) / "out"

    def configured(self, *argv):
        clock = BoundedClock()
        with (mock.patch.object(common, "FakeSim", lambda: FakeSim(clock=clock)),
              mock.patch.object(common, "wall_clock", clock.now), mock.patch.object(common, "wall_sleep", clock.sleep),
              contextlib.redirect_stdout(io.StringIO())):
            import spike_sensors
            code = spike_sensors.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", str(self.out), *argv])
        self.assertEqual(code, 0)
        saved = json.loads((self.out / "sensors.json").read_text(encoding="utf-8"))
        return saved["mode"], {r["sensor"]: r["configured_rate_hz"] for r in saved["sensors"]}

    def test_free_mode_has_its_own_defaults_and_lockstep_keeps_its_own(self):
        self.assertEqual(self.configured(), ("lockstep", {"lidar": 20.0, "camera": 20.0, "radar": 20.0,
                                                          "gps": 10.0, "imu": 10.0}))
        self.assertEqual(self.configured("--mode", "free"), ("free", {"lidar": 20.0, "camera": 40.0, "radar": 10.0,
                                                                      "gps": 10.0, "imu": 10.0}))

    def test_a_named_rate_in_free_mode_keeps_the_free_defaults_for_the_rest(self):
        _, configured = self.configured("--mode", "free", "--rates", "lidar=30")
        self.assertEqual(configured, {"lidar": 30.0, "camera": 40.0, "radar": 10.0, "gps": 10.0, "imu": 10.0})

    def test_the_help_names_both_defaults(self):
        _, stdout, _ = run_script("spike_sensors", "--help")
        flat = stdout.replace("\n", "").replace(" ", "")
        self.assertIn(rates.DEFAULT_RATES_TEXT, flat)
        self.assertIn(rates.FREE_DEFAULT_RATES_TEXT, flat)


class RawRecorder:
    def __init__(self):
        self.calls = []
        self.closed = 0

    def set_time(self, seconds):
        self.calls.append(("time", seconds))

    def log_lidar(self, path, cloud, centre):
        self.calls.append(("points", path, cloud, centre))

    def log_points(self, path, positions, colors):
        self.calls.append(("points", path, positions, colors))

    def log_translation(self, path, translation):
        self.calls.append(("translation", path, translation))

    def log_image(self, path, image):
        self.calls.append(("image", path, image))

    def close(self):
        self.closed += 1


class NoColourCamera(Timed):
    """Its streamed camera gives a dict whose only image is None, which passes stream_check."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.camera_reads = 0

    def stream(self, car, kind):
        if kind != "camera":
            return super().stream(car, kind)
        self.camera_reads += 1
        return reading_from_stream(kind, {"colour": None}, self.sim_time)


class LiveViewerDefaultTest(unittest.TestCase):
    def run_main(self, *argv, sim_class=Timed):
        import spike_lidar_rerun

        clock = BoundedClock()
        sims, recorder = [], RawRecorder()

        def make():
            sims.append(sim_class(clock=clock, rates=SENSOR_RATES))
            return sims[-1]

        stdout, stderr = io.StringIO(), io.StringIO()
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(common, "FakeSim", make))
            stack.enter_context(mock.patch.object(common, "wall_clock", clock.now))
            stack.enter_context(mock.patch.object(common, "wall_sleep", clock.sleep))
            stack.enter_context(mock.patch.object(spike_lidar_rerun, "make_recorder", lambda *args: recorder))
            stack.enter_context(contextlib.redirect_stdout(stdout))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            code = spike_lidar_rerun.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", tmp, *argv])
        return code, stdout.getvalue(), stderr.getvalue(), sims, recorder

    def test_the_default_is_free_mode_with_streamed_lidar_and_camera(self):
        code, stdout, stderr, (sim,), recorder = self.run_main()
        self.assertEqual((code, stderr), (0, ""))
        self.assertIn("mode: free", stdout)
        self.assertRegex(stdout, r"(?m)^kite1\s+lidar\s.*\sstream\s+0\s+\d+$")
        self.assertRegex(stdout, r"(?m)^kite1\s+camera\s.*\sstream\s+0\s+\d+$")
        self.assertIn("real-time factor", stdout)
        self.assertIn("schedule:", stdout)
        self.assertRegex(stdout, r"(?m)^sensor: lidar .* is_streaming=True$")
        self.assertNotIn("lidar", sim.starts)
        self.assertNotIn("camera", sim.starts)
        self.assertEqual(recorder.closed, 1)

    def test_each_new_streamed_scan_and_image_is_logged_once_at_its_timer_time_without_ground_truth(self):
        _, stdout, _, (sim,), recorder = self.run_main()
        lidar = [c for c in recorder.calls if c[0] == "points"]
        images = [c for c in recorder.calls if c[0] == "image"]
        # The free defaults read the lidar 20 times per wall second, at k / 20 for k = 0 to 20 in
        # a 1 s run. The fake lidar updates 10 times per simulated second (SENSOR_RATES), so each
        # update is read twice and logged once: updates 0 to 10.
        self.assertEqual(len(sim.lidar_reads), 21)
        self.assertEqual(len(lidar), len({update for _, update in sim.lidar_reads}))
        self.assertEqual(len(lidar), 11)
        self.assertEqual({(c[1], c[3]) for c in lidar}, {(LIDAR_PATH, GROUND_ORIGIN)})
        self.assertEqual({c[1] for c in images}, {CAMERA_PATH})
        self.assertIn(f"logged 11 lidar scans and {len(images)} camera images at Timer times", stdout)
        self.assertNotIn(EGO_PATH, [c[1] for c in recorder.calls if c[0] != "time"])
        self.assertNotIn(GROUND_TRUTH_PATH, [c[1] for c in recorder.calls if c[0] != "time"])
        times = [c[1] for c in recorder.calls if c[0] == "time"]
        self.assertEqual(times, sorted(times))
        self.assertEqual(len(times), len(lidar) + len(images))

    def test_camera_readings_without_a_colour_image_are_counted_and_printed(self):
        code, stdout, _, (sim,), recorder = self.run_main(sim_class=NoColourCamera)
        self.assertEqual(code, 0)
        self.assertEqual([c for c in recorder.calls if c[0] == "image"], [])
        # The free default reads the camera 40 times per wall second, at k / 40 for k = 0 to 40
        # in a 1 s run, and none of those readings has a colour image. All 41 hold the same
        # content, so only the first is new and reaches the logger.
        self.assertEqual(sim.camera_reads, 41)
        self.assertIn("logged 11 lidar scans and 0 camera images at Timer times", stdout)
        self.assertIn("camera gave 1 new readings without a colour image; not logged", stdout)

    def test_every_camera_image_logged_prints_no_not_logged_line(self):
        _, stdout, _, _, _ = self.run_main()
        self.assertNotIn("not logged", stdout)

    def test_no_stream_polls_the_lidar_and_camera_in_free_mode(self):
        code, stdout, _, (sim,), _ = self.run_main("--no-stream")
        self.assertEqual(code, 0)
        self.assertIn("mode: free", stdout)
        self.assertNotIn("stream:", stdout)
        self.assertEqual(sim.lidar_reads, [])
        self.assertGreater(len(sim.starts["lidar"]), 0)

    def test_mode_lockstep_runs_the_demo_as_before(self):
        code, stdout, _, (sim,), recorder = self.run_main("--mode", "lockstep")
        self.assertEqual(code, 0)
        self.assertIn("logged 60 steps", stdout)
        self.assertNotIn("mode: free", stdout)
        self.assertEqual(len([c for c in recorder.calls if c[0] == "time"]), 60)

    def test_lockstep_with_stream_or_rates_is_a_usage_error(self):
        for extra in (("--stream",), ("--rates", "all=10")):
            code, stdout, stderr = run_script("spike_lidar_rerun", "--fake", "--mode", "lockstep", *extra)
            self.assertEqual((code, stdout), (2, ""), extra)
            self.assertIn("need --mode free", stderr)

    def test_help_says_free_and_stream_are_the_default(self):
        _, stdout, _ = run_script("spike_lidar_rerun", "--help")
        flat = " ".join(stdout.split())
        self.assertIn("--mode {lockstep,free}", flat)
        self.assertIn("free (default) or lockstep", flat)
        self.assertIn("--stream, --no-stream", flat)
        self.assertIn("default: on with --mode free", flat)
        self.assertIn("reads per wall second for each sensor", flat)
        self.assertIn(f"(default {rates.FREE_DEFAULT_RATES_TEXT}); needs --mode free", flat)
        self.assertNotIn("polls per simulated second", flat)
        self.assertNotIn(rates.DEFAULT_RATES_TEXT, flat)


if __name__ == "__main__":
    unittest.main()
