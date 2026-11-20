import contextlib
import importlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from kite_bridge import common, rates
from kite_bridge.fake_sim import FakeClock, FakeSim
from kite_bridge.readings import reading_from_raw
from kite_bridge.rerun_view import ListArrays
from kite_bridge.self_hits import DEFAULT_EGO_RADIUS_M, SelfHits
from kite_bridge.sensor_settings import SENSOR_SETTINGS

CAR = "kite1"
SENSOR_RATES = {"lidar": 10.0, "camera": 20.0, "radar": 10.0, "gps": 10.0, "imu": 100.0}
ALL_60 = rates.parse_rates("all=60")
DEFAULT_POS = (0, 0, 1.7)


def gps_at(x, y):
    return [{"time": 0.0, "x": x, "y": y}]


def self_hits(radius=2.5, lidar_pos=DEFAULT_POS):
    return SelfHits(radius, lidar_pos, ListArrays())


class SelfHitCountTest(unittest.TestCase):
    def test_a_scan_counts_the_points_within_the_radius_in_x_and_y_at_any_height(self):
        hits = self_hits(2.5)
        hits.observe(CAR, "gps", gps_at(10.0, 20.0))
        inside = [(10.0, 20.0, 1.7), (11.0, 21.0, 2.0), (10.0, 20.0, 100.0), (12.5, 20.0, 0.0)]
        outside = [(12.6, 20.0, 0.0), (10.0, 17.4, 0.0), (0.0, 0.0, 0.0), (float("nan"), 20.0, 1.0)]
        hits.observe(CAR, "lidar", {"pointCloud": outside[:2] + inside + outside[2:]})
        # (12.5, 20) is exactly 2.5 m from (10, 20): a point on the radius counts.
        # (12.6, 20) is 2.6 m and (10, 17.4) is 2.6 m away: not counted.
        # The point with a nan x has no distance: not counted.
        summary = hits.summary(CAR)
        self.assertEqual((summary["scans_counted"], summary["mean"], summary["max"]), (1, 4.0, 4))

    def test_the_mean_and_the_maximum_run_over_the_scans_of_a_run(self):
        hits = self_hits(2.5)
        hits.observe(CAR, "gps", gps_at(0.0, 0.0))
        for near in (4, 1, 1):
            hits.observe(CAR, "lidar", {"pointCloud": [(1.0, 0.0, 1.0)] * near + [(50.0, 0.0, 1.0)] * 3})
        summary = hits.summary(CAR)
        self.assertEqual((summary["scans_counted"], summary["max"]), (3, 4))
        self.assertAlmostEqual(summary["mean"], 2.0)

    def test_a_scan_is_counted_against_the_latest_gps_position(self):
        hits = self_hits(2.5)
        scan = {"pointCloud": [(0.0, 0.0, 1.0), (30.0, 0.0, 1.0)]}
        hits.observe(CAR, "gps", gps_at(0.0, 0.0))
        hits.observe(CAR, "lidar", scan)
        hits.observe(CAR, "gps", gps_at(30.0, 0.0))
        hits.observe(CAR, "lidar", scan)
        hits.observe(CAR, "gps", gps_at(100.0, 0.0))
        hits.observe(CAR, "lidar", scan)
        summary = hits.summary(CAR)
        self.assertEqual((summary["scans_counted"], summary["mean"], summary["max"]), (3, 2 / 3, 1))

    def test_scans_without_a_gps_position_are_not_counted_and_are_reported(self):
        hits = self_hits(2.5)
        scan = {"pointCloud": [(0.0, 0.0, 1.0)]}
        hits.observe(CAR, "lidar", scan)
        hits.observe(CAR, "gps", [{"time": 0.0, "lat": 1.0, "lon": 2.0}])
        hits.observe(CAR, "lidar", scan)
        hits.observe(CAR, "gps", gps_at(0.0, 0.0))
        hits.observe(CAR, "lidar", scan)
        hits.observe(CAR, "gps", [{"time": 0.2, "lat": 1.0, "lon": 2.0}])
        hits.observe(CAR, "lidar", scan)
        summary = hits.summary(CAR)
        self.assertEqual((summary["scans_counted"], summary["max"], summary["scans_without_position"]), (1, 1, 3))

    def test_a_run_with_no_counted_scan_has_no_mean_or_maximum(self):
        summary = self_hits().summary(CAR)
        self.assertEqual((summary["scans_counted"], summary["mean"], summary["max"]), (0, None, None))

    def test_each_car_is_counted_against_its_own_position(self):
        hits = self_hits(2.5)
        hits.observe("kite1", "gps", gps_at(0.0, 0.0))
        hits.observe("kite2", "gps", gps_at(100.0, 0.0))
        hits.observe("kite1", "lidar", {"pointCloud": [(1.0, 0.0, 1.0)] * 3})
        hits.observe("kite2", "lidar", {"pointCloud": [(1.0, 0.0, 1.0)] * 3})
        self.assertEqual(hits.summary("kite1")["max"], 3)
        self.assertEqual(hits.summary("kite2")["max"], 0)

    def test_a_scan_that_is_not_x_y_z_rows_is_reported_and_never_stops_the_run(self):
        hits = self_hits(2.5)
        hits.observe(CAR, "gps", gps_at(0.0, 0.0))
        hits.observe(CAR, "lidar", {"pointCloud": [1.0, 2.0, 3.0]})
        hits.observe(CAR, "lidar", {"pointCloud": [(1.0, 0.0, 1.0)]})
        summary = hits.summary(CAR)
        self.assertEqual((summary["scans_unreadable"], summary["scans_counted"]), (1, 1))

    def test_the_summary_says_the_radius_and_the_mount_it_was_counted_for(self):
        summary = self_hits(3.5, (0.0, 0.5, 2.1)).summary(CAR)
        self.assertEqual((summary["radius_m"], summary["lidar_pos"]), (3.5, [0.0, 0.5, 2.1]))


class GpsAt3(FakeSim):
    """Its GPS readings carry x = 3 and y = 0, and its lidar scan of sample t has the 5 points
    (t + 1, r, 2) for r = 1 to 5, on the polled and the streamed path alike."""

    def __init__(self, **kwargs):
        super().__init__(lidar_rows=5, rates=SENSOR_RATES, **kwargs)

    def poll(self, car, kind):
        reading = super().poll(car, kind)
        if kind != "gps" or reading.raw is None:
            return reading
        return reading_from_raw("gps", [dict(sample, x=3.0, y=0.0) for sample in reading.raw], self.sim_time)


class NoSimulatorPosition(GpsAt3):
    def position(self, car):
        raise AssertionError("the self-hit count asked the simulator for our car's position")

    def other_positions(self):
        raise AssertionError("the self-hit count asked the simulator for the other cars' positions")


def measure(stream, sim_class=GpsAt3, hits=None, lidar_pos=None):
    clock = FakeClock()
    sim = sim_class(clock=clock)
    cars = sim.load_scenario("fake", 1)
    result = common.measure_sensors(sim, cars, 1.0, clock=clock.now, poll_rates=ALL_60, stream=stream,
                                    lidar_pos=lidar_pos, self_hits=hits)
    return result, sim, {r["sensor"]: r for r in result["sensors"]}


class MeasuredSelfHitsTest(unittest.TestCase):
    # One simulated second polled after every step (60 steps) sees the 10 Hz lidar samples 0 to
    # 10, each new once: 11 scans. The lidar is read before the GPS in a step, so sample 0, seen at
    # step 1, comes before any GPS position and is not counted: 10 scans are counted, samples 1 to 10.
    # Sample t has the points (t + 1, r) for r = 1 to 5 and our car is at (3, 0), so a point is
    # sqrt((t - 2)^2 + r^2) from it, inside 2.5 for:
    #   t = 1, 3: (1, 1) and (1, 2) apart: sqrt(2) and sqrt(5) are in, sqrt(10) is out -> 2
    #   t = 2:    (0, 1) and (0, 2) apart: 1 and 2 are in, 3 is out -> 2
    #   t = 4:    (2, 1) apart is sqrt(5) in, (2, 2) is sqrt(8) = 2.83 out -> 1
    #   t = 5 to 10: 3 or more apart in x -> 0
    COUNTS = [2, 2, 2, 1, 0, 0, 0, 0, 0, 0]

    def test_every_new_scan_is_counted_once_on_the_polled_and_the_streamed_path(self):
        for stream in (False, True):
            _, _, rows = measure(stream, hits=self_hits())
            summary = rows["lidar"]["self_hits_approx"]
            self.assertEqual(summary["scans_counted"], len(self.COUNTS), stream)
            self.assertEqual(summary["scans_without_position"], 1, stream)
            self.assertAlmostEqual(summary["mean"], sum(self.COUNTS) / len(self.COUNTS), msg=stream)
            self.assertEqual(summary["max"], 2, stream)

    def test_only_the_lidar_row_has_the_count(self):
        _, _, rows = measure(True, hits=self_hits())
        self.assertEqual([kind for kind, row in rows.items() if "self_hits_approx" in row], ["lidar"])

    def test_the_run_prints_the_count_labelled_approximate_with_the_mount_and_the_radius(self):
        result, _, _ = measure(True, hits=self_hits(2.5, (0.0, 0.0, 2.3)), lidar_pos=(0.0, 0.0, 2.3))
        lines = [line for line in common.format_result(result).splitlines() if "self hits" in line]
        self.assertEqual(len(lines), 2)
        self.assertIn("(approximate)", lines[0])
        self.assertIn("within 2.5 m", lines[0])
        self.assertEqual(lines[1], "self hits (approximate): kite1 lidar at (0, 0, 2.3): 10 scans counted, "
                                   "mean 0.7 points, max 2")
        self.assertIn("kite1 lidar: 1 new scans had no GPS position yet and were not counted",
                      common.format_result(result))

    def test_without_a_counter_the_output_has_no_self_hits(self):
        result, _, rows = measure(True)
        self.assertNotIn("self_hits_approx", rows["lidar"])
        self.assertNotIn("self hits", common.format_result(result))

    def test_the_count_never_asks_the_simulator_for_a_position(self):
        _, _, rows = measure(True, NoSimulatorPosition, hits=self_hits())
        self.assertEqual(rows["lidar"]["self_hits_approx"]["scans_counted"], len(self.COUNTS))

    def test_the_mount_reaches_the_lidar_the_fake_was_given_and_no_other_setting_changes(self):
        _, sim, _ = measure(True, lidar_pos=(0.0, 0.0, 2.3))
        passed = sim.lidar_settings[CAR]
        self.assertEqual(passed["pos"], (0.0, 0.0, 2.3))
        self.assertEqual({k: v for k, v in passed.items() if k != "pos"},
                         {**{k: v for k, v in SENSOR_SETTINGS["lidar"].items() if k != "pos"}, "is_streaming": True})

    def test_without_a_mount_the_lidar_keeps_the_table_position(self):
        _, sim, _ = measure(False)
        self.assertEqual(sim.lidar_settings[CAR]["pos"], DEFAULT_POS)


def run_script(module, *argv):
    sims = []

    def make():
        sims.append(FakeSim())
        return sims[-1]

    stdout, stderr = io.StringIO(), io.StringIO()
    with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
        stack.enter_context(mock.patch.object(common, "FakeSim", make))
        stack.enter_context(contextlib.redirect_stdout(stdout))
        stack.enter_context(contextlib.redirect_stderr(stderr))
        out = Path(tmp)
        code = module.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", str(out), *argv])
        saved = {path.name: json.loads(path.read_text(encoding="utf-8")) for path in out.glob("*.json")}
    return code, stdout.getvalue(), stderr.getvalue(), sims, saved


class LidarOptionsTest(unittest.TestCase):
    def modules(self):
        import spike_lidar_rerun
        import spike_multicar
        import spike_sensors

        return ((spike_sensors, ()), (spike_multicar, ()), (spike_lidar_rerun, ("--mode", "lockstep")))

    def test_the_mount_reaches_every_lidar_every_script_creates_and_shows_on_the_sensor_line(self):
        for module, extra in self.modules():
            code, stdout, stderr, sims, _ = run_script(module, "--lidar-pos", "0.5,-0.25,2.3", *extra)
            self.assertEqual(code, 0, (module.__name__, stderr))
            self.assertTrue(sims, module.__name__)
            for sim in sims:
                for settings in sim.lidar_settings.values():
                    self.assertEqual(settings["pos"], (0.5, -0.25, 2.3), module.__name__)
            lidar_line = next(line for line in stdout.splitlines() if line.startswith("sensor: lidar "))
            self.assertIn(" pos=(0.5, -0.25, 2.3) ", lidar_line + " ", module.__name__)
            radar_line = next(line for line in stdout.splitlines() if line.startswith("sensor: radar "))
            self.assertIn(" pos=(0, 0, 1.7) ", radar_line + " ", module.__name__)

    def test_without_the_option_every_lidar_stays_where_the_table_puts_it(self):
        for module, extra in self.modules():
            code, stdout, _, sims, _ = run_script(module, *extra)
            self.assertEqual(code, 0, module.__name__)
            for sim in sims:
                for settings in sim.lidar_settings.values():
                    self.assertEqual(settings["pos"], DEFAULT_POS, module.__name__)
            self.assertIn("sensor: lidar requested_update_time=0.1 pos=(0, 0, 1.7) ", stdout, module.__name__)

    def test_the_json_has_the_mount_and_the_self_hit_count(self):
        import spike_multicar
        import spike_sensors

        _, _, _, _, saved = run_script(spike_sensors, "--lidar-pos", "0,0,2.1", "--ego-radius", "4",
                                       "--rates", "all=60")
        data = saved["sensors.json"]
        self.assertEqual(data["sensor_settings"]["lidar"]["pos"], [0.0, 0.0, 2.1])
        lidar = next(row for row in data["sensors"] if row["sensor"] == "lidar")
        self.assertEqual(lidar["self_hits_approx"]["radius_m"], 4.0)
        self.assertEqual(lidar["self_hits_approx"]["lidar_pos"], [0.0, 0.0, 2.1])
        _, _, _, _, saved = run_script(spike_multicar, "--lidar-pos", "0,0,2.1")
        for case in saved["multicar.json"]["cases"]:
            lidars = [row for row in case["sensors"] if row["sensor"] == "lidar"]
            self.assertEqual(len(lidars), case["cars"])
            for row in lidars:
                self.assertEqual(row["self_hits_approx"]["radius_m"], DEFAULT_EGO_RADIUS_M)
                self.assertEqual(row["self_hits_approx"]["lidar_pos"], [0.0, 0.0, 2.1])

    def test_a_multi_case_script_counts_each_case_on_its_own(self):
        import spike_multicar

        _, _, _, _, saved = run_script(spike_multicar)
        for case in saved["multicar.json"]["cases"]:
            for row in (r for r in case["sensors"] if r["sensor"] == "lidar"):
                seen = row["self_hits_approx"]
                self.assertEqual(seen["scans_counted"] + seen["scans_without_position"], row["new_readings"])

    def test_the_radius_is_printed_and_defaults_to_2_5_metres(self):
        import spike_sensors

        _, stdout, _, _, _ = run_script(spike_sensors)
        self.assertIn("within 2.5 m of our car in x and y", stdout)
        _, stdout, _, _, _ = run_script(spike_sensors, "--ego-radius", "4")
        self.assertIn("within 4 m of our car in x and y", stdout)

    def test_bad_values_are_usage_errors(self):
        bad_mounts = ("nan,0,1", "0,inf,1", "0,0,-inf", "0,0", "0,0,1,2", "a,b,c", "", "1,,2", "0 0 1")
        for module, extra in self.modules():
            for bad in bad_mounts:
                with self.assertRaises(SystemExit) as raised, contextlib.redirect_stderr(io.StringIO()):
                    run_script(module, f"--lidar-pos={bad}", *extra)
                self.assertEqual(raised.exception.code, 2, (module.__name__, bad))
            for bad in ("0", "-1", "nan", "inf", "x"):
                with self.assertRaises(SystemExit) as raised, contextlib.redirect_stderr(io.StringIO()):
                    run_script(module, f"--ego-radius={bad}", *extra)
                self.assertEqual(raised.exception.code, 2, (module.__name__, bad))

    def test_a_negative_mount_is_given_with_an_equals_sign(self):
        import spike_sensors

        code, _, _, sims, _ = run_script(spike_sensors, "--lidar-pos=-0.5,0,1.9")
        self.assertEqual(code, 0)
        self.assertEqual(sims[0].lidar_settings[CAR]["pos"], (-0.5, 0.0, 1.9))

    def test_the_connect_script_creates_no_lidar_so_it_has_no_mount_option(self):
        import spike_connect

        with self.assertRaises(SystemExit) as raised, contextlib.redirect_stderr(io.StringIO()):
            run_script(spike_connect, "--lidar-pos", "0,0,2")
        self.assertEqual(raised.exception.code, 2)

    def test_the_lockstep_rerun_demo_counts_no_self_hits_so_a_radius_there_is_a_usage_error(self):
        import spike_lidar_rerun

        with self.assertRaises(SystemExit) as raised, contextlib.redirect_stderr(io.StringIO()):
            run_script(spike_lidar_rerun, "--mode", "lockstep", "--ego-radius", "3")
        self.assertEqual(raised.exception.code, 2)
        code, stdout, _, _, _ = run_script(spike_lidar_rerun, "--mode", "lockstep")
        self.assertEqual(code, 0)
        self.assertNotIn("self hits", stdout)

    def test_the_help_names_both_options_on_the_scripts_that_have_them(self):
        for name in ("spike_sensors", "spike_multicar", "spike_lidar_rerun"):
            stdout = io.StringIO()
            with self.assertRaises(SystemExit), contextlib.redirect_stdout(stdout):
                importlib.import_module(name).main(["--help"])
            for option in ("--lidar-pos", "--ego-radius"):
                self.assertIn(option, stdout.getvalue(), name)


if __name__ == "__main__":
    unittest.main()
