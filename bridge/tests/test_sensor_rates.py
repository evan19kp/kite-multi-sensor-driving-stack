import json
import tempfile
import unittest
from fractions import Fraction
from pathlib import Path

from tests.test_scripts import run_script
from kite_bridge import common, measure, rates
from kite_bridge.fake_sim import FakeClock, FakeSim
from kite_bridge.sensor_settings import requested_update_hz
from kite_bridge.sim_api import SENSOR_KINDS, STEP_HZ

# The fake sensors update at the rates the adapter asks for (lidar and radar 10 Hz, camera 20 Hz)
# and GPS and IMU buffer 100 readings a second.
SENSOR_RATES = {"lidar": 10.0, "camera": 20.0, "radar": 10.0, "gps": 100.0, "imu": 100.0}
REQUEST_COST = {"lidar": 0.020, "camera": 0.022, "radar": 0.021, "gps": 0.020, "imu": 0.021}
STEP_COST = 0.047
ALL_60 = rates.parse_rates("all=60")
DEFAULT = rates.parse_rates(rates.DEFAULT_RATES_TEXT)


def measure_at(poll_rates, seconds=1.0, sensor_rates=SENSOR_RATES, sim_class=FakeSim):
    clock = FakeClock()
    sim = sim_class(rates=sensor_rates, clock=clock, request_cost=REQUEST_COST, step_cost=STEP_COST,
                    lidar_rows=100)
    cars = sim.load_scenario("fake_cherrier_tograc", 1)
    result = common.measure_sensors(sim, cars, seconds, clock=clock.now, poll_rates=poll_rates)
    return result, {r["sensor"]: r for r in result["sensors"]}


class ParseRatesTest(unittest.TestCase):
    def test_the_default_is_20_for_lidar_camera_radar_and_10_for_gps_and_imu(self):
        self.assertEqual(rates.parse_rates(rates.DEFAULT_RATES_TEXT),
                         {"lidar": 20, "camera": 20, "radar": 20, "gps": 10, "imu": 10})
        self.assertEqual(rates.DEFAULT_RATES_TEXT, "lidar=20,camera=20,radar=20,gps=10,imu=10")

    def test_all_sets_every_sensor(self):
        self.assertEqual(rates.parse_rates("all=60"), dict.fromkeys(SENSOR_KINDS, 60))

    def test_a_named_sensor_changes_only_that_sensor_from_the_defaults(self):
        self.assertEqual(rates.parse_rates("gps=60"),
                         {"lidar": 20, "camera": 20, "radar": 20, "gps": 60, "imu": 10})

    def test_a_named_sensor_overrides_all_wherever_it_is_written(self):
        expected = {"lidar": 60, "camera": 60, "radar": 60, "gps": 5, "imu": 60}
        self.assertEqual(rates.parse_rates("all=60,gps=5"), expected)
        self.assertEqual(rates.parse_rates("gps=5,all=60"), expected)

    def test_decimal_rates_are_exact_and_spaces_are_ignored(self):
        self.assertEqual(rates.parse_rates(" imu = 2.5 ")["imu"], Fraction(5, 2))
        self.assertEqual(rates.parse_rates("lidar=60")["lidar"], 60)
        self.assertEqual(rates.parse_rates("lidar=0.001")["lidar"], Fraction(1, 1000))

    def test_anything_outside_the_domain_is_refused(self):
        for text in ("", "lidar", "lidar=", "=10", "lidar=10,", "lidar=10=20", "fast=10", "lidar=abc",
                     "lidar=0", "lidar=-1", "lidar=60.001", "lidar=61", "lidar=1e999", "all=nan",
                     "all=inf", "all=-inf", "lidar=10,lidar=20", "all=10,all=20", "lidar=1/3"):
            with self.assertRaises(ValueError, msg=text):
                rates.parse_rates(text)

    def test_a_rate_that_is_above_60_only_in_exact_arithmetic_is_refused(self):
        # float("60.00000000000000001") is exactly 60.0, but the decimal text is above 60.
        for text in ("lidar=60.00000000000000001", "all=60.000000000000000000001"):
            with self.assertRaises(ValueError, msg=text):
                rates.parse_rates(text)

    def test_the_message_names_the_sensor_and_the_value(self):
        with self.assertRaisesRegex(ValueError, "lidar.*61"):
            rates.parse_rates("lidar=61")
        with self.assertRaisesRegex(ValueError, "unknown sensor 'fast'"):
            rates.parse_rates("fast=10")


class ScheduleTest(unittest.TestCase):
    def steps(self, text, kind, count):
        return [k for k in range(1, count + 1) if rates.polled(k, rates.parse_rates(text)[kind])]

    def test_the_first_step_is_always_polled_then_each_step_where_floor_of_k_r_over_60_advances(self):
        # r = 20: floor(k / 3) advances at k = 3, 6, 9, ..., so with step 1 the steps are 1, 3, 6, 9, ...
        self.assertEqual(self.steps("lidar=20", "lidar", 12), [1, 3, 6, 9, 12])
        # r = 10: floor(k / 6) advances at 6, 12, ...
        self.assertEqual(self.steps("gps=10", "gps", 18), [1, 6, 12, 18])

    def test_a_rate_that_does_not_divide_60_still_gives_the_documented_count(self):
        # r = 7, 60 steps: floor(7k / 60) goes up at k = 9 (63), 18 (126), 26 (182), 35 (245),
        # 43 (301), 52 (364) and 60 (420): 7 advances, plus step 1.
        self.assertEqual(self.steps("imu=7", "imu", 60), [1, 9, 18, 26, 35, 43, 52, 60])
        # r = 2.5 = 5 / 2: floor(k / 24) goes up at 24 and 48, plus step 1.
        self.assertEqual(self.steps("imu=2.5", "imu", 60), [1, 24, 48])

    def test_poll_counts_follow_floor_n_r_over_60_plus_one_below_60_and_n_at_60(self):
        for text, n, polls in (("lidar=20", 1800, 601), ("lidar=10", 1800, 301), ("lidar=1", 1800, 31),
                               ("lidar=59", 1800, 1771), ("lidar=60", 1800, 1800),
                               ("lidar=1.1", 600, 12), ("lidar=0.001", 1800, 1)):
            self.assertEqual(len(self.steps(text, "lidar", n)), polls, text)

    def test_every_step_is_polled_at_60(self):
        self.assertEqual(self.steps("all=60", "gps", 100), list(range(1, 101)))

    def test_polling_at_twice_a_sensors_rate_leaves_half_its_interval_between_polls(self):
        # A sensor at 10 Hz updates every 6 steps; polling at 20 Hz polls every 3 steps.
        steps = self.steps("lidar=20", "lidar", 600)
        self.assertEqual({b - a for a, b in zip(steps[1:], steps[2:])}, {3})


class FewerPollsNoLossTest(unittest.TestCase):
    def test_every_step_polls_each_sensor_60_times_a_second(self):
        result, rows = measure_at(ALL_60, seconds=2.0)
        self.assertEqual({kind: row["polls"] for kind, row in rows.items()}, dict.fromkeys(SENSOR_KINDS, 120))
        self.assertEqual({row["configured_rate_hz"] for row in rows.values()}, {60.0})

    def test_the_default_rates_poll_less_and_lose_no_new_reading(self):
        _, every = measure_at(ALL_60, seconds=2.0)
        _, default = measure_at(DEFAULT, seconds=2.0)
        # 120 steps. Lidar, camera and radar at 20 polls a second: floor(120 * 20 / 60) + 1 = 41 polls.
        # GPS and IMU at 10: floor(120 * 10 / 60) + 1 = 21 polls.
        self.assertEqual({k: r["polls"] for k, r in default.items()},
                         {"lidar": 41, "camera": 41, "radar": 41, "gps": 21, "imu": 21})
        # New readings in 2 s. Lidar and radar at 10 Hz: samples 0 to 20 = 21. Camera at 20 Hz:
        # samples 0 to 40 = 41. GPS and IMU at 100 Hz buffered: samples 1 to 200 = 200, each returned once
        # whichever polls collect them (the first poll takes sample 1, later ones the samples made since).
        expected = {"lidar": 21, "camera": 41, "radar": 21, "gps": 200, "imu": 200}
        self.assertEqual({k: r["new_readings"] for k, r in every.items()}, expected)
        self.assertEqual({k: r["new_readings"] for k, r in default.items()}, expected)
        for kind in SENSOR_KINDS:
            self.assertLess(default[kind]["polls"], every[kind]["polls"])
            self.assertEqual(default[kind]["new_per_simulated_s"], every[kind]["new_per_simulated_s"])
            self.assertFalse(default[kind]["possible_loss"], kind)

    def test_buffered_sensors_return_the_readings_in_fewer_bigger_polls(self):
        _, every = measure_at(ALL_60, seconds=2.0)
        _, default = measure_at(DEFAULT, seconds=2.0)
        # 100 Hz at 60 polls a second: 1 or 2 samples a poll. At 10 polls a second: 10 a poll.
        self.assertEqual(every["gps"]["readings_per_poll_max"], 2)
        self.assertEqual(default["gps"]["readings_per_poll_max"], 10)

    def test_the_summary_shows_the_poll_rate_beside_new_readings_and_flags_no_loss(self):
        result, _ = measure_at(DEFAULT, seconds=2.0)
        text = common.format_result(result)
        self.assertRegex(text, r"(?m)^car\s+sensor\s+read_s\s+per_poll_ms\s+poll_hz\s+new\s+new_per_s\s+req_hz\s")
        # lidar: 41 polls * 0.020 = 0.820 s, 20.00 ms a poll, 20.00 polls a second, 21 new, 10.50 a second,
        # 10.00 Hz requested. GPS: 21 * 0.020 = 0.420 s, 10 polls a second, 200 new, 100.00 a second.
        self.assertRegex(text, r"(?m)^kite1\s+lidar\s+0\.820\s+20\.00\s+20\.00\s+21\s+10\.50\s+10\.00\s")
        self.assertRegex(text, r"(?m)^kite1\s+gps\s+0\.420\s+20\.00\s+10\.00\s+200\s+100\.00\s+n/a\s")
        self.assertNotIn("possible loss", text)
        self.assertIn("loss not checkable for gps, imu", text)

    def test_every_step_prints_no_loss_note_at_all(self):
        result, _ = measure_at(ALL_60, seconds=1.0)
        text = common.format_result(result)
        self.assertNotIn("possible loss", text)
        self.assertNotIn("loss not checkable", text)


class TimingStillAddsUpTest(unittest.TestCase):
    def test_fewer_reads_shrink_the_read_time_and_the_parts_still_add_up(self):
        result, rows = measure_at(DEFAULT, seconds=1.0)
        t = result["timing"]
        # 1 s = 60 steps. Polls: 21 each for lidar, camera, radar and 11 each for GPS and IMU.
        # Reads: 21 * 0.020 + 21 * 0.022 + 21 * 0.021 + 11 * 0.020 + 11 * 0.021
        #      = 0.420 + 0.462 + 0.441 + 0.220 + 0.231 = 1.774 s. Steps: 60 * 0.047 = 2.82 s.
        # Wall: 2.82 + 1.774 = 4.594 s, so the real-time factor is 1 / 4.594.
        self.assertAlmostEqual(t["sensor_reads_s"], 1.774)
        self.assertAlmostEqual(t["step_wait_s"], 2.82)
        self.assertAlmostEqual(t["processing_s"], 0.0)
        self.assertAlmostEqual(t["wall_s"], 4.594)
        self.assertAlmostEqual(t["real_time_factor"], 1 / 4.594)
        self.assertAlmostEqual(t["step_wait_s"] + t["sensor_reads_s"] + t["processing_s"], t["wall_s"])
        self.assertAlmostEqual(rows["lidar"]["read_total_s"], 21 * 0.020)
        self.assertAlmostEqual(rows["lidar"]["read_per_poll_s"], 0.020)
        self.assertAlmostEqual(rows["imu"]["read_total_s"], 11 * 0.021)
        self.assertAlmostEqual(rows["imu"]["read_per_poll_s"], 0.021)

    def test_every_step_still_costs_what_it_did_before_the_rates_existed(self):
        result, _ = measure_at(ALL_60, seconds=1.0)
        # 60 * 0.104 = 6.24 s of reads and 2.82 s of steps, as in the timing-breakdown tests.
        self.assertAlmostEqual(result["timing"]["sensor_reads_s"], 6.24)
        self.assertAlmostEqual(result["timing"]["wall_s"], 9.06)


class EveryStepIsTodaysBehaviourTest(unittest.TestCase):
    def calls(self, **kwargs):
        events = []

        class Recording(FakeSim):
            def step(self, n=1):
                events.append("step")
                super().step(n)

            def poll(self, car, kind):
                events.append(f"{car} {kind}")
                return super().poll(car, kind)

        sim = Recording()
        cars = sim.load_scenario("fake_cherrier_tograc", 2)
        result = common.measure_sensors(sim, cars, 3 / STEP_HZ, clock=FakeClock().now, **kwargs)
        return events, result

    def test_all_60_steps_then_polls_every_sensor_of_every_car_in_order_each_step(self):
        events, _ = self.calls(poll_rates=ALL_60)
        one_step = ["step"] + [f"{car} {kind}" for car in ("kite1", "kite2") for kind in SENSOR_KINDS]
        self.assertEqual(events, one_step * 3)

    def test_all_60_gives_the_same_result_as_giving_no_rates(self):
        events, result = self.calls(poll_rates=ALL_60)
        before_events, before = self.calls()
        self.assertEqual(events, before_events)
        self.assertEqual(result["sensors"], before["sensors"])

    def test_a_reduced_rate_skips_the_polls_in_between(self):
        events, _ = self.calls(poll_rates=rates.parse_rates("all=60,lidar=20"))
        # 3 steps at 20 polls a second: step 1 and step 3 (floor(3 * 20 / 60) goes from 0 to 1)
        self.assertEqual(events.count("kite1 lidar"), 2)
        self.assertEqual(events.count("kite1 camera"), 3)


class PossibleLossTest(unittest.TestCase):
    def test_a_lidar_read_at_5_polls_a_second_misses_updates_and_is_flagged(self):
        result, rows = measure_at(rates.parse_rates("lidar=5"), seconds=1.0)
        # 5 polls a second: steps 1, 12, 24, 36, 48, 60, at 0, 0.2, 0.4, ... s, on 10 Hz samples
        # 0, 2, 4, 6, 8, 10: 6 new readings. 10 Hz requested over 1 s is 10, less 1 of slack = 9.
        self.assertEqual(rows["lidar"]["polls"], 6)
        self.assertEqual(rows["lidar"]["new_readings"], 6)
        self.assertTrue(rows["lidar"]["possible_loss"])
        self.assertFalse(rows["camera"]["possible_loss"])
        text = common.format_result(result)
        self.assertIn("possible loss: kite1 lidar gave 6.00 new readings per simulated second at 5 polls per "
                      "second, fewer than the 10 Hz requested.", text)
        self.assertEqual(text.count("possible loss"), 1)

    def test_polling_at_twice_the_update_rate_is_not_flagged(self):
        _, rows = measure_at(rates.parse_rates("lidar=20,camera=40,radar=20"), seconds=2.0)
        for kind in ("lidar", "camera", "radar"):
            self.assertFalse(rows[kind]["possible_loss"], kind)

    def test_a_sensor_polled_every_step_is_never_flagged_even_if_it_updates_slowly(self):
        _, rows = measure_at(ALL_60, seconds=1.0, sensor_rates={**SENSOR_RATES, "lidar": 2.0})
        self.assertEqual(rows["lidar"]["new_readings"], 3)
        self.assertFalse(rows["lidar"]["possible_loss"])

    def test_the_same_slow_sensor_at_a_reduced_rate_is_flagged(self):
        _, rows = measure_at(DEFAULT, seconds=1.0, sensor_rates={**SENSOR_RATES, "lidar": 2.0})
        self.assertTrue(rows["lidar"]["possible_loss"])

    def test_gps_and_imu_are_never_flagged_because_no_requested_rate_is_known(self):
        _, rows = measure_at(rates.parse_rates("gps=1,imu=1"), seconds=1.0)
        self.assertFalse(rows["gps"]["possible_loss"])
        self.assertFalse(rows["imu"]["possible_loss"])
        self.assertIsNone(rows["gps"]["requested_update_hz"])

    def test_the_rule_allows_one_reading_of_slack(self):
        # 10 Hz over 1 s: 9 or more is fine, 8 is flagged.
        self.assertFalse(measure.possible_loss(10, 1.0, 10.0))
        self.assertFalse(measure.possible_loss(9, 1.0, 10.0))
        self.assertTrue(measure.possible_loss(8, 1.0, 10.0))
        self.assertTrue(measure.possible_loss(0, 30.0, 10.0))
        self.assertFalse(measure.possible_loss(299, 30.0, 10.0))
        self.assertTrue(measure.possible_loss(298, 30.0, 10.0))

    def test_the_rule_refuses_values_outside_its_domain(self):
        for args in ((-1, 1.0, 10.0), (5, float("nan"), 10.0), (5, 1.0, float("inf"))):
            with self.assertRaises(ValueError, msg=str(args)):
                measure.possible_loss(*args)


class RequestedRateTest(unittest.TestCase):
    def test_it_is_one_over_the_requested_update_time_the_adapter_passes(self):
        self.assertEqual(requested_update_hz("lidar"), 10.0)
        self.assertEqual(requested_update_hz("camera"), 20.0)
        self.assertEqual(requested_update_hz("radar"), 10.0)

    def test_gps_and_imu_pass_no_requested_update_time_so_have_no_requested_rate(self):
        self.assertIsNone(requested_update_hz("gps"))
        self.assertIsNone(requested_update_hz("imu"))


class RatesOptionTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name) / "out"

    def run_fake(self, script, *extra):
        code, stdout, stderr = run_script(script, "--fake", "--seconds", "1", "--out", str(self.out), *extra)
        self.assertEqual(code, 0, stderr)
        return stdout

    def test_a_rate_outside_the_domain_is_a_usage_error_before_anything_runs(self):
        for bad in ("lidar=61", "lidar=60.5", "lidar=60.00000000000000001", "gps=0", "imu=-3", "all=nan",
                    "all=inf", "lidar=x", "fast=10", "lidar", ""):
            code, stdout, stderr = run_script("spike_sensors", "--fake", "--out", str(self.out), "--rates", bad)
            self.assertEqual(code, 2, bad)
            self.assertIn("usage", stderr)
            self.assertIn("--rates", stderr)
            self.assertEqual(stdout, "", bad)
            self.assertFalse(self.out.exists(), bad)

    def test_the_multicar_script_refuses_a_bad_rate_too(self):
        code, _, stderr = run_script("spike_multicar", "--fake", "--out", str(self.out), "--rates", "lidar=100")
        self.assertEqual(code, 2)
        self.assertIn("--rates", stderr)

    def test_help_lists_the_option_and_the_default_for_both_measuring_scripts(self):
        for script in ("spike_sensors", "spike_multicar"):
            _, stdout, _ = run_script(script, "--help")
            self.assertIn("--rates", stdout)
            self.assertIn(rates.DEFAULT_RATES_TEXT, stdout.replace("\n", "").replace(" ", ""))

    def test_the_default_run_polls_at_the_default_rates_and_writes_them_to_the_json(self):
        stdout = self.run_fake("spike_sensors", "--seconds", "2")
        saved = json.loads((self.out / "sensors.json").read_text(encoding="utf-8"))
        rows = {r["sensor"]: r for r in saved["sensors"]}
        self.assertEqual({k: r["configured_rate_hz"] for k, r in rows.items()},
                         {"lidar": 20.0, "camera": 20.0, "radar": 20.0, "gps": 10.0, "imu": 10.0})
        # 2 s = 120 steps: floor(120 * 20 / 60) + 1 = 41 and floor(120 * 10 / 60) + 1 = 21 polls
        self.assertEqual({k: r["polls"] for k, r in rows.items()},
                         {"lidar": 41, "camera": 41, "radar": 41, "gps": 21, "imu": 21})
        self.assertEqual(rows["lidar"]["requested_update_hz"], 10.0)
        self.assertIsNone(rows["imu"]["requested_update_hz"])
        self.assertIn("poll_hz", stdout)

    def test_all_60_polls_every_step_and_a_named_sensor_overrides_it(self):
        self.run_fake("spike_sensors", "--seconds", "2", "--rates", "all=60,gps=30")
        saved = json.loads((self.out / "sensors.json").read_text(encoding="utf-8"))
        rows = {r["sensor"]: r for r in saved["sensors"]}
        self.assertEqual({k: r["polls"] for k, r in rows.items()},
                         {"lidar": 120, "camera": 120, "radar": 120, "gps": 61, "imu": 120})
        self.assertEqual(rows["gps"]["configured_rate_hz"], 30.0)

    def test_the_multicar_script_applies_the_rates_to_every_case(self):
        self.run_fake("spike_multicar", "--rates", "all=30")
        saved = json.loads((self.out / "multicar.json").read_text(encoding="utf-8"))
        for case in saved["cases"]:
            # 1 s = 60 steps: floor(60 * 30 / 60) + 1 = 31 polls
            self.assertEqual({r["polls"] for r in case["sensors"]}, {31})
            self.assertEqual({r["configured_rate_hz"] for r in case["sensors"]}, {30.0})


if __name__ == "__main__":
    unittest.main()
