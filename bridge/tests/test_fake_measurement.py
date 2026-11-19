import contextlib
import io
import json
import math
import unittest

from kite_bridge import common
from kite_bridge.fake_sim import FakeSim
from kite_bridge.readings import reading_from_raw
from kite_bridge.sim_api import SENSOR_KINDS, STEP_HZ


def measured(sim, n_cars=1, seconds=1.0):
    cars = sim.load_scenario("fake_cherrier_tograc", n_cars)
    result = common.measure_sensors(sim, cars, seconds)
    return {(r["car"], r["sensor"]): r for r in result["sensors"]}, result


class SensorRateTest(unittest.TestCase):
    def test_polling_faster_than_the_sensor_gives_the_sensor_rate_and_exact_repeats(self):
        rows, result = measured(FakeSim(rates=dict.fromkeys(SENSOR_KINDS, 10.0)))
        self.assertEqual(result["steps"], STEP_HZ)
        for row in rows.values():
            self.assertAlmostEqual(row["rate_hz"], 10.0)
            self.assertEqual(row["polls"], 60)
            # polls after steps 1 to 60 see samples 0 to 10 (11 distinct), so 60 - 11 = 49 repeat
            self.assertEqual(row["repeated"], 49)
            self.assertEqual(row["empty"], 0)

    def test_default_rates_per_sensor(self):
        rows, _ = measured(FakeSim())
        self.assertAlmostEqual(rows["kite1", "lidar"]["rate_hz"], 10.0)
        self.assertAlmostEqual(rows["kite1", "camera"]["rate_hz"], 20.0)
        self.assertAlmostEqual(rows["kite1", "radar"]["rate_hz"], 20.0)
        self.assertAlmostEqual(rows["kite1", "gps"]["rate_hz"], 10.0)
        self.assertGreater(rows["kite1", "imu"]["rate_hz"], 59.0)

    def test_lidar_reports_points_and_other_sensors_do_not(self):
        rows, _ = measured(FakeSim())
        self.assertTrue(900 <= rows["kite1", "lidar"]["points_mean"] < 1100)
        for kind in SENSOR_KINDS[1:]:
            self.assertIsNone(rows["kite1", kind]["points_mean"])

    def test_same_seed_gives_the_same_readings(self):
        first, _ = measured(FakeSim(seed=3, dropout=0.1))
        second, _ = measured(FakeSim(seed=3, dropout=0.1))
        for key in first:
            self.assertEqual(first[key]["empty"], second[key]["empty"])
            self.assertEqual(first[key]["points_mean"], second[key]["points_mean"])


class InjectedFaultTest(unittest.TestCase):
    def test_dropouts_and_repeats_are_counted_exactly(self):
        sim = FakeSim(seed=1, rates=dict.fromkeys(SENSOR_KINDS, 120.0), dropout=0.2, repeat=0.2)
        rows, _ = measured(sim, seconds=5.0)
        for key, row in rows.items():
            self.assertEqual(row["empty"], sim.injected[key]["empty"])
            self.assertEqual(row["repeated"], sim.injected[key]["repeated"])
        self.assertGreater(sum(r["empty"] for r in rows.values()), 0)
        self.assertGreater(sum(r["repeated"] for r in rows.values()), 0)

    def test_every_poll_dropped(self):
        rows, _ = measured(FakeSim(dropout=1.0))
        for row in rows.values():
            self.assertEqual(row["empty"], row["polls"])
            self.assertIsNone(row["rate_hz"])
            self.assertEqual(row["repeated"], 0)

    def test_latency_shows_up_in_the_poll_latency_stats(self):
        rows, _ = measured(FakeSim(latency=0.002), seconds=0.2)
        for row in rows.values():
            self.assertGreaterEqual(row["latency_s"]["mean"], 0.0015)
            self.assertGreaterEqual(row["latency_s"]["max"], row["latency_s"]["p95"])


class TimeSourceTest(unittest.TestCase):
    def test_sensor_timestamps_are_labelled_sensor_and_print_no_warning(self):
        rows, result = measured(FakeSim())
        for row in rows.values():
            self.assertEqual(row["time_source"], "sensor")
        self.assertNotIn("polling rate", common.format_result(result))

    def test_readings_without_sensor_time_are_labelled_poll_in_json_and_table(self):
        sim = FakeSim(sensor_time=False)
        rows, result = measured(sim)
        saved = json.loads(json.dumps(result))
        for row in saved["sensors"]:
            self.assertEqual(row["time_source"], "poll")
            self.assertEqual(row["repeated"], 0)
            self.assertAlmostEqual(row["rate_hz"], STEP_HZ, delta=1.0)
        table = common.format_result(result)
        self.assertRegex(table, r"(?m)^kite1\s+camera\s+\d+\s+\d+\.\d+\s+poll\s")
        self.assertIn("polling rate, not the sensor's", table)
        for kind in SENSOR_KINDS:
            self.assertIn(f"kite1 {kind}", table.splitlines()[-1])

    def test_a_sensor_with_both_kinds_of_timestamp_is_labelled_mixed(self):
        class Switching(FakeSim):
            def poll(self, car, kind):
                if kind == "gps" and self.sim_time > 0.5:
                    return reading_from_raw("gps", {"lat": 0.0}, self.sim_time)
                return super().poll(car, kind)

        rows, _ = measured(Switching())
        self.assertEqual(rows["kite1", "gps"]["time_source"], "mixed")
        self.assertEqual(rows["kite1", "lidar"]["time_source"], "sensor")

    def test_a_sensor_that_never_answers_has_no_time_source(self):
        rows, result = measured(FakeSim(dropout=1.0))
        for row in rows.values():
            self.assertIsNone(row["time_source"])
        self.assertNotIn("polling rate", common.format_result(result))


class UnexpectedReadingTest(unittest.TestCase):
    MESSAGE = "lidar reading has no 'pointCloud'; got ['colours', 'time']"

    def broken_lidar(self):
        class BrokenLidar(FakeSim):
            def poll(self, car, kind):
                if kind == "lidar":
                    return reading_from_raw("lidar", {"time": self.sim_time, "colours": []}, self.sim_time)
                return super().poll(car, kind)

        return BrokenLidar()

    def test_an_unexpected_reading_is_reported_and_is_not_an_empty_poll(self):
        rows, result = measured(self.broken_lidar())
        lidar = rows["kite1", "lidar"]
        self.assertEqual(lidar["unexpected"], {"count": STEP_HZ, "first": self.MESSAGE})
        self.assertEqual(lidar["empty"], 0)
        self.assertEqual(lidar["polls"], 0)
        self.assertIsNone(lidar["rate_hz"])
        self.assertIn(f"unexpected format: kite1 lidar {STEP_HZ} readings, first: {self.MESSAGE}",
                      common.format_result(result))
        json.dumps(result)

    def test_other_sensors_are_unaffected(self):
        rows, _ = measured(self.broken_lidar())
        self.assertEqual(rows["kite1", "camera"]["unexpected"], {"count": 0, "first": None})
        self.assertAlmostEqual(rows["kite1", "camera"]["rate_hz"], 20.0)


class ReadingsPerPollTest(unittest.TestCase):
    def test_a_sensor_faster_than_the_poll_rate_reports_readings_per_poll(self):
        _, result = measured(FakeSim(rates={**dict.fromkeys(SENSOR_KINDS, 10.0), "imu": 100.0}))
        rows = {r["sensor"]: r for r in json.loads(json.dumps(result))["sensors"]}
        # 100 Hz polled at 60 Hz gives 1 or 2 new samples per poll
        self.assertEqual(rows["imu"]["readings_per_poll_max"], math.ceil(100 / STEP_HZ))
        self.assertLessEqual(rows["imu"]["rate_hz"], STEP_HZ)
        self.assertEqual(rows["lidar"]["readings_per_poll_max"], 1)
        table = common.format_result(result)
        self.assertIn("kite1 imu: polls returned up to 2 readings. rate_hz counts at most one per "
                      "poll, so it undercounts this sensor.", table)
        self.assertNotIn("kite1 lidar: polls returned", table)
        self.assertIn(f"limited to about the poll rate of {STEP_HZ} Hz. per_poll shows when a "
                      "sensor delivered more.", table)
        self.assertRegex(table, r"(?m)^kite1\s+imu\s+\d+\s+\d+\.\d+\s+sensor\s+2\s")

    def test_no_sensor_faster_than_the_poll_rate_prints_no_readings_per_poll_line(self):
        _, result = measured(FakeSim(rates=dict.fromkeys(SENSOR_KINDS, 20.0)))
        self.assertEqual({r["readings_per_poll_max"] for r in result["sensors"]}, {1})
        self.assertNotIn("polls returned", common.format_result(result))

    def test_a_sensor_that_never_answers_has_no_readings_per_poll(self):
        rows, _ = measured(FakeSim(dropout=1.0))
        for row in rows.values():
            self.assertIsNone(row["readings_per_poll_max"])


class SecondsRuleTest(unittest.TestCase):
    def test_seconds_round_to_the_nearest_whole_step_and_at_least_one(self):
        self.assertEqual(common.steps_for(1.0), STEP_HZ)
        self.assertEqual(common.steps_for(0.0001), 1)
        self.assertEqual(common.steps_for(1.01), 61)


class MultiCarTest(unittest.TestCase):
    def test_every_car_gets_every_sensor(self):
        rows, result = measured(FakeSim(), n_cars=4)
        self.assertEqual(result["cars"], 4)
        self.assertEqual(len(rows), 4 * len(SENSOR_KINDS))
        self.assertEqual({car for car, _ in rows}, {"kite1", "kite2", "kite3", "kite4"})

    def test_steps_per_second_is_reported(self):
        _, result = measured(FakeSim())
        self.assertGreater(result["steps_per_second"], 0)


class TrafficTest(unittest.TestCase):
    def test_a_failure_is_reported_and_not_fatal(self):
        class NoTraffic(FakeSim):
            def spawn_traffic(self, n):
                raise RuntimeError("no traffic api")

        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            spawned = common.start_traffic(NoTraffic(), 8)
        self.assertFalse(spawned)
        self.assertIn("traffic not spawned", out.getvalue())
        self.assertIn("no traffic api", out.getvalue())

    def test_success_and_zero(self):
        sim = FakeSim()
        self.assertTrue(common.start_traffic(sim, 8))
        self.assertEqual(sim.traffic_requested, 8)
        self.assertFalse(common.start_traffic(FakeSim(), 0))


if __name__ == "__main__":
    unittest.main()
