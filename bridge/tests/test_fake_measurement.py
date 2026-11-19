import contextlib
import io
import unittest

from kite_bridge import common
from kite_bridge.fake_sim import FakeSim
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
            # polls come after steps 1..60, at t = 1/60 ... 1; the newest 10 Hz sample is
            # floor(6 t) / 10 = 0.0, 0.1, ..., 1.0: 11 distinct samples, so 49 repeats
            self.assertEqual(row["polls"], 60)
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


class MultiCarTest(unittest.TestCase):
    def test_every_car_gets_every_sensor(self):
        rows, result = measured(FakeSim(), n_cars=4)
        self.assertEqual(result["cars"], 4)
        self.assertEqual(len(rows), 4 * len(SENSOR_KINDS))
        self.assertEqual({car for car, _ in rows}, {"kite1", "kite2", "kite3", "kite4"})

    def test_steps_per_second_is_reported(self):
        _, result = measured(FakeSim())
        self.assertGreater(result["steps_per_second"], 0)


class ResolveModelTest(unittest.TestCase):
    def resolve(self, sim, requested=None):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            try:
                return common.resolve_model(sim, requested), out.getvalue()
            except common.SpikeError as error:
                return error, out.getvalue()

    def test_the_single_match_is_chosen_and_printed(self):
        model, printed = self.resolve(FakeSim())
        self.assertEqual(model, "fake_cherrier_tograc")
        self.assertIn("fake_cherrier_tograc", printed)
        self.assertNotIn("fake_van", printed)

    def test_matching_is_case_insensitive(self):
        model, _ = self.resolve(FakeSim(models=["Plain", "ToGrAc_One"]))
        self.assertEqual(model, "ToGrAc_One")

    def test_no_match_prints_all_names_and_fails(self):
        error, printed = self.resolve(FakeSim(models=["alpha", "beta"]))
        self.assertIsInstance(error, common.SpikeError)
        self.assertIn("alpha", printed)
        self.assertIn("beta", printed)
        self.assertIn("--model", str(error))

    def test_several_matches_print_only_the_matches_and_fail(self):
        error, printed = self.resolve(FakeSim(models=["tograc_a", "cherrier_b", "other"]))
        self.assertIsInstance(error, common.SpikeError)
        self.assertIn("tograc_a", printed)
        self.assertIn("cherrier_b", printed)
        self.assertNotIn("other", printed)

    def test_a_listing_failure_asks_for_model(self):
        class Broken(FakeSim):
            def list_models(self):
                raise RuntimeError("no such call")

        error, _ = self.resolve(Broken())
        self.assertIsInstance(error, common.SpikeError)
        self.assertIn("--model", str(error))
        self.assertIn("no such call", str(error))
        model, _ = self.resolve(Broken(), "chosen")
        self.assertEqual(model, "chosen")


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
