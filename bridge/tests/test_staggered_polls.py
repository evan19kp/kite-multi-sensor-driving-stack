import contextlib
import io
import json
import tempfile
import unittest
from fractions import Fraction
from pathlib import Path
from unittest import mock

from tests.test_free_running import SENSOR_RATES, BoundedClock
from kite_bridge import common, measure, rates
from kite_bridge.fake_sim import FakeSim
from kite_bridge.sensor_settings import buffered_update_hz

# The cost of one slow poll in these tests: about what Evan's real runs measured.
SLOW_S = 0.023
SLOW = ("radar", "gps", "imu", "timer")
NEW_FREE_DEFAULTS = rates.parse_rates(None, rates.FREE_DEFAULT_RATES)


def free_run(seconds=1.0, stagger=True, request_cost=0.0, poll_rates=NEW_FREE_DEFAULTS, **sim_args):
    clock = BoundedClock()
    sim = FakeSim(clock=clock, request_cost=request_cost, **{"rates": SENSOR_RATES, **sim_args})
    cars = sim.load_scenario("fake_cherrier_tograc", 1)
    result = common.measure_sensors(sim, cars, seconds, clock=clock.now, sleep=clock.sleep, poll_rates=poll_rates,
                                    stream=True, mode=common.FREE, stagger=stagger)
    rows = {r["sensor"]: r for r in result["sensors"]}
    return result, rows


def schedules(result, rows):
    out = {kind: rows[kind]["schedule"] for kind in SLOW if kind != "timer"}
    out["timer"] = result["timer"]["schedule"]
    return out


class NewFreeDefaultsTest(unittest.TestCase):
    def test_gps_and_imu_default_to_5_radar_stays_10_and_lockstep_is_unchanged(self):
        self.assertEqual(rates.FREE_DEFAULT_RATES, {"lidar": 20, "camera": 40, "radar": 10, "gps": 5, "imu": 5})
        self.assertEqual(rates.FREE_DEFAULT_RATES_TEXT, "lidar=20,camera=40,radar=10,gps=5,imu=5")
        self.assertEqual(rates.DEFAULT_RATES, {"lidar": 20, "camera": 20, "radar": 20, "gps": 10, "imu": 10})

    def test_the_timer_follows_gps_and_imu_down_to_5(self):
        result, _ = free_run()
        # The Timer reads at 0, 0.2, ..., 1.0 s; the reading at 1.0 ends the run.
        self.assertEqual((result["timer"]["polls"], result["timer"]["rate_hz"]), (6, 5.0))


class PhasesTest(unittest.TestCase):
    def test_n_slow_polls_spread_evenly_over_the_shortest_period(self):
        rate = {"timer": Fraction(5), "radar": Fraction(10), "gps": Fraction(5), "imu": Fraction(5)}
        # The highest rate is 10, so P = 100 ms, and 4 polls get 0, 25, 50 and 75 ms.
        self.assertEqual(rates.phases(["timer", "radar", "gps", "imu"], rate),
                         {"timer": 0, "radar": Fraction(1, 40), "gps": Fraction(1, 20), "imu": Fraction(3, 40)})

    def test_a_phase_shifts_every_due_time_by_itself(self):
        # Rate 10, phase 25 ms: due at 0.025, 0.125, 0.225, ...
        self.assertEqual(rates.next_due(0, Fraction(10), Fraction(1, 40)), Fraction(1, 40))
        self.assertEqual(rates.next_due(Fraction(1, 40), Fraction(10), Fraction(1, 40)), Fraction(1, 8))
        self.assertEqual(rates.next_due(0.13, Fraction(10), Fraction(1, 40)), Fraction(9, 40))
        # A read at 0.33 served the due time 0.125 and passed over 0.225 and 0.325.
        self.assertEqual(rates.skipped(Fraction(1, 8), 0.33, Fraction(10), Fraction(1, 40)), 2)

    def test_a_phase_outside_one_period_is_refused(self):
        for phase in (Fraction(-1, 1000), Fraction(1, 10), Fraction(1, 5)):
            with self.assertRaises(ValueError, msg=str(phase)):
                rates.next_due(0, Fraction(10), phase)
            with self.assertRaises(ValueError, msg=str(phase)):
                rates.skipped(0, 1, Fraction(10), phase)

    def test_a_staggered_run_prints_and_records_its_phases(self):
        result, _ = free_run()
        self.assertEqual(result["phases"], {"staggered": True, "period_ms": 100.0, "slow": [
            {"car": "kite1", "sensor": "timer", "phase_ms": 0.0},
            {"car": "kite1", "sensor": "radar", "phase_ms": 25.0},
            {"car": "kite1", "sensor": "gps", "phase_ms": 50.0},
            {"car": "kite1", "sensor": "imu", "phase_ms": 75.0}]})
        self.assertIn("phases: ms after the run starts that each slow poll is first due, spread evenly over 100.00 ms "
                      "(1 / the highest slow rate), then once per its own period: kite1 timer 0.00, kite1 radar "
                      "25.00, kite1 gps 50.00, kite1 imu 75.00", common.format_result(result))

    def test_an_unstaggered_run_says_so(self):
        result, _ = free_run(stagger=False)
        self.assertFalse(result["phases"]["staggered"])
        self.assertEqual({p["phase_ms"] for p in result["phases"]["slow"]}, {0.0})
        self.assertIn("phases: not staggered; every slow poll is first due when the run starts",
                      common.format_result(result))


class StaggerLatenessTest(unittest.TestCase):
    """Four slow polls (Timer 5, radar 10, GPS 5, IMU 5 per wall second) at 23 ms each, the lidar
    and camera streamed at no cost, 0.9 Timer seconds. The Timer's first reading comes at 0.023
    (its cost is charged before it reads), so the run ends at its reading in the pass at 1.0 s,
    1.023 - 0.023 = 1.0 >= 0.9; the pass at 0.8 s reads 0.8, below 0.9."""

    def test_staggered_polls_start_on_time_where_unstaggered_ones_queue_up_to_69_ms(self):
        unstaggered, u_rows = free_run(0.9, stagger=False, request_cost=dict.fromkeys(SLOW, SLOW_S))
        staggered, s_rows = free_run(0.9, stagger=True, request_cost=dict.fromkeys(SLOW, SLOW_S))
        u, s = schedules(unstaggered, u_rows), schedules(staggered, s_rows)

        # Unstaggered, all four are due at 0.2 k s and radar also at 0.1 + 0.2 k. A pass makes one
        # poll, the most overdue; on a tie the Timer, then the sensors least recently polled.
        # 0.000 Timer (before the loop). 0.023 radar, 23 ms late. 0.046 GPS, 46. 0.069 IMU, 69.
        # 0.1 radar alone, on time. Then at 0.2, 0.4, 0.6 and 0.8: Timer on time, GPS at +23 ms,
        # IMU at +46, radar at +69; radar alone at 0.3, 0.5, 0.7 and 0.9. At 1.0 the Timer ends it.
        # Radar: 10 polls, 5 late (0.023 and the four +69), worst 69 ms. GPS: 5 polls, all late,
        # worst 46 (at 0.046). IMU: 5 polls, all late, worst 69 (at 0.069). Timer: 6, none late.
        # Compared at the 2 decimals printed: where float rounding leaves a due time a hair after
        # the clock, the loop sleeps its 1 us minimum (MIN_SLEEP_S), so a start can be 0.001 ms off.
        self.assertEqual({k: (v["late"], round(v["worst_late_ms"], 2)) for k, v in u.items()},
                         {"radar": (5, 69.0), "gps": (5, 46.0), "imu": (5, 69.0), "timer": (0, 0.0)})

        # Staggered, the phases are Timer 0, radar 25, GPS 50 and IMU 75 ms (P = 1 / 10 s), so the
        # due times are 0 T, 25 R, 50 G, 75 I, 125 R, 200 T, 225 R, 250 G, 275 I, 325 R, 400 T, ...
        # 25 ms apart at the closest. A 23 ms poll ends 2 ms before the next is due, so every
        # poll starts on its due time: no poll is late and the latest start is 0 ms.
        for kind, schedule in s.items():
            self.assertEqual(schedule["late"], 0, kind)
            self.assertEqual(round(schedule["worst_late_ms"], 2), 0.0, kind)
            self.assertEqual(schedule["skipped"], 0, kind)

        # Both runs make the same number of polls (radar 10, GPS and IMU 5 each, Timer 6), so
        # staggering moves the polls, not their number.
        polls = {kind: u_rows[kind]["polls"] for kind in ("radar", "gps", "imu")}
        self.assertEqual(polls, {"radar": 10, "gps": 5, "imu": 5})
        self.assertEqual({kind: s_rows[kind]["polls"] for kind in polls}, polls)
        self.assertEqual((unstaggered["timer"]["polls"], staggered["timer"]["polls"]), (6, 6))
        self.assertLess(max(v["worst_late_ms"] for v in s.values()), max(v["worst_late_ms"] for v in u.values()))
        self.assertLess(sum(v["late"] for v in s.values()), sum(v["late"] for v in u.values()))


class BufferLossTest(unittest.TestCase):
    """The fake IMU makes 100 readings per simulated second and its polls return every reading
    made since the one before (SENSOR_RATES). Staggered, the IMU is polled at 0.075 + 0.2 k s,
    k = 0 to 4, at no cost, and the fake runs at real time, so the polls see samples 7, 27, 47,
    67 and 87. The first poll returns sample 7 alone; the four after it return 20 each."""

    def test_an_imu_at_100_hz_polled_5_times_a_second_loses_nothing(self):
        self.assertEqual(buffered_update_hz("imu"), 100.0)
        result, rows = free_run()
        imu = rows["imu"]["buffer"]
        # 80 readings after the first poll, made between samples 7 and 87, 0.8 simulated s: 100.
        self.assertEqual(imu["readings_per_poll_max"], 20)
        self.assertAlmostEqual(imu["new_per_simulated_s_between_polls"], 100.0, places=9)
        self.assertEqual(imu["own_rate_hz"], 100.0)
        self.assertFalse(imu["possible_buffer_loss"])
        # The whole-run figure, 81 new over 1.0 Timer s, leaves out the 0.2 s after the last
        # poll, which is why the buffer check counts between the first and last poll.
        self.assertAlmostEqual(rows["imu"]["new_per_simulated_s"], 81.0)
        text = common.format_result(result)
        self.assertIn("buffer: kite1 imu returned up to 20 readings in one poll and 100.00 new readings per "
                      "simulated second between its first and last poll; own rate 100 Hz from its "
                      "physics_update_time", text)
        self.assertNotIn("possible buffer loss", text)

    def test_a_buffer_that_keeps_15_readings_is_flagged(self):
        result, rows = free_run(buffer_cap=15)
        imu = rows["imu"]["buffer"]
        # The four later polls each find 20 new readings and get the latest 15: 60 over 0.8 s, 75
        # per simulated second, 25 % below 100.
        self.assertEqual(imu["readings_per_poll_max"], 15)
        self.assertAlmostEqual(imu["new_per_simulated_s_between_polls"], 75.0, places=9)
        self.assertTrue(imu["possible_buffer_loss"])
        self.assertIn("possible buffer loss: kite1 imu gave 75.00 new readings per simulated second, more than 2% "
                      "below its own 100 Hz", common.format_result(result))

    def test_the_gps_gets_its_number_only_because_its_own_rate_is_not_known(self):
        # The same 15-reading cap on a 100 Hz fake GPS: 75 per simulated second, and no note.
        result, rows = free_run(buffer_cap=15)
        gps = rows["gps"]["buffer"]
        self.assertIsNone(buffered_update_hz("gps"))
        self.assertAlmostEqual(gps["new_per_simulated_s_between_polls"], 75.0, places=9)
        self.assertEqual((gps["own_rate_hz"], gps["possible_buffer_loss"]), (None, False))
        text = common.format_result(result)
        self.assertIn("own rate not known: the adapter passes it no physics_update_time", text)
        self.assertNotIn("possible buffer loss: kite1 gps", text)

    def test_the_rate_needs_two_polls_and_counts_a_returned_reading_once(self):
        self.assertIsNone(measure.buffered_rate([]))
        self.assertIsNone(measure.buffered_rate([(1.0, ("a",))]))
        self.assertIsNone(measure.buffered_rate([(1.0, ("a",)), (1.0, ("b",))]))
        # "a" comes back in the second poll and is not new; "b" and "c" are: 2 over 0.5 s.
        self.assertEqual(measure.buffered_rate([(1.0, ("a",)), (1.5, ("a", "b", "c"))]), 4.0)

    def test_the_loss_rule_is_more_than_2_percent_below(self):
        self.assertFalse(measure.possible_buffer_loss(98.0, 100.0))
        self.assertTrue(measure.possible_buffer_loss(97.9, 100.0))
        self.assertFalse(measure.possible_buffer_loss(None, 100.0))
        self.assertFalse(measure.possible_buffer_loss(10.0, None))
        with self.assertRaises(ValueError):
            measure.possible_buffer_loss(float("nan"), 100.0)


class ScriptTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name) / "out"

    def run_main(self, *argv):
        import spike_sensors

        clock = BoundedClock()
        stdout = io.StringIO()
        with (mock.patch.object(common, "FakeSim", lambda: FakeSim(clock=clock)),
              mock.patch.object(common, "wall_clock", clock.now), mock.patch.object(common, "wall_sleep", clock.sleep),
              contextlib.redirect_stdout(stdout)):
            code = spike_sensors.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", str(self.out),
                                       "--mode", "free", "--stream", *argv])
        self.assertEqual(code, 0)
        return stdout.getvalue(), json.loads((self.out / "sensors.json").read_text(encoding="utf-8"))

    def test_the_script_staggers_and_writes_the_phases_and_buffer_to_json(self):
        stdout, saved = self.run_main()
        self.assertEqual([(p["sensor"], p["phase_ms"]) for p in saved["phases"]["slow"]],
                         [("timer", 0.0), ("radar", 25.0), ("gps", 50.0), ("imu", 75.0)])
        self.assertIn("phases: ms after the run starts", stdout)
        imu = next(r for r in saved["sensors"] if r["sensor"] == "imu")
        self.assertEqual(set(imu["buffer"]), {"readings_per_poll_max", "new_per_simulated_s_between_polls",
                                              "own_rate_hz", "possible_buffer_loss"})
        self.assertNotIn("buffer", next(r for r in saved["sensors"] if r["sensor"] == "radar"))

    def test_rates_still_override_and_the_phases_follow(self):
        _, saved = self.run_main("--rates", "radar=5")
        self.assertEqual({r["sensor"]: r["configured_rate_hz"] for r in saved["sensors"]},
                         {"lidar": 20.0, "camera": 40.0, "radar": 5.0, "gps": 5.0, "imu": 5.0})
        # Every slow poll at 5: P = 200 ms, split 4 ways.
        self.assertEqual(saved["phases"]["period_ms"], 200.0)
        self.assertEqual([p["phase_ms"] for p in saved["phases"]["slow"]], [0.0, 50.0, 100.0, 150.0])


if __name__ == "__main__":
    unittest.main()
