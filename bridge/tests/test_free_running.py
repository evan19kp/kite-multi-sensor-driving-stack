import contextlib
import io
import json
import socket
import tempfile
import unittest
from fractions import Fraction
from pathlib import Path
from unittest import mock

from tests.test_scripts import run_script
from kite_bridge import common, rates
from kite_bridge.fake_sim import FakeClock, FakeSim
from kite_bridge.readings import timer_value

# The fake sensors: lidar and radar make 10 samples per simulated second, the camera 20, and GPS
# and IMU buffer 100.
SENSOR_RATES = {"lidar": 10.0, "camera": 20.0, "radar": 10.0, "gps": 100.0, "imu": 100.0}
DEFAULT = rates.parse_rates(rates.DEFAULT_RATES_TEXT)
MAX_SLEEPS = 100_000
MAX_READS = 1_000_000
TIMING_KEYS_FREE = {"simulated_s", "wall_s", "real_time_factor", "idle_s", "timer_reads_s", "sensor_reads_s",
                    "processing_s"}


class BoundedClock(FakeClock):
    def __init__(self):
        super().__init__()
        self.sleeps = 0
        self.reads = 0

    def now(self):
        self.reads += 1
        if self.reads > MAX_READS:
            raise AssertionError(f"read the clock more than {MAX_READS} times")
        return super().now()

    def sleep(self, seconds):
        self.sleeps += 1
        if self.sleeps > MAX_SLEEPS:
            raise AssertionError(f"slept more than {MAX_SLEEPS} times")
        self.advance(seconds)


class Recording(FakeSim):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.events = []

    def step(self, n=1):
        self.events.append("step")
        super().step(n)

    def poll(self, car, kind):
        self.events.append(f"poll {kind}")
        return super().poll(car, kind)

    def stream(self, car, kind):
        self.events.append(f"stream {kind}")
        return super().stream(car, kind)

    def start_free_running(self, car):
        self.events.append("start free running")
        super().start_free_running(car)

    def timer_seconds(self):
        self.events.append("timer")
        return super().timer_seconds()

    def pause(self):
        self.events.append("pause")
        super().pause()

    def remove_sensors(self):
        self.events.append("remove sensors")
        super().remove_sensors()

    def close(self):
        self.events.append("close")
        super().close()


def free_run(poll_rates=DEFAULT, seconds=1.0, sim_class=FakeSim, stream=False, dump=None, steps_before=0, **sim_args):
    clock = BoundedClock()
    sim = sim_class(clock=clock, **{"rates": SENSOR_RATES, **sim_args})
    cars = sim.load_scenario("fake_cherrier_tograc", 1)
    sim.step(steps_before)
    result = common.measure_sensors(sim, cars, seconds, clock=clock.now, sleep=clock.sleep, poll_rates=poll_rates,
                                    stream=stream, dump=dump, mode=common.FREE)
    return result, {r["sensor"]: r for r in result["sensors"]}, clock


def polls(rows):
    return {kind: row["polls"] for kind, row in rows.items()}


class WallClockScheduleTest(unittest.TestCase):
    def test_the_next_due_time_is_the_first_multiple_of_one_over_the_rate_after_now(self):
        r = Fraction(20)
        self.assertEqual(rates.next_due(0, r), Fraction(1, 20))
        self.assertEqual(rates.next_due(Fraction(1, 20), r), Fraction(1, 10))
        self.assertEqual(rates.next_due(0.07, r), Fraction(1, 10))
        self.assertEqual(rates.next_due(0, Fraction(1, 1000)), 1000)

    def test_a_late_poll_skips_the_due_times_it_missed(self):
        # Due at 0.05 but polled at 0.17: the slots at 0.10 and 0.15 are skipped, the next is 0.20.
        self.assertEqual(rates.next_due(0.17, Fraction(20)), Fraction(1, 5))

    def test_a_clock_reading_just_below_a_due_time_is_not_past_it(self):
        # The float 0.3 is a little below 3/10, so it is still in the slot that starts at 0.2.
        self.assertLess(Fraction(0.3), Fraction(3, 10))
        self.assertEqual(rates.next_due(0.3, Fraction(10)), Fraction(3, 10))


class FreeRunCountsTest(unittest.TestCase):
    def test_each_sensor_is_polled_at_its_rate_per_wall_second(self):
        result, rows, _ = free_run()
        # Polls cost nothing and the fake runs at real time, so 1 Timer second is 1 wall second.
        # Lidar, camera and radar at 20 per wall second are due at k / 20 for k = 0 to 19, below
        # 1 s: 20 polls. GPS and IMU at 10 are due at k / 10, k = 0 to 9: 10 polls. The Timer
        # follows them at 10: it reads 0, 0.1, ..., 1.0, 11 polls, and the run ends at the
        # reading 1.0, before that pass polls anything else.
        self.assertEqual(polls(rows), {"lidar": 20, "camera": 20, "radar": 20, "gps": 10, "imu": 10})
        self.assertEqual((result["timer"]["polls"], result["timer"]["rate_hz"]), (11, 10.0))
        self.assertEqual(result["mode"], "free")
        self.assertIsNone(result["steps"])
        self.assertIsNone(result["steps_per_second"])
        self.assertAlmostEqual(result["timing"]["simulated_s"], 1.0, places=5)
        self.assertAlmostEqual(result["timing"]["wall_s"], 1.0, places=5)
        self.assertAlmostEqual(result["timing"]["real_time_factor"], 1.0, places=5)
        # Lidar polls at k / 20 s see 10 Hz samples floor(k / 2) = 0 to 9: 10 new.
        self.assertEqual(rows["lidar"]["new_readings"], 10)

    def test_a_simulator_at_half_real_time_runs_twice_the_wall_time_and_gets_twice_the_polls(self):
        result, rows, _ = free_run(free_speed=0.5)
        # 1 Timer second takes 2 wall seconds. Lidar, camera and radar: k / 20 below 2 s, 40
        # polls. GPS and IMU: 20. The Timer reads every 0.1 wall s up to 2.0: 21 polls.
        self.assertEqual(polls(rows), {"lidar": 40, "camera": 40, "radar": 40, "gps": 20, "imu": 20})
        self.assertEqual(result["timer"]["polls"], 21)
        self.assertAlmostEqual(result["timing"]["wall_s"], 2.0, places=5)
        self.assertAlmostEqual(result["timing"]["real_time_factor"], 0.5, places=5)
        # The rate stays per wall second; new readings are per Timer second: 40 lidar polls at
        # k / 20 wall s see samples floor(k / 4) = 0 to 9, 10 new in 1 Timer second.
        self.assertEqual(rows["lidar"]["configured_rate_hz"], 20.0)
        self.assertEqual(rows["lidar"]["new_per_simulated_s"], 10.0)

    def test_a_pass_that_runs_late_polls_once_and_skips_instead_of_catching_up(self):
        result, rows, _ = free_run(rates.parse_rates("all=20"), request_cost={"lidar": 0.07})
        # A lidar poll takes 0.07 wall s, longer than the 0.05 s between due times. Every sensor is
        # polled (none streams), so each pass makes one poll. At 0 s the Timer reads 0, then the
        # lidar, first of the sensors due at 0, is polled until 0.07. From then on each round at
        # 0.07 k s polls the Timer, camera, radar, GPS and IMU (free) and the lidar (0.07 s), each
        # once: none is polled twice, and the due times they passed are skipped. The Timer reads
        # 0.07 k and first reaches 1 at k = 15 (1.05), which ends the run: the lidar was polled at
        # k = 0 to 14 (15), the others at k = 1 to 14 (14), not the 20 due in the first second.
        self.assertEqual(polls(rows), {"lidar": 15, "camera": 14, "radar": 14, "gps": 14, "imu": 14})
        self.assertEqual(result["timer"]["polls"], 16)
        # The last polls start at 0.98 s, so the due times k / 20 up to 0.95 (k = 0 to 19, 20 of
        # them) were each polled or counted as skipped: 20 - 15 = 5 for the lidar, 20 - 14 = 6.
        self.assertEqual({kind: row["schedule"]["skipped"] for kind, row in rows.items()},
                         {"lidar": 5, "camera": 6, "radar": 6, "gps": 6, "imu": 6})
        t = result["timing"]
        self.assertAlmostEqual(t["sensor_reads_s"], 15 * 0.07)
        self.assertAlmostEqual(t["wall_s"], 1.05)
        self.assertEqual(t["idle_s"], 0.0)

    def test_after_one_slow_poll_the_missed_due_times_are_skipped_not_made_up(self):
        class SlowFirstLidar(FakeSim):
            slow = True

            def poll(self, car, kind):
                if kind == "lidar" and self.slow:
                    self.slow = False
                    self._clock.advance(0.2)
                return super().poll(car, kind)

        result, rows, _ = free_run(rates.parse_rates("all=20"), sim_class=SlowFirstLidar)
        # Every sensor and the Timer are at 20 per wall second, one poll per pass. After the
        # Timer's first reading, the lidar poll at 0 s takes 0.2 s, so the due times 0.05, 0.10
        # and 0.15 pass with no poll. At 0.2 s each sensor and the Timer is polled once and moved
        # to 0.25; then the polls at 0.25, 0.30, ..., 0.95 run on time and the Timer reading 1.0
        # ends the run. Lidar: 1 + 1 + 15 = 17; the others were not reached at 0 s: 1 + 15 = 16.
        # Making the missed times up would add 3 more polls each at 0.2 s.
        self.assertEqual(polls(rows), {"lidar": 17, "camera": 16, "radar": 16, "gps": 16, "imu": 16})
        self.assertEqual(result["timer"]["polls"], 18)

    def test_the_timer_follows_the_higher_of_the_gps_and_imu_rates(self):
        result, rows, _ = free_run(rates.parse_rates("gps=5,imu=25"))
        # Timer at 25: k / 25 for k = 0 to 25, the reading at 1.0 ends the run: 26 polls.
        self.assertEqual((result["timer"]["polls"], result["timer"]["rate_hz"]), (26, 25.0))
        self.assertEqual((rows["gps"]["polls"], rows["imu"]["polls"]), (5, 25))

    def test_the_run_ends_after_seconds_of_timer_time_counted_from_its_first_reading(self):
        # 180 steps first: the Timer already reads 3.0 s when the run starts.
        result, rows, clock = free_run(steps_before=180)
        self.assertAlmostEqual(result["timing"]["simulated_s"], 1.0, places=5)
        self.assertAlmostEqual(clock.now(), 1.0, places=5)
        self.assertEqual(rows["lidar"]["polls"], 20)


class TimerTimeTest(unittest.TestCase):
    def test_readings_without_their_own_time_get_the_latest_timer_reading(self):
        result, rows, _ = free_run(sensor_time=False)
        for kind, row in rows.items():
            self.assertEqual(row["time_source"], "timer", kind)
        # Lidar polls at k / 20 s get the Timer reading of k / 10 s rounded down to 0.1: 0.0 to
        # 0.9, 10 distinct times over 0.9 s, so rate_hz is (10 - 1) / 0.9 = 10, the Timer's rate.
        self.assertAlmostEqual(rows["lidar"]["rate_hz"], 10.0, places=5)
        text = common.format_result(result)
        self.assertRegex(text, r"(?m)^kite1\s+lidar\s+20\s+10\.00\s+timer\s")
        self.assertIn("no sensor time for kite1 lidar, kite1 camera, kite1 radar, kite1 gps, kite1 imu", text)

    def test_readings_with_their_own_time_keep_it(self):
        _, rows, _ = free_run()
        self.assertEqual({row["time_source"] for row in rows.values()}, {"sensor"})

    def test_streamed_readings_carry_no_time_so_they_get_the_timer(self):
        result, rows, _ = free_run(stream=True)
        self.assertEqual((rows["lidar"]["read"], rows["lidar"]["time_source"]), ("stream", "timer"))
        self.assertEqual((rows["camera"]["read"], rows["camera"]["time_source"]), ("stream", "timer"))
        self.assertEqual(rows["radar"]["time_source"], "sensor")
        # A pass reads the due streamed sensors before its one poll, so the pass at 1.0 s reads
        # the lidar and camera, then polls the Timer, whose reading 1.0 ends the run: k / 20 for
        # k = 0 to 20, 21 reads. The polled sensors stop at 0.95 and 0.9, as without --stream.
        self.assertEqual(polls(rows), {"lidar": 21, "camera": 21, "radar": 20, "gps": 10, "imu": 10})
        self.assertIn("stream_reads_s", result["timing"])

    def test_a_timer_that_goes_back_stops_the_run(self):
        class Backwards(FakeSim):
            readings = iter([5.0, 4.0])

            def timer_seconds(self):
                return next(self.readings)

        with self.assertRaisesRegex(common.SpikeError, "the Timer went back from 5 s to 4 s"):
            free_run(sim_class=Backwards)


class FreeTimingTest(unittest.TestCase):
    # Costs are powers of two, so the sums below are exact.
    COST = {"lidar": 2 ** -8, "camera": 2 ** -8, "radar": 2 ** -8, "gps": 2 ** -9, "imu": 2 ** -9, "timer": 2 ** -10}

    def test_the_wall_time_splits_into_waiting_timer_polls_sensor_reads_and_processing(self):
        result, _, _ = free_run(request_cost=self.COST)
        t = result["timing"]
        self.assertEqual(set(t), TIMING_KEYS_FREE)
        # Every pass ends before the next is due (at most 2^-10 + 3 * 2^-8 + 2 * 2^-9 = 0.0166 s
        # of polls against 0.05 s between due times), so the counts are those of a free run with
        # no cost: 20 polls each of lidar, camera, radar, 10 each of GPS and IMU, 11 of the Timer.
        # Sensor reads: 20 * 3 * 2^-8 + 10 * 2 * 2^-9 = 0.234375 + 0.0390625 = 0.2734375 s.
        # Timer polls: 11 * 2^-10 = 0.0107421875 s. The last pass starts at 1.0 s and ends after
        # its Timer poll, so the wall time is 1 + 2^-10 = 1.0009765625 s. The fake spends no
        # time elsewhere: processing is 0 and the rest, 0.716796875 s, is waiting to poll.
        self.assertAlmostEqual(t["sensor_reads_s"], 0.2734375)
        self.assertAlmostEqual(t["timer_reads_s"], 0.0107421875)
        self.assertAlmostEqual(t["wall_s"], 1.0009765625, places=5)
        self.assertAlmostEqual(t["idle_s"], 0.716796875, places=5)
        self.assertAlmostEqual(t["processing_s"], 0.0, places=5)
        self.assertAlmostEqual(t["idle_s"] + t["timer_reads_s"] + t["sensor_reads_s"] + t["processing_s"],
                               t["wall_s"])
        # The Timer advanced 1.0 s between its first reading (at 2^-10 s) and its last.
        self.assertAlmostEqual(t["simulated_s"], 1.0, places=5)
        self.assertAlmostEqual(t["real_time_factor"], 1.0 / 1.0009765625, places=5)

    def test_the_summary_says_which_clock_each_number_uses(self):
        result, _, _ = free_run(request_cost=self.COST)
        text = common.format_result(result)
        self.assertRegex(text, r"(?m)^cars: 1  mode: free, the Timer polled 11 times at 10 per wall second$")
        self.assertRegex(text, r"(?m)^timing: 1\.000 simulated s \(Timer\) in 1\.001 wall s, real-time factor 0\.999$")
        # 0.716796875 / 1.0009765625 = 71.6 %, 0.0107421875 / 1.0009765625 = 1.1 %, 0.2734375 / 1.0009765625 = 27.3 %
        self.assertRegex(text, r"(?m)^  waiting to poll\s+0\.717 s\s+71\.6%$")
        self.assertRegex(text, r"(?m)^  in Timer polls\s+0\.011 s\s+1\.1%$")
        self.assertRegex(text, r"(?m)^  in sensor reads\s+0\.273 s\s+27\.3%$")
        self.assertNotIn("waiting for steps", text)
        self.assertIn("mode free: the simulator ran on its own and was polled on a wall-clock schedule. poll_hz is "
                      "polls per wall second.", text)
        self.assertIn("not a target", text)


class FreeLossTest(unittest.TestCase):
    def test_a_lidar_polled_slower_than_it_updates_is_flagged_per_wall_second(self):
        result, rows, _ = free_run(rates.parse_rates("lidar=5"))
        # Polls at 0, 0.2, 0.4, 0.6, 0.8 s see 10 Hz samples 0, 2, 4, 6, 8: 5 new, below 10 - 1.
        self.assertEqual((rows["lidar"]["polls"], rows["lidar"]["new_readings"]), (5, 5))
        self.assertTrue(rows["lidar"]["possible_loss"])
        self.assertIn("possible loss: kite1 lidar gave 5.00 new readings per simulated second at 5 polls per wall "
                      "second, fewer than the 10 Hz requested.", common.format_result(result))

    def test_gps_and_imu_loss_is_never_checkable_in_free_mode_even_at_60(self):
        result, rows, _ = free_run(rates.parse_rates("all=60"))
        self.assertIn("loss not checkable for gps, imu", common.format_result(result))
        self.assertFalse(rows["lidar"]["possible_loss"])


class FreeDumpTest(unittest.TestCase):
    def test_dump_entries_have_no_step_and_the_wall_time_since_the_run_started(self):
        dump = common.RawDump(3)
        free_run(dump=dump)
        gps = [(e["step"], e["wall_s"], e["reading"]["time"]) for e in dump.entries if e["sensor"] == "gps"]
        # GPS polls at 0 and 0.1 wall s. The first returns sample 0, the second samples 1 to 10
        # of the 100 Hz fake; the dump keeps the first 3 readings.
        self.assertEqual([step for step, _, _ in gps], [None, None, None])
        for (_, wall, time), (want_wall, want_time) in zip(gps, [(0.0, 0.0), (0.1, 0.01), (0.1, 0.02)]):
            self.assertAlmostEqual(wall, want_wall, places=5)
            self.assertAlmostEqual(time, want_time)


class WallClockLimitTest(unittest.TestCase):
    def test_a_paused_simulator_hits_the_limit_instead_of_hanging(self):
        class StaysPaused(FakeSim):
            def start_free_running(self, car):
                super().start_free_running(car)
                self.pause()

        with self.assertRaises(common.SpikeError) as caught:
            free_run(sim_class=StaysPaused)
        self.assertEqual(str(caught.exception), "free run: the Timer advanced 0.000 of 1 simulated s in the "
                                                "wall-clock limit of 20 s (--seconds x 20)")

    def test_a_slow_simulator_that_needs_more_than_the_limit_fails_at_the_limit(self):
        # At 1/32 of real time, 1 Timer second needs 32 wall s, more than 1 x 20. The pass at 20 s
        # polls the Timer (due every 0.1 s), which reads 20 / 32 = 0.625, then hits the limit.
        with self.assertRaisesRegex(common.SpikeError, r"^free run: the Timer advanced 0\.625 of 1 simulated s"):
            free_run(free_speed=1 / 32)

    def test_a_slow_simulator_that_finishes_inside_the_limit_is_not_stopped(self):
        # At 1/16 of real time, 1 Timer second takes 16 wall s, inside the 20 s limit.
        result, _, _ = free_run(free_speed=1 / 16)
        self.assertAlmostEqual(result["timing"]["wall_s"], 16.0, places=4)
        self.assertAlmostEqual(result["timing"]["real_time_factor"], 1 / 16, places=5)


class LockstepIsUnchangedTest(unittest.TestCase):
    def test_lockstep_never_runs_free_or_polls_the_timer(self):
        sim = Recording(clock=BoundedClock(), rates=SENSOR_RATES)
        cars = sim.load_scenario("fake_cherrier_tograc", 1)
        result = common.measure_sensors(sim, cars, 3 / 60, clock=FakeClock().now, poll_rates=DEFAULT)
        self.assertEqual(result["mode"], "lockstep")
        self.assertNotIn("timer", result)
        self.assertEqual([e for e in sim.events if e in ("start free running", "timer", "pause")], [])
        self.assertEqual(sim.events.count("step"), 3)


class FakeTimerTest(unittest.TestCase):
    def test_the_fake_timer_advances_with_its_wall_clock_only_while_running(self):
        clock = BoundedClock()
        sim = FakeSim(clock=clock, free_speed=0.5)
        car = sim.load_scenario("fake_cherrier_tograc", 1)[0]
        with self.assertRaises(KeyError):
            sim.timer_seconds()
        sim.step(60)
        sim.start_free_running(car)
        self.assertTrue(sim.free_running)
        clock.advance(2.0)
        self.assertEqual(sim.timer_seconds(), 2.0)  # 1 s of steps + 2 wall s at half speed
        sim.pause()
        self.assertFalse(sim.free_running)
        clock.advance(5.0)
        self.assertEqual(sim.timer_seconds(), 2.0)


    def test_without_a_clock_the_fake_runs_free_on_real_time_and_its_timer_never_goes_back(self):
        sim = FakeSim()
        car = sim.load_scenario("fake_cherrier_tograc", 1)[0]
        sim.start_free_running(car)
        first = sim.timer_seconds()
        self.assertGreaterEqual(first, 0.0)
        self.assertGreaterEqual(sim.timer_seconds(), first)


class TimerValueTest(unittest.TestCase):
    def test_a_finite_number_of_seconds_from_zero_up_is_accepted(self):
        self.assertEqual(timer_value(0), 0.0)
        self.assertEqual(timer_value(12.5), 12.5)

    def test_anything_else_is_refused(self):
        for raw in (None, "1.0", True, float("nan"), float("inf"), -0.5, [1.0]):
            with self.assertRaises(ValueError, msg=repr(raw)):
                timer_value(raw)


class ScriptTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name) / "out"

    def run_main(self, *argv, sim_class=Recording, script="spike_sensors", clock=None, **sim_args):
        module = __import__(script)
        clock = clock or BoundedClock()
        made = []

        def make():
            made.append(sim_class(clock=clock, **sim_args))
            return made[-1]

        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(common, "FakeSim", make))
            stack.enter_context(mock.patch.object(common, "wall_clock", clock.now))
            stack.enter_context(mock.patch.object(common, "wall_sleep", clock.sleep))
            stack.enter_context(contextlib.redirect_stdout(stdout))
            stack.enter_context(contextlib.redirect_stderr(stderr))
            code = module.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", str(self.out), *argv])
        return code, stdout.getvalue(), stderr.getvalue().splitlines(), made

    def saved(self, name="sensors.json"):
        return json.loads((self.out / name).read_text(encoding="utf-8"))

    def test_mode_lockstep_is_the_same_run_as_no_mode(self):
        code, stdout, stderr, (sim,) = self.run_main()
        before = self.saved()
        code_l, stdout_l, stderr_l, (sim_l,) = self.run_main("--mode", "lockstep")
        self.assertEqual((code, stderr), (0, []))
        self.assertEqual((code_l, stdout_l, stderr_l), (code, stdout, stderr))
        self.assertEqual(self.saved(), before)
        self.assertEqual(sim_l.events, sim.events)
        self.assertEqual(before["mode"], "lockstep")
        shutdown = [e for e in sim.events if e in ("pause", "remove sensors", "close")]
        self.assertEqual(shutdown, ["remove sensors", "close"])

    def test_a_free_run_pauses_before_it_removes_the_sensors(self):
        code, _, stderr, (sim,) = self.run_main("--mode", "free")
        self.assertEqual((code, stderr), (0, []))
        self.assertEqual(sim.events[-3:], ["pause", "remove sensors", "close"])
        self.assertEqual(sim.events.index("start free running"), sim.events.index("timer") - 1)

    def test_the_json_records_free_mode_and_its_timing(self):
        _, stdout, _, _ = self.run_main("--mode", "free", "--stream", "--dump-raw", "2")
        saved = self.saved()
        self.assertIn("provenance", saved)
        self.assertEqual(saved["mode"], "free")
        self.assertIsNone(saved["steps"])
        self.assertEqual(set(saved["timer"]), {"polls", "rate_hz", "schedule"})
        self.assertEqual((saved["timer"]["polls"], saved["timer"]["rate_hz"]), (11, 10.0))
        self.assertEqual(set(saved["timing"]), TIMING_KEYS_FREE | {"stream_reads_s"})
        # The free-mode defaults: lidar 20 and camera 40 per wall second, streamed, so the pass at
        # 1.0 s reads them before the Timer ends the run (k / 20 for k = 0 to 20, k / 40 for k = 0
        # to 40); radar, GPS and IMU at 10 are polled at 0 to 0.9 s.
        self.assertEqual({r["sensor"]: r["polls"] for r in saved["sensors"]},
                         {"lidar": 21, "camera": 41, "radar": 10, "gps": 10, "imu": 10})
        self.assertEqual({r["sensor"]: r["configured_rate_hz"] for r in saved["sensors"]},
                         {"lidar": 20.0, "camera": 40.0, "radar": 10.0, "gps": 10.0, "imu": 10.0})
        self.assertEqual({(r["schedule"]["late"], r["schedule"]["skipped"]) for r in saved["sensors"]}, {(0, 0)})
        dump = self.saved("raw_gps_imu.json")
        self.assertIn("provenance", dump)
        self.assertEqual({e["step"] for e in dump["cases"][0]["entries"]}, {None})
        self.assertIn("mode: free", stdout)

    def test_the_multicar_script_runs_every_case_free(self):
        code, _, _, sims = self.run_main("--mode", "free", script="spike_multicar")
        self.assertEqual(code, 0)
        self.assertEqual([c["mode"] for c in self.saved("multicar.json")["cases"]], ["free"] * 3)
        self.assertEqual([s.events[-3:] for s in sims], [["pause", "remove sensors", "close"]] * 3)

    def test_the_wall_clock_limit_is_one_clean_error_line_and_a_normal_shutdown(self):
        code, _, stderr, (sim,) = self.run_main("--mode", "free", free_speed=1 / 32)
        self.assertEqual(code, 1)
        self.assertEqual(stderr, ["error: free run: the Timer advanced 0.625 of 1 simulated s in the wall-clock "
                                  "limit of 20 s (--seconds x 20)"])
        self.assertEqual(sim.events[-3:], ["pause", "remove sensors", "close"])
        self.assertFalse((self.out / "sensors.json").exists())

    def test_ctrl_c_while_polling_free_closes_without_sending_more_requests_and_exits_130(self):
        class CtrlCInPoll(Recording):
            def poll(self, car, kind):
                if self.free_running and kind == "gps":
                    raise KeyboardInterrupt
                return super().poll(car, kind)

        code, _, stderr, (sim,) = self.run_main("--mode", "free", sim_class=CtrlCInPoll)
        self.assertEqual((code, stderr), (130, ["interrupted: BeamNG.tech closed"]))
        self.assertEqual(sim.events[-1], "close")
        self.assertNotIn("pause", sim.events)
        self.assertNotIn("remove sensors", sim.events)
        self.assertFalse(sim.process_ended)

    def test_ctrl_c_while_waiting_between_polls_closes_cleanly_and_exits_130(self):
        clock = BoundedClock()

        def interrupted(seconds):
            raise KeyboardInterrupt

        clock.sleep = interrupted
        code, _, stderr, (sim,) = self.run_main("--mode", "free", clock=clock)
        self.assertEqual((code, stderr), (130, ["interrupted: BeamNG.tech closed"]))
        self.assertIn("poll imu", sim.events)
        self.assertEqual(sim.events[-1], "close")
        self.assertNotIn("remove sensors", sim.events)

    def test_a_timer_poll_that_times_out_stops_the_run(self):
        class TimedOutTimer(Recording):
            def timer_seconds(self):
                raise socket.timeout("timed out")

        code, _, stderr, (sim,) = self.run_main("--mode", "free", sim_class=TimedOutTimer)
        self.assertEqual((code, stderr), (1, ["timed out after 300 s: BeamNG.tech closed"]))
        self.assertEqual(sim.events[-1], "close")
        self.assertNotIn("pause", sim.events)

    def test_a_pause_that_fails_skips_the_removal_still_closes_and_reports_it(self):
        class FailingPause(Recording):
            def pause(self):
                self.events.append("pause")
                raise RuntimeError("pause failed")

        code, _, stderr, (sim,) = self.run_main("--mode", "free", sim_class=FailingPause)
        self.assertEqual(code, 1)
        self.assertIn("warning: could not pause: RuntimeError: pause failed; sensors not removed", stderr)
        self.assertEqual(stderr[-1], "error: RuntimeError: pause failed")
        self.assertEqual(sim.events[-2:], ["pause", "close"])

    def test_a_failed_run_whose_pause_also_fails_reports_both_and_still_closes(self):
        class FailingPause(Recording):
            def pause(self):
                self.events.append("pause")
                raise RuntimeError("pause failed")

        # At 1/32 of real time the run hits the wall-clock limit (see WallClockLimitTest).
        code, _, stderr, (sim,) = self.run_main("--mode", "free", sim_class=FailingPause, free_speed=1 / 32)
        self.assertEqual(code, 1)
        self.assertEqual(stderr, ["warning: could not pause: RuntimeError: pause failed; sensors not removed",
                                  "error: free run: the Timer advanced 0.625 of 1 simulated s in the wall-clock "
                                  "limit of 20 s (--seconds x 20)"])
        self.assertEqual(sim.events[sim.events.index("pause"):], ["pause", "close"])

    def test_a_pause_that_times_out_is_reported_and_ends_the_run_as_a_timeout(self):
        class TimedOutPause(Recording):
            def pause(self):
                self.events.append("pause")
                raise socket.timeout("timed out")

        code, _, stderr, (sim,) = self.run_main("--mode", "free", sim_class=TimedOutPause)
        self.assertEqual((code, stderr), (1, ["warning: could not pause: TimeoutError: timed out; sensors not removed",
                                              "timed out after 300 s: BeamNG.tech closed"]))
        self.assertEqual(sim.events[sim.events.index("pause"):], ["pause", "close"])

    def test_a_timer_reading_that_is_not_a_number_is_one_clean_error_line(self):
        class TextTimer(Recording):
            def timer_seconds(self):
                self.events.append("timer")
                return timer_value("1.0")

        code, _, stderr, (sim,) = self.run_main("--mode", "free", sim_class=TextTimer)
        self.assertEqual((code, stderr), (1, ["error: free run: unusable Timer reading: Timer time is not a number: "
                                              "got str"]))
        self.assertEqual(sim.events[-3:], ["pause", "remove sensors", "close"])
        self.assertFalse((self.out / "sensors.json").exists())

    def test_an_unknown_mode_is_a_usage_error(self):
        code, stdout, stderr = run_script("spike_sensors", "--fake", "--mode", "fast", "--out", str(self.out))
        self.assertEqual((code, stdout), (2, ""))
        self.assertIn("--mode", stderr)
        self.assertFalse(self.out.exists())

    def test_help_lists_the_mode_for_both_measuring_scripts(self):
        for script in ("spike_sensors", "spike_multicar"):
            _, stdout, _ = run_script(script, "--help")
            flat = " ".join(stdout.split())
            self.assertIn("--mode {lockstep,free}", flat, script)
            self.assertIn("--seconds x 20 wall seconds", flat, script)


if __name__ == "__main__":
    unittest.main()
