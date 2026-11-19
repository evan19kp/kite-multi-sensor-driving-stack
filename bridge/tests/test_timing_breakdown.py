import json
import tempfile
import unittest
from pathlib import Path

from tests.test_scripts import run_script
from kite_bridge import common
from kite_bridge.fake_sim import FakeClock, FakeSim
from kite_bridge.readings import reading_from_raw
from kite_bridge.sim_api import SENSOR_KINDS, STEP_HZ, Reading

# One simulated second is 60 steps, and each step is followed by one poll of each sensor.
REQUEST_COST = {"lidar": 0.020, "camera": 0.022, "radar": 0.021, "gps": 0.020, "imu": 0.021}
STEP_COST = 0.047
RATES = {"lidar": 10.0, "camera": 20.0, "radar": 30.0, "gps": 10.0, "imu": 100.0}
LIDAR_ROWS = 100


def measure_with_fake_clock(sim_class=FakeSim, seconds=1.0, n_cars=1, dump=None, **sim_args):
    clock = FakeClock()
    sim = sim_class(clock=clock, **sim_args)
    cars = sim.load_scenario("fake_cherrier_tograc", n_cars)
    result = common.measure_sensors(sim, cars, seconds, clock=clock.now, dump=dump)
    return result, {(r["car"], r["sensor"]): r for r in result["sensors"]}, clock


def costed(**more):
    return measure_with_fake_clock(rates=RATES, request_cost=REQUEST_COST, step_cost=STEP_COST,
                                   lidar_rows=LIDAR_ROWS, **more)


class TimingBreakdownTest(unittest.TestCase):
    def test_wall_time_splits_into_steps_reads_and_processing(self):
        result, rows, _ = costed()
        timing = result["timing"]
        # 1 s = 60 steps. Waiting for steps: 60 * 0.047 = 2.82 s.
        # Sensor reads: 60 * (0.020 + 0.022 + 0.021 + 0.020 + 0.021) = 60 * 0.104 = 6.24 s.
        # The fake spends no time anywhere else, so processing is 0 and the wall time is
        # 2.82 + 6.24 = 9.06 s. Real-time factor: 1 simulated s / 9.06 wall s = 0.110375.
        self.assertAlmostEqual(timing["simulated_s"], 1.0)
        self.assertAlmostEqual(timing["step_wait_s"], 2.82)
        self.assertAlmostEqual(timing["sensor_reads_s"], 6.24)
        self.assertAlmostEqual(timing["processing_s"], 0.0)
        self.assertAlmostEqual(timing["wall_s"], 9.06)
        self.assertAlmostEqual(timing["real_time_factor"], 1 / 9.06)

    def test_each_sensor_reports_its_own_read_total_and_time_per_poll(self):
        _, rows, _ = costed()
        for kind, cost in REQUEST_COST.items():
            row = rows["kite1", kind]
            # 60 polls at the sensor's cost
            self.assertAlmostEqual(row["read_total_s"], 60 * cost, msg=kind)
            self.assertAlmostEqual(row["read_per_poll_s"], cost, msg=kind)

    def test_two_cars_double_the_polls_and_so_the_read_time_but_not_the_step_wait(self):
        result, rows, _ = costed(n_cars=2)
        # 2 cars: 2 * 6.24 = 12.48 s of reads; the steps still cost 2.82 s
        self.assertAlmostEqual(result["timing"]["sensor_reads_s"], 12.48)
        self.assertAlmostEqual(result["timing"]["step_wait_s"], 2.82)
        self.assertAlmostEqual(result["timing"]["wall_s"], 15.30)
        self.assertAlmostEqual(result["timing"]["real_time_factor"], 1 / 15.30)

    def test_our_own_processing_is_the_time_outside_steps_and_reads(self):
        class SlowToHash:
            """Hashing this payload takes the fake clock 1 ms: our work, not the simulator's."""

            def __init__(self, clock, tick):
                self._clock, self._tick = clock, tick

            def __len__(self):
                return 10

            def tobytes(self):
                self._clock.advance(0.001)
                return bytes([self._tick % 256]) * 120

        class SlowLidar(FakeSim):
            def poll(self, car, kind):
                reading = super().poll(car, kind)
                if kind != "lidar":
                    return reading
                return reading_from_raw(kind, {"pointCloud": SlowToHash(self._clock, self._steps)},
                                        self.sim_time)

        result, _, _ = measure_with_fake_clock(SlowLidar, request_cost=REQUEST_COST, step_cost=STEP_COST)
        timing = result["timing"]
        # 60 lidar polls, each hashed once at 1 ms: 60 * 0.001 = 0.06 s. Steps and reads are as before.
        self.assertAlmostEqual(timing["processing_s"], 0.06)
        self.assertAlmostEqual(timing["step_wait_s"], 2.82)
        self.assertAlmostEqual(timing["sensor_reads_s"], 6.24)
        self.assertAlmostEqual(timing["wall_s"], 9.12)

    def test_the_parts_always_add_up_to_the_wall_time(self):
        result, _, _ = costed()
        t = result["timing"]
        self.assertAlmostEqual(t["step_wait_s"] + t["sensor_reads_s"] + t["processing_s"], t["wall_s"])

    def test_with_no_wall_time_the_real_time_factor_is_undefined_not_infinite(self):
        result, rows, _ = measure_with_fake_clock()
        self.assertEqual(result["timing"]["wall_s"], 0.0)
        self.assertIsNone(result["timing"]["real_time_factor"])
        self.assertEqual(rows["kite1", "lidar"]["read_per_poll_s"], 0.0)
        saved = json.loads(json.dumps(result, allow_nan=False))
        self.assertIsNone(saved["timing"]["real_time_factor"])
        self.assertIn("real-time factor n/a", common.format_result(result))


class FreshReadingTest(unittest.TestCase):
    def test_new_readings_and_bytes_per_simulated_second_match_the_hand_count(self):
        _, rows, _ = costed()
        # A poll after step s (1 to 60) is at s/60 simulated s, and a sensor at R Hz is on sample
        # floor(s * R / 60). Lidar 10 Hz: s/6 gives samples 0 to 10, so 11 new. Camera 20 Hz: s/3, 0 to 20,
        # 21. Radar 30 Hz: s/2, 0 to 30, 31.
        # GPS 10 Hz: samples 0 to 10, 11 distinct readings; the polls between samples return the
        # last one again. IMU 100 Hz: the first poll returns sample 1, each later poll the samples
        # made since, so samples 1 to 100 are returned once each: 100 new.
        self.assertEqual(rows["kite1", "lidar"]["new_readings"], 11)
        self.assertEqual(rows["kite1", "camera"]["new_readings"], 21)
        self.assertEqual(rows["kite1", "radar"]["new_readings"], 31)
        self.assertEqual(rows["kite1", "gps"]["new_readings"], 11)
        self.assertEqual(rows["kite1", "imu"]["new_readings"], 100)
        for kind, new in (("lidar", 11), ("camera", 21), ("radar", 31), ("gps", 11), ("imu", 100)):
            self.assertEqual(rows["kite1", kind]["new_per_simulated_s"], new)

    def test_bytes_read_per_simulated_second_follow_the_payload_sizes(self):
        _, rows, _ = costed()
        # Payload sizes, counted as raw_data documents (a float or int is 8 bytes, a key its length):
        # lidar poll: "time" 4 + 8, "pointCloud" 10 + 100 rows * 3 columns * 4 bytes = 1222
        # camera poll: "time" 4 + 8, "colour" 6 + 8 * 4 * 3 values * 4 bytes = 402
        # radar poll: "time" 4 + 8, "returns" 7 + 20 rows * 7 columns * 4 bytes = 579
        # GPS and IMU reading: "time" 4 + 8, "tick" 4 + 8, "value" 5 + 8 = 37
        # Every poll returns a reading: 60 polls each for lidar, camera, radar and GPS (one reading per
        # poll); the IMU returns 100 readings in total.
        self.assertEqual(rows["kite1", "lidar"]["bytes_read"], 60 * 1222)
        self.assertEqual(rows["kite1", "camera"]["bytes_read"], 60 * 402)
        self.assertEqual(rows["kite1", "radar"]["bytes_read"], 60 * 579)
        self.assertEqual(rows["kite1", "gps"]["bytes_read"], 60 * 37)
        self.assertEqual(rows["kite1", "imu"]["bytes_read"], 100 * 37)
        self.assertEqual(rows["kite1", "lidar"]["bytes_per_simulated_s"], 60 * 1222)

    def test_the_counts_are_per_simulated_second_so_a_longer_run_gives_the_same_rate(self):
        _, rows, _ = measure_with_fake_clock(rates=RATES, lidar_rows=LIDAR_ROWS, seconds=2.0)
        # 120 steps: lidar samples 0 to 20 = 21 new in 2 s; bytes are 120 polls of 1222 in 2 s
        self.assertEqual(rows["kite1", "lidar"]["new_readings"], 21)
        self.assertEqual(rows["kite1", "lidar"]["new_per_simulated_s"], 10.5)
        self.assertEqual(rows["kite1", "lidar"]["bytes_per_simulated_s"], 120 * 1222 / 2)

    def identical(self, kind, raw_for_poll):
        class Same(FakeSim):
            def poll(self, car, kind_):
                if kind_ != kind:
                    return super().poll(car, kind_)
                return reading_from_raw(kind_, raw_for_poll(self._steps), self.sim_time)

        _, rows, _ = measure_with_fake_clock(Same, rates=RATES)
        return rows["kite1", kind]

    def test_a_lidar_that_returns_the_same_scan_every_poll_has_one_new_reading(self):
        row = self.identical("lidar", lambda step: {"pointCloud": b"x" * 12, "colours": b"y"})
        self.assertEqual(row["polls"], STEP_HZ)
        self.assertEqual(row["new_readings"], 1)
        self.assertEqual(row["bytes_read"], STEP_HZ * (10 + 12 + 7 + 1))

    def test_a_scan_that_changes_by_one_byte_is_new(self):
        row = self.identical("lidar", lambda step: {"pointCloud": b"x" * 11 + bytes([step // 30])})
        self.assertEqual(row["new_readings"], 3)  # steps 1-29, 30-59, 60

    def test_a_scan_that_returns_to_an_earlier_content_is_new_again(self):
        row = self.identical("lidar", lambda step: {"pointCloud": bytes([step // 20 % 2])})
        # steps 1-19 A, 20-39 B, 40-59 A, 60 B
        self.assertEqual(row["new_readings"], 4)

    def test_a_gps_that_returns_the_same_reading_every_poll_has_one_new_reading(self):
        row = self.identical("gps", lambda step: [{"time": 0.5, "lat": 1.0, "lon": 2.0}])
        self.assertEqual(row["new_readings"], 1)

    def test_a_gps_reading_with_the_same_time_and_new_values_is_new(self):
        row = self.identical("gps", lambda step: [{"time": 0.5, "lat": step // 30}])
        self.assertEqual(row["new_readings"], 3)

    def test_a_gps_reading_with_new_time_and_the_same_values_is_new(self):
        row = self.identical("gps", lambda step: {"time": step // 30 / 10, "lat": 1.0})
        self.assertEqual(row["new_readings"], 3)

    def test_the_same_gps_readings_returned_again_in_a_later_poll_are_not_new(self):
        pair = [{"time": 0.1, "lat": 1.0}, {"time": 0.2, "lat": 2.0}]
        row = self.identical("gps", lambda step: pair)
        self.assertEqual(row["new_readings"], 2)

    def test_an_empty_poll_adds_no_new_reading_and_no_bytes(self):
        row = self.identical("lidar", lambda step: None)
        self.assertEqual(row["new_readings"], 0)
        self.assertEqual(row["bytes_read"], 0)
        self.assertEqual(row["empty"], STEP_HZ)

    def test_a_poll_whose_raw_data_cannot_be_measured_counts_as_unexpected(self):
        class Odd(FakeSim):
            def poll(self, car, kind):
                if kind == "radar":
                    return Reading(self.sim_time, raw=object())
                return super().poll(car, kind)

        _, rows, _ = measure_with_fake_clock(Odd)
        radar = rows["kite1", "radar"]
        self.assertEqual(radar["unexpected"]["count"], STEP_HZ)
        self.assertEqual(radar["unexpected"]["first"], "raw data not measurable: cannot measure raw data of type object")
        self.assertEqual((radar["polls"], radar["new_readings"], radar["bytes_read"]), (0, 0, 0))
        self.assertGreater(rows["kite1", "lidar"]["new_readings"], 0)


class PrintedBreakdownTest(unittest.TestCase):
    def test_the_summary_prints_the_split_the_factor_and_a_row_per_sensor(self):
        result, _, _ = costed()
        text = common.format_result(result)
        self.assertIn("timing: 1.000 simulated s in 9.060 wall s, real-time factor 0.110", text)
        # 2.82 / 9.06 = 31.1 %, 6.24 / 9.06 = 68.9 %
        self.assertRegex(text, r"(?m)^  waiting for steps\s+2\.820 s\s+31\.1%$")
        self.assertRegex(text, r"(?m)^  in sensor reads\s+6\.240 s\s+68\.9%$")
        self.assertRegex(text, r"(?m)^  our processing\s+0\.000 s\s+0\.0%$")
        # lidar: 60 * 0.020 = 1.200 s of reads, 20.000 ms a poll, 11 new readings, 11.00 a second,
        # 73320 bytes = 73.3 kB for the second
        self.assertRegex(text, r"(?m)^kite1\s+lidar\s+1\.200\s+20\.00\s+11\s+11\.00\s+73320\s+73\.3$")
        self.assertRegex(text, r"(?m)^kite1\s+imu\s+1\.260\s+21\.00\s+100\s+100\.00\s+3700\s+3\.7$")


class RawDumpTest(unittest.TestCase):
    def test_the_dump_holds_n_gps_and_n_imu_readings_per_car_and_nothing_else(self):
        dump = common.RawDump(3)
        result, _, _ = measure_with_fake_clock(rates=RATES, n_cars=2, dump=dump)
        by_car_and_sensor = {}
        for entry in dump.entries:
            by_car_and_sensor.setdefault((entry["car"], entry["sensor"]), []).append(entry)
        self.assertEqual(sorted(by_car_and_sensor), [("kite1", "gps"), ("kite1", "imu"),
                                                     ("kite2", "gps"), ("kite2", "imu")])
        for entries in by_car_and_sensor.values():
            self.assertEqual(len(entries), 3)
        self.assertEqual(len(dump.entries), 12)

    def test_each_entry_is_the_raw_reading_with_its_polls_wall_time_and_step(self):
        dump = common.RawDump(3)
        measure_with_fake_clock(rates=RATES, request_cost=0.02, step_cost=0.05, dump=dump)
        gps = [e for e in dump.entries if e["sensor"] == "gps"]
        imu = [e for e in dump.entries if e["sensor"] == "imu"]
        # GPS 10 Hz: steps 1 to 5 all return sample 0. IMU 100 Hz: step 1 returns sample 1, step 2
        # samples 2 and 3, so the first three IMU readings are samples 1, 2, 3 at steps 1, 2, 2.
        self.assertEqual([(e["step"], e["reading"]) for e in gps],
                         [(s, {"time": 0.0, "tick": 0, "value": 0.0}) for s in (1, 2, 3)])
        self.assertEqual([(e["step"], e["reading"]["tick"], e["reading"]["time"]) for e in imu],
                         [(1, 1, 0.01), (2, 2, 0.02), (2, 3, 0.03)])
        # Step 1: 0.05 s of stepping, then four polls of 0.02 s (lidar, camera, radar, gps), so the
        # GPS poll ends at 0.05 + 4 * 0.02 = 0.13 s and the IMU poll at 0.15 s. Each poll took 0.02 s.
        # Step 2 ends at 0.15 + 0.05 = 0.20 s, so its GPS poll ends at 0.28 s and its IMU poll at 0.30 s.
        self.assertAlmostEqual(gps[0]["wall_s"], 0.13)
        self.assertAlmostEqual(imu[0]["wall_s"], 0.15)
        self.assertAlmostEqual(gps[1]["wall_s"], 0.28)
        self.assertAlmostEqual(imu[1]["wall_s"], 0.30)
        for entry in dump.entries:
            self.assertAlmostEqual(entry["poll_wall_s"], 0.02)
            self.assertEqual(set(entry), {"car", "sensor", "step", "wall_s", "poll_wall_s", "reading"})

    def test_readings_from_one_poll_share_that_polls_wall_time_and_step(self):
        dump = common.RawDump(3)
        measure_with_fake_clock(rates=RATES, request_cost=0.02, step_cost=0.05, dump=dump)
        second, third = [e for e in dump.entries if e["sensor"] == "imu"][1:]
        self.assertEqual((second["step"], second["wall_s"], second["poll_wall_s"]),
                         (third["step"], third["wall_s"], third["poll_wall_s"]))

    def test_zero_collects_nothing(self):
        dump = common.RawDump(0)
        measure_with_fake_clock(dump=dump)
        self.assertEqual(dump.entries, [])

    def test_a_run_with_fewer_readings_than_n_dumps_what_there_is(self):
        dump = common.RawDump(10_000)
        measure_with_fake_clock(rates=RATES, dump=dump)
        self.assertEqual(sum(e["sensor"] == "imu" for e in dump.entries), 100)
        self.assertEqual(sum(e["sensor"] == "gps" for e in dump.entries), STEP_HZ)


class DumpScriptTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name) / "out"

    def run_fake(self, script, *extra):
        code, stdout, stderr = run_script(script, "--fake", "--seconds", "1", "--out", str(self.out), *extra)
        self.assertEqual(code, 0, stderr)
        return stdout

    def test_dump_raw_writes_the_file_with_the_provenance_field_and_only_the_entries(self):
        stdout = self.run_fake("spike_sensors", "--dump-raw", "3")
        saved = json.loads((self.out / "raw_gps_imu.json").read_text(encoding="utf-8"))
        self.assertEqual(set(saved), {"provenance", "readings_per_car_and_sensor", "cases"})
        self.assertEqual(saved["readings_per_car_and_sensor"], 3)
        self.assertIn("provenance", json.loads((self.out / "sensors.json").read_text(encoding="utf-8")))
        (case,) = saved["cases"]
        self.assertEqual(set(case), {"cars", "entries"})
        self.assertEqual(case["cars"], 1)
        self.assertEqual([(e["car"], e["sensor"]) for e in case["entries"]].count(("kite1", "gps")), 3)
        self.assertEqual([(e["car"], e["sensor"]) for e in case["entries"]].count(("kite1", "imu")), 3)
        self.assertEqual(len(case["entries"]), 6)
        self.assertIn("raw_gps_imu.json", stdout)
        self.assertEqual(sorted(p.name for p in self.out.iterdir()), ["raw_gps_imu.json", "sensors.json"])

    def test_without_dump_raw_no_dump_file_is_written(self):
        self.run_fake("spike_sensors")
        self.assertEqual(sorted(p.name for p in self.out.iterdir()), ["sensors.json"])

    def test_the_multicar_dump_has_n_per_car_for_each_case(self):
        self.run_fake("spike_multicar", "--dump-raw", "2")
        saved = json.loads((self.out / "raw_gps_imu.json").read_text(encoding="utf-8"))
        self.assertEqual([case["cars"] for case in saved["cases"]], [1, 2, 4])
        for case in saved["cases"]:
            self.assertEqual(len(case["entries"]), case["cars"] * 2 * 2)

    def test_a_negative_dump_count_is_a_usage_error(self):
        code, _, stderr = run_script("spike_sensors", "--fake", "--dump-raw", "-1", "--out", str(self.out))
        self.assertEqual(code, 2)
        self.assertIn("--dump-raw", stderr)

    def test_help_lists_the_option_for_the_measuring_scripts(self):
        for script in ("spike_sensors", "spike_multicar"):
            _, stdout, _ = run_script(script, "--help")
            self.assertIn("--dump-raw", stdout)


class ScriptOutputTest(unittest.TestCase):
    def test_every_measuring_script_prints_and_writes_the_breakdown(self):
        for script, name in (("spike_sensors", "sensors.json"), ("spike_multicar", "multicar.json")):
            with tempfile.TemporaryDirectory() as tmp:
                code, stdout, stderr = run_script(script, "--fake", "--seconds", "1", "--out", tmp)
                self.assertEqual(code, 0, stderr)
                saved = json.loads((Path(tmp) / name).read_text(encoding="utf-8"))
            results = saved["cases"] if script == "spike_multicar" else [saved]
            for result in results:
                self.assertEqual(set(result["timing"]), {"simulated_s", "wall_s", "real_time_factor",
                                                         "step_wait_s", "sensor_reads_s", "processing_s"})
                self.assertAlmostEqual(result["timing"]["simulated_s"], 1.0)
                for row in result["sensors"]:
                    for field in ("read_total_s", "read_per_poll_s", "new_readings", "new_per_simulated_s",
                                  "bytes_read", "bytes_per_simulated_s"):
                        self.assertIn(field, row)
                self.assertEqual({r["sensor"] for r in result["sensors"]}, set(SENSOR_KINDS))
            self.assertEqual(stdout.count("real-time factor"), len(results))
            self.assertEqual(stdout.count("waiting for steps"), len(results))


if __name__ == "__main__":
    unittest.main()
