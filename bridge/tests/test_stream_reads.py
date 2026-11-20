import array
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from tests.test_scripts import run_script
from kite_bridge import common, rates
from kite_bridge.fake_sim import FakeClock, FakeImage, FakeSim
from kite_bridge.sim_api import SENSOR_KINDS, STEP_HZ
from kite_bridge.stream_check import reading_from_stream

SENSOR_RATES = {"lidar": 10.0, "camera": 20.0, "radar": 10.0, "gps": 10.0, "imu": 100.0}
REQUEST_COST = {"lidar": 0.020, "camera": 0.022, "radar": 0.021, "gps": 0.020, "imu": 0.021}
STEP_COST = 0.047
ALL_60 = rates.parse_rates("all=60")
TIMING_KEYS_WITHOUT_STREAM = {"simulated_s", "wall_s", "real_time_factor", "step_wait_s", "sensor_reads_s",
                              "processing_s"}


def measure(stream, sim_class=FakeSim, poll_rates=ALL_60, seconds=1.0, **sim_args):
    clock = FakeClock()
    sim = sim_class(clock=clock, request_cost=REQUEST_COST, step_cost=STEP_COST,
                    **{"rates": SENSOR_RATES, **sim_args})
    cars = sim.load_scenario("fake_cherrier_tograc", 1)
    result = common.measure_sensors(sim, cars, seconds, clock=clock.now, poll_rates=poll_rates, stream=stream)
    return result, {r["sensor"]: r for r in result["sensors"]}


class Rows:
    def __init__(self, rows, code="f", columns=3):
        self._rows, self._code = [tuple(row) for row in rows], code
        self.shape = (len(self._rows), columns)

    def __len__(self):
        return self.shape[0]

    def __getitem__(self, rows):
        return Rows(self._rows[rows], self._code, self.shape[1])

    def tobytes(self):
        return array.array(self._code, [v for row in self._rows for v in row]).tobytes()

    def values(self):
        return array.array(self._code, self.tobytes()).tolist()


class Shaped:
    def __init__(self, *shape):
        self.shape = shape


def lidar(rows, **more):
    return reading_from_stream("lidar", {"pointCloud": rows, **more}, 0.5)


def camera(**images):
    return reading_from_stream("camera", images, 0.5)


def stream_returning(fixed_kind, value):
    class Fixed(FakeSim):
        def stream(self, car, kind):
            if kind == fixed_kind:
                return reading_from_stream(kind, value, self.sim_time)
            return super().stream(car, kind)

    return Fixed


class StreamedLidarMemoryTest(unittest.TestCase):
    """The fake's lidar memory: a block of 1100 rows that alternating scans of 1000 and 800 rows
    overwrite from the start. Sample t writes rows (t + 1, r + 1, 2)."""

    def test_trailing_zero_rows_are_cut_and_counted_on_every_read(self):
        result, rows = measure(True, lidar_rows=(1000, 800), stream_rows=1100)
        row = rows["lidar"]
        # The longest scan is 1000 rows, so rows 1000 to 1099 are never written: each of the 60
        # reads keeps 1000 rows and cuts 100 zero rows, 60 * 100 = 6000.
        self.assertEqual(row["suspect"], {"readings": 0, "rows": 6000, "first": None})
        self.assertEqual(row["polls"], STEP_HZ)
        self.assertEqual(row["empty"], 0)
        # Sample 1, 3, ... wrote 800 rows, so 200 rows of the sample before stay behind them and
        # are counted as points: every read has 1000, never 800.
        self.assertEqual(row["points_mean"], 1000)
        # Reads after steps 1 to 60 see samples floor(s / 6) = 0 to 10: 11 new.
        self.assertEqual(row["new_readings"], 11)
        # A kept read: the key "pointCloud" (10 bytes) and 1000 rows of 3 float32 (12000 bytes).
        self.assertEqual(row["bytes_read"], 60 * 12010)
        text = common.format_result(result)
        self.assertRegex(text, r"(?m)^kite1\s+lidar\s+60\s.*\s1000\s+n/a\s+stream\s+0\s+6000$")
        self.assertIn("suspect: kite1 lidar dropped 0 readings and cut 6000 all-zero rows\n", text)

    def test_rows_left_from_an_earlier_longer_scan_are_kept_because_they_cannot_be_told_apart(self):
        sim = FakeSim(rates=SENSOR_RATES, lidar_rows=(1000, 800), stream_rows=1100)
        car = sim.load_scenario("fake_cherrier_tograc", 1)[0]
        sim.attach_sensors(car, stream=True)
        sim.step(6)  # 0.1 s: lidar sample 1, an 800-row scan written over sample 0's 1000 rows
        reading = sim.stream(car, "lidar")
        self.assertEqual((reading.points, reading.suspect_rows, reading.suspect), (1000, 100, None))
        xs = array.array("f", reading.raw["pointCloud"].tobytes())[0::3]
        self.assertEqual(set(xs[:800]), {2.0})  # sample 1
        self.assertEqual(set(xs[800:]), {1.0})  # sample 0, stale

    def test_memory_that_has_not_changed_is_not_new_and_not_suspect(self):
        # At 0.5 updates a second the lidar and camera make one sample in the first second.
        _, rows = measure(True, rates={**SENSOR_RATES, "lidar": 0.5, "camera": 0.5}, lidar_rows=1000)
        for kind in ("lidar", "camera"):
            self.assertEqual(rows[kind]["polls"], STEP_HZ, kind)
            self.assertEqual(rows[kind]["new_readings"], 1, kind)
            self.assertEqual(rows[kind]["suspect"]["readings"], 0, kind)
        self.assertEqual(rows["lidar"]["suspect"]["rows"], 60 * 100)
        self.assertEqual(rows["camera"]["suspect"]["rows"], 0)

    def test_a_block_with_no_rows_is_an_empty_reading(self):
        _, rows = measure(True, lidar_rows=0, stream_rows=0)
        row = rows["lidar"]
        self.assertEqual((row["polls"], row["empty"], row["new_readings"]), (STEP_HZ, STEP_HZ, 0))
        self.assertEqual(row["unexpected"]["count"], 0)
        self.assertEqual(row["suspect"], {"readings": 0, "rows": 0, "first": None})

    def test_a_block_every_scan_fills_prints_no_suspect_line(self):
        result, rows = measure(True, lidar_rows=1100, stream_rows=1100)
        self.assertEqual(rows["lidar"]["suspect"], {"readings": 0, "rows": 0, "first": None})
        self.assertEqual(rows["camera"]["suspect"], {"readings": 0, "rows": 0, "first": None})
        text = common.format_result(result)
        self.assertRegex(text, r"(?m)^stream: ")
        self.assertNotRegex(text, r"(?m)^suspect:")
        self.assertRegex(text, r"(?m)^car\s+sensor\s+polls\s.*\sdetect\s+read\s+suspect\s+sus_rows$")
        self.assertRegex(text, r"(?m)^kite1\s+lidar\s.*\s1100\s+n/a\s+stream\s+0\s+0$")
        self.assertRegex(text, r"(?m)^kite1\s+radar\s.*\s+poll\s+n/a\s+n/a$")

    def test_rows_cut_from_a_scan_that_ends_up_unexpected_are_not_counted(self):
        # The check cuts one zero row and keeps one. raw_data cannot size an object(), so the reading is unexpected.
        raw = {"pointCloud": Rows([(1, 2, 3), (0, 0, 0)]), "pose": object()}
        _, rows = measure(True, stream_returning("lidar", raw))
        row = rows["lidar"]
        self.assertEqual(row["unexpected"], {"count": STEP_HZ,
                                             "first": "raw data not measurable: cannot measure raw data of type object"})
        self.assertEqual(row["suspect"], {"readings": 0, "rows": 0, "first": None})
        self.assertEqual((row["polls"], row["bytes_read"]), (0, 0))

    def test_the_configured_rates_still_schedule_the_streamed_reads(self):
        _, rows = measure(True, poll_rates=rates.parse_rates(rates.DEFAULT_RATES_TEXT))
        # 60 steps: floor(60 * 20 / 60) + 1 = 21 reads for lidar, camera, radar, 11 for GPS and IMU.
        self.assertEqual({k: r["polls"] for k, r in rows.items()},
                         {"lidar": 21, "camera": 21, "radar": 21, "gps": 11, "imu": 11})


class StreamedLidarCheckTest(unittest.TestCase):
    def test_zero_rows_after_the_scan_are_cut_with_the_matching_rows_of_other_arrays(self):
        cloud = Rows([(1, 2, 3), (4, 5, 6), (0, 0, 0), (0, 0, 0)])
        colours = Rows([(9, 9, 9, 1)] * 4, columns=4)
        intensity = Rows([(7, 7, 7)] * 2)
        reading = lidar(cloud, colours=colours, intensity=intensity)
        self.assertEqual((reading.points, reading.suspect_rows, reading.suspect), (2, 2, None))
        self.assertEqual(reading.raw["pointCloud"].values(), [1, 2, 3, 4, 5, 6])
        self.assertEqual(len(reading.raw["colours"]), 2)
        self.assertIs(reading.raw["intensity"], intensity)

    def test_values_without_a_row_count_are_never_cut(self):
        # No value has a row count: no shape, an empty shape, or a shape that is not a tuple. Three
        # of them still have 3 items or a first dimension of 3, like the cloud.
        extras = {"name": "abc", "counts": [5, 6, 7], "scalar": Shaped(), "listed": SimpleNamespace(shape=[3, 1])}
        reading = lidar(Rows([(1, 2, 3), (4, 5, 6), (0, 0, 0)]), **extras)
        self.assertEqual((reading.points, reading.suspect_rows, reading.suspect), (2, 1, None))
        for key, value in extras.items():
            self.assertIs(reading.raw[key], value, key)

    def test_a_cloud_with_no_rows_is_an_empty_reading(self):
        reading = lidar(Rows([]))
        self.assertEqual((reading.timestamp, reading.suspect, reading.suspect_rows, reading.unexpected),
                         (None, None, 0, None))

    def test_a_zero_row_inside_the_scan_is_kept_and_not_counted(self):
        reading = lidar(Rows([(1, 2, 3), (0, 0, 0), (4, 5, 6)]))
        self.assertEqual((reading.points, reading.suspect_rows), (3, 0))

    def test_a_negative_zero_is_not_unwritten_memory(self):
        reading = lidar(Rows([(1, 2, 3), (-0.0, 0, 0)]))
        self.assertEqual((reading.points, reading.suspect_rows), (2, 0))

    def test_memory_that_is_all_zero_is_an_empty_reading_and_cuts_no_rows(self):
        reading = lidar(Rows([(0, 0, 0)] * 5))
        self.assertEqual((reading.timestamp, reading.suspect, reading.suspect_rows), (None, None, 0))

    def test_memory_that_is_all_zero_in_a_run_counts_under_empty_only(self):
        result, rows = measure(True, lidar_rows=0, stream_rows=100)
        row = rows["lidar"]
        self.assertEqual((row["polls"], row["empty"], row["new_readings"]), (STEP_HZ, STEP_HZ, 0))
        self.assertEqual(row["suspect"], {"readings": 0, "rows": 0, "first": None})
        self.assertRegex(common.format_result(result), r"(?m)^kite1\s+lidar\s.*\sstream\s+0\s+0$")
        self.assertNotRegex(common.format_result(result), r"(?m)^suspect: kite1 lidar")

    def test_one_written_row_among_zero_rows_is_still_cut_and_counted(self):
        reading = lidar(Rows([(1, 2, 3)] + [(0, 0, 0)] * 4))
        self.assertEqual((reading.points, reading.suspect_rows), (1, 4))

    def test_a_value_that_is_not_finite_drops_the_whole_reading(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            reading = lidar(Rows([(1, 2, 3), (4, bad, 6), (0, 0, 0)]))
            self.assertEqual(reading.suspect, "pointCloud holds a value that is not finite (nan or inf)", bad)
            self.assertEqual((reading.timestamp, reading.points, reading.raw), (None, None, None))

    def test_float64_rows_are_read_and_large_finite_values_are_kept(self):
        # 1e308 + 1e308 overflows a float sum to inf, but both values are finite.
        reading = lidar(Rows([(1e308, 1e308, 1), (0, 0, 0)], code="d"))
        self.assertEqual((reading.points, reading.suspect_rows, reading.suspect), (1, 1, None))

    def test_a_cloud_of_the_wrong_shape_or_size_drops_the_whole_reading(self):
        class Lying(Rows):
            def tobytes(self):
                return super().tobytes() + b"\x01\x02"

        class Doubled(Rows):
            def tobytes(self):
                return super().tobytes() * 2

        cases = {
            "pointCloud is not an array of x, y, z rows: shape (2, 2)": Rows([(1, 2), (3, 4)], columns=2),
            "pointCloud is not an array of x, y, z rows: shape None": [(1.0, 2.0, 3.0)],
            "pointCloud is not an array of x, y, z rows: shape (6,)": Shaped(6),
            "pointCloud is not an array of x, y, z rows: shape (2, 3)": Shaped(2, 3),
            "pointCloud holds 26 bytes for shape (2, 3), not 4- or 8-byte floats": Lying([(1, 2, 3), (4, 5, 6)]),
            # 6 values: 12 bytes is 2 bytes each (int16), 96 bytes 16 each (float64 written twice).
            "pointCloud holds 12 bytes for shape (2, 3), not 4- or 8-byte floats": Rows([(1, 2, 3), (4, 5, 6)], "h"),
            "pointCloud holds 96 bytes for shape (2, 3), not 4- or 8-byte floats": Doubled([(1, 2, 3), (4, 5, 6)], "d"),
        }
        for reason, cloud in cases.items():
            reading = lidar(cloud)
            self.assertEqual(reading.suspect, reason)
            self.assertIsNone(reading.raw)

    def test_a_reading_without_a_point_cloud_is_unexpected_as_a_poll_would_be(self):
        self.assertIn("no 'pointCloud'", reading_from_stream("lidar", {"colours": Rows([(1, 2, 3)])}, 0.5).unexpected)
        self.assertIsNone(reading_from_stream("lidar", None, 0.5).timestamp)
        self.assertIsNone(reading_from_stream("lidar", None, 0.5).suspect)


class StreamedCameraCheckTest(unittest.TestCase):
    def test_an_image_of_the_configured_size_is_kept(self):
        for image in (FakeImage((640, 360), 1), Shaped(360, 640, 3), Shaped(360, 640)):
            reading = camera(colour=image, depth=None)
            self.assertEqual((reading.suspect, reading.timestamp), (None, 0.5))

    def test_an_image_of_another_size_drops_the_whole_reading(self):
        cases = {
            "camera image 'colour' is 320x180, expected 640x360": FakeImage((320, 180), 1),
            "camera image 'colour' is 360x640, expected 640x360": Shaped(640, 360, 3),
            "camera image 'colour' has no size, expected 640x360": b"\x00" * 10,
        }
        for reason, image in cases.items():
            reading = camera(colour=image)
            self.assertEqual(reading.suspect, reason)
            self.assertIsNone(reading.raw)

    def test_an_image_with_no_usable_size_drops_the_whole_reading(self):
        # A one-dimensional numpy array has a one-item shape and an int size.
        for image in (SimpleNamespace(shape=(640,), size=640), SimpleNamespace(size=(640, 360, 3))):
            reading = camera(colour=image)
            self.assertEqual(reading.suspect, "camera image 'colour' has no size, expected 640x360", image)
            self.assertIsNone(reading.raw)

    def test_one_wrong_image_drops_the_others_too(self):
        reading = camera(colour=FakeImage((640, 360), 1), depth=Shaped(180, 320))
        self.assertEqual(reading.suspect, "camera image 'depth' is 320x180, expected 640x360")

    def test_a_wrong_size_image_in_a_run_is_counted_and_never_passed_on(self):
        class SmallCamera(FakeSim):
            def stream(self, car, kind):
                if kind == "camera":
                    return reading_from_stream(kind, {"colour": FakeImage((320, 180), self._steps)}, self.sim_time)
                return super().stream(car, kind)

        result, rows = measure(True, SmallCamera)
        reason = "camera image 'colour' is 320x180, expected 640x360"
        cam = rows["camera"]
        self.assertEqual(cam["suspect"], {"readings": STEP_HZ, "rows": 0, "first": reason})
        self.assertEqual((cam["polls"], cam["empty"], cam["new_readings"], cam["bytes_read"]), (0, 0, 0, 0))
        self.assertEqual(cam["unexpected"]["count"], 0)
        self.assertEqual(rows["lidar"]["suspect"]["readings"], 0)
        self.assertGreater(rows["lidar"]["new_readings"], 0)
        text = common.format_result(result)
        self.assertIn(f"suspect: kite1 camera dropped {STEP_HZ} readings and cut 0 all-zero rows, "
                      f"first reason: {reason}", text)

    def test_a_camera_with_no_frame_yet_gives_empty_readings(self):
        _, rows = measure(True, stream_returning("camera", None))
        cam = rows["camera"]
        self.assertEqual((cam["polls"], cam["empty"], cam["new_readings"]), (STEP_HZ, STEP_HZ, 0))
        self.assertEqual(cam["unexpected"]["count"], 0)
        self.assertEqual(cam["suspect"]["readings"], 0)

    def test_a_dict_whose_images_are_all_none_is_an_empty_reading(self):
        for images in ({"colour": None}, {"colour": None, "depth": None}, {}):
            reading = camera(**images)
            self.assertEqual((reading.timestamp, reading.raw, reading.suspect, reading.unexpected),
                             (None, None, None, None), images)

    def test_a_dict_with_one_image_among_none_values_is_a_reading(self):
        reading = camera(colour=None, depth=FakeImage((640, 360), 1))
        self.assertEqual((reading.timestamp, reading.suspect), (0.5, None))

    def test_a_camera_whose_images_are_all_none_counts_under_empty_in_a_run(self):
        result, rows = measure(True, stream_returning("camera", {"colour": None, "depth": None}))
        cam = rows["camera"]
        self.assertEqual((cam["polls"], cam["empty"], cam["new_readings"], cam["bytes_read"]),
                         (STEP_HZ, STEP_HZ, 0, 0))
        self.assertEqual((cam["unexpected"]["count"], cam["suspect"]["readings"]), (0, 0))

    def test_a_camera_result_that_is_not_a_dict_is_unexpected_and_never_passed_on(self):
        for value, name in (([FakeImage((320, 180), 1)], "list"), (b"\x00" * 10, "bytes"), (7, "int")):
            _, rows = measure(True, stream_returning("camera", value))
            cam = rows["camera"]
            self.assertEqual(cam["unexpected"], {"count": STEP_HZ,
                                                 "first": f"camera stream() returned {name}, not a dict of images"})
            self.assertEqual((cam["polls"], cam["empty"], cam["bytes_read"]), (0, 0, 0), name)
            self.assertEqual(cam["suspect"]["readings"], 0, name)


class FakeStreamTest(unittest.TestCase):
    def attached(self, stream, **sim_args):
        sim = FakeSim(rates=SENSOR_RATES, **sim_args)
        car = sim.load_scenario("fake_cherrier_tograc", 1)[0]
        sim.attach_sensors(car, stream=stream)
        return sim, car

    def test_a_sensor_that_is_not_attached_cannot_be_streamed(self):
        sim = FakeSim(rates=SENSOR_RATES)
        car = sim.load_scenario("fake_cherrier_tograc", 1)[0]
        with self.assertRaises(KeyError):
            sim.stream(car, "lidar")

    def test_a_sensor_attached_without_stream_cannot_be_streamed(self):
        sim, car = self.attached(False)
        for kind in ("lidar", "camera"):
            with self.assertRaises(ValueError):
                sim.stream(car, kind)
            self.assertIsNotNone(sim.poll(car, kind).timestamp, kind)

    def test_a_kind_that_has_no_streaming_cannot_be_streamed(self):
        sim, car = self.attached(True)
        self.assertIsNone(sim.stream(car, "lidar").suspect)
        for kind in ("radar", "gps", "imu"):
            with self.assertRaises(ValueError):
                sim.stream(car, kind)

    def test_a_scan_longer_than_the_lidar_block_is_an_error(self):
        sim, car = self.attached(True, lidar_rows=1101, stream_rows=1100)
        with self.assertRaises(ValueError):
            sim.stream(car, "lidar")
        sim, car = self.attached(True, lidar_rows=1100, stream_rows=1100)
        self.assertEqual(sim.stream(car, "lidar").points, 1100)


class StreamTimingTest(unittest.TestCase):
    def test_streamed_reads_cost_no_request_and_are_timed_apart_from_the_polls(self):
        result, rows = measure(True)
        t = result["timing"]
        # Only radar, GPS and IMU are polled: 60 * (0.021 + 0.020 + 0.021) = 3.72 s. The fake's
        # streamed reads take no fake time. Steps: 60 * 0.047 = 2.82 s. Wall: 6.54 s.
        self.assertAlmostEqual(t["sensor_reads_s"], 3.72)
        self.assertEqual(t["stream_reads_s"], 0.0)
        self.assertAlmostEqual(t["step_wait_s"], 2.82)
        self.assertAlmostEqual(t["processing_s"], 0.0)
        self.assertAlmostEqual(t["wall_s"], 6.54)
        self.assertAlmostEqual(t["real_time_factor"], 1 / 6.54)
        text = common.format_result(result)
        # 3.72 / 6.54 = 56.9 %, 2.82 / 6.54 = 43.1 %
        self.assertRegex(text, r"(?m)^  in sensor polls\s+3\.720 s\s+56\.9%$")
        self.assertRegex(text, r"(?m)^  in stream reads\s+0\.000 s\s+0\.0%$")
        self.assertNotIn("in sensor reads", text)
        self.assertEqual((rows["lidar"]["read"], rows["camera"]["read"], rows["radar"]["read"]),
                         ("stream", "stream", "poll"))
        self.assertIsNone(rows["radar"]["suspect"])

    def test_time_inside_a_streamed_read_goes_to_stream_reads_only(self):
        class TwoMsStream(FakeSim):
            def stream(self, car, kind):
                self._clock.advance(0.002)
                return super().stream(car, kind)

        result, rows = measure(True, TwoMsStream)
        t = result["timing"]
        # 60 steps * 2 streamed sensors * 0.002 s = 0.24 s; the polls stay 3.72 s. Wall 6.78 s.
        self.assertAlmostEqual(t["stream_reads_s"], 0.24)
        self.assertAlmostEqual(t["sensor_reads_s"], 3.72)
        self.assertAlmostEqual(t["wall_s"], 6.78)
        self.assertAlmostEqual(t["step_wait_s"] + t["sensor_reads_s"] + t["stream_reads_s"] + t["processing_s"],
                               t["wall_s"])
        self.assertAlmostEqual(rows["lidar"]["read_per_poll_s"], 0.002)
        # 0.24 / 6.78 = 3.5 %, 3.72 / 6.78 = 54.9 %
        text = common.format_result(result)
        self.assertRegex(text, r"(?m)^  in stream reads\s+0\.240 s\s+3\.5%$")
        self.assertRegex(text, r"(?m)^  in sensor polls\s+3\.720 s\s+54\.9%$")

    def test_a_streamed_read_that_raises_is_unexpected_and_says_stream(self):
        class Broken(FakeSim):
            def stream(self, car, kind):
                raise RuntimeError("no shared memory")

        _, rows = measure(True, Broken)
        self.assertEqual(rows["lidar"]["unexpected"],
                         {"count": STEP_HZ, "first": "stream raised RuntimeError: no shared memory"})
        self.assertEqual(rows["lidar"]["suspect"]["readings"], 0)


class StreamOffIsUnchangedTest(unittest.TestCase):
    def calls(self, stream, poll_rates=ALL_60):
        events = []

        class Recording(FakeSim):
            def poll(self, car, kind):
                events.append(f"poll {kind}")
                return super().poll(car, kind)

            def stream(self, car, kind):
                events.append(f"stream {kind}")
                return super().stream(car, kind)

        result, _ = measure(stream, Recording, poll_rates=poll_rates, seconds=3 / STEP_HZ)
        return events, result

    def test_without_stream_every_sensor_is_polled_and_nothing_is_streamed(self):
        events, _ = self.calls(False)
        self.assertEqual(events, [f"poll {kind}" for kind in SENSOR_KINDS] * 3)

    def test_with_stream_lidar_and_camera_are_streamed_on_the_same_schedule(self):
        events, _ = self.calls(True, rates.parse_rates("all=60,lidar=20"))
        # 3 steps: lidar at 20 is read after steps 1 and 3, the rest after every step.
        self.assertEqual(events.count("stream lidar"), 2)
        self.assertEqual(events.count("stream camera"), 3)
        self.assertEqual(events.count("poll radar"), 3)
        self.assertFalse([e for e in events if e in ("poll lidar", "poll camera")])

    def test_without_stream_the_result_and_the_printout_have_no_stream_fields(self):
        result, rows = measure(False)
        self.assertIs(result["stream"], False)
        self.assertEqual(set(result["timing"]), TIMING_KEYS_WITHOUT_STREAM)
        for row in rows.values():
            self.assertNotIn("read", row)
            self.assertNotIn("suspect", row)
        # The same costs as the timing-breakdown tests: 60 * 0.104 = 6.24 s of reads.
        self.assertAlmostEqual(result["timing"]["sensor_reads_s"], 6.24)
        text = common.format_result(result)
        self.assertRegex(text, r"(?m)^  in sensor reads\s+6\.240 s\s+68\.9%$")
        for absent in ("in sensor polls", "in stream reads", "sus_rows", "suspect", "stream"):
            self.assertNotIn(absent, text)


class StreamOptionTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name) / "out"

    def run_fake(self, script, *extra):
        code, stdout, stderr = run_script(script, "--fake", "--seconds", "1", "--out", str(self.out), *extra)
        self.assertEqual(code, 0, stderr)
        return stdout

    def test_help_lists_the_option_for_both_measuring_scripts(self):
        for script in ("spike_sensors", "spike_multicar"):
            _, stdout, _ = run_script(script, "--help")
            self.assertIn("--stream", stdout)

    def test_the_json_records_streaming_and_the_suspect_counts(self):
        stdout = self.run_fake("spike_sensors", "--stream", "--dump-raw", "2")
        saved = json.loads((self.out / "sensors.json").read_text(encoding="utf-8"))
        self.assertIn("provenance", saved)
        self.assertIs(saved["stream"], True)
        self.assertIn("stream_reads_s", saved["timing"])
        rows = {r["sensor"]: r for r in saved["sensors"]}
        self.assertEqual(set(rows["lidar"]["suspect"]), {"readings", "rows", "first"})
        self.assertGreater(rows["lidar"]["suspect"]["rows"], 0)
        self.assertIsNone(rows["gps"]["suspect"])
        self.assertIn("in stream reads", stdout)
        dump = json.loads((self.out / "raw_gps_imu.json").read_text(encoding="utf-8"))
        self.assertEqual(len(dump["cases"][0]["entries"]), 4)

    def test_without_the_option_the_json_says_streaming_was_off(self):
        self.run_fake("spike_sensors")
        saved = json.loads((self.out / "sensors.json").read_text(encoding="utf-8"))
        self.assertIs(saved["stream"], False)
        self.assertEqual(set(saved["timing"]), TIMING_KEYS_WITHOUT_STREAM)

    def test_the_multicar_script_streams_in_every_case(self):
        self.run_fake("spike_multicar", "--stream")
        saved = json.loads((self.out / "multicar.json").read_text(encoding="utf-8"))
        for case in saved["cases"]:
            self.assertIs(case["stream"], True)
            self.assertEqual({r["read"] for r in case["sensors"] if r["sensor"] in ("lidar", "camera")}, {"stream"})


if __name__ == "__main__":
    unittest.main()
