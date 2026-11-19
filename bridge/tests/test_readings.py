import unittest

from kite_bridge.readings import reading_from_raw
from kite_bridge.sim_api import POLL_TIME, SENSOR_TIME

POLL_AT = 7.5


class ReadingShapeTest(unittest.TestCase):
    def test_one_reading_with_a_time(self):
        reading = reading_from_raw("gps", {"time": 2, "lat": 0.1}, POLL_AT)
        self.assertEqual(reading.timestamp, 2.0)
        self.assertIsInstance(reading.timestamp, float)
        self.assertEqual(reading.time_source, SENSOR_TIME)
        self.assertEqual(reading.count, 1)
        self.assertIsNone(reading.points)
        self.assertIsNone(reading.unexpected)

    def test_a_list_gives_the_last_time_and_how_many_readings_came(self):
        raw = [{"time": 1.0}, {"time": 1.01}, {"time": 1.02}]
        reading = reading_from_raw("imu", raw, POLL_AT)
        self.assertEqual((reading.timestamp, reading.time_source, reading.count), (1.02, SENSOR_TIME, 3))

    def test_a_dict_of_readings_gives_the_last_time_and_how_many_readings_came(self):
        raw = {0: {"time": 3.0}, 1: {"time": 3.01}}
        reading = reading_from_raw("radar", raw, POLL_AT)
        self.assertEqual((reading.timestamp, reading.time_source, reading.count), (3.01, SENSOR_TIME, 2))

    def test_empty_results_are_empty_polls(self):
        for kind in ("lidar", "camera", "imu"):
            for raw in (None, [], {}):
                reading = reading_from_raw(kind, raw, POLL_AT)
                self.assertIsNone(reading.timestamp, (kind, raw))
                self.assertIsNone(reading.unexpected, (kind, raw))


class PollTimeTest(unittest.TestCase):
    def test_a_reading_without_time_keeps_the_poll_time_and_says_so(self):
        reading = reading_from_raw("camera", {"colour": b"..."}, POLL_AT)
        self.assertEqual((reading.timestamp, reading.time_source, reading.count), (POLL_AT, POLL_TIME, 1))

    def test_a_list_whose_latest_reading_has_no_time_uses_the_poll_time(self):
        reading = reading_from_raw("imu", [{"time": 1.0}, {"accel": 0.0}], POLL_AT)
        self.assertEqual((reading.timestamp, reading.time_source, reading.count), (POLL_AT, POLL_TIME, 2))


class LidarTest(unittest.TestCase):
    def test_points_are_the_rows_of_point_cloud(self):
        reading = reading_from_raw("lidar", {"time": 4.0, "pointCloud": [[0, 0, 0]] * 3}, POLL_AT)
        self.assertEqual((reading.timestamp, reading.points, reading.count), (4.0, 3, 1))

    def test_zero_points_is_an_empty_poll(self):
        reading = reading_from_raw("lidar", {"time": 4.0, "pointCloud": []}, POLL_AT)
        self.assertIsNone(reading.timestamp)
        self.assertIsNone(reading.unexpected)

    def test_no_point_cloud_is_unexpected_and_names_the_keys(self):
        reading = reading_from_raw("lidar", {"time": 4.0, "colours": []}, POLL_AT)
        self.assertIsNone(reading.timestamp)
        self.assertEqual(reading.unexpected, "lidar reading has no 'pointCloud'; got ['colours', 'time']")

    def test_a_lidar_result_that_is_not_a_dict_is_unexpected(self):
        reading = reading_from_raw("lidar", [{"time": 4.0}], POLL_AT)
        self.assertIsNone(reading.timestamp)
        self.assertEqual(reading.unexpected, "lidar reading has no 'pointCloud'; got list")

    def test_other_sensors_need_no_point_cloud(self):
        reading = reading_from_raw("radar", {"time": 4.0}, POLL_AT)
        self.assertIsNone(reading.unexpected)
        self.assertIsNone(reading.points)


if __name__ == "__main__":
    unittest.main()
