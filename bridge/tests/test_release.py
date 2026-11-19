import contextlib
import io
import socket
import unittest

from kite_bridge.sim_api import remove_all


class Sensor:
    def __init__(self, error=None):
        self.error = error
        self.removed = False

    def remove(self):
        self.removed = True
        if self.error:
            raise self.error


def run_remove_all(sensors):
    stderr = io.StringIO()
    with contextlib.redirect_stderr(stderr):
        try:
            remove_all(sensors)
        except Exception as error:
            return error, stderr.getvalue()
    return None, stderr.getvalue()


class RemoveAllTest(unittest.TestCase):
    def test_every_removal_failure_is_printed_and_the_first_is_raised_after_trying_all(self):
        first, second = KeyError("gone"), OSError("busy")
        sensors = {("kite1", "lidar"): Sensor(first), ("kite1", "camera"): Sensor(),
                   ("kite2", "radar"): Sensor(second)}
        raised, stderr = run_remove_all(sensors)
        self.assertIs(raised, first)
        self.assertTrue(all(s.removed for s in sensors.values()))
        self.assertEqual(stderr.splitlines(), [
            "warning: could not remove kite1 lidar: KeyError: 'gone'",
            "warning: could not remove kite2 radar: OSError: busy"])

    def test_a_timeout_is_raised_at_once_and_no_later_sensor_is_tried(self):
        timeout = socket.timeout("timed out")
        sensors = {("kite1", "lidar"): Sensor(OSError("busy")), ("kite1", "camera"): Sensor(timeout),
                   ("kite1", "radar"): Sensor()}
        raised, stderr = run_remove_all(sensors)
        self.assertIs(raised, timeout)
        self.assertFalse(sensors["kite1", "radar"].removed)
        self.assertEqual(stderr.splitlines(), [
            "warning: could not remove kite1 lidar: OSError: busy",
            "warning: could not remove kite1 camera: TimeoutError: timed out"])

    def test_clean_removal_removes_everything_and_prints_nothing(self):
        sensors = {("kite1", "imu"): Sensor()}
        raised, stderr = run_remove_all(sensors)
        self.assertIsNone(raised)
        self.assertTrue(sensors["kite1", "imu"].removed)
        self.assertEqual(stderr, "")


if __name__ == "__main__":
    unittest.main()
