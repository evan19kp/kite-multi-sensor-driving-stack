import contextlib
import io
import unittest

from kite_bridge.sim_api import release


class Sensor:
    def __init__(self, error=None):
        self.error = error
        self.removed = False

    def remove(self):
        self.removed = True
        if self.error:
            raise self.error


class Closer:
    def __init__(self, error=None):
        self.error = error
        self.closed = False

    def __call__(self):
        self.closed = True
        if self.error:
            raise self.error


def run_release(sensors, closer):
    stderr = io.StringIO()
    with contextlib.redirect_stderr(stderr):
        try:
            release(sensors, closer)
        except Exception as error:
            return error, stderr.getvalue()
    return None, stderr.getvalue()


class ReleaseTest(unittest.TestCase):
    def test_every_removal_failure_is_printed_and_the_first_is_raised_after_close(self):
        first, second = KeyError("gone"), OSError("busy")
        sensors = {("kite1", "lidar"): Sensor(first), ("kite1", "camera"): Sensor(),
                   ("kite2", "radar"): Sensor(second)}
        closer = Closer()
        raised, stderr = run_release(sensors, closer)
        self.assertIs(raised, first)
        self.assertTrue(all(s.removed for s in sensors.values()))
        self.assertTrue(closer.closed)
        self.assertEqual(stderr.splitlines(), [
            "warning: could not remove kite1 lidar: KeyError: 'gone'",
            "warning: could not remove kite2 radar: OSError: busy"])

    def test_a_close_failure_is_raised_even_after_removal_failures(self):
        close_error = RuntimeError("close failed")
        raised, stderr = run_release({("kite1", "gps"): Sensor(OSError("busy"))}, Closer(close_error))
        self.assertIs(raised, close_error)
        self.assertIn("warning: could not remove kite1 gps: OSError: busy", stderr)

    def test_clean_release_removes_everything_closes_and_prints_nothing(self):
        sensors = {("kite1", "imu"): Sensor()}
        closer = Closer()
        raised, stderr = run_release(sensors, closer)
        self.assertIsNone(raised)
        self.assertTrue(sensors["kite1", "imu"].removed)
        self.assertTrue(closer.closed)
        self.assertEqual(stderr, "")


if __name__ == "__main__":
    unittest.main()
