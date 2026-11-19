import json
import math
import unittest

from tests.fake_array import FakeArray
from kite_bridge import raw_data


class Blob:
    """Has tobytes() like a numpy array or a PIL image, and no length."""

    def __init__(self, data):
        self._data = data

    def tobytes(self):
        return self._data


class SizeTest(unittest.TestCase):
    def nbytes(self, raw):
        return raw_data.describe(raw)[0]

    def test_each_raw_form_counts_the_bytes_the_module_documents(self):
        self.assertEqual(self.nbytes(None), 0)
        self.assertEqual(self.nbytes(b"abcd"), 4)
        self.assertEqual(self.nbytes(bytearray(5)), 5)
        self.assertEqual(self.nbytes(Blob(bytes(12))), 12)
        self.assertEqual(self.nbytes(True), 1)
        self.assertEqual(self.nbytes(7), 8)
        self.assertEqual(self.nbytes(2.5), 8)
        # "héllo" is 6 bytes in UTF-8: h, two for é, l, l, o
        self.assertEqual(self.nbytes("héllo"), 6)

    def test_a_dict_counts_its_key_names_and_its_values(self):
        # keys "a" (1) and "bb" (2), a float (8) and an int (8): 1 + 2 + 8 + 8 = 19
        self.assertEqual(self.nbytes({"a": 1, "bb": 2.5}), 19)

    def test_lists_and_array_like_rows_add_up_their_items(self):
        # [True, "héllo"] = 1 + 6; a 2 x 3 array-like of floats = 6 * 8
        self.assertEqual(self.nbytes([True, "héllo"]), 7)
        self.assertEqual(self.nbytes((1, 2, 3)), 24)
        self.assertEqual(self.nbytes(FakeArray(2, 3)), 48)

    def test_a_lidar_shaped_poll(self):
        # "pointCloud" (10) + 100 points of 3 float32 (1200) + "time" (4) + a float (8) = 1222
        poll = {"pointCloud": Blob(bytes(1200)), "time": 0.5}
        self.assertEqual(self.nbytes(poll), 1222)

    def test_an_empty_raw_value_is_zero_bytes(self):
        for empty in (b"", [], {}, Blob(b""), FakeArray(0, 3)):
            self.assertEqual(self.nbytes(empty), 0)

    def test_a_value_whose_truth_test_raises_is_never_truth_tested(self):
        self.assertEqual(self.nbytes({"rows": FakeArray(3, 7)}), len("rows") + 3 * 7 * 8)


class FingerprintTest(unittest.TestCase):
    def key(self, raw):
        return raw_data.describe(raw)[1]

    def test_the_same_content_gives_the_same_fingerprint_whatever_the_key_order(self):
        self.assertEqual(self.key({"a": 1, "b": [2.0, "x"]}), self.key({"b": [2.0, "x"], "a": 1}))
        self.assertEqual(self.key(Blob(b"abc")), self.key(Blob(b"abc")))

    def test_a_different_value_gives_a_different_fingerprint(self):
        base = self.key({"time": 0.1, "x": 1.0})
        self.assertNotEqual(base, self.key({"time": 0.1, "x": 1.5}))
        self.assertNotEqual(base, self.key({"time": 0.2, "x": 1.0}))
        self.assertNotEqual(base, self.key({"time": 0.1, "y": 1.0}))
        self.assertNotEqual(self.key(Blob(b"abc")), self.key(Blob(b"abd")))

    def test_the_same_bytes_in_a_different_shape_are_different(self):
        self.assertNotEqual(self.key([1, 2]), self.key([[1], [2]]))
        self.assertNotEqual(self.key(["ab", "c"]), self.key(["a", "bc"]))
        self.assertNotEqual(self.key(1), self.key(1.0))
        self.assertNotEqual(self.key(1), self.key(True))


class OutsideTheDomainTest(unittest.TestCase):
    def test_an_unknown_type_raises(self):
        for bad in (object(), {1: 2}, 2 ** 63, -(2 ** 63) - 1, {"a": object()}, [None, object()]):
            with self.assertRaises(ValueError, msg=repr(bad)):
                raw_data.describe(bad)

    def test_the_edges_of_a_64_bit_integer_are_inside(self):
        self.assertEqual(raw_data.describe(2 ** 63 - 1)[0], 8)
        self.assertEqual(raw_data.describe(-(2 ** 63))[0], 8)


class PollStatsTest(unittest.TestCase):
    def test_a_buffered_sensor_gets_one_fingerprint_per_reading(self):
        first, second = {"time": 0.1, "v": 1.0}, {"time": 0.2, "v": 2.0}
        stats = raw_data.poll_stats("imu", [first, second])
        self.assertEqual(stats.nbytes, raw_data.describe(first)[0] + raw_data.describe(second)[0])
        self.assertEqual(stats.keys, (raw_data.describe(first)[1], raw_data.describe(second)[1]))

    def test_a_dict_of_dicts_is_split_into_its_readings(self):
        stats = raw_data.poll_stats("gps", {"gps_0": {"time": 0.1}, "gps_1": {"time": 0.2}})
        self.assertEqual(len(stats.keys), 2)

    def test_one_dict_is_one_reading(self):
        self.assertEqual(len(raw_data.poll_stats("gps", {"time": 0.1, "lat": 1.0}).keys), 1)

    def test_any_other_sensor_gets_one_fingerprint_for_the_whole_poll(self):
        poll = {"pointCloud": Blob(bytes(12)), "colours": Blob(bytes(12))}
        stats = raw_data.poll_stats("lidar", poll)
        self.assertEqual(stats.nbytes, len("pointCloud") + 12 + len("colours") + 12)
        self.assertEqual(stats.keys, (raw_data.describe(poll)[1],))


class ToPlainTest(unittest.TestCase):
    def test_numbers_names_and_structure_survive(self):
        reading = {"time": 0.5, "n": 3, "ok": True, "name": "imu", "none": None, "acc": [1.0, (2, 3)]}
        self.assertEqual(raw_data.to_plain(reading),
                         {"time": 0.5, "n": 3, "ok": True, "name": "imu", "none": None, "acc": [1.0, [2, 3]]})

    def test_a_float_that_is_not_finite_becomes_text_so_the_json_can_be_written(self):
        plain = raw_data.to_plain({"a": math.nan, "b": math.inf, "c": -math.inf})
        self.assertEqual(plain, {"a": "nan", "b": "inf", "c": "-inf"})
        json.dumps(plain, allow_nan=False)

    def test_an_object_with_tolist_is_converted_through_it(self):
        class Array:
            def tolist(self):
                return [1.0, 2.0]

        self.assertEqual(raw_data.to_plain({"v": Array()}), {"v": [1.0, 2.0]})

    def test_anything_else_is_named_and_never_copied(self):
        self.assertEqual(raw_data.to_plain({"image": Blob(b"secret")}), {"image": "<unsupported Blob>"})
        self.assertEqual(raw_data.to_plain(b"bytes"), "<unsupported bytes>")


if __name__ == "__main__":
    unittest.main()
