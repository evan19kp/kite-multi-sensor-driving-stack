import json
import math
import unittest

from tests.fake_array import FakeArray
from kite_bridge import raw_data
from kite_bridge.readings import reading_from_raw


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

    def test_a_dict_key_counts_like_a_value_of_its_type(self):
        # Each value is None (0 bytes), so the count is the key's alone: a str its UTF-8 length,
        # an int or float 8, a bool 1, None 0, bytes their length, a tuple the sum of its items.
        for key, expected in (("ab", 2), (7, 8), (2.5, 8), (True, 1), (None, 0), (b"gps", 3),
                              ((1, "a"), 8 + 1)):
            self.assertEqual(self.nbytes({key: None}), expected, msg=repr(key))

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

    def test_a_key_of_another_type_is_another_key(self):
        reading = {"time": 0.5}
        keys = [self.key({key: reading}) for key in (1, "1", 1.0, True, b"1", (1,))]
        self.assertEqual(len(set(keys)), 6)

    def test_keys_of_mixed_types_give_the_same_fingerprint_whatever_their_order(self):
        self.assertEqual(self.key({1: "a", "b": 2.0, None: [3], (4, 5): True}),
                         self.key({(4, 5): True, None: [3], "b": 2.0, 1: "a"}))
        self.assertNotEqual(self.key({1: "a", 2: "b"}), self.key({1: "b", 2: "a"}))


class OutsideTheDomainTest(unittest.TestCase):
    def test_an_unknown_type_raises(self):
        for bad in (object(), {object(): 2}, {2 ** 63: 1}, 2 ** 63, -(2 ** 63) - 1, {"a": object()},
                    [None, object()]):
            with self.assertRaises(ValueError, msg=repr(bad)):
                raw_data.describe(bad)

    def test_the_polls_readings_reads_but_this_module_cannot_size_are_refused(self):
        class LengthOnly:
            def __len__(self):
                return 3

        for raw in ({"time": 1.0, "v": complex(1, 2)}, {"time": 1.0, "n": 2 ** 64}, LengthOnly(),
                    {0: {"time": 1.0}, 1: {"time": 2.0, "id": object()}}):
            self.assertIsNone(reading_from_raw("gps", raw, 1.0).unexpected, msg=repr(raw))
            with self.assertRaises(ValueError, msg=repr(raw)):
                raw_data.poll_stats("gps", raw)

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
        # The poll counts as the one dict it is: outer keys "gps_0" and "gps_1" (5 + 5) and, in
        # each reading, the key "time" (4) and a float (8): 10 + 2 * (4 + 8) = 34
        self.assertEqual(stats.nbytes, 34)

    def test_the_size_of_a_buffered_poll_is_the_size_of_the_whole_poll(self):
        first, second = {"time": 0.1, "v": 1.0}, {"time": 0.2, "v": 2.0}
        # each reading: "time" (4) + float (8) + "v" (1) + float (8) = 21; a list adds nothing
        self.assertEqual(raw_data.poll_stats("imu", [first, second]).nbytes, 42)
        for raw in ([first, second], {"imu_0": first, "imu_1": second}, first):
            self.assertEqual(raw_data.poll_stats("imu", raw).nbytes, raw_data.describe(raw)[0])

    def test_every_poll_shape_readings_reads_is_sized_and_split_into_its_readings(self):
        one = {"time": 1.0}  # "time" 4 + a float 8 = 12
        two = {"time": 2.0}
        # A dict key counts like a value of its type.
        shapes = (
            (one, 12, 1),
            ([one, two], 24, 2),
            ((one, two), 24, 1),  # a tuple is one reading, as readings.readings_in has it
            ({0: one, 1: two}, 8 + 8 + 24, 2),
            ({"gps_0": one}, 5 + 12, 1),
            ({2.5: one}, 8 + 12, 1),
            ({True: one}, 1 + 12, 1),
            ({None: one}, 0 + 12, 1),
            ({b"gps": one}, 3 + 12, 1),
            ({(0, 1): one}, 16 + 12, 1),
            (b"abc", 3, 1),
            (bytearray(b"abcd"), 4, 1),
            ("text", 4, 1),
            (FakeArray(2, 3), 2 * 3 * 8, 1),
            # "time" 4 + 8, "acc" 3 + a float 8 + "x" 1 + an int 8, "m" 1 + two ints 16 = 49
            ({"time": 1.0, "acc": [1.0, {"x": 2}], "m": (1, 2)}, 49, 1),
        )
        for raw, nbytes, readings in shapes:
            self.assertIsNone(reading_from_raw("gps", raw, 1.0).unexpected, msg=repr(raw))
            stats = raw_data.poll_stats("gps", raw)
            self.assertEqual((stats.nbytes, len(stats.keys)), (nbytes, readings), msg=repr(raw))

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

    def test_a_key_that_is_not_a_str_is_written_as_its_text_so_the_json_can_be_written(self):
        plain = raw_data.to_plain({7: 1.0, 2.5: "a", True: None, None: 2, b"k": 3, (0, 1): {4: 5}})
        self.assertEqual(plain, {"7": 1.0, "2.5": "a", "True": None, "None": 2, "b'k'": 3,
                                 "(0, 1)": {"4": 5}})
        self.assertEqual(json.loads(json.dumps(plain, allow_nan=False)), plain)

    def test_anything_else_is_named_and_never_copied(self):
        self.assertEqual(raw_data.to_plain({"image": Blob(b"secret")}), {"image": "<unsupported Blob>"})
        self.assertEqual(raw_data.to_plain(b"bytes"), "<unsupported bytes>")


if __name__ == "__main__":
    unittest.main()
