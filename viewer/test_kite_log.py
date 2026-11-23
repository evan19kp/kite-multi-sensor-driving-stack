import unittest
from pathlib import Path

import kite_log
from kite_log import LogError, canonical_double, parse_log, read_log

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").is_file())
DATA = ROOT / "sim" / "tests" / "data"
HEADER = b"# kite-log v0\n"


def serialise(log):
    def line(kind, record):
        fields = [canonical_double(v) if isinstance(v, float) else str(v) for v in record]
        return f"{kind},{','.join(fields)}\n"
    text = "".join(line("gt", r) for r in log.gt) + "".join(line("meas", r) for r in log.meas)
    return (HEADER.decode() + text + "".join(line("track", r) for r in log.tracks)).encode()


class GoldenLogs(unittest.TestCase):
    def test_seed42_has_the_records_the_generator_wrote(self):
        log = read_log(DATA / "seed42.log")
        self.assertEqual((len(log.gt), len(log.meas), len(log.tracks)), (63, 57, 0))
        self.assertEqual(log.gt[0], kite_log.Gt(0.0, 0, 0, 25.5155532954539, 13.903139385469743,
                                                7.994703933613708, -11.53260977523988))
        self.assertEqual({r.actor_id for r in log.gt}, {0, 1, 2})

    def test_logs_from_kite_track_keep_gt_and_meas_and_add_tracks(self):
        golden = read_log(DATA / "seed42.log")
        for name, tracks, track_ids in (("kalman", 56, {0, 1, 2}), ("baseline", 57, {0, 1, 2, 3, 4, 5, 6})):
            with self.subTest(tracker=name):
                log = read_log(DATA / f"seed42_{name}_tracks.log")
                self.assertEqual((log.gt, log.meas), (golden.gt, golden.meas))
                self.assertEqual(len(log.tracks), tracks)
                self.assertEqual({r.track_id for r in log.tracks}, track_ids)

    def test_every_golden_file_reads_back_to_its_exact_bytes(self):
        for name in ("seed42.log", "seed42_kalman_tracks.log", "seed42_baseline_tracks.log",
                     "hard_seed7.log"):
            with self.subTest(name=name):
                data = (DATA / name).read_bytes()
                self.assertEqual(serialise(parse_log(data)), data)


class CanonicalDouble(unittest.TestCase):
    def test_documented_examples(self):
        # 0.0001 is 6 characters plain and 5 as 1e-04, so scientific wins; 0.001 is 5 either way,
        # and plain wins a tie.
        for value, text in ((0.1, "0.1"), (-3.25, "-3.25"), (8.0, "8"), (1e5, "1e+05"), (1.0, "1"),
                            (1e300, "1e+300"), (-1e-300, "-1e-300"), (-0.0, "-0"), (0.0, "0"),
                            (123456.0, "123456"), (1.5e-5, "1.5e-05"),
                            (0.0001, "1e-04"), (0.001, "0.001")):
            with self.subTest(text=text):
                self.assertEqual(canonical_double(value), text)


class Errors(unittest.TestCase):
    def assertLineError(self, data, line, fragment):
        with self.assertRaises(LogError) as caught:
            parse_log(data)
        message = str(caught.exception)
        self.assertTrue(message.startswith(f"line {line}: "), message)
        self.assertIn(fragment, message)

    def test_header_problems_name_line_1(self):
        self.assertLineError(b"", 1, "empty")
        self.assertLineError(b"\n", 1, "header")
        self.assertLineError(b"# kite-log v1\n", 1, "header")
        self.assertLineError(b"gt,0,0,1,1,1,1,1\n", 1, "header")

    def test_unknown_and_empty_record_types(self):
        self.assertLineError(HEADER + b"gt,0,0,1,1,1,1,1\nfoo,1\n", 3, "unknown record type 'foo'")
        self.assertLineError(HEADER + b"\n", 2, "unknown record type ''")

    def test_wrong_field_counts(self):
        self.assertLineError(HEADER + b"gt,0,0,1,1,1,1\n", 2, "found 6")
        self.assertLineError(HEADER + b"meas,0,0,0,1,1\nmeas,0,0,0,1\n", 3, "found 4")
        self.assertLineError(HEADER + b"track,0,0,1,1,1,1,1,1\n", 2, "found 8")

    def test_numbers_that_are_not_numbers(self):
        for field in (b"abc", b"", b" 1", b"+1", b"1 ", b"1_0", b"nan", b"inf", b"-inf", b"0x10", b"1e"):
            with self.subTest(field=field):
                self.assertLineError(HEADER + b"meas,0,0,0," + field + b",1\n", 2, "number")

    def test_non_canonical_numbers(self):
        for field in (b"1.50", b"1E5", b"1e5", b"1e+5", b"100000", b".5", b"5.", b"1.0", b"1e+05.0"):
            with self.subTest(field=field):
                self.assertLineError(HEADER + b"meas,0,0,0," + field + b",1\n", 2, "number")

    def test_integers_must_be_canonical_and_in_range(self):
        for field in (b"007", b"-0", b"+1", b"1.5", b"2147483648", b"-2147483649"):
            with self.subTest(field=field):
                self.assertLineError(HEADER + b"meas,0,0," + field + b",1,1\n", 2, "integer")

    def test_out_of_range_double_is_not_finite(self):
        self.assertLineError(HEADER + b"meas,1e+999,0,0,1,1\n", 2, "not finite")

    def test_carriage_return_is_an_error(self):
        self.assertLineError(HEADER + b"meas,0,0,0,1,1\r\n", 2, "carriage return")
        self.assertLineError(b"# kite-log v0\r\n", 1, "header")

    def test_a_log_cut_off_mid_line_names_the_last_line(self):
        self.assertLineError(HEADER + b"meas,0,0,0,1,1\nmeas,0,0,0,1", 3, "newline")
        self.assertLineError(b"# kite-log v0", 1, "newline")

    def test_invalid_utf8_names_its_line(self):
        self.assertLineError(HEADER + b"meas,0,0,0,\xff,1\n", 2, "UTF-8")

    def test_record_types_may_come_in_any_order(self):
        log = parse_log(HEADER + b"track,0.1,0,1,13.3,-3.25,7.9,0.05\nmeas,0.1,0,0,13.31,-3.2\n"
                        b"gt,0,0,1,12.5,-3.25,8,0\n")
        self.assertEqual((len(log.gt), len(log.meas), len(log.tracks)), (1, 1, 1))

    def test_header_only_log_is_empty(self):
        log = parse_log(HEADER)
        self.assertEqual((log.gt, log.meas, log.tracks), ([], [], []))


if __name__ == "__main__":
    unittest.main()
