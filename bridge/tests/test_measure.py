import math
import unittest

from kite_bridge import measure

BAD_NUMBERS = [math.nan, math.inf, -math.inf, -0.5]


class EffectiveRateTest(unittest.TestCase):
    def test_five_readings_over_two_seconds(self):
        self.assertEqual(measure.effective_rate([0.0, 0.5, 1.0, 1.5, 2.0]), 2.0)

    def test_starts_at_a_nonzero_time(self):
        self.assertEqual(measure.effective_rate([10.0, 10.25, 10.5]), 4.0)

    def test_undefined_cases_give_none(self):
        self.assertIsNone(measure.effective_rate([]))
        self.assertIsNone(measure.effective_rate([1.0]))
        self.assertIsNone(measure.effective_rate([2.0, 2.0, 2.0]))

    def test_outside_the_domain_raises(self):
        for bad in BAD_NUMBERS:
            with self.assertRaises(ValueError):
                measure.effective_rate([0.0, bad])
        with self.assertRaises(ValueError):
            measure.effective_rate([1.0, 0.5])


class PercentileTest(unittest.TestCase):
    def test_median_of_four_interpolates(self):
        self.assertEqual(measure.percentile([1, 2, 3, 4], 50), 2.5)

    def test_p95_of_four_is_hand_computed(self):
        # rank 0.95 * 3 = 2.85, so 3 + 0.85 * (4 - 3) = 3.85
        self.assertAlmostEqual(measure.percentile([1, 2, 3, 4], 95), 3.85)

    def test_ends_and_unsorted_input(self):
        self.assertEqual(measure.percentile([4, 1, 3, 2], 0), 1)
        self.assertEqual(measure.percentile([4, 1, 3, 2], 100), 4)
        self.assertEqual(measure.percentile([4, 1, 3, 2], 50), 2.5)

    def test_one_element_and_all_equal(self):
        self.assertEqual(measure.percentile([7.0], 95), 7.0)
        self.assertEqual(measure.percentile([3.0, 3.0, 3.0], 50), 3.0)

    def test_empty_gives_none(self):
        self.assertIsNone(measure.percentile([], 50))

    def test_outside_the_domain_raises(self):
        for bad in BAD_NUMBERS:
            with self.assertRaises(ValueError):
                measure.percentile([1.0, bad], 50)
        for q in (-1, 100.5, math.nan):
            with self.assertRaises(ValueError):
                measure.percentile([1.0], q)


class LatencyStatsTest(unittest.TestCase):
    def test_four_latencies(self):
        stats = measure.latency_stats([0.001, 0.002, 0.003, 0.004])
        self.assertAlmostEqual(stats["mean"], 0.0025)
        self.assertAlmostEqual(stats["p50"], 0.0025)
        self.assertAlmostEqual(stats["p95"], 0.00385)
        self.assertEqual(stats["max"], 0.004)

    def test_empty_gives_none_everywhere(self):
        self.assertEqual(measure.latency_stats([]),
                         {"mean": None, "p50": None, "p95": None, "max": None})

    def test_negative_latency_raises(self):
        with self.assertRaises(ValueError):
            measure.latency_stats([0.1, -0.1])


class CountsTest(unittest.TestCase):
    SERIES = [0.0, 0.0, None, 0.0, 0.1, 0.1, None, None, 0.2]

    def test_empty_readings(self):
        self.assertEqual(measure.count_empty(self.SERIES), 3)

    def test_repeated_readings_compare_with_the_previous_non_empty_one(self):
        # 0.0 twice in a row, 0.0 again after a None, 0.1 twice in a row
        self.assertEqual(measure.count_repeated(self.SERIES), 3)

    def test_fresh_timestamps_drop_repeats_and_gaps(self):
        self.assertEqual(measure.fresh_timestamps(self.SERIES), [0.0, 0.1, 0.2])

    def test_empty_series(self):
        self.assertEqual(measure.count_empty([]), 0)
        self.assertEqual(measure.count_repeated([]), 0)
        self.assertEqual(measure.fresh_timestamps([]), [])

    def test_decreasing_or_invalid_series_raises(self):
        for series in ([0.2, None, 0.1], [0.0, math.nan], [0.0, math.inf], [-1.0]):
            for function in (measure.count_empty, measure.count_repeated, measure.fresh_timestamps):
                with self.assertRaises(ValueError):
                    function(series)


class StepsPerSecondTest(unittest.TestCase):
    def test_rate(self):
        self.assertEqual(measure.steps_per_second(120, 2.0), 60.0)

    def test_zero_seconds_gives_none(self):
        self.assertIsNone(measure.steps_per_second(10, 0.0))

    def test_outside_the_domain_raises(self):
        for steps, seconds in [(-1, 1.0), (10, -1.0), (10, math.nan), (10, math.inf)]:
            with self.assertRaises(ValueError):
                measure.steps_per_second(steps, seconds)


class SummariseTest(unittest.TestCase):
    def test_one_sensor(self):
        series = [0.0, 0.0, None, 0.5, 1.0]
        summary = measure.summarise(series, [0.001] * 5, points=[100, 200])
        self.assertEqual(summary["polls"], 5)
        self.assertEqual(summary["rate_hz"], 2.0)
        self.assertEqual(summary["empty"], 1)
        self.assertEqual(summary["repeated"], 1)
        self.assertEqual(summary["points_mean"], 150.0)
        self.assertAlmostEqual(summary["latency_s"]["mean"], 0.001)

    def test_no_points_gives_none(self):
        self.assertIsNone(measure.summarise([0.0], [0.0])["points_mean"])


if __name__ == "__main__":
    unittest.main()
