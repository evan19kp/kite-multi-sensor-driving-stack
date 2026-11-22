import sys
import unittest
from pathlib import Path

import scene
from kite_log import Gt, Log, Meas, Track, read_log

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").is_file())
DATA = ROOT / "sim" / "tests" / "data"


class FakeSink:
    def __init__(self):
        self.steps = []

    def set_time(self, seconds):
        self.steps.append((seconds, {}))

    def log_points(self, path, positions, colors, labels):
        assert len(positions) == len(colors) and (labels is None or len(labels) == len(positions))
        assert path not in self.steps[-1][1], f"{path} logged twice in one step"
        self.steps[-1][1][path] = (positions, colors, labels)


def run(log):
    sink = FakeSink()
    summary = scene.log_scene(sink, log)
    return sink.steps, summary


class Entities(unittest.TestCase):
    def setUp(self):
        self.log = read_log(DATA / "seed42_kalman_tracks.log")
        self.steps, self.summary = run(self.log)

    def test_neither_rerun_nor_numpy_is_imported(self):
        self.assertNotIn("rerun", sys.modules)
        self.assertNotIn("numpy", sys.modules)

    def test_one_step_per_distinct_time_in_order(self):
        times = [t for t, _ in self.steps]
        self.assertEqual(times, sorted({r.t for r in self.log.gt + self.log.meas + self.log.tracks}))
        self.assertEqual(self.summary, scene.Summary(len(times), 63, 57, 56))

    def test_every_step_logs_the_same_four_entities_and_nothing_else(self):
        wanted = {scene.GROUND_TRUTH_PATH, scene.MEASUREMENTS_PATH, scene.TRACKS_PATH, scene.VELOCITY_PATH}
        for t, entities in self.steps:
            self.assertEqual(set(entities), wanted, t)

    def test_counts_per_step_match_the_records_at_that_time(self):
        for t, entities in self.steps:
            gt = [r for r in self.log.gt if r.t == t]
            meas = [r for r in self.log.meas if r.t == t]
            tracks = [r for r in self.log.tracks if r.t == t]
            self.assertEqual(len(entities[scene.GROUND_TRUTH_PATH][0]), len(gt), t)
            self.assertEqual(len(entities[scene.MEASUREMENTS_PATH][0]), len(meas), t)
            self.assertEqual(len(entities[scene.TRACKS_PATH][0]), len(tracks), t)
            self.assertEqual(len(entities[scene.VELOCITY_PATH][0]), len(tracks) * len(scene.VELOCITY_TRAIL_S), t)

    def test_ground_truth_is_only_under_ground_truth(self):
        gt_positions = {(r.x, r.y, 0.0) for r in self.log.gt}
        for _, entities in self.steps:
            for path, (positions, _, _) in entities.items():
                if path != scene.GROUND_TRUTH_PATH:
                    self.assertFalse(gt_positions & set(positions), path)
        self.assertTrue(scene.GROUND_TRUTH_PATH.startswith("ground_truth/"))
        for path in (scene.MEASUREMENTS_PATH, scene.TRACKS_PATH, scene.VELOCITY_PATH):
            self.assertFalse(path.startswith("ground_truth"))

    def test_labels_match_ids_in_record_order(self):
        for t, entities in self.steps:
            self.assertEqual(entities[scene.GROUND_TRUTH_PATH][2],
                             [str(r.actor_id) for r in self.log.gt if r.t == t])
            self.assertEqual(entities[scene.TRACKS_PATH][2],
                             [str(r.track_id) for r in self.log.tracks if r.t == t])

    def test_positions_are_the_record_positions_on_the_ground(self):
        t, entities = self.steps[3]
        self.assertEqual(entities[scene.GROUND_TRUTH_PATH][0],
                         [(r.x, r.y, 0.0) for r in self.log.gt if r.t == t])
        self.assertEqual(entities[scene.MEASUREMENTS_PATH][0],
                         [(r.x, r.y, 0.0) for r in self.log.meas if r.t == t])


class Colours(unittest.TestCase):
    def test_one_colour_per_track_id_on_every_step(self):
        steps, _ = run(read_log(DATA / "seed42_baseline_tracks.log"))
        seen = {}
        for _, entities in steps:
            _, colours, labels = entities[scene.TRACKS_PATH]
            for label, colour in zip(labels, colours):
                self.assertEqual(seen.setdefault(label, colour), colour)
        self.assertEqual(len(set(seen.values())), len(seen))

    def test_track_colours_differ_from_ground_truth_and_measurement_colours(self):
        for colour in scene.TRACK_COLOURS:
            self.assertNotIn(colour, (scene.GROUND_TRUTH_COLOUR, scene.MEASUREMENT_COLOUR))
        self.assertNotEqual(scene.GROUND_TRUTH_COLOUR, scene.MEASUREMENT_COLOUR)
        self.assertEqual(len(set(scene.TRACK_COLOURS)), len(scene.TRACK_COLOURS))

    def test_colour_is_stable_and_cycles_after_the_palette(self):
        n = len(scene.TRACK_COLOURS)
        self.assertEqual(scene.track_colour(3), scene.track_colour(3))
        self.assertEqual(scene.track_colour(3), scene.track_colour(3 + n))
        self.assertNotEqual(scene.track_colour(3), scene.track_colour(4))

    def test_velocity_trail_has_its_track_s_colour(self):
        steps, _ = run(read_log(DATA / "seed42_kalman_tracks.log"))
        _, entities = steps[5]
        colours = entities[scene.TRACKS_PATH][1]
        trail = entities[scene.VELOCITY_PATH][1]
        n = len(scene.VELOCITY_TRAIL_S)
        for i, colour in enumerate(colours):
            self.assertEqual(trail[i * n:(i + 1) * n], [colour] * n)


class VelocityLine(unittest.TestCase):
    def test_trail_follows_the_velocity_from_the_track_position(self):
        log = Log([], [], [Track(1.0, 0, 7, 10.0, -2.0, 4.0, -1.0)])
        (_, entities), = run(log)[0]
        positions, _, labels = entities[scene.VELOCITY_PATH]
        self.assertEqual([(round(x, 9), round(y, 9)) for x, y, _ in positions],
                         [(10.0 + 4.0 * s, -2.0 - 1.0 * s) for s in scene.VELOCITY_TRAIL_S])
        self.assertTrue(all(z == 0.0 for *_, z in positions))
        self.assertIsNone(labels)
        self.assertEqual(entities[scene.TRACKS_PATH][2], ["7"])


class Timeline(unittest.TestCase):
    def test_records_are_grouped_by_time_whatever_their_type_order(self):
        log = Log([Gt(0.2, 0, 1, 0.0, 0.0, 0.0, 0.0)], [Meas(0.1, 0, 0, 1.0, 1.0)], [])
        steps, summary = run(log)
        self.assertEqual([t for t, _ in steps], [0.1, 0.2])
        self.assertEqual(len(steps[0][1][scene.GROUND_TRUTH_PATH][0]), 0)
        self.assertEqual(len(steps[1][1][scene.MEASUREMENTS_PATH][0]), 0)
        self.assertEqual(summary, scene.Summary(2, 1, 1, 0))

    def test_an_empty_log_logs_nothing(self):
        steps, summary = run(Log([], [], []))
        self.assertEqual((steps, summary), ([], scene.Summary(0, 0, 0, 0)))


if __name__ == "__main__":
    unittest.main()
