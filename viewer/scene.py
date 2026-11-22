"""Turns a Log into what the viewer shows, one time step at a time, without knowing Rerun. A sink
takes `set_time(seconds)` and `log_points(path, positions, colors, labels)`; `rerun_sink.RerunSink`
is the real one.

Every step logs every entity, an empty one included, because a viewer keeps the last thing logged
to an entity until something replaces it: a step with no measurement must show none. Ground truth
goes only under GROUND_TRUTH_PATH, so hiding that one entity hides all of it, and a tracker's
output never sits under it.
"""

from typing import NamedTuple

GROUND_TRUTH_PATH = "ground_truth/actors"
MEASUREMENTS_PATH = "measurements"
TRACKS_PATH = "tracks"
VELOCITY_PATH = "tracks/velocity"
GROUND_TRUTH_COLOUR = (255, 165, 0)
MEASUREMENT_COLOUR = (150, 150, 150)
TRACK_COLOURS = (
    (0, 114, 255), (0, 200, 83), (230, 50, 200), (0, 200, 220), (230, 40, 40), (140, 90, 255),
    (170, 220, 0), (255, 105, 135), (0, 150, 136), (200, 160, 255), (110, 70, 20), (255, 255, 255),
)
VELOCITY_TRAIL_S = (0.1, 0.2, 0.3, 0.4, 0.5)


class Summary(NamedTuple):
    steps: int
    gt: int
    meas: int
    tracks: int


def track_colour(track_id):
    """The colour of track `track_id`, the same on every step and every run: the id picks an entry
    of TRACK_COLOURS, and ids 12 apart share one."""
    return TRACK_COLOURS[track_id % len(TRACK_COLOURS)]


def _by_time(records):
    groups = {}
    for record in records:
        groups.setdefault(record.t, []).append(record)
    return groups


def log_scene(sink, log):
    """Logs `log` to `sink` on the time of each distinct `t` in it, in order, and returns the
    Summary of what it logged. Points are (x, y, 0). A track's velocity is the points it would
    reach at each of VELOCITY_TRAIL_S seconds if it kept its velocity, in the track's colour."""
    gt, meas, tracks = _by_time(log.gt), _by_time(log.meas), _by_time(log.tracks)
    times = sorted(set(gt) | set(meas) | set(tracks))
    for t in times:
        actors, measurements, current = gt.get(t, []), meas.get(t, []), tracks.get(t, [])
        sink.set_time(t)
        sink.log_points(GROUND_TRUTH_PATH, [(a.x, a.y, 0.0) for a in actors],
                        [GROUND_TRUTH_COLOUR] * len(actors), [str(a.actor_id) for a in actors])
        sink.log_points(MEASUREMENTS_PATH, [(m.x, m.y, 0.0) for m in measurements],
                        [MEASUREMENT_COLOUR] * len(measurements), None)
        sink.log_points(TRACKS_PATH, [(k.x, k.y, 0.0) for k in current],
                        [track_colour(k.track_id) for k in current], [str(k.track_id) for k in current])
        trail = [(k, s) for k in current for s in VELOCITY_TRAIL_S]
        sink.log_points(VELOCITY_PATH, [(k.x + k.vx * s, k.y + k.vy * s, 0.0) for k, s in trail],
                        [track_colour(k.track_id) for k, _ in trail], None)
    return Summary(len(times), len(log.gt), len(log.meas), len(log.tracks))
