"""Turns one BeamNGpy sensor poll result into a `Reading`.

Kept apart from `beamng_adapter.py` so that tests can drive it with plain dicts and lists,
without `beamngpy`.
"""

from kite_bridge.sim_api import POLL_TIME, Reading


def reading_from_raw(kind, raw, poll_time):
    """`raw` is what the sensor's `poll()` returned: one reading (a dict), a list of readings, a
    dict of readings, or something empty. `poll_time` is the simulator time of the poll."""
    points = None
    if kind == "lidar" and raw:
        # ASSUMPTION: a 1.36 lidar poll is one dict whose "pointCloud" has one row per point
        if not isinstance(raw, dict) or "pointCloud" not in raw:
            keys = sorted(map(str, raw)) if isinstance(raw, dict) else type(raw).__name__
            return Reading(None, unexpected=f"lidar reading has no 'pointCloud'; got {keys}")
        points = len(raw["pointCloud"])
        if points == 0:
            return Reading(None)
    readings = _readings(raw)
    if not readings:
        return Reading(None)
    latest = readings[-1]
    # ASSUMPTION: readings carry "time" in simulator seconds, and the last one is the latest
    if isinstance(latest, dict) and "time" in latest:
        return Reading(float(latest["time"]), points, count=len(readings))
    return Reading(poll_time, points, POLL_TIME, count=len(readings))


def _readings(raw):
    if isinstance(raw, list):
        return raw
    if isinstance(raw, dict) and raw and all(isinstance(v, dict) for v in raw.values()):
        return list(raw.values())
    return [raw] if raw else []
