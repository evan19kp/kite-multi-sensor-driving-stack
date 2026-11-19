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


def points_from_raw(raw):
    """Lidar `raw` (a poll result) to a list of (x, y, z) float tuples. An empty poll gives []."""
    if not raw:
        return []
    # ASSUMPTION: "pointCloud" rows are world-space x, y, z (extra columns are ignored)
    if not isinstance(raw, dict) or "pointCloud" not in raw:
        raise ValueError(f"unexpected lidar reading: no 'pointCloud' in {_describe(raw)}")
    return [(float(row[0]), float(row[1]), float(row[2])) for row in raw["pointCloud"]]


def image_from_raw(raw):
    """Camera `raw` (a poll result) to rows of (red, green, blue) tuples."""
    # ASSUMPTION: the colour image is under "colour" and is a PIL-style image (convert, size, getdata)
    if not isinstance(raw, dict) or raw.get("colour") is None:
        raise ValueError(f"unexpected camera reading: no 'colour' in {_describe(raw)}")
    image = raw["colour"].convert("RGB")
    width, height = image.size
    pixels = [tuple(int(v) for v in pixel) for pixel in image.getdata()]
    return [pixels[row * width:(row + 1) * width] for row in range(height)]


def _describe(raw):
    return sorted(map(str, raw)) if isinstance(raw, dict) else type(raw).__name__


def _readings(raw):
    if isinstance(raw, list):
        return raw
    if isinstance(raw, dict) and raw and all(isinstance(v, dict) for v in raw.values()):
        return list(raw.values())
    return [raw] if raw else []
