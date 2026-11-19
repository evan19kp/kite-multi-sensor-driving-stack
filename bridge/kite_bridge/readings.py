"""Turns one BeamNGpy sensor poll result into plain data: a `Reading` for any sensor, the lidar
points, or the camera image. An empty poll, including a lidar scan with no points, is no reading;
a radar array with no rows is a reading with 0 detections.

Input domain, and nothing beyond it: a raw poll is None, a dict, a list, bytes, or an array-like
object, which is any object with `__len__` (its truth value may raise, like a numpy array's, so
a raw poll is never truth-tested). Anything else is an unexpected format.
"""

from kite_bridge.sim_api import POLL_TIME, Reading


def reading_from_raw(kind, raw, poll_time):
    """`raw` is what the sensor's `poll()` returned, within the input domain above. `poll_time` is
    the simulator time of the poll.

    A dict is one reading, a list is its readings, a dict of dicts is its readings, and any
    other non-empty object is one reading. A radar array-like is one reading with one detection
    per row, in `detections`, even with zero rows; the columns are not read. A lidar poll must
    be a dict with a "pointCloud". Any other empty poll, including a lidar scan with no points,
    is no reading. A poll outside the domain, or a lidar poll
    without "pointCloud", gives a reading with `unexpected` set."""
    try:
        return _reading(kind, raw, poll_time)
    except ValueError as error:
        return Reading(None, unexpected=str(error))


def points_from_raw(raw):
    """Lidar `raw` (a poll result) to a list of (x, y, z) float tuples, or None for no reading.
    Raises ValueError for a poll outside the input domain or without "pointCloud"."""
    cloud = _point_cloud(raw)
    if cloud is None:
        return None
    # ASSUMPTION: "pointCloud" rows are world-space x, y, z (extra columns are ignored)
    return [(float(row[0]), float(row[1]), float(row[2])) for row in cloud]


def image_from_raw(raw):
    """Camera `raw` (a poll result) to rows of (red, green, blue) tuples, or None for no reading.
    Raises ValueError for a poll outside the input domain or without "colour"."""
    if _is_empty(raw):
        return None
    # ASSUMPTION: the colour image is under "colour" and is a PIL-style image (convert, size, getdata)
    if not isinstance(raw, dict) or raw.get("colour") is None:
        raise ValueError(f"unexpected camera reading: no 'colour' in {_describe(raw)}")
    image = raw["colour"].convert("RGB")
    width, height = image.size
    pixels = [tuple(int(v) for v in pixel) for pixel in image.getdata()]
    return [pixels[row * width:(row + 1) * width] for row in range(height)]


def _reading(kind, raw, poll_time):
    if kind == "radar" and hasattr(raw, "__len__") and not isinstance(raw, (dict, list, bytes)):
        return Reading(poll_time, time_source=POLL_TIME, detections=len(raw))
    if _is_empty(raw):
        return Reading(None)
    points = None
    if kind == "lidar":
        cloud = _point_cloud(raw)
        if cloud is None:
            return Reading(None)
        points = len(cloud)
    readings = _readings(raw)
    latest = readings[-1]
    # ASSUMPTION: readings carry "time" in simulator seconds, and the last one is the latest
    if isinstance(latest, dict) and "time" in latest:
        return Reading(float(latest["time"]), points, count=len(readings))
    return Reading(poll_time, points, POLL_TIME, count=len(readings))


def _is_empty(raw):
    """True for None and for an object of length 0. Raises ValueError outside the input domain."""
    if raw is None:
        return True
    if not hasattr(raw, "__len__"):
        raise ValueError(f"unexpected poll result: got {type(raw).__name__}")
    return len(raw) == 0


def _point_cloud(raw):
    if _is_empty(raw):
        return None
    # ASSUMPTION: a 1.36 lidar poll is one dict whose "pointCloud" has one row per point
    if not isinstance(raw, dict) or "pointCloud" not in raw:
        raise ValueError(f"lidar reading has no 'pointCloud'; got {_describe(raw)}")
    cloud = raw["pointCloud"]
    return cloud if len(cloud) else None


def _describe(raw):
    return sorted(map(str, raw)) if isinstance(raw, dict) else type(raw).__name__


def _readings(raw):
    if isinstance(raw, list):
        return raw
    if isinstance(raw, dict) and all(isinstance(v, dict) for v in raw.values()):
        return list(raw.values())
    return [raw]
