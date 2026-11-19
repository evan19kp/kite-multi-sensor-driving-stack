"""Checks a streamed lidar or camera reading before anything uses it.

A streamed sensor writes each reading into shared memory, and BeamNGpy's `stream()` reads that
memory without asking the simulator what is in it. For the lidar it reads the whole block,
without asking how many points the scan has, so the rows after the scan may be memory that was
never written (zeros) or rows left from an earlier, longer scan. This module is the one place
that decides what of a streamed reading is passed on. What it cannot trust it drops and counts;
it never passes it on silently. Standard library only: it never imports numpy or PIL and reads
arrays and images by duck typing.

Lidar. Input domain: what `readings.reading_from_raw` accepts for a lidar. Only a dict with a
"pointCloud" is checked; anything else goes to `reading_from_raw` unchanged and comes out as an
empty or unexpected reading, as a poll would.
- "pointCloud" must have `shape` == (rows, columns) with columns >= 3, and `tobytes()` that
  returns rows * columns values of 4 or 8 bytes each (float32 or float64, in the machine's byte
  order, row after row). Otherwise the whole reading is dropped as suspect.
- An all-zero row is a row whose bytes are all zero. The all-zero rows after the last row with
  any non-zero byte are cut off and counted in `suspect_rows`. Every array in the dict whose
  `shape` has the same row count (the "colours", for example) is cut to the same rows. An
  all-zero row before the last non-zero row is kept and not counted: it cannot be told from a
  point.
- If a kept value is not finite (nan or inf), the whole reading is dropped as suspect.
- Not detectable: rows left from an earlier, longer scan. They are non-zero, finite rows of the
  right shape, and the streamed dict says nothing about how many rows the new scan wrote, so
  they are kept and counted as points.

Camera. Input domain: what `readings.reading_from_raw` accepts for a camera. Only a dict is
checked. Every value that is not None is an image, and its size must be the configured
`resolution` (width, height) in `sensor_settings.py`: an object with `shape` is
(height, width, ...) like a numpy array, else an object with a two-item `size` is
(width, height) like a PIL image. An image of another size, or with neither, drops the whole
reading as suspect. Not detectable: an image of the right size that was never rendered into
(all black, for example) or holds an earlier frame.

Freshness is not decided here: a reading whose memory has not changed since the last read
passes the check, and the measuring loop's content fingerprint (`raw_data`) says it is not new.
"""

import array
import dataclasses
import math

from kite_bridge.readings import reading_from_raw
from kite_bridge.sensor_settings import SENSOR_SETTINGS
from kite_bridge.sim_api import Reading

FLOAT_CODES = {4: "f", 8: "d"}


class Suspect(Exception):
    """A streamed reading that cannot be trusted; the message says why."""


def reading_from_stream(kind, raw, read_time):
    """`raw` is what a streamed sensor's `stream()` returned; `kind` is "lidar" or "camera";
    `read_time` is the simulator time of the read. Returns the `Reading` of what was kept,
    with `suspect_rows` set, or a `Reading` with only `suspect` set when the whole reading
    was dropped."""
    try:
        kept, cut = _check_lidar(raw) if kind == "lidar" else (_check_camera(raw), 0)
    except Suspect as why:
        return Reading(None, suspect=str(why))
    return dataclasses.replace(reading_from_raw(kind, kept, read_time), suspect_rows=cut)


def _check_lidar(raw):
    if not isinstance(raw, dict) or "pointCloud" not in raw:
        return raw, 0
    cloud = raw["pointCloud"]
    rows = _rows(cloud)
    kept = trusted_rows(cloud)
    if kept == rows:
        return raw, 0
    return {key: value[:kept] if _rows(value) == rows else value for key, value in raw.items()}, rows - kept


def _rows(value):
    shape = getattr(value, "shape", None)
    return shape[0] if isinstance(shape, tuple) and shape else None


def trusted_rows(cloud):
    """How many leading rows of a point cloud to keep, by the lidar rules above. Raises Suspect
    when the whole cloud cannot be trusted."""
    shape = getattr(cloud, "shape", None)
    if not (isinstance(shape, tuple) and len(shape) == 2 and shape[1] >= 3 and hasattr(cloud, "tobytes")):
        raise Suspect(f"pointCloud is not an array of x, y, z rows: shape {shape!r}")
    rows, columns = shape
    if rows == 0:
        return 0
    data = cloud.tobytes()
    itemsize, extra = divmod(len(data), rows * columns)
    if extra or itemsize not in FLOAT_CODES:
        raise Suspect(f"pointCloud holds {len(data)} bytes for shape {shape!r}, not 4- or 8-byte floats")
    row_bytes = itemsize * columns
    kept = -(-len(data.rstrip(b"\x00")) // row_bytes)
    values = array.array(FLOAT_CODES[itemsize], data[:kept * row_bytes])
    # A sum of finite floats is finite unless it overflows, so all() runs only to tell overflow from nan or inf.
    if not math.isfinite(sum(values)) and not all(map(math.isfinite, values)):
        raise Suspect("pointCloud holds a value that is not finite (nan or inf)")
    return kept


def _check_camera(raw):
    if not isinstance(raw, dict):
        return raw
    width, height = SENSOR_SETTINGS["camera"]["resolution"]
    for name, image in raw.items():
        if image is None:
            continue
        size = _image_size(image)
        if size is None:
            raise Suspect(f"camera image {name!r} has no size, expected {width}x{height}")
        if size != (width, height):
            raise Suspect(f"camera image {name!r} is {size[0]}x{size[1]}, expected {width}x{height}")
    return raw


def _image_size(image):
    shape = getattr(image, "shape", None)
    if isinstance(shape, tuple) and len(shape) >= 2:
        return shape[1], shape[0]
    size = getattr(image, "size", None)
    if isinstance(size, tuple) and len(size) == 2:
        return size
    return None
