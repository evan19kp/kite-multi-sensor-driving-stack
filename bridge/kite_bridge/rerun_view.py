"""Shows one simulator run in Rerun. Sensor layers (lidar, camera) have their own roots. Our car's
and the other cars' positions come from simulator state, so they are ground truth: for display
only and always under `ground_truth/`, which is what separates them from sensor data.

`world/ego` is our car as the sensors place it, with the lidar as its child. In free mode it sits
at the x, y the GPS measures, at height 0, because a GPS reading gives no height; with no such
position it stays at the origin. Nothing from simulator state goes under `world/`.

`RerunRecorder` hands lidar points and camera images to Rerun as numpy arrays (`NumpyArrays`):
each step is one operation on the whole array, with no Python loop over points or pixels.
`ListArrays` does the same work on plain lists, one point at a time. It exists only for tests
and the fake, whose stand-in for Rerun takes lists and which have no numpy.
"""

import functools
import io
import math
import numbers
from pathlib import Path, PurePosixPath, PureWindowsPath

from kite_bridge.output_guard import PROVENANCE
from kite_bridge.readings import readings_in

PROVENANCE_PATH = "meta/provenance"
EGO_FRAME_PATH = "world/ego"
LIDAR_PATH = "world/ego/lidar"
CAMERA_PATH = "camera/image"
EGO_PATH = "ground_truth/ego/position"
GROUND_TRUTH_PATH = "ground_truth/other_cars"
TIMELINE = "sim_time"
FAR_M = 50.0
GROUND_ORIGIN = (0.0, 0.0)
EGO_COLOUR = (255, 255, 255)
GROUND_TRUTH_COLOUR = (255, 165, 0)
RECORDING_SUFFIX = ".rrd"
PNG_MEDIA_TYPE = "image/png"


def distance_colour(distance_m):
    """RGB for a lidar point `distance_m` metres from our car: blue at 0 m, red at FAR_M (50 m)
    or more, blending linearly between. Domain: a finite distance >= 0; anything else raises
    ValueError."""
    if not math.isfinite(distance_m) or distance_m < 0:
        raise ValueError(f"distance must be a finite number >= 0, got {distance_m}")
    far = round(255 * min(distance_m / FAR_M, 1.0))
    return (far, 0, 255 - far)


def log_step(recorder, sim_time, ego, lidar_points, image, others=None):
    """Logs one simulator step: the time, the lidar points coloured by distance from `ego`, `ego`
    itself under EGO_PATH and the camera `image`. `lidar_points` is None when the lidar gave no
    reading, and then no lidar is logged; a list, even an empty one, is a scan. `image` is None
    when the camera gave no reading, and then no image is logged. `others` is the traffic's
    ground truth, or None when it is unavailable; a list, even an empty one, is logged under
    GROUND_TRUTH_PATH."""
    recorder.set_time(sim_time)
    if lidar_points is not None:
        recorder.log_lidar(LIDAR_PATH, lidar_points, ego)
    recorder.log_points(EGO_PATH, [ego], [EGO_COLOUR])
    if image is not None:
        recorder.log_image(CAMERA_PATH, image)
    if others is not None:
        recorder.log_points(GROUND_TRUTH_PATH, others, [GROUND_TRUTH_COLOUR] * len(others))


def log_reading(recorder, kind, reading, ego=None):
    """Free mode: logs one new `reading` (a `sim_api.Reading` with a timestamp) of `kind` "lidar"
    or "camera" at its time, which is the Timer's for a reading without its own. Returns True when
    it logged the reading. A lidar scan with a point whose x, y or z is not finite, and a camera
    reading without a "colour" image, log nothing and return False, so a bad reading never stops
    the run (R28).

    The lidar's "pointCloud" goes under LIDAR_PATH, coloured by distance on the ground (x and y
    only) from `ego`, the (x, y) the GPS last measured, or from the origin while `ego` is None.
    Its other arrays, BeamNG's per-point "colours" among them, are never logged. The points are
    in world space (see `readings.points_from_raw`). With an `ego`, EGO_FRAME_PATH moves to it at
    height 0 and LIDAR_PATH moves back by the same amount, so the points stay where the lidar saw
    them in the world view, and a view whose origin is EGO_FRAME_PATH follows our car."""
    raw = reading.raw
    if kind == "lidar":
        if not recorder.all_finite(raw["pointCloud"]):
            return False
        recorder.set_time(reading.timestamp)
        if ego is not None:
            x, y = ego
            recorder.log_translation(EGO_FRAME_PATH, (x, y, 0.0))
            recorder.log_translation(LIDAR_PATH, (-x, -y, 0.0))
        recorder.log_lidar(LIDAR_PATH, raw["pointCloud"], GROUND_ORIGIN if ego is None else ego)
        return True
    image = raw.get("colour") if isinstance(raw, dict) else None
    if image is None:
        return False
    recorder.set_time(reading.timestamp)
    recorder.log_image(CAMERA_PATH, image)
    return True


def gps_position(raw):
    """The position the GPS measured in the latest reading of `raw`, a non-empty GPS poll in the
    domain of `readings.py`, as an (x, y) tuple of floats in metres. None when that reading has
    no finite numbers "x" and "y": a reading with latitude and longitude only gives None, because
    nothing here projects them to metres."""
    latest = readings_in(raw)[-1]
    if not isinstance(latest, dict):
        return None
    # ASSUMPTION: a BeamNGpy 1.36 GPS reading has the sensor's position in metres under "x" and "y", in the
    # world frame of the lidar points, beside "lat" and "lon"; not seen in a real run yet
    position = latest.get("x"), latest.get("y")
    if not all(isinstance(value, numbers.Real) and not isinstance(value, bool) for value in position):
        return None
    x, y = (float(value) for value in position)
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None


def recording_name(text):
    """Checks a `--save` value: a relative path that stays inside the output folder and ends
    in `.rrd`. Returns it unchanged, else raises ValueError saying why."""
    if not text or not text.lower().endswith(RECORDING_SUFFIX) or text.lower() == RECORDING_SUFFIX:
        raise ValueError(f"must be a file name ending in {RECORDING_SUFFIX}, got {text!r}")
    for flavour in (PurePosixPath, PureWindowsPath):
        path = flavour(text)
        if path.is_absolute() or path.drive or path.root or ".." in path.parts:
            raise ValueError(f"must stay inside --out (no absolute path, drive or ..), got {text!r}")
    return text


def recording_path(out, name):
    return Path(out) / recording_name(name)


class NumpyArrays:
    """The array work of RerunRecorder and of `self_hits.SelfHits`, on numpy arrays."""

    @functools.cached_property
    def np(self):
        import numpy

        return numpy

    def points(self, cloud):
        """x, y, z of each row of `cloud` (further columns dropped) as an (N, 3) float32 array."""
        rows = self.np.asarray(cloud, dtype=self.np.float32)
        return rows.reshape(-1, rows.shape[1] if rows.ndim == 2 else 3)[:, :3]

    def distance_colours(self, points, centre):
        """`distance_colour` of every point's distance from `centre`, as an (N, 3) uint8 array.
        A `centre` of two coordinates gives the distance in x and y only. Raises ValueError, as
        `distance_colour` does, when a distance is not finite."""
        np = self.np
        offsets = points[:, :len(centre)] - np.asarray(centre, dtype=np.float32)
        distance = np.sqrt((offsets ** 2).sum(axis=1))
        if not np.isfinite(distance).all():
            raise ValueError("a lidar point's distance is not finite (nan or inf)")
        far = np.rint(255 * np.minimum(distance / FAR_M, 1.0)).astype(np.uint8)
        return np.stack([far, np.zeros_like(far), 255 - far], axis=1)

    def all_finite(self, points):
        return bool(self.np.isfinite(points).all())

    def count_within(self, points, centre, radius):
        """How many of `points` (what `points()` returned) are at most `radius` metres from
        `centre` in x and y, in one array operation. A point whose x or y is nan or inf is never
        counted."""
        np = self.np
        offsets = points[:, :2] - np.asarray(centre, dtype=np.float32)
        return int(np.count_nonzero((offsets ** 2).sum(axis=1) <= radius * radius))

    def image(self, image):
        """A PIL image, an array or rows of pixels as a (height, width, channels) uint8 array."""
        # ASSUMPTION: BeamNGpy's colour image, polled or streamed, holds 8-bit values (0 to 255) per channel
        return self.np.asarray(image, dtype=self.np.uint8)


class ListArrays:
    """The work of NumpyArrays on plain lists, one point at a time. For tests and the fake only,
    never for a real scan."""

    def points(self, cloud):
        return [tuple(float(value) for value in row[:3]) for row in cloud]

    def distance_colours(self, points, centre):
        return [distance_colour(math.dist(point[:len(centre)], centre)) for point in points]

    def all_finite(self, points):
        return all(math.isfinite(value) for point in points for value in point)

    def count_within(self, points, centre, radius):
        return sum(math.dist(point[:2], centre) <= radius for point in points)

    def image(self, image):
        return image


def pillow_png(pixels):
    """`pixels`, a (height, width, 3 or 4) uint8 array, as PNG bytes. PNG is lossless."""
    from PIL import Image

    buffer = io.BytesIO()
    Image.fromarray(pixels).save(buffer, format="PNG")
    return buffer.getvalue()


class RerunRecorder:
    """Without `save_path`: spawns a live viewer, shows every `live_stride`-th lidar point and
    passes camera images as they are. With `save_path`: the recording goes to that file only,
    with every lidar point, and each camera image is encoded by `encode_png` (lossless,
    `pillow_png` by default) before it is logged. `encode_s` is the time spent encoding, by
    `clock`, and `encoded` the number of images encoded. `arrays` does the array work
    (NumpyArrays by default)."""

    def __init__(self, save_path=None, live_stride=1, *, clock, arrays=None, encode_png=None):
        import rerun

        self._rr = rerun
        self._arrays = arrays or NumpyArrays()
        self._stride = live_stride if save_path is None else 1
        self._encode_png = None if save_path is None else (encode_png or pillow_png)
        self._clock = clock
        self.encode_s = 0.0
        self.encoded = 0
        if save_path is None:
            rerun.init("kite_lidar", spawn=True)  # ASSUMPTION: init(app_id, spawn=True) spawns and connects a viewer
        else:
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            rerun.init("kite_lidar")
            rerun.save(str(save_path))  # ASSUMPTION: save(path) sends the recording to a file instead of a viewer
        rerun.log(PROVENANCE_PATH, rerun.TextLog(PROVENANCE))  # ASSUMPTION: TextLog(text) is a log entry that takes one string
        self._send_layout()

    def _send_layout(self):
        """Sends a layout whose 3D view has EGO_FRAME_PATH as its origin, so that it follows our
        car, beside the camera image and the provenance text. Says so when it cannot."""
        try:
            import rerun.blueprint as rrb

            # ASSUMPTION: rerun.blueprint has Blueprint, Horizontal, Vertical, Spatial3DView(origin=, contents=),
            # Spatial2DView(origin=) and TextLogView(origin=); a view shows its contents relative to its origin,
            # and send_blueprint(blueprint) makes it the viewer's layout
            layout = rrb.Blueprint(rrb.Horizontal(
                rrb.Spatial3DView(origin=f"/{EGO_FRAME_PATH}", contents=["/world/**", "/ground_truth/**"]),
                rrb.Vertical(rrb.Spatial2DView(origin="/camera"), rrb.TextLogView(origin="/meta"))))
            self._rr.send_blueprint(layout)
        except (ImportError, AttributeError, TypeError, ValueError) as error:
            print(f"default layout not sent ({type(error).__name__}: {error}); to follow our car, set the "
                  f"3D view's origin to /{EGO_FRAME_PATH} in the viewer")

    def set_time(self, seconds):
        if hasattr(self._rr, "set_time"):
            self._rr.set_time(TIMELINE, duration=seconds)  # ASSUMPTION: set_time(name, duration=) in newer SDKs
        else:
            self._rr.set_time_seconds(TIMELINE, seconds)  # ASSUMPTION: set_time_seconds(name, s) in older SDKs

    def all_finite(self, cloud):
        """True when every point of `cloud`, not only those a stride shows, has a finite x, y and z."""
        return self._arrays.all_finite(self._arrays.points(cloud))

    def log_lidar(self, path, cloud, centre):
        points = self._arrays.points(cloud)[::self._stride]
        colours = self._arrays.distance_colours(points, centre)
        # ASSUMPTION: Points3D takes an (N, 3) float32 array of positions and an (N, 3) uint8 array of colors
        self._rr.log(path, self._rr.Points3D(points, colors=colours))

    def log_points(self, path, positions, colors):
        # ASSUMPTION: Points3D takes plain lists of (x, y, z) positions and (r, g, b) colors
        self._rr.log(path, self._rr.Points3D(positions, colors=colors))

    def log_translation(self, path, translation):
        # ASSUMPTION: Transform3D(translation=(x, y, z)) places an entity, and its children, relative to its parent
        self._rr.log(path, self._rr.Transform3D(translation=translation))

    def log_image(self, path, image):
        pixels = self._arrays.image(image)
        if self._encode_png is None:
            self._rr.log(path, self._rr.Image(pixels))  # ASSUMPTION: Image takes a (height, width, 3 or 4) uint8 array
            return
        started = self._clock()
        png = self._encode_png(pixels)
        self.encode_s += self._clock() - started
        self.encoded += 1
        # ASSUMPTION: EncodedImage(contents=, media_type=) logs encoded image bytes that the viewer decodes
        self._rr.log(path, self._rr.EncodedImage(contents=png, media_type=PNG_MEDIA_TYPE))

    def close(self):
        self._rr.disconnect()  # ASSUMPTION: disconnect() flushes and closes the file or connection


class NullRecorder:
    def all_finite(self, cloud):
        return True

    def set_time(self, seconds):
        pass

    def log_lidar(self, path, cloud, centre):
        pass

    def log_points(self, path, positions, colors):
        pass

    def log_translation(self, path, translation):
        pass

    def log_image(self, path, image):
        pass

    def close(self):
        pass
