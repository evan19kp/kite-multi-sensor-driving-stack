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

A saved recording's camera images are PNG-encoded on one thread of their own (`PngWorker`). That
thread only encodes: it is given the encoder and the clock, never the simulator or Rerun. Every
Rerun call stays on the run's thread, which logs each encoded image at the time it was read.
"""

import collections
import contextlib
import dataclasses
import enum
import functools
import io
import math
import numbers
import signal
import sys
import threading
import typing
from copy import deepcopy
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
# The camera makes at most 20 images per simulated second (requested_update_time 0.05 s), so 8
# frames are 0.4 s of camera. A 640 x 360 RGBA frame is 921,600 bytes, so a full queue holds
# under 7.4 MB. The loop's own frame joins the queue while it waits for space, so closing waits
# for at most 9 encodes.
PNG_QUEUE_FRAMES = 8
WAIT_SLICE_S = 0.1


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

    def image(self, image, copy=False):
        """A PIL image, an array or rows of pixels as a (height, width, channels) uint8 array. With
        `copy` the array is always a new one, never a view of the reading's memory."""
        # ASSUMPTION: BeamNGpy's colour image, polled or streamed, holds 8-bit values (0 to 255) per channel
        return (self.np.array if copy else self.np.asarray)(image, dtype=self.np.uint8)


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

    def image(self, image, copy=False):
        return deepcopy(image) if copy else image


def pillow_png(pixels):
    """`pixels`, a (height, width, 3 or 4) uint8 array, as PNG bytes. PNG is lossless."""
    from PIL import Image

    buffer = io.BytesIO()
    Image.fromarray(pixels).save(buffer, format="PNG")
    return buffer.getvalue()


class Frame(typing.NamedTuple):
    """A camera image for the recording: its entity path, its time on TIMELINE, and its pixels
    before encoding or its PNG bytes after."""
    path: str
    time: float
    data: object


@dataclasses.dataclass
class EncodeStats:
    """`encoded` frames took `encode_s` on the encoder thread, by the clock given, from the start
    to the end of each encode. The run's loop found the queue full `backlog_waits` times and
    waited `backlog_wait_s` in all for space."""
    encoded: int = 0
    encode_s: float = 0.0
    backlog_waits: int = 0
    backlog_wait_s: float = 0.0


class EncodeError(RuntimeError):
    pass


class Phase(enum.Enum):
    """Where a PngWorker is. RUNNING encodes frames as they are put. `finish` moves RUNNING to
    CLOSING, which encodes the frames already put and stops. An exception from the encoder or the
    clock moves RUNNING or CLOSING to FAILED, which encodes nothing more. The first `put` or
    `next_encoded` that finds FAILED raises EncodeError and moves to REPORTED. In REPORTED `put`
    still raises and `next_encoded` does not. No other move happens."""
    RUNNING = "running"
    CLOSING = "closing"
    FAILED = "failed"
    REPORTED = "reported"


class PngWorker:
    """Encodes frames with `encode` on one thread of its own, in the order they are put. `put`
    adds the frame to the queue before anything else, so an interrupted `put` still leaves its
    frame to be encoded. While more than PNG_QUEUE_FRAMES frames wait unencoded, the one being
    encoded among them, `put` waits for space and counts the wait in `stats`; it never drops a
    frame. `first_encoded` is the oldest encoded frame not yet released, and `release_first` lets
    it go once it is recorded. `finish` lets the thread stop once every frame put is encoded, and
    `next_encoded` waits for each of them. `unsaved` counts the frames put and not yet released.

    When the encoder raises, the thread stops, and Phase says what `put` and `next_encoded` do
    after. The EncodeError says how many frames were not recorded: the one that failed and every
    frame after it. The frames encoded before it are still there to be recorded."""

    def __init__(self, encode, clock):
        self.stats = EncodeStats()
        self._encode, self._clock = encode, clock
        self._changed = threading.Condition()
        self._waiting = collections.deque()
        self._encoded = collections.deque()
        self._phase = Phase.RUNNING
        self._failure = None
        self._thread = threading.Thread(target=self._work, name="png-encoder", daemon=True)
        self._thread.start()

    def put(self, frame):
        with self._changed:
            self._waiting.append(frame)
            self._changed.notify_all()
            if self._over():
                started = self._clock()
                self.stats.backlog_waits += 1
                # Short waits: on Windows a Ctrl+C is not raised until a blocking wait returns.
                while self._over():
                    self._changed.wait(WAIT_SLICE_S)
                self.stats.backlog_wait_s += self._clock() - started
            if self._phase in (Phase.FAILED, Phase.REPORTED):
                raise self._report()

    def first_encoded(self):
        with self._changed:
            return self._encoded[0] if self._encoded else None

    def release_first(self):
        with self._changed:
            self._encoded.popleft()

    def finish(self):
        with self._changed:
            if self._phase is Phase.RUNNING:
                self._phase = Phase.CLOSING
                self._changed.notify_all()

    def next_encoded(self):
        """After `finish`: the oldest encoded frame not yet released, waiting while the thread
        encodes the next one. None once the thread has ended with no encoded frame left."""
        with self._changed:
            while not self._encoded and self._waiting and self._phase is Phase.CLOSING:
                self._changed.wait(WAIT_SLICE_S)
            if self._encoded:
                return self._encoded[0]
            if self._phase is Phase.FAILED:
                raise self._report()
        while self._thread.is_alive():
            self._thread.join(WAIT_SLICE_S)
        return None

    def unsaved(self):
        with self._changed:
            return len(self._waiting) + len(self._encoded)

    def _over(self):
        return len(self._waiting) > PNG_QUEUE_FRAMES and self._phase in (Phase.RUNNING, Phase.CLOSING)

    def _report(self):
        self._phase = Phase.REPORTED
        frame, error = self._failure
        lost = len(self._waiting)
        report = EncodeError(f"PNG encoding failed on the camera frame at {frame.time:.3f} s "
                             f"({type(error).__name__}: {error}); {lost} camera frames were not recorded: "
                             f"that one and every frame after it")
        report.__cause__ = error
        return report

    def _work(self):
        while True:
            with self._changed:
                while not self._waiting and self._phase is Phase.RUNNING:
                    self._changed.wait()
                if not self._waiting:
                    return
                frame = self._waiting[0]
            try:
                started = self._clock()
                png = self._encode(frame.data)
                elapsed = self._clock() - started
            except BaseException as error:
                with self._changed:
                    self._phase, self._failure = Phase.FAILED, (frame, error)
                    self._changed.notify_all()
                return
            with self._changed:
                self._waiting.popleft()
                self._encoded.append(frame._replace(data=png))
                self.stats.encoded += 1
                self.stats.encode_s += elapsed
                self._changed.notify_all()


@contextlib.contextmanager
def _ctrl_c_held():
    """Holds back a Ctrl+C that comes inside the block and raises it as soon as the block ends, so
    a frame's log and its release from the queue happen together or not at all. Python delivers
    Ctrl+C only to the main thread, so on any other thread the block just runs."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    held = []
    previous = signal.signal(signal.SIGINT, lambda signum, frame: held.append(signum))
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, previous)
        if held:
            signal.raise_signal(signal.SIGINT)


class RerunRecorder:
    """Without `save_path`: spawns a live viewer, shows every `live_stride`-th lidar point and
    passes camera images as they are. With `save_path`: the recording goes to that file only,
    with every lidar point, and each camera image is encoded by `encode_png` (lossless,
    `pillow_png` by default) on a PngWorker thread with a queue of PNG_QUEUE_FRAMES, timed by
    `clock`. `stats` says what the encoding took. The run's thread logs each encoded image at the
    time set when it was queued, at the next `set_time` or at `close`, in the order queued. `close`
    waits for every queued image, logs it and flushes the file. `arrays` does the array work
    (NumpyArrays by default)."""

    def __init__(self, save_path=None, live_stride=1, *, clock, arrays=None, encode_png=None):
        import rerun

        self._rr = rerun
        self._arrays = arrays or NumpyArrays()
        self._stride = live_stride if save_path is None else 1
        self._time = None
        self._png = None
        self.stats = EncodeStats()
        if save_path is None:
            rerun.init("kite_lidar", spawn=True)  # ASSUMPTION: init(app_id, spawn=True) spawns and connects a viewer
        else:
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            rerun.init("kite_lidar")
            rerun.save(str(save_path))  # ASSUMPTION: save(path) sends the recording to a file instead of a viewer
        rerun.log(PROVENANCE_PATH, rerun.TextLog(PROVENANCE))  # ASSUMPTION: TextLog(text) is a log entry that takes one string
        self._send_layout()
        if save_path is not None:
            self._png = PngWorker(encode_png or pillow_png, clock)
            self.stats = self._png.stats

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
        self._log_encoded()
        self._time = seconds
        self._set_time(seconds)

    def _set_time(self, seconds):
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
        if self._png is None:
            # ASSUMPTION: Image takes a (height, width, 3 or 4) uint8 array
            self._rr.log(path, self._rr.Image(self._arrays.image(image)))
            return
        self._png.put(Frame(path, self._time, self._arrays.image(image, copy=True)))

    def _log_encoded(self, wait=False):
        if self._png is None:
            return
        take = self._png.next_encoded if wait else self._png.first_encoded
        while (frame := take()) is not None:
            with _ctrl_c_held():
                # ASSUMPTION: an image logged after later data, at its own earlier time, sits at that time on the timeline
                self._set_time(frame.time)
                try:
                    # ASSUMPTION: EncodedImage(contents=, media_type=) logs encoded image bytes that the viewer decodes
                    self._rr.log(frame.path, self._rr.EncodedImage(contents=frame.data, media_type=PNG_MEDIA_TYPE))
                except BaseException as error:
                    # ASSUMPTION: a log call that raises stored nothing, so the frame stays queued and is logged
                    # again; Ctrl+C is held back here, so only an error of Rerun's own can raise inside it
                    print(f"camera frame at {frame.time:.3f} s: Rerun's log raised {type(error).__name__}; the frame "
                          "stays queued and is logged again, on the rule that a log that raises stored nothing",
                          file=sys.stderr)
                    raise
                self._png.release_first()

    def close(self):
        """Waits for every queued image, logs each as it is encoded and flushes the file. If an
        exception ends that early, a second Ctrl+C for example, prints how many camera frames were
        not saved. An EncodeError is the exception: its message already counts them."""
        try:
            try:
                if self._png is not None:
                    self._png.finish()
                    self._log_encoded(wait=True)
            finally:
                self._rr.disconnect()  # ASSUMPTION: disconnect() flushes and closes the file or connection
        except EncodeError:
            raise
        except BaseException as error:
            unsaved = 0 if self._png is None else self._png.unsaved()
            if unsaved:
                why = "Ctrl+C" if isinstance(error, KeyboardInterrupt) else type(error).__name__
                print(f"{unsaved} camera frames were not saved: {why} while the recording closed", file=sys.stderr)
            raise


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
