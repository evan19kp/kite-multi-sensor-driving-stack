"""Shows one simulator run in Rerun. Sensor layers (lidar, camera) have their own roots. Our car's
and the other cars' positions come from simulator state, so they are ground truth: for display
only and always under `ground_truth/`, which is what separates them from sensor data."""

import math
from pathlib import Path, PurePosixPath, PureWindowsPath

from kite_bridge.output_guard import PROVENANCE

PROVENANCE_PATH = "meta/provenance"
LIDAR_PATH = "world/ego/lidar"
CAMERA_PATH = "camera/image"
EGO_PATH = "ground_truth/ego/position"
GROUND_TRUTH_PATH = "ground_truth/other_cars"
TIMELINE = "sim_time"
FAR_M = 50.0
EGO_COLOUR = (255, 255, 255)
GROUND_TRUTH_COLOUR = (255, 165, 0)
RECORDING_SUFFIX = ".rrd"


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
        recorder.log_points(LIDAR_PATH, lidar_points,
                            [distance_colour(math.dist(point, ego)) for point in lidar_points])
    recorder.log_points(EGO_PATH, [ego], [EGO_COLOUR])
    if image is not None:
        recorder.log_image(CAMERA_PATH, image)
    if others is not None:
        recorder.log_points(GROUND_TRUTH_PATH, others, [GROUND_TRUTH_COLOUR] * len(others))


def log_reading(recorder, kind, reading):
    """Free mode: logs one `reading` (a `sim_api.Reading` with a timestamp) of `kind` "lidar" or
    "camera" at its time, which is the Timer's for a reading without its own. The lidar's
    "pointCloud" goes under LIDAR_PATH and the camera's "colour" image under CAMERA_PATH as the
    simulator gave them, with no colours and no ground truth. Returns True when it logged the
    reading. A camera reading without a "colour" image logs nothing and returns False."""
    raw = reading.raw
    if kind == "lidar":
        recorder.set_time(reading.timestamp)
        recorder.log_points(LIDAR_PATH, raw["pointCloud"], None)
        return True
    image = raw.get("colour") if isinstance(raw, dict) else None
    if image is None:
        return False
    recorder.set_time(reading.timestamp)
    recorder.log_image(CAMERA_PATH, image)
    return True


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


class RerunRecorder:
    """Giving `save_path` turns the live viewer off: the recording goes to that file only."""

    def __init__(self, save_path=None):
        import rerun

        self._rr = rerun
        if save_path is None:
            rerun.init("kite_lidar", spawn=True)  # ASSUMPTION: init(app_id, spawn=True) spawns and connects a viewer
        else:
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            rerun.init("kite_lidar")
            rerun.save(str(save_path))  # ASSUMPTION: save(path) sends the recording to a file instead of a viewer
        rerun.log(PROVENANCE_PATH, rerun.TextLog(PROVENANCE))  # ASSUMPTION: TextLog(text) is a log entry that takes one string

    def set_time(self, seconds):
        if hasattr(self._rr, "set_time"):
            self._rr.set_time(TIMELINE, duration=seconds)  # ASSUMPTION: set_time(name, duration=) in newer SDKs
        else:
            self._rr.set_time_seconds(TIMELINE, seconds)  # ASSUMPTION: set_time_seconds(name, s) in older SDKs

    def log_points(self, path, positions, colors):
        # ASSUMPTION: Points3D takes plain lists of (x, y, z) positions and (r, g, b) colors, and in free
        # mode a BeamNGpy pointCloud array of x, y, z rows with colors=None
        self._rr.log(path, self._rr.Points3D(positions, colors=colors))

    def log_image(self, path, image):
        # ASSUMPTION: Image takes rows of (r, g, b) tuples as nested lists, and in free mode the camera's
        # "colour" image as BeamNGpy gives it (a PIL image or an array of rows)
        self._rr.log(path, self._rr.Image(image))

    def close(self):
        self._rr.disconnect()  # ASSUMPTION: disconnect() flushes and closes the file or connection


class NullRecorder:
    def set_time(self, seconds):
        pass

    def log_points(self, path, positions, colors):
        pass

    def log_image(self, path, image):
        pass

    def close(self):
        pass
