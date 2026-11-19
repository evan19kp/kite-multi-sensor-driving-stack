"""Shows one simulator run in Rerun.

This is the only module that imports `rerun`, and it does so lazily, inside `RerunRecorder`, so
importing the module needs no installed package. Every Rerun call lives in `RerunRecorder`, and
none has run against an installed Rerun yet. Calls whose name or arguments are guesses carry an
`# ASSUMPTION:` comment; the first real run checks them.

`log_step` takes any recorder with `set_time`, `log_points`, `log_image` and `close`, so tests
pass a fake and never need Rerun.

Entity paths: sensor layers sit under `world/` and `camera/`. Other cars' positions are
simulator ground truth, shown only for display, and sit under `ground_truth/`, which can be
hidden in the viewer without touching a sensor layer. They never go to anything that drives.
"""

import math
from pathlib import Path, PurePosixPath, PureWindowsPath

LIDAR_PATH = "world/ego/lidar"
EGO_PATH = "world/ego/position"
CAMERA_PATH = "camera/image"
GROUND_TRUTH_PATH = "ground_truth/other_cars"
TIMELINE = "sim_time"
FAR_M = 50.0
EGO_COLOUR = (255, 255, 255)
GROUND_TRUTH_COLOUR = (255, 165, 0)
RECORDING_SUFFIX = ".rrd"


def distance_colour(distance_m):
    """RGB for a lidar point `distance_m` metres from our car: blue at 0 m, red at FAR_M (50 m)
    or more, blending linearly between. Red never falls and blue never rises as the distance
    grows. Domain: a finite distance >= 0; anything else raises ValueError."""
    if not math.isfinite(distance_m) or distance_m < 0:
        raise ValueError(f"distance must be a finite number >= 0, got {distance_m}")
    far = round(255 * min(distance_m / FAR_M, 1.0))
    return (far, 0, 255 - far)


def log_step(recorder, sim_time, ego, lidar_points, image, others=None):
    """Logs one simulator step: the time, then the lidar points coloured by distance from
    `ego`, `ego`, and the camera `image`. `others` is the traffic's ground truth, or None when
    it is unavailable; a list, even an empty one, is logged under GROUND_TRUTH_PATH only."""
    recorder.set_time(sim_time)
    recorder.log_points(LIDAR_PATH, lidar_points,
                        [distance_colour(math.dist(point, ego)) for point in lidar_points])
    recorder.log_points(EGO_PATH, [ego], [EGO_COLOUR])
    recorder.log_image(CAMERA_PATH, image)
    if others is not None:
        recorder.log_points(GROUND_TRUTH_PATH, others, [GROUND_TRUTH_COLOUR] * len(others))


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
    """Writes to a spawned Rerun viewer, or to `save_path` (a file) when one is given.
    Giving a path turns the live viewer off: the recording goes to the file only."""

    def __init__(self, save_path=None):
        import numpy
        import rerun

        self._rr = rerun
        self._numpy = numpy
        if save_path is None:
            rerun.init("kite_lidar", spawn=True)  # ASSUMPTION: init(app_id, spawn=True) spawns and connects a viewer
        else:
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            rerun.init("kite_lidar")
            rerun.save(str(save_path))  # ASSUMPTION: save(path) sends the recording to a file instead of a viewer

    def set_time(self, seconds):
        if hasattr(self._rr, "set_time"):
            self._rr.set_time(TIMELINE, duration=seconds)  # ASSUMPTION: set_time(name, duration=) in newer SDKs
        else:
            self._rr.set_time_seconds(TIMELINE, seconds)  # ASSUMPTION: set_time_seconds(name, s) in older SDKs

    def log_points(self, path, positions, colors):
        # ASSUMPTION: Points3D takes (n, 3) float positions and (n, 3) uint8 colors
        self._rr.log(path, self._rr.Points3D(
            self._numpy.asarray(positions, dtype="float32").reshape(-1, 3),
            colors=self._numpy.asarray(colors, dtype="uint8").reshape(-1, 3)))

    def log_image(self, path, image):
        self._rr.log(path, self._rr.Image(self._numpy.asarray(image, dtype="uint8")))  # ASSUMPTION: Image takes a (h, w, 3) uint8 array

    def close(self):
        self._rr.disconnect()  # ASSUMPTION: disconnect() flushes and closes the file or connection


class NullRecorder:
    """Takes every call and keeps nothing. `--fake` uses it so no viewer or file is made."""

    def set_time(self, seconds):
        pass

    def log_points(self, path, positions, colors):
        pass

    def log_image(self, path, image):
        pass

    def close(self):
        pass
