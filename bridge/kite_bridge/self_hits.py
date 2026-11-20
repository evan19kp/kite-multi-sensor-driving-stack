"""An approximate count of the lidar points that hit our own car.

A lidar point is a self-hit, approximately, when it is at most `radius_m` metres from our car in
x and y, the distance on the ground. The car's position is the x, y of its latest GPS reading,
so the count uses sensor data only: it never asks the simulator for our car's position, adds no
request, and nothing here is ground truth. The numbers are printed and written to the JSON,
never logged under `world/` and never an input to anything that drives.

What the number measures: points inside a vertical cylinder around the GPS position, per new
scan. What it cannot say:
- It is not the car's shape. A box or a car-shaped filter comes with perception. Ground and other
  objects inside the cylinder count too, so a radius smaller than where the lowest beams reach
  the ground keeps those out.
- The GPS sits at its own mount, `sensor_settings.py`'s GPS `pos`, not at the car's origin, so
  the cylinder is centred up to that far from the car's middle.
- The GPS is read less often than the lidar, so its position can be older than a scan. A car
  driving at speed v drives v times that age before the scan is counted, so the cylinder lags the
  car. This is inference, not a measurement. Points of the car at the front can leave the
  cylinder and points at the rear can enter it, so the count can be off in either direction.
- The points and the GPS position must be in one frame. The lidar points are assumed to be in the
  world frame and the GPS x, y in the same one (see `readings.points_from_raw` and
  `rerun_view.gps_position`); the first real run shows whether they are.
- The height is ignored, so it cannot tell a roof hit from a ground hit.

A scan read before the first GPS position, or while the latest GPS reading has no position, is
not counted and is reported as such. A scan the counter cannot read is counted apart too.
"""

import math

from kite_bridge.rerun_view import gps_position

DEFAULT_EGO_RADIUS_M = 2.5


class SelfHits:
    """Counts the self-hits of each new lidar scan, per car. `arrays` is a `rerun_view` arrays
    object: `NumpyArrays` for a real run (one array operation per scan, no loop over points) and
    `ListArrays` for the fake and tests (one point at a time). `lidar_pos` is only reported."""

    def __init__(self, radius_m, lidar_pos, arrays):
        self.radius_m = radius_m
        self.lidar_pos = tuple(lidar_pos)
        self._arrays = arrays
        self._position = {}
        self._counts = {}
        self._no_position = {}
        self._unreadable = {}

    def observe(self, car, kind, raw):
        """Takes one new reading of `car`'s `kind`. A GPS reading sets the position (to None when
        its latest reading has none); a lidar `raw` is a dict with "pointCloud" and is counted."""
        if kind == "gps":
            self._position[car] = gps_position(raw)
        elif kind == "lidar":
            self._scan(car, raw["pointCloud"])

    def _scan(self, car, cloud):
        centre = self._position.get(car)
        if centre is None:
            self._no_position[car] = self._no_position.get(car, 0) + 1
            return
        try:
            points = self._arrays.points(cloud)
            self._counts.setdefault(car, []).append(self._arrays.count_within(points, centre, self.radius_m))
        except (ValueError, TypeError, IndexError):
            self._unreadable[car] = self._unreadable.get(car, 0) + 1

    def summary(self, car):
        counts = self._counts.get(car, [])
        return {"radius_m": self.radius_m, "lidar_pos": list(self.lidar_pos), "scans_counted": len(counts),
                "mean": math.fsum(counts) / len(counts) if counts else None,
                "max": max(counts, default=None),
                "scans_without_position": self._no_position.get(car, 0),
                "scans_unreadable": self._unreadable.get(car, 0)}
