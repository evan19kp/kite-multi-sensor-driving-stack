"""Every setting the adapter passes to a sensor, in one table.

ASSUMPTION: the setting names, and the values of the lidar and radar settings that are
BeamNGpy 1.36's own defaults, were read from BeamNGpy 1.36; the first real run checks them.
"""

FORWARD = (0, -1, 0)
UP = (0, 0, 1)

SENSOR_SETTINGS = {
    "lidar": {
        "requested_update_time": 0.1,
        "pos": (0, 0, 1.7),
        "vertical_resolution": 64,
        "frequency": 20,
        "horizontal_angle": 360,
        "max_distance": 120,
        "is_360_mode": True,
        "is_using_shared_memory": True,
    },
    "camera": {
        "requested_update_time": 0.05,
        "resolution": (640, 360),
        "field_of_view_y": 70,
        "pos": (0, -0.5, 1.5),
        "dir": FORWARD,
        "is_render_colours": True,
        "is_render_annotations": False,
        "is_render_depth": False,
        "is_using_shared_memory": True,
    },
    "radar": {
        "requested_update_time": 0.1,
        "pos": (0, 0, 1.7),
        "dir": FORWARD,
        "up": UP,
        "range_min": 0.1,
        "range_max": 100.0,
        "half_angle_deg": 30.0,
        "field_of_view_y": 70,
    },
    "gps": {
        "pos": (0, 1.5, 2.0),
        "ref_lon": 0.0,
        "ref_lat": 0.0,
    },
    "imu": {
        "pos": (0, 0, 0.5),
        "dir": FORWARD,
        "up": UP,
        "gfx_update_time": 0.01,
        "physics_update_time": 0.01,
    },
}


def requested_update_hz(kind):
    """The update rate the adapter asks for, 1 / requested_update_time. None for a sensor with no
    such setting (GPS and IMU): no requested rate is known for it."""
    interval = SENSOR_SETTINGS[kind].get("requested_update_time")
    return None if interval is None else 1 / interval


def settings_lines():
    return [f"sensor: {kind} " + " ".join(f"{name}={value}" for name, value in settings.items())
            for kind, settings in SENSOR_SETTINGS.items()]
