"""Every setting the adapter passes to a sensor, in one table.

ASSUMPTION: the setting names, and the values of the lidar and radar settings that are
BeamNGpy 1.36's own defaults, were read from BeamNGpy 1.36; the first real run checks them.
"""

from kite_bridge.sim_api import STREAMED_KINDS

FORWARD = (0, -1, 0)
UP = (0, 0, 1)

SENSOR_SETTINGS = {
    "lidar": {
        "requested_update_time": 0.1,
        "pos": (0, 0, 1.9),
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


STREAM_SETTINGS = {"is_streaming": True}


DEFAULT_LIDAR_POS = SENSOR_SETTINGS["lidar"]["pos"]


def settings_for(kind, stream=False, lidar_pos=None):
    """What the adapter passes to a `kind` sensor: SENSOR_SETTINGS[kind], and with `stream` also
    STREAM_SETTINGS for the streamed sensors (lidar and camera). A `lidar_pos` (x, y, z in
    vehicle space, metres) replaces the lidar's `pos`; it changes no other sensor. Without
    `stream` and `lidar_pos` it is SENSOR_SETTINGS[kind] itself."""
    settings = SENSOR_SETTINGS[kind]
    if lidar_pos is not None and kind == "lidar":
        settings = {**settings, "pos": lidar_pos}
    if stream and kind in STREAMED_KINDS:
        return {**settings, **STREAM_SETTINGS}
    return settings


def settings_table(stream=False, lidar_pos=None):
    return {kind: settings_for(kind, stream, lidar_pos) for kind in SENSOR_SETTINGS}


def requested_update_hz(kind):
    """The update rate the adapter asks for, 1 / requested_update_time. None for a sensor with no
    such setting (GPS and IMU): no requested rate is known for it."""
    interval = SENSOR_SETTINGS[kind].get("requested_update_time")
    return None if interval is None else 1 / interval


def buffered_update_hz(kind):
    """How many readings a buffered sensor (GPS, IMU) makes per simulated second, 1 /
    physics_update_time, the setting the adapter passes for how often it measures: 100 for the IMU.
    None for the GPS, which is passed no such setting, so its own rate is not known.
    ASSUMPTION: the IMU buffers one reading per physics_update_time; its gfx_update_time is the
    same 0.01 s, so either setting gives 100."""
    interval = SENSOR_SETTINGS[kind].get("physics_update_time")
    return None if interval is None else 1 / interval


def settings_lines(stream=False, lidar_pos=None):
    return [f"sensor: {kind} " + " ".join(f"{name}={value}" for name, value in settings.items())
            for kind, settings in settings_table(stream, lidar_pos).items()]
