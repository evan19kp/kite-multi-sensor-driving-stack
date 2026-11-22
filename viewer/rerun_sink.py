"""The only module in `viewer/` that imports `rerun`, and only inside `RerunSink.__init__`."""

from pathlib import Path

APP_ID = "kite_log"
TIMELINE = "t"


class RerunSink:
    """Without `save_path`: spawns a live viewer. With it: the recording goes to that file only."""

    def __init__(self, save_path=None):
        import rerun

        self._rr = rerun
        if save_path is None:
            rerun.init(APP_ID, spawn=True)  # ASSUMPTION: init(app_id, spawn=True) spawns and connects a viewer
        else:
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            rerun.init(APP_ID)
            rerun.save(str(save_path))  # ASSUMPTION: save(path) sends the recording to a file instead of a viewer

    def set_time(self, seconds):
        if hasattr(self._rr, "set_time"):
            self._rr.set_time(TIMELINE, duration=seconds)  # ASSUMPTION: set_time(name, duration=) in newer SDKs
        else:
            self._rr.set_time_seconds(TIMELINE, seconds)  # ASSUMPTION: set_time_seconds(name, s) in older SDKs

    def log_points(self, path, positions, colors, labels):
        # ASSUMPTION: Points3D takes plain lists of (x, y, z) positions and (r, g, b) colors, and
        # labels= takes one string per point or None; an empty list of positions clears the entity
        # at that time. Not yet run: labels= and the empty list are new here.
        self._rr.log(path, self._rr.Points3D(positions, colors=colors, labels=labels))

    def close(self):
        self._rr.disconnect()  # ASSUMPTION: disconnect() flushes and closes the file or connection
