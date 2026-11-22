import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import rerun_sink

NEW_RERUN = ("init", "save", "log", "Points3D", "disconnect", "set_time")
OLD_RERUN = ("init", "save", "log", "Points3D", "disconnect", "set_time_seconds")


def fake_rerun(api=NEW_RERUN):
    fake = types.ModuleType("rerun")
    for name in api:
        setattr(fake, name, mock.Mock(name=name))
    return fake


def sink_for(fake, save_path=None):
    with mock.patch.dict(sys.modules, {"rerun": fake}):
        sink = rerun_sink.RerunSink(save_path)
    return sink


class RerunSinkTest(unittest.TestCase):
    def test_without_a_save_path_it_spawns_a_viewer_and_saves_nothing(self):
        fake = fake_rerun()
        sink_for(fake)
        fake.init.assert_called_once_with("kite_log", spawn=True)
        fake.save.assert_not_called()

    def test_with_a_save_path_it_records_to_that_file_and_spawns_no_viewer(self):
        fake = fake_rerun()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "out" / "day1" / "demo.rrd"
            sink_for(fake, path)
            self.assertTrue(path.parent.is_dir())
            self.assertFalse(path.exists())
        fake.init.assert_called_once_with("kite_log")
        fake.save.assert_called_once_with(str(path))

    def test_these_tests_leave_no_rerun_in_sys_modules(self):
        sink_for(fake_rerun())
        self.assertNotIn("rerun", sys.modules)

    def test_time_uses_set_time_when_rerun_has_it(self):
        fake = fake_rerun(NEW_RERUN)
        sink_for(fake).set_time(1.5)
        fake.set_time.assert_called_once_with("t", duration=1.5)

    def test_time_falls_back_to_set_time_seconds_on_an_older_rerun(self):
        fake = fake_rerun(OLD_RERUN)
        sink_for(fake).set_time(1.5)
        fake.set_time_seconds.assert_called_once_with("t", 1.5)

    def test_points_reach_rerun_as_points3d_with_colours_and_labels(self):
        fake = fake_rerun()
        positions = [(1.0, 2.0, 0.0), (3.0, 4.0, 0.0)]
        colours = [(255, 0, 0), (0, 255, 0)]
        sink_for(fake).log_points("tracks", positions, colours, ["7", "9"])
        fake.Points3D.assert_called_once_with(positions, colors=colours, labels=["7", "9"])
        fake.log.assert_called_once_with("tracks", fake.Points3D.return_value)

    def test_no_labels_reach_rerun_as_none_and_an_empty_step_as_an_empty_list(self):
        fake = fake_rerun()
        sink_for(fake).log_points("measurements", [], [], None)
        fake.Points3D.assert_called_once_with([], colors=[], labels=None)
        fake.log.assert_called_once_with("measurements", fake.Points3D.return_value)

    def test_close_disconnects(self):
        fake = fake_rerun()
        sink = sink_for(fake)
        fake.disconnect.assert_not_called()
        sink.close()
        fake.disconnect.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
