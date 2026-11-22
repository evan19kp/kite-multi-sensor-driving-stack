import contextlib
import importlib
import io
import unittest

from tests.test_self_hits import run_script
from kite_bridge.sensor_settings import settings_for, settings_lines, settings_table
from kite_bridge.sim_api import SENSOR_KINDS

DRAWABLE = ("lidar", "radar", "gps", "imu")


def expected_drawing(flag):
    return {kind: flag and kind in DRAWABLE for kind in SENSOR_KINDS}


def line_values(stdout):
    lines = [line for line in stdout.splitlines() if line.startswith("sensor:")]
    return {line.split()[1]: line for line in lines}


class SettingsTableTest(unittest.TestCase):
    def test_every_sensor_is_created_with_drawing_off_by_default(self):
        for kind in SENSOR_KINDS:
            self.assertIs(settings_for(kind)["is_visualised"], False, kind)
            self.assertIs(settings_for(kind, stream=True, lidar_pos=(0, 0, 2))["is_visualised"], False, kind)

    def test_the_flag_turns_drawing_on_for_the_four_that_support_it_and_never_for_the_camera(self):
        for kind in SENSOR_KINDS:
            self.assertIs(settings_for(kind, visualise=True)["is_visualised"], kind in DRAWABLE, kind)
            self.assertIs(settings_for(kind, stream=True, visualise=True)["is_visualised"], kind in DRAWABLE, kind)

    def test_the_flag_changes_no_other_setting(self):
        for kind in SENSOR_KINDS:
            on, off = settings_for(kind, visualise=True), settings_for(kind)
            self.assertEqual({k: v for k, v in on.items() if k != "is_visualised"},
                             {k: v for k, v in off.items() if k != "is_visualised"}, kind)

    def test_the_printed_lines_show_the_value_for_all_five(self):
        for flag in (False, True):
            by_kind = line_values("\n".join(settings_lines(visualise=flag)))
            self.assertEqual(list(by_kind), list(SENSOR_KINDS))
            for kind, drawn in expected_drawing(flag).items():
                self.assertIn(f" is_visualised={drawn}", by_kind[kind], (kind, flag))
        self.assertEqual({kind: s["is_visualised"] for kind, s in settings_table(visualise=True).items()},
                         expected_drawing(True))


class ScriptsTest(unittest.TestCase):
    def runs(self):
        import spike_lidar_rerun
        import spike_multicar
        import spike_sensors

        return ((spike_sensors, (), False), (spike_multicar, (), False),
                (spike_lidar_rerun, ("--mode", "lockstep"), False), (spike_lidar_rerun, (), True))

    def check(self, flag):
        extra_flag = ("--visualise-sensors",) if flag else ()
        for module, extra, free in self.runs():
            name = module.__name__, flag, extra
            code, stdout, stderr, sims, saved = run_script(module, *extra_flag, *extra, free=free)
            self.assertEqual(code, 0, (name, stderr))
            self.assertTrue(sims, name)
            for sim in sims:
                self.assertTrue(sim.sensor_settings, name)
                for settings in sim.sensor_settings.values():
                    self.assertEqual({k: s["is_visualised"] for k, s in settings.items()}, expected_drawing(flag), name)
            by_kind = line_values(stdout)
            self.assertEqual(list(by_kind), list(SENSOR_KINDS), name)
            for kind, drawn in expected_drawing(flag).items():
                self.assertIn(f" is_visualised={drawn} ", by_kind[kind] + " ", (name, kind))
            for data in saved.values():
                self.assertEqual({k: s["is_visualised"] for k, s in data["sensor_settings"].items()},
                                 expected_drawing(flag), name)

    def test_by_default_the_fake_receives_drawing_off_and_the_lines_and_json_show_it(self):
        self.check(False)

    def test_with_the_flag_the_four_are_drawn_and_the_camera_is_not(self):
        self.check(True)

    def test_the_scripts_that_write_json_put_the_value_in_it(self):
        import spike_multicar
        import spike_sensors

        for module, name in ((spike_sensors, "sensors.json"), (spike_multicar, "multicar.json")):
            _, _, _, _, saved = run_script(module, "--visualise-sensors")
            self.assertEqual({k: s["is_visualised"] for k, s in saved[name]["sensor_settings"].items()},
                             expected_drawing(True), name)

    def test_the_flag_combines_with_stream_and_the_lidar_mount(self):
        import spike_sensors

        code, stdout, stderr, sims, _ = run_script(spike_sensors, "--visualise-sensors", "--stream",
                                                   "--lidar-pos", "0,0,2.1")
        self.assertEqual(code, 0, stderr)
        lidar = sims[0].sensor_settings["kite1"]["lidar"]
        self.assertEqual((lidar["is_visualised"], lidar["is_streaming"], lidar["pos"]), (True, True, (0, 0, 2.1)))
        self.assertTrue(line_values(stdout)["lidar"].endswith(" is_streaming=True"))

    def test_the_help_names_the_flag_on_the_scripts_that_create_sensors(self):
        for name in ("spike_sensors", "spike_multicar", "spike_lidar_rerun"):
            stdout = io.StringIO()
            with self.assertRaises(SystemExit), contextlib.redirect_stdout(stdout):
                importlib.import_module(name).main(["--help"])
            self.assertIn("--visualise-sensors", stdout.getvalue(), name)

    def test_the_connect_script_creates_no_sensors_so_it_has_no_flag(self):
        import spike_connect

        with self.assertRaises(SystemExit) as raised, contextlib.redirect_stderr(io.StringIO()):
            run_script(spike_connect, "--visualise-sensors")
        self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
