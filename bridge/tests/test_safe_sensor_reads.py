import ast
import contextlib
import io
import itertools
import json
import re
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tests import BRIDGE_DIR
from tests.fake_array import FakeArray
from tests.test_car_and_versions import LOCKSTEP_ARGS
from tests.test_scripts import run_script
from kite_bridge import common
from kite_bridge.fake_sim import FakeSim
from kite_bridge.readings import reading_from_raw
from kite_bridge.sensor_settings import SENSOR_SETTINGS, settings_for
from kite_bridge.sim_api import SENSOR_KINDS, STEP_HZ

SIMULATED_SECONDS = 1.0


def array_polls(radar_rows, lidar_rows):
    class ArrayPolls(FakeSim):
        def poll(self, car, kind):
            if kind == "radar":
                return reading_from_raw(kind, FakeArray(radar_rows, 7), self.sim_time)
            if kind == "lidar":
                raw = {"pointCloud": FakeArray(lidar_rows, 3), "colours": FakeArray(lidar_rows, 3)}
                return reading_from_raw(kind, raw, self.sim_time)
            return super().poll(car, kind)

    return ArrayPolls()


def measured_rows(sim):
    cars = sim.load_scenario("fake_cherrier_tograc", 1)
    result = common.measure_sensors(sim, cars, SIMULATED_SECONDS)
    return {r["sensor"]: r for r in result["sensors"]}, result


class ArrayPollTest(unittest.TestCase):
    def test_radar_and_lidar_arrays_are_read_and_counted(self):
        rows, result = measured_rows(array_polls(radar_rows=6, lidar_rows=250))
        self.assertEqual(rows["radar"]["polls"], STEP_HZ)
        self.assertEqual(rows["radar"]["detections_mean"], 6)
        self.assertEqual(rows["radar"]["unexpected"]["count"], 0)
        self.assertEqual(rows["lidar"]["points_mean"], 250)
        self.assertEqual(rows["lidar"]["unexpected"]["count"], 0)
        table = common.format_result(result)
        self.assertRegex(table, r"(?m)^kite1\s+radar\s.*\s6$")
        self.assertRegex(table, r"(?m)^kite1\s+lidar\s.*\s250\s+n/a$")

    def test_a_radar_array_with_no_rows_is_a_reading_and_a_lidar_scan_with_no_points_is_empty(self):
        rows, _ = measured_rows(array_polls(radar_rows=0, lidar_rows=0))
        self.assertEqual(rows["radar"]["polls"], STEP_HZ)
        self.assertEqual(rows["radar"]["empty"], 0)
        self.assertEqual(rows["radar"]["detections_mean"], 0)
        self.assertEqual(rows["radar"]["unexpected"]["count"], 0)
        self.assertEqual(rows["lidar"]["polls"], STEP_HZ)
        self.assertEqual(rows["lidar"]["empty"], STEP_HZ)
        self.assertEqual(rows["lidar"]["unexpected"]["count"], 0)
        self.assertIsNone(rows["lidar"]["points_mean"])

    def test_detections_mean_counts_the_zero_row_polls(self):
        row_cycle = itertools.cycle((0, 2, 7))

        class VaryingRadar(FakeSim):
            def poll(self, car, kind):
                if kind == "radar":
                    return reading_from_raw(kind, FakeArray(next(row_cycle), 7), self.sim_time)
                return super().poll(car, kind)

        rows, _ = measured_rows(VaryingRadar())
        # 60 polls cycle 0, 2, 7 rows twenty times each: (20 * 0 + 20 * 2 + 20 * 7) / 60 = 180 / 60 = 3.
        # Dropping the zero-row polls would give (20 * 2 + 20 * 7) / 40 = 4.5 instead.
        self.assertEqual(rows["radar"]["detections_mean"], 3)
        self.assertEqual(rows["radar"]["empty"], 0)
        self.assertEqual(rows["radar"]["polls"], STEP_HZ)


class BadSensorTest(unittest.TestCase):
    def run_sensors(self, sim_class):
        out, err = io.StringIO(), io.StringIO()
        with tempfile.TemporaryDirectory() as tmp:
            import spike_sensors

            with (mock.patch.object(common, "FakeSim", sim_class), contextlib.redirect_stdout(out),
                  contextlib.redirect_stderr(err)):
                code = spike_sensors.main(["--fake", "--seconds", "1", "--rates", "all=60", "--out", tmp])
            saved = json.loads((Path(tmp) / "sensors.json").read_text(encoding="utf-8"))
        return code, out.getvalue(), err.getvalue(), {r["sensor"]: r for r in saved["sensors"]}

    def check_only_radar_is_unexpected(self, sim_class, first):
        code, stdout, stderr, rows = self.run_sensors(sim_class)
        self.assertEqual(code, 0, stderr)
        self.assertEqual(rows["radar"]["unexpected"], {"count": STEP_HZ, "first": first})
        self.assertEqual(rows["radar"]["polls"], 0)
        self.assertEqual(rows["radar"]["empty"], 0)
        for kind in ("lidar", "camera", "gps", "imu"):
            self.assertEqual(rows[kind]["unexpected"], {"count": 0, "first": None}, kind)
            self.assertEqual(rows[kind]["polls"], STEP_HZ, kind)
            self.assertRegex(stdout, rf"(?m)^kite1\s+{kind}\s+{STEP_HZ}\s", kind)
        self.assertRegex(stdout, rf"(?m)^kite1\s+radar\s+0\s.*\s{STEP_HZ}\s", "radar")
        self.assertIn(f"unexpected format: kite1 radar {STEP_HZ} readings, first: {first}", stdout)

    def test_a_poll_that_raises_is_counted_and_the_other_sensors_carry_on(self):
        class RaisingRadar(FakeSim):
            def poll(self, car, kind):
                if kind == "radar":
                    raise RuntimeError(f"radar broke at {self.sim_time:.2f}")
                return super().poll(car, kind)

        self.check_only_radar_is_unexpected(RaisingRadar, f"poll raised RuntimeError: radar broke at {1 / STEP_HZ:.2f}")

    def test_the_error_of_a_numpy_like_truth_test_is_not_a_crash(self):
        class TruthTestingRadar(FakeSim):
            def poll(self, car, kind):
                if kind == "radar":
                    if FakeArray(3, 7):
                        pass
                return super().poll(car, kind)

        self.check_only_radar_is_unexpected(
            TruthTestingRadar,
            "poll raised ValueError: The truth value of an array with more than one element is ambiguous")

    def test_a_poll_outside_the_input_domain_is_counted_with_its_type(self):
        class WrongType(FakeSim):
            def poll(self, car, kind):
                if kind == "radar":
                    return reading_from_raw(kind, 42, self.sim_time)
                return super().poll(car, kind)

        self.check_only_radar_is_unexpected(WrongType, "unexpected poll result: got int")


class SettingsLinesTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name)

    def sensor_lines(self, script, *extra):
        code, stdout, stderr = run_script(script, "--fake", "--seconds", "1", "--out", str(self.out), *extra)
        self.assertEqual(code, 0, stderr)
        lines = [line for line in stdout.splitlines() if line.startswith("sensor:")]
        return {line.split()[1]: line for line in lines}, stdout.splitlines()

    def test_every_script_prints_one_line_per_sensor_before_the_run(self):
        for script in ("spike_connect", "spike_sensors", "spike_multicar", "spike_lidar_rerun"):
            by_kind, lines = self.sensor_lines(script, *LOCKSTEP_ARGS.get(script, ()))
            self.assertEqual(list(by_kind), list(SENSOR_KINDS), script)
            spawned = next(i for i, line in enumerate(lines) if line.startswith("spawned"))
            for line in by_kind.values():
                self.assertLess(lines.index(line), spawned, script)

    def test_a_multi_case_script_prints_the_settings_once(self):
        _, lines = self.sensor_lines("spike_multicar")
        self.assertEqual(sum(line.startswith("sensor:") for line in lines), len(SENSOR_KINDS))

    def test_the_lidar_and_radar_lines_pass_every_listed_setting(self):
        by_kind, _ = self.sensor_lines("spike_sensors")
        for setting in ("vertical_resolution=64", "frequency=20", "horizontal_angle=360", "max_distance=120",
                        "pos=(0, 0, 1.7)", "requested_update_time=0.1"):
            self.assertIn(f" {setting}", by_kind["lidar"] + " ")
        for setting in ("requested_update_time=0.1", "pos=(0, 0, 1.7)", "range_min=0.1", "range_max=100.0",
                        "half_angle_deg=30.0", "field_of_view_y=70"):
            self.assertIn(f" {setting}", by_kind["radar"] + " ")

    def test_the_camera_line_says_resolution_view_position_direction_update_time_and_colour_only(self):
        by_kind, _ = self.sensor_lines("spike_sensors")
        camera = by_kind["camera"] + " "
        for setting in ("resolution=(640, 360)", "field_of_view_y=70", "pos=(0, -0.5, 1.5)", "dir=(0, -1, 0)",
                        "requested_update_time=0.05", "is_render_colours=True", "is_render_annotations=False",
                        "is_render_depth=False"):
            self.assertIn(f" {setting} ", camera)

    def test_the_json_holds_the_same_settings_as_the_lines(self):
        by_kind, _ = self.sensor_lines("spike_sensors")
        saved = json.loads((self.out / "sensors.json").read_text(encoding="utf-8"))["sensor_settings"]
        self.assertEqual(list(saved), list(SENSOR_KINDS))
        for kind, line in by_kind.items():
            self.assertEqual(len(saved[kind]), len(re.findall(r" \w+=", line)), kind)
        self.assertEqual(saved["camera"]["is_render_colours"], True)
        self.assertEqual(saved["camera"]["is_render_annotations"], False)
        self.assertEqual(saved["camera"]["is_render_depth"], False)
        self.assertEqual(saved["camera"]["resolution"], [640, 360])
        self.assertEqual(saved["lidar"]["vertical_resolution"], 64)
        self.assertEqual(saved["radar"]["half_angle_deg"], 30.0)

    def test_the_adapter_passes_sensor_settings_only_from_the_table(self):
        tree = ast.parse((BRIDGE_DIR / "kite_bridge" / "beamng_adapter.py").read_text(encoding="utf-8"))
        attach = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "attach_sensors")
        keywords = [k for call in ast.walk(attach) if isinstance(call, ast.Call) for k in call.keywords]
        self.assertTrue(keywords)
        for keyword in keywords:
            self.assertIsNone(keyword.arg, "a sensor setting is passed by name in the adapter")
            self.assertEqual(ast.unparse(keyword.value), "settings_for(kind, stream, lidar_pos)")

    def test_without_stream_the_adapter_passes_the_table_itself(self):
        for kind in SENSOR_KINDS:
            self.assertIs(settings_for(kind), SENSOR_SETTINGS[kind], kind)
            self.assertIs(settings_for(kind, stream=False), SENSOR_SETTINGS[kind], kind)

    def test_with_stream_only_lidar_and_camera_gain_is_streaming_true(self):
        for kind in SENSOR_KINDS:
            added = {k: v for k, v in settings_for(kind, stream=True).items() if k not in SENSOR_SETTINGS[kind]}
            self.assertEqual(added, {"is_streaming": True} if kind in ("lidar", "camera") else {}, kind)
            for name, value in SENSOR_SETTINGS[kind].items():
                self.assertEqual(settings_for(kind, stream=True)[name], value, kind)

    def test_with_stream_the_lines_and_the_json_show_is_streaming(self):
        by_kind, _ = self.sensor_lines("spike_sensors", "--stream")
        saved = json.loads((self.out / "sensors.json").read_text(encoding="utf-8"))["sensor_settings"]
        for kind in SENSOR_KINDS:
            streamed = kind in ("lidar", "camera")
            self.assertEqual(by_kind[kind].endswith(" is_streaming=True"), streamed, kind)
            self.assertEqual(saved[kind].get("is_streaming"), True if streamed else None, kind)

    def test_without_stream_no_line_mentions_streaming(self):
        _, lines = self.sensor_lines("spike_sensors")
        self.assertFalse([line for line in lines if "is_streaming" in line])


if __name__ == "__main__":
    unittest.main()
