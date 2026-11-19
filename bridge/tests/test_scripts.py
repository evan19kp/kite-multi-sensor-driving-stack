import ast
import contextlib
import io
import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from tests import BRIDGE_DIR

SCRIPTS = ("spike_connect", "spike_sensors", "spike_multicar")
OUTPUT_FILE = {"spike_connect": "connect.json", "spike_sensors": "sensors.json",
               "spike_multicar": "multicar.json"}


def run_script(name, *args):
    done = subprocess.run([sys.executable, str(BRIDGE_DIR / f"{name}.py"), *args],
                          capture_output=True, text=True, timeout=120)
    return done.returncode, done.stdout, done.stderr


class FakeRunTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name) / "nested" / "out"

    def run_fake(self, name, *extra):
        code, stdout, stderr = run_script(name, "--fake", "--seconds", "1", "--out", str(self.out), *extra)
        self.assertEqual(code, 0, stderr)
        return stdout, json.loads((self.out / OUTPUT_FILE[name]).read_text(encoding="utf-8"))

    def test_connect_prints_versions_and_positions_and_records_ground_truth(self):
        stdout, data = self.run_fake("spike_connect")
        self.assertIn("BeamNG.tech version: fake", stdout)
        self.assertIn("BeamNGpy version: fake", stdout)
        self.assertEqual(len(re.findall(r"^t=\d+s position=", stdout, re.MULTILINE)), 1)
        self.assertEqual(data["model"], "fake_cherrier_tograc")
        self.assertEqual(len(data["ground_truth"]), 1)
        self.assertAlmostEqual(data["ground_truth"][0]["pos"][0], 10.0)

    def test_connect_prints_one_position_per_simulated_second(self):
        code, stdout, _ = run_script("spike_connect", "--fake", "--seconds", "3", "--out", str(self.out))
        self.assertEqual(code, 0)
        self.assertEqual(len(re.findall(r"^t=\d+s position=", stdout, re.MULTILINE)), 3)

    def test_sensors_prints_a_table_and_writes_the_same_numbers(self):
        stdout, data = self.run_fake("spike_sensors")
        self.assertEqual([r["sensor"] for r in data["sensors"]], ["lidar", "camera", "radar", "gps", "imu"])
        for sensor in data["sensors"]:
            self.assertRegex(stdout, rf"(?m)^kite1\s+{sensor['sensor']}\s")
        by_sensor = {r["sensor"]: r for r in data["sensors"]}
        self.assertAlmostEqual(by_sensor["lidar"]["rate_hz"], 10.0)
        self.assertEqual(by_sensor["lidar"]["repeated"], 49)
        self.assertTrue(900 <= by_sensor["lidar"]["points_mean"] < 1100)
        self.assertEqual(set(by_sensor["lidar"]["latency_s"]), {"mean", "p50", "p95", "max"})

    def test_multicar_runs_one_two_and_four_cars(self):
        stdout, data = self.run_fake("spike_multicar")
        self.assertEqual([c["cars"] for c in data["cases"]], [1, 2, 4])
        self.assertEqual([len(c["sensors"]) for c in data["cases"]], [5, 10, 20])
        for case in data["cases"]:
            self.assertGreater(case["steps_per_second"], 0)
            self.assertAlmostEqual(case["sensors"][0]["rate_hz"], 10.0)
        self.assertEqual(len(re.findall(r"steps per second", stdout)), 3)

    def test_model_flag_is_used(self):
        _, data = self.run_fake("spike_sensors", "--model", "chosen_model", "--traffic", "0")
        self.assertEqual(data["model"], "chosen_model")


class CommandLineTest(unittest.TestCase):
    def test_missing_home_without_fake_is_a_usage_error(self):
        for name in SCRIPTS:
            code, stdout, stderr = run_script(name)
            self.assertEqual(code, 2, name)
            self.assertIn("--home", stderr)
            self.assertEqual(stdout, "")

    def test_bad_option_values_are_usage_errors(self):
        for bad in (["--seconds", "0"], ["--seconds", "nan"], ["--traffic", "-1"], ["--seconds", "x"]):
            code, _, stderr = run_script("spike_sensors", "--fake", *bad)
            self.assertEqual(code, 2, bad)
            self.assertIn("usage", stderr)

    def test_a_home_that_is_not_a_folder_fails_clearly(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "missing"
            code, _, stderr = run_script("spike_connect", "--home", str(missing), "--out", tmp)
        self.assertEqual(code, 1)
        self.assertIn("error: --home is not a folder", stderr)

    def test_help_lists_every_common_option(self):
        for name in SCRIPTS:
            code, stdout, _ = run_script(name, "--help")
            self.assertEqual(code, 0)
            for option in ("--home", "--headless", "--seconds", "--traffic", "--out", "--fake", "--model"):
                self.assertIn(option, stdout)


class NoRealSimulatorTest(unittest.TestCase):
    def test_running_every_script_against_the_fake_never_imports_the_adapter(self):
        import spike_connect
        import spike_multicar
        import spike_sensors

        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            for module in (spike_connect, spike_sensors, spike_multicar):
                self.assertEqual(module.main(["--fake", "--seconds", "1", "--out", tmp]), 0)
        self.assertNotIn("beamngpy", sys.modules)
        self.assertNotIn("kite_bridge.beamng_adapter", sys.modules)

    def test_the_adapter_source_parses_and_implements_every_interface_method(self):
        tree = ast.parse((BRIDGE_DIR / "kite_bridge" / "beamng_adapter.py").read_text(encoding="utf-8"))
        defined = {n.name for c in tree.body if isinstance(c, ast.ClassDef) for n in c.body
                   if isinstance(n, ast.FunctionDef)}
        interface = ast.parse((BRIDGE_DIR / "kite_bridge" / "sim_api.py").read_text(encoding="utf-8"))
        required = {n.name for c in interface.body if isinstance(c, ast.ClassDef) and c.name == "Simulator"
                    for n in c.body if isinstance(n, ast.FunctionDef)}
        self.assertEqual(required - defined, set())

    def test_only_the_adapter_imports_beamngpy(self):
        sources = [*BRIDGE_DIR.glob("*.py"), *(BRIDGE_DIR / "kite_bridge").glob("*.py")]
        self.assertGreater(len(sources), 5)
        for path in sources:
            imports = re.search(r"(?m)^\s*(import|from)\s+beamngpy", path.read_text(encoding="utf-8"))
            self.assertEqual(bool(imports), path.name == "beamng_adapter.py", path.name)


if __name__ == "__main__":
    unittest.main()
