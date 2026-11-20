import contextlib
import importlib
import io
import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from tests import BRIDGE_DIR
from kite_bridge import common
from kite_bridge.fake_sim import FakeSim

SCRIPTS = ("spike_connect", "spike_sensors", "spike_multicar", "spike_lidar_rerun")
TOGRAC_QE = "vehicles/vivace/tograc_qE.pc"
# The Rerun demo runs free by default, on the real clock; lock-step needs no waiting.
LOCKSTEP_ARGS = {"spike_lidar_rerun": ("--mode", "lockstep")}


class FakeRunCase(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.out = Path(tmp.name) / "out"

    def run_script(self, script, *argv, models=None):
        sims = []

        def make_fake():
            sim = FakeSim() if models is None else FakeSim(models=models)
            sims.append(sim)
            return sim

        stdout, stderr = io.StringIO(), io.StringIO()
        module = importlib.import_module(script)
        with (mock.patch.object(common, "FakeSim", make_fake), contextlib.redirect_stdout(stdout),
              contextlib.redirect_stderr(stderr)):
            code = module.main(["--fake", "--seconds", "1", "--traffic", "0", "--out", str(self.out),
                                *LOCKSTEP_ARGS.get(script, ()), *argv])
        return SimpleNamespace(code=code, stdout=stdout.getvalue(), stderr=stderr.getvalue(), sims=sims)


class CarSelectionTest(FakeRunCase):
    def test_every_script_spawns_the_tograc_qe_by_default(self):
        for script in SCRIPTS:
            with self.subTest(script=script):
                done = self.run_script(script)
                self.assertEqual(done.code, 0, done.stderr)
                self.assertTrue(done.sims)
                for sim in done.sims:
                    self.assertEqual(sim.spawned_model, "vivace")
                    self.assertEqual(sim.spawned_config, TOGRAC_QE)

    def test_an_empty_config_spawns_the_model_default_configuration(self):
        for script in SCRIPTS:
            with self.subTest(script=script):
                done = self.run_script(script, "--config", "")
                self.assertEqual(done.code, 0, done.stderr)
                for sim in done.sims:
                    self.assertEqual(sim.spawned_model, "vivace")
                    self.assertIsNone(sim.spawned_config)

    def test_config_with_an_equals_sign_and_no_value_spawns_the_model_default(self):
        for script in SCRIPTS:
            with self.subTest(script=script):
                done = self.run_script(script, "--config=")
                self.assertEqual(done.code, 0, done.stderr)
                self.assertTrue(done.sims)
                for sim in done.sims:
                    self.assertEqual(sim.spawned_model, "vivace")
                    self.assertIsNone(sim.spawned_config)
                self.assertIn("config: model default", done.stdout)

    def test_model_and_config_from_the_command_line_reach_the_spawn(self):
        for script in SCRIPTS:
            with self.subTest(script=script):
                done = self.run_script(script, "--model", "etk800", "--config", "vehicles/etk800/other.pc")
                self.assertEqual(done.code, 0, done.stderr)
                for sim in done.sims:
                    self.assertEqual(sim.spawned_model, "etk800")
                    self.assertEqual(sim.spawned_config, "vehicles/etk800/other.pc")

    def test_the_spawned_line_prints_the_model_and_the_configuration(self):
        done = self.run_script("spike_connect")
        self.assertIn(f"spawned kite1 (model: vivace, config: {TOGRAC_QE})", done.stdout.splitlines())
        done = self.run_script("spike_connect", "--config", "")
        self.assertIn("spawned kite1 (model: vivace, config: model default)", done.stdout.splitlines())

    def test_the_json_records_the_model_and_the_configuration(self):
        files = {"spike_connect": "connect.json", "spike_sensors": "sensors.json",
                 "spike_multicar": "multicar.json"}
        for script, name in files.items():
            with self.subTest(script=script):
                self.run_script(script)
                saved = json.loads((self.out / name).read_text(encoding="utf-8"))
                self.assertEqual((saved["model"], saved["config"]), ("vivace", TOGRAC_QE))
                self.run_script(script, "--config", "")
                saved = json.loads((self.out / name).read_text(encoding="utf-8"))
                self.assertIsNone(saved["config"])

    def test_the_name_search_is_gone(self):
        done = self.run_script("spike_sensors", models=["tograc_a", "cherrier_b"])
        self.assertEqual(done.code, 0, done.stderr)
        self.assertEqual(done.sims[0].spawned_model, "vivace")


class ListModelsTest(FakeRunCase):
    def test_every_script_prints_the_names_sorted_and_spawns_nothing(self):
        for script in SCRIPTS:
            with self.subTest(script=script):
                done = self.run_script(script, "--list-models",
                                       models=["vivace", "fake_van", "etk800", "bolide"])
                self.assertEqual(done.code, 0, done.stderr)
                self.assertEqual(done.stdout.splitlines(), ["bolide", "etk800", "fake_van", "vivace"])
                for sim in done.sims:
                    self.assertIsNone(sim.spawned_model)
                    self.assertIsNone(sim.traffic_requested)
                    self.assertEqual(sim.sim_time, 0)
                self.assertFalse(self.out.exists())

    def test_it_works_with_the_fake_from_the_command_line(self):
        for script in SCRIPTS:
            with self.subTest(script=script):
                done = subprocess.run([sys.executable, str(BRIDGE_DIR / f"{script}.py"), "--fake",
                                       "--list-models", "--out", str(self.out)], capture_output=True, text=True, timeout=120)
                names = done.stdout.splitlines()
                self.assertEqual(done.returncode, 0, done.stderr)
                self.assertIn("vivace", names)
                self.assertEqual(names, sorted(names))

    def test_a_listing_failure_is_an_error_not_an_empty_list(self):
        class Broken(FakeSim):
            def list_models(self):
                raise RuntimeError("no such call")

        stderr = io.StringIO()
        with (mock.patch.object(common, "FakeSim", Broken), contextlib.redirect_stdout(io.StringIO()),
              contextlib.redirect_stderr(stderr)):
            code = importlib.import_module("spike_connect").main(["--fake", "--list-models"])
        self.assertEqual(code, 1)
        self.assertEqual(stderr.getvalue().splitlines()[-1], "error: RuntimeError: no such call")


class VersionLinesTest(FakeRunCase):
    expected = ["BeamNGpy version: fake", "BeamNGpy protocol: fake",
                "BeamNG.tech version: not reported by BeamNGpy 1.36"]

    def test_connect_prints_the_package_protocol_and_unreported_tech_lines_first(self):
        done = self.run_script("spike_connect")
        self.assertEqual(done.code, 0, done.stderr)
        self.assertEqual(done.stdout.splitlines()[:3], self.expected)

    def test_connect_records_the_same_lines_in_its_json(self):
        self.run_script("spike_connect")
        saved = json.loads((self.out / "connect.json").read_text(encoding="utf-8"))
        self.assertEqual([f"{label}: {value}" for label, value in saved["versions"].items()], self.expected)


class NoTestImportsBeamngpyTest(unittest.TestCase):
    def test_no_test_file_imports_beamngpy(self):
        tests = sorted((BRIDGE_DIR / "tests").glob("*.py"))
        self.assertGreater(len(tests), 5)
        for path in tests:
            found = re.search(r"(?m)^\s*(import|from)\s+beamngpy", path.read_text(encoding="utf-8"))
            self.assertIsNone(found, path.name)


if __name__ == "__main__":
    unittest.main()
