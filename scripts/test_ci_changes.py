import json
import re
import tempfile
import unittest
from pathlib import Path

import ci_changes

ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / ".github" / "workflows" / "ci.yml"

CFG = {
    "jobs": ["app", "app-windows", "python"],
    "always_all": [".github/workflows/", "scripts/ci_changes.py"],
    "rules": [{"prefix": "src/", "jobs": ["app", "app-windows"]}, {"prefix": "scripts/", "jobs": ["python"]}],
    "md_read_by_tests": {"DESIGN.md": ["app"]},
    "seed_paths": ["package-lock.json", ".github/workflows/"],
}


class ClassifyTest(unittest.TestCase):
    def test_no_files_runs_everything(self):
        self.assertEqual(ci_changes.classify([], CFG), {"app": True, "app-windows": True, "python": True})
        self.assertTrue(ci_changes.needs_seed([], CFG))

    def test_docs_only_runs_nothing(self):
        self.assertEqual(set(k for k, v in ci_changes.classify(["README.md", "docs/x.md"], CFG).items() if v), set())

    def test_md_read_by_tests(self):
        self.assertTrue(ci_changes.classify(["DESIGN.md"], CFG)["app"])

    def test_prefix_rule(self):
        self.assertEqual(ci_changes.classify(["src/a.ts"], CFG), {"app": True, "app-windows": True, "python": False})
        self.assertEqual(ci_changes.classify(["scripts\\x.py"], CFG), {"app": False, "app-windows": False, "python": True})

    def test_unknown_and_workflow_run_everything(self):
        self.assertTrue(all(ci_changes.classify(["weird/file"], CFG).values()))
        self.assertTrue(all(ci_changes.classify([".github/workflows/ci.yml"], CFG).values()))

    def test_seed(self):
        self.assertTrue(ci_changes.needs_seed(["package-lock.json"], CFG))
        self.assertFalse(ci_changes.needs_seed(["src/a.ts"], CFG))

    def test_config_rejects_unknown_job(self):
        bad = dict(CFG, rules=[{"prefix": "x/", "jobs": ["nope"]}])
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "c.json"
            p.write_text(json.dumps(bad), encoding="utf-8")
            with self.assertRaises(ValueError):
                ci_changes.load(p)

    def test_repo_config_loads(self):
        ci_changes.load()

    def test_repo_config_maps_cpp_paths_to_both_builds(self):
        cfg = ci_changes.load()
        self.assertNotIn("e2e", cfg["jobs"])
        self.assertIn("app-windows", cfg["jobs"])
        for path in ["core/x.cpp", "third_party/googletest/CMakeLists.txt", "CMakeLists.txt",
                     "CMakePresets.json", "cmake/x.cmake"]:
            self.assertEqual(ci_changes.jobs_for(path, cfg), frozenset({"app", "app-windows"}), path)

    def test_repo_config_python_only_change_skips_cpp_builds(self):
        self.assertEqual(
            ci_changes.classify(["scripts/x.py"], ci_changes.load()),
            {"app": False, "app-windows": False, "python": True},
        )


@unittest.skipUnless(WORKFLOW.is_file(), "no ci.yml")
class WorkflowShapeTest(unittest.TestCase):
    def test_every_gated_job_exists_and_uses_the_classifier(self):
        text = WORKFLOW.read_text(encoding="utf-8")
        for job in ci_changes.load()["jobs"]:
            self.assertRegex(text, rf"(?m)^  {re.escape(job)}:$", job)
            self.assertIn(f"needs.gitleaks.outputs.{job} != 'false'", text, job)

    def test_no_e2e_job_and_windows_job_present(self):
        text = WORKFLOW.read_text(encoding="utf-8")
        self.assertNotRegex(text, r"(?m)^  e2e:$")
        self.assertRegex(text, r"(?m)^  app-windows:$")
        self.assertIn("steps.classify.outputs.app-windows", text)

    def test_every_job_has_a_timeout(self):
        text = WORKFLOW.read_text(encoding="utf-8")
        jobs = re.findall(r"(?m)^  ([a-z][a-z0-9-]*):$", text.split("\njobs:", 1)[1])
        job_timeouts = re.findall(r"(?m)^    timeout-minutes:", text)
        self.assertEqual(len(job_timeouts), len(jobs))


if __name__ == "__main__":
    unittest.main()
