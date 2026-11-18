import json
import re
import tempfile
import unittest
from pathlib import Path

import ci_changes


def find_repo_root() -> Path:
    here = Path(__file__).resolve().parent
    for folder in (here, *here.parents):
        if (folder / "AGENTS.md").is_file():
            return folder
    raise FileNotFoundError("AGENTS.md not found above " + str(here))


ROOT = find_repo_root()
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

    def test_repo_config_cpp_folders_trigger_both_builds_only(self):
        cfg = ci_changes.load()
        for path in ["sim/gen.cpp", "eval/score.cpp", "tools/gen.py"]:
            self.assertEqual(ci_changes.jobs_for(path, cfg), frozenset({"app", "app-windows"}), path)
            self.assertEqual(ci_changes.classify([path], cfg), {"app": True, "app-windows": True, "python": False}, path)

    def test_repo_config_python_folders_trigger_python_only(self):
        cfg = ci_changes.load()
        for path in ["bridge/x.py", "perception/x.py", "scenarios/x.py", "viewer/x.py"]:
            self.assertEqual(ci_changes.jobs_for(path, cfg), frozenset({"python"}), path)
            self.assertEqual(ci_changes.classify([path], cfg), {"app": False, "app-windows": False, "python": True}, path)

    def test_repo_config_backslash_paths_match_new_folders(self):
        cfg = ci_changes.load()
        self.assertEqual(ci_changes.jobs_for("sim\\a.cpp", cfg), frozenset({"app", "app-windows"}))
        self.assertEqual(ci_changes.jobs_for("viewer\\a.py", cfg), frozenset({"python"}))

    def test_repo_config_docs_still_trigger_app_only(self):
        cfg = ci_changes.load()
        self.assertEqual(ci_changes.jobs_for("docs/diagram.png", cfg), frozenset({"app"}))

    def test_repo_config_markdown_read_by_ctest_triggers_both_builds_only(self):
        cfg = ci_changes.load()
        for path in ["docs/BASELINE.md", "eval/README.md"]:
            self.assertEqual(ci_changes.classify([path], cfg), {"app": True, "app-windows": True, "python": False}, path)

    def test_repo_config_removed_folders_fall_back_to_every_job(self):
        cfg = ci_changes.load()
        everything = frozenset(cfg["jobs"])
        for path in ["frontend/app.ts", "backend/Main.java", "src/a.ts"]:
            self.assertEqual(ci_changes.jobs_for(path, cfg), everything, path)
            self.assertTrue(all(ci_changes.classify([path], cfg).values()), path)

    def test_repo_config_removed_seed_paths_no_longer_seed(self):
        cfg = ci_changes.load()
        for path in ["package-lock.json", "frontend/package-lock.json", "backend/pom.xml"]:
            self.assertFalse(ci_changes.needs_seed([path], cfg), path)
        self.assertTrue(ci_changes.needs_seed([".github/workflows/ci.yml"], cfg))
        self.assertFalse(ci_changes.needs_seed(["sim/a.cpp"], cfg))


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
