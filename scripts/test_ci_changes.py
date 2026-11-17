import json
import re
import tempfile
import unittest
from pathlib import Path

import ci_changes

ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / ".github" / "workflows" / "ci.yml"

CFG = {
    "jobs": ["app", "python", "e2e"],
    "always_all": [".github/workflows/", "scripts/ci_changes.py"],
    "rules": [{"prefix": "src/", "jobs": ["app", "e2e"]}, {"prefix": "scripts/", "jobs": ["python"]}],
    "md_read_by_tests": {"DESIGN.md": ["app"]},
    "seed_paths": ["package-lock.json", ".github/workflows/"],
}


class ClassifyTest(unittest.TestCase):
    def test_no_files_runs_everything(self):
        self.assertEqual(ci_changes.classify([], CFG), {"app": True, "python": True, "e2e": True})
        self.assertTrue(ci_changes.needs_seed([], CFG))

    def test_docs_only_runs_nothing(self):
        self.assertEqual(set(k for k, v in ci_changes.classify(["README.md", "docs/x.md"], CFG).items() if v), set())

    def test_md_read_by_tests(self):
        self.assertTrue(ci_changes.classify(["DESIGN.md"], CFG)["app"])

    def test_prefix_rule(self):
        self.assertEqual(ci_changes.classify(["src/a.ts"], CFG), {"app": True, "python": False, "e2e": True})
        self.assertEqual(ci_changes.classify(["scripts\\x.py"], CFG), {"app": False, "python": True, "e2e": False})

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


@unittest.skipUnless(WORKFLOW.is_file(), "no ci.yml")
class WorkflowShapeTest(unittest.TestCase):
    def test_every_gated_job_exists_and_uses_the_classifier(self):
        text = WORKFLOW.read_text(encoding="utf-8")
        for job in ci_changes.load()["jobs"]:
            self.assertRegex(text, rf"(?m)^  {re.escape(job)}:$", job)
            self.assertIn(f"needs.gitleaks.outputs.{job} != 'false'", text, job)

    def test_every_job_has_a_timeout(self):
        text = WORKFLOW.read_text(encoding="utf-8")
        jobs = re.findall(r"(?m)^  ([a-z][a-z0-9-]*):$", text.split("\njobs:", 1)[1])
        job_timeouts = re.findall(r"(?m)^    timeout-minutes:", text)
        self.assertEqual(len(job_timeouts), len(jobs))


if __name__ == "__main__":
    unittest.main()
