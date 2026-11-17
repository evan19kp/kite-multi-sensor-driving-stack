"""Tests for check_forbidden_files.py. Run: python -m unittest discover -s scripts -v"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_forbidden_files as c  # noqa: E402

RULES = c.load_rules()


class Check(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.root, ignore_errors=True)

    def put(self, rel: str, content: str | bytes = "x") -> str:
        full = os.path.join(self.root, rel)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        mode = "wb" if isinstance(content, bytes) else "w"
        with open(full, mode, **({} if mode == "wb" else {"encoding": "utf-8"})) as f:
            f.write(content)
        return rel

    def problems(self, *rels):
        return c.check(self.root, list(rels), RULES)

    def test_beamng_file_types_fail(self):
        for rel in ("vehicles/tograc/tograc.jbeam", "data/tograc_qE.pc", "x/a.dds", "lua/ge/x.lua",
                    "levels/a/main.materials.json", "tech.key", "dist/app.exe", "rec/run1.rrd",
                    "sample/run.mp4", "art/info.json"):
            self.assertTrue(self.problems(self.put(rel)), rel)

    def test_beamng_paths_fail(self):
        self.assertTrue(self.problems(self.put("third_party/BeamNG/readme.txt")))
        self.assertTrue(self.problems(self.put("copy/Bin64/notes.txt")))

    def test_content_markers_fail(self):
        for text in ("-- BeamNG COMMON DEVELOPMENT AND DISTRIBUTION LICENSE (bCDDL) Version 1.1",
                     "Copyright (c) 2012-2026 BeamNG GmbH",
                     "home = 'D:\\\\BeamNG'", "home = 'D:/BeamNG'", "ls /d/BeamNG", "/mnt/d/beamng/x",
                     "C:\\Users\\e\\AppData\\Local\\BeamNG\\tech", "os.environ['BNG_HOME']"):
            self.assertTrue(self.problems(self.put("bridge/a.py", text)), text)

    def test_size_limit(self):
        self.assertTrue(self.problems(self.put("data/big.csv", b"0" * (RULES["max_file_kb"] * 1024 + 1))))

    def test_normal_files_pass(self):
        files = [
            self.put("bridge/beamng_bridge.py",
                     "from beamngpy import BeamNGpy\nbng = BeamNGpy('localhost', 25252, home=args.home)\n"),
            self.put("README.md", "Credit: BeamNG.tech by BeamNG GmbH. Tested in simulation.\n"),
            self.put("core/src/kalman.cpp", "// constant-velocity Kalman filter\n"),
            self.put("data/sample/README.md", "This work includes data generated with the software "
                                              "BeamNG.tech by BeamNG GmbH, www.beamng.gmbh\n"),
            self.put("data/sample/run1.csv", "t,x,y\n0,0,0\n"),
        ]
        self.assertEqual(self.problems(*files), [])

    def test_exempt_files_skip_content_only(self):
        rel = self.put("scripts/forbidden_files.json", '{"x": "bCDDL D:/BeamNG"}')
        self.assertEqual(self.problems(rel), [])
        self.assertTrue(self.problems(self.put("scripts/other.json", '{"x": "bCDDL"}')))

    def test_repo_itself_is_clean(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if not os.path.isdir(os.path.join(root, ".git")):
            self.skipTest("not inside a git checkout")
        self.assertEqual(c.check(root, c.tracked_files(root), RULES), [])


if __name__ == "__main__":
    unittest.main()
