import tempfile
import unittest
from pathlib import Path

from kite_bridge.output_guard import PROVENANCE, repo_roots

SCANNED_FOLDERS = ("bridge", "docs")
MAY_SPELL_IT = ("bridge/kite_bridge/output_guard.py", "bridge/tests/test_output_guard.py")
SKIPPED_FOLDERS = ("__pycache__",)


def fold_case_and_whitespace(data):
    return " ".join(data.decode("utf-8", errors="ignore").casefold().split())


def files_spelling_the_marker(root):
    marker = fold_case_and_whitespace(PROVENANCE.encode("utf-8"))
    scanned, spelling = [], []
    for folder in SCANNED_FOLDERS:
        for path in sorted((root / folder).rglob("*")):
            if not path.is_file() or any(part in SKIPPED_FOLDERS for part in path.parts):
                continue
            relative = path.relative_to(root).as_posix()
            scanned.append(relative)
            if marker in fold_case_and_whitespace(path.read_bytes()):
                spelling.append(relative)
    return scanned, spelling


class MarkerSpellingTest(unittest.TestCase):
    def test_only_the_guard_and_its_test_spell_the_marker(self):
        scanned, spelling = files_spelling_the_marker(repo_roots(__file__)[0])
        self.assertIn("bridge/README.md", scanned)
        self.assertIn("docs/SPIKE-0B.md", scanned)
        self.assertEqual(sorted(set(spelling) - set(MAY_SPELL_IT)), [])

    def test_the_scan_finds_a_file_that_spells_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "bridge").mkdir()
            (root / "docs").mkdir()
            wrapped = PROVENANCE.replace(" - ", " -\n")
            (root / "docs" / "note.md").write_text(f"see `{wrapped}`\n", encoding="utf-8")
            (root / "docs" / "shout.md").write_text(PROVENANCE.upper(), encoding="utf-8")
            (root / "bridge" / "clean.md").write_text("provenance field\n", encoding="utf-8")
            _, spelling = files_spelling_the_marker(root)
        self.assertEqual(spelling, ["docs/note.md", "docs/shout.md"])


if __name__ == "__main__":
    unittest.main()
