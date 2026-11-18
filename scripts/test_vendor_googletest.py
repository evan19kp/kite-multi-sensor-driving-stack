import contextlib
import hashlib
import io
import os
import tarfile
import tempfile
import unittest
from pathlib import Path

import vendor_googletest

PREFIX = "googletest-9.9.9"
GOOD_FILES = {
    "CMakeLists.txt": b"project(googletest-distribution)\n",
    "LICENSE": b"BSD 3-Clause\n",
    "googletest/CMakeLists.txt": b"project(gtest)\n",
    "googletest/src/gtest.cc": b"// gtest\n",
    "googlemock/CMakeLists.txt": b"project(gmock)\n",
    "README.md": b"not wanted\n",
    "ci/run.sh": b"not wanted\n",
    "docs/index.md": b"not wanted\n",
}


def add_file(tar, name, data):
    info = tarfile.TarInfo(name)
    info.size = len(data)
    tar.addfile(info, io.BytesIO(data))


def make_archive(path, files=None, extra=()):
    files = GOOD_FILES if files is None else files
    with tarfile.open(path, "w:gz") as tar:
        for name, data in files.items():
            add_file(tar, f"{PREFIX}/{name}", data)
        for name, data in extra:
            add_file(tar, name, data)
    return path


def run_main(*argv):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = vendor_googletest.main(list(argv))
    return code, out.getvalue(), err.getvalue()


class VendorGoogletestTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = Path(tmp.name)
        self.archive = make_archive(str(self.tmp / "gt.tar.gz"))
        self.dest = self.tmp / "out" / "googletest"

    def vendor(self, *extra, archive=None):
        return run_main(
            "--tag", "v9.9.9", "--dest", str(self.dest), "--archive", archive or self.archive, *extra
        )

    def test_extracts_only_wanted_entries(self):
        code, _, err = self.vendor()
        self.assertEqual(code, 0, err)
        names = sorted(p.name for p in self.dest.iterdir())
        self.assertEqual(names, ["CMakeLists.txt", "LICENSE", "VENDORED.md", "googlemock", "googletest"])
        self.assertEqual((self.dest / "googletest" / "src" / "gtest.cc").read_bytes(), b"// gtest\n")
        self.assertEqual((self.dest / "LICENSE").read_bytes(), b"BSD 3-Clause\n")

    def test_prints_sha256_of_the_archive(self):
        expected = hashlib.sha256(Path(self.archive).read_bytes()).hexdigest()
        code, out, _ = self.vendor()
        self.assertEqual(code, 0)
        self.assertIn(f"SHA-256: {expected}", out)

    def test_writes_vendored_record(self):
        expected = hashlib.sha256(Path(self.archive).read_bytes()).hexdigest()
        self.vendor()
        text = (self.dest / "VENDORED.md").read_text(encoding="utf-8")
        self.assertIn("v9.9.9", text)
        self.assertIn("https://github.com/google/googletest/archive/refs/tags/v9.9.9.tar.gz", text)
        self.assertIn(expected, text)
        self.assertRegex(text, r"Date: \d{4}-\d{2}-\d{2}")
        self.assertIn("BSD-3-Clause, see LICENSE", text)
        self.assertIn("vendored by scripts/vendor_googletest.py; do not edit by hand", text.lower())

    def test_refuses_non_empty_destination(self):
        self.dest.mkdir(parents=True)
        (self.dest / "mine.txt").write_text("keep", encoding="utf-8")
        code, _, err = self.vendor()
        self.assertEqual(code, 1)
        self.assertIn("not empty", err)
        self.assertEqual((self.dest / "mine.txt").read_text(encoding="utf-8"), "keep")
        self.assertFalse((self.dest / "LICENSE").exists())

    def test_empty_destination_is_fine(self):
        self.dest.mkdir(parents=True)
        code, _, err = self.vendor()
        self.assertEqual(code, 0, err)

    def test_force_replaces_destination(self):
        self.dest.mkdir(parents=True)
        (self.dest / "stale.txt").write_text("old", encoding="utf-8")
        code, _, err = self.vendor("--force")
        self.assertEqual(code, 0, err)
        self.assertFalse((self.dest / "stale.txt").exists())
        self.assertTrue((self.dest / "LICENSE").is_file())

    def test_refuses_parent_folder_member(self):
        bad = make_archive(str(self.tmp / "bad.tar.gz"), extra=[(f"{PREFIX}/googletest/../../evil.txt", b"x")])
        code, _, err = self.vendor(archive=bad)
        self.assertEqual(code, 1)
        self.assertIn("unsafe", err)
        self.assertFalse((self.tmp / "out" / "evil.txt").exists())
        self.assertFalse(self.dest.exists())

    def test_refuses_parent_folder_member_outside_kept_entries(self):
        bad = make_archive(str(self.tmp / "bad.tar.gz"), extra=[("../evil.txt", b"x")])
        code, _, err = self.vendor(archive=bad)
        self.assertEqual(code, 1)
        self.assertIn("unsafe", err)

    def test_refuses_absolute_member(self):
        bad = make_archive(str(self.tmp / "bad.tar.gz"), extra=[("/evil.txt", b"x")])
        code, _, err = self.vendor(archive=bad)
        self.assertEqual(code, 1)
        self.assertIn("absolute", err)
        self.assertFalse(self.dest.exists())

    def test_refuses_symlink_member(self):
        path = str(self.tmp / "link.tar.gz")
        with tarfile.open(path, "w:gz") as tar:
            for name, data in GOOD_FILES.items():
                add_file(tar, f"{PREFIX}/{name}", data)
            link = tarfile.TarInfo(f"{PREFIX}/googletest/link")
            link.type = tarfile.SYMTYPE
            link.linkname = "target"
            tar.addfile(link)
        code, _, err = self.vendor(archive=path)
        self.assertEqual(code, 1)
        self.assertIn("unsupported", err)
        self.assertFalse(self.dest.exists())

    def test_fails_when_wanted_entry_missing(self):
        files = {k: v for k, v in GOOD_FILES.items() if not k.startswith("googlemock/")}
        bad = make_archive(str(self.tmp / "bad.tar.gz"), files=files)
        code, _, err = self.vendor(archive=bad)
        self.assertEqual(code, 1)
        self.assertIn("googlemock", err)
        self.assertFalse(self.dest.exists())

    def test_fails_on_missing_archive_file(self):
        code, _, err = self.vendor(archive=str(self.tmp / "nope.tar.gz"))
        self.assertEqual(code, 1)
        self.assertIn("not found", err)

    def test_fails_on_corrupt_archive(self):
        bad = self.tmp / "bad.tar.gz"
        bad.write_bytes(b"this is not a tar file")
        code, _, err = self.vendor(archive=str(bad))
        self.assertEqual(code, 1)
        self.assertIn("cannot read archive", err)

    def test_archive_url_for_tag(self):
        self.assertEqual(
            vendor_googletest.archive_url("v1.2.3"),
            "https://github.com/google/googletest/archive/refs/tags/v1.2.3.tar.gz",
        )

    def test_rejects_bad_tag(self):
        for tag in ("../x", "v1/../../x", "", "a b", "-v1"):
            with self.assertRaises(vendor_googletest.VendorError, msg=tag):
                vendor_googletest.archive_url(tag)

    def test_default_destination_is_in_the_repo(self):
        expected = os.path.join(vendor_googletest.REPO_ROOT, "third_party", "googletest")
        self.assertEqual(vendor_googletest.DEFAULT_DEST, expected)
        self.assertTrue(os.path.isfile(os.path.join(vendor_googletest.REPO_ROOT, "AGENTS.md")))


if __name__ == "__main__":
    unittest.main()
