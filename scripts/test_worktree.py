import subprocess
import tempfile
import unittest
from pathlib import Path

import proc_registry
import worktree


def git(cwd, *args):
    return subprocess.run(["git", "-C", str(cwd), *args], capture_output=True, text=True, check=True).stdout


class WorktreeTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        base = Path(tmp.name).resolve()
        self.root = base / "repo"
        self.root.mkdir()
        git(self.root, "init", "-b", "main")
        git(self.root, "config", "user.email", "t@example.com")
        git(self.root, "config", "user.name", "t")
        (self.root / "AGENTS.md").write_text("x\n")
        git(self.root, "add", "AGENTS.md")
        git(self.root, "commit", "-m", "init")

    def test_create_list_remove(self):
        path = worktree.create(self.root, "R40")
        self.assertEqual(path, self.root / ".worktrees" / "R40")
        self.assertTrue((path / "AGENTS.md").is_file())
        self.assertEqual(git(path, "branch", "--show-current").strip(), "feat/R40")
        self.assertEqual(worktree.listing(self.root), [("R40", "feat/R40", path.resolve())])
        worktree.remove(self.root, "R40")
        self.assertFalse(path.exists())
        self.assertEqual(worktree.listing(self.root), [])
        self.assertIn("feat/R40", git(self.root, "branch", "--list"), "branch is kept")

    def test_create_is_idempotent_and_honours_branch(self):
        first = worktree.create(self.root, "R41", branch="feat/R41-thing")
        self.assertEqual(worktree.create(self.root, "R41", branch="feat/R41-thing"), first)
        with self.assertRaises(worktree.WorktreeError):
            worktree.create(self.root, "R41", branch="feat/other")

    def test_branch_made_from_origin_main(self):
        remote = self.root.parent / "remote.git"
        subprocess.run(["git", "init", "--bare", "-b", "main", str(remote)], check=True, capture_output=True)
        git(self.root, "remote", "add", "origin", str(remote))
        git(self.root, "push", "origin", "main")
        other = self.root.parent / "other"
        subprocess.run(["git", "clone", str(remote), str(other)], check=True, capture_output=True)
        git(other, "config", "user.email", "t@example.com")
        git(other, "config", "user.name", "t")
        (other / "new.txt").write_text("n\n")
        git(other, "add", "new.txt")
        git(other, "commit", "-m", "newer")
        git(other, "push", "origin", "main")
        path = worktree.create(self.root, "R42")
        self.assertTrue((path / "new.txt").is_file(), "branched from the fetched origin/main, not stale local main")

    def test_remove_refuses_dirty_unless_forced(self):
        path = worktree.create(self.root, "R43")
        (path / "scratch.txt").write_text("wip\n")
        with self.assertRaises(worktree.WorktreeError):
            worktree.remove(self.root, "R43")
        self.assertTrue(path.exists())
        worktree.remove(self.root, "R43", force=True)
        self.assertFalse(path.exists())

    def test_bad_item_ids_are_rejected(self):
        for item in ("../x", "a/b", "", "R40;rm", "-R1", "a b", ".."):
            with self.assertRaises(worktree.WorktreeError, msg=item):
                worktree.create(self.root, item)
            with self.assertRaises(worktree.WorktreeError, msg=item):
                worktree.remove(self.root, item)

    def test_remove_refuses_the_main_checkout_and_unknown_items(self):
        with self.assertRaises(worktree.WorktreeError):
            worktree.remove(self.root, "R99")
        self.assertTrue((self.root / "AGENTS.md").is_file())

    def test_main_checkout_resolves_from_inside_a_worktree(self):
        path = worktree.create(self.root, "R44")
        self.assertEqual(proc_registry.main_checkout(path).resolve(), self.root)
        self.assertEqual(proc_registry.main_checkout(self.root).resolve(), self.root)


if __name__ == "__main__":
    unittest.main()
