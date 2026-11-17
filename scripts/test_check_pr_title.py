import unittest

from check_pr_title import is_valid


class PrTitleTest(unittest.TestCase):
    def test_valid(self):
        for title in [
            "feat(R1): add spring boot backend skeleton",
            "fix(R12): handle empty shot list",
            "refactor(R3): split ci jobs",
            "test(R4): cover schema example",
            "docs(R6): document title check",
            "ci(R6): add pr title conventional commits check",
            "perf(R2): cache node modules",
            "chore(R7): bump deps",
            "chore: tidy gitignore",
            "chore(deps): bump the maven group across 1 directory",
            "chore(deps-dev): bump the npm group",
        ]:
            with self.subTest(title=title):
                self.assertTrue(is_valid(title))

    def test_invalid(self):
        for title in [
            "",
            "add pr title check",
            "feat: add thing",
            "feat(R1) add thing",
            "feat(R1):add thing",
            "feat(R1): Add thing",
            "feat(R1): ",
            "feat(R1):  double space",
            "feat(R1): trailing space ",
            "feat(r1): lowercase scope",
            "feat(R): missing number",
            "feat(R1)!: breaking",
            "feature(R1): wrong type",
            "Feat(R1): capitalised type",
            "chore(): empty scope",
            "chore(foo): other non-r scope",
            "chore(deps) bump: no colon",
            "chore(deps): Bump capitalised",
            "feat(deps): deps scope only for chore",
            "fix: bare fix",
            "chore:no space",
            "feat(R1): 1st thing",
            "feat(R1): a\nsecond line",
        ]:
            with self.subTest(title=title):
                self.assertFalse(is_valid(title))


if __name__ == "__main__":
    unittest.main()
