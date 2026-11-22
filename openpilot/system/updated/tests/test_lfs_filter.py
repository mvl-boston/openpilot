import subprocess
import tempfile
import unittest
from pathlib import Path

from openpilot.system.updated.branch_switch_compat import reset_foreign_lfs_filter

LFS_PY_FILTER = '"$(git rev-parse --git-common-dir)/lfs.py" filter-process'


def git(cwd: str, *args: str) -> str:
  cmd = ["git", "-c", "user.name=test", "-c", "user.email=test@test", *args]
  return subprocess.check_output(cmd, cwd=cwd, stderr=subprocess.STDOUT, encoding="utf8").strip()


def local_filter(cwd: str) -> str:
  return subprocess.run(["git", "config", "--local", "--get", "filter.lfs.process"], cwd=cwd, capture_output=True, encoding="utf8").stdout.strip()


class TestForeignLfsFilter(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.repo = self.tmp.name
    git(self.repo, "init", "-q", "-b", "with-client")
    Path(self.repo, "lfs.py").write_text("")
    git(self.repo, "add", "lfs.py")
    git(self.repo, "commit", "-q", "-m", "ships lfs.py")
    git(self.repo, "checkout", "-q", "-b", "without-client")
    git(self.repo, "rm", "-q", "lfs.py")
    git(self.repo, "commit", "-q", "-m", "stock git-lfs")

  def install_client_filter(self):
    git(self.repo, "config", "--local", "filter.lfs.process", LFS_PY_FILTER)
    git(self.repo, "config", "--local", "filter.lfs.required", "true")

  def test_removed_when_target_has_no_client(self):
    self.install_client_filter()
    self.assertTrue(reset_foreign_lfs_filter(self.repo, "without-client"))
    self.assertEqual(local_filter(self.repo), "")
    self.assertNotIn("filter.lfs", git(self.repo, "config", "--local", "--list"))

  def test_kept_when_target_ships_client(self):
    self.install_client_filter()
    self.assertFalse(reset_foreign_lfs_filter(self.repo, "with-client"))
    self.assertEqual(local_filter(self.repo), LFS_PY_FILTER)

  def test_other_local_filters_untouched(self):
    self.assertFalse(reset_foreign_lfs_filter(self.repo, "without-client"))

    git(self.repo, "config", "--local", "filter.lfs.process", "git-lfs filter-process")
    self.assertFalse(reset_foreign_lfs_filter(self.repo, "without-client"))
    self.assertEqual(local_filter(self.repo), "git-lfs filter-process")


if __name__ == "__main__":
  unittest.main()
