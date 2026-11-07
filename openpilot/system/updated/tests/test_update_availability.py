import unittest
from pathlib import Path
from unittest import mock

from openpilot.system.updated.updated import BASEDIR, FINALIZED, OVERLAY_MERGED, Updater


class TestUpdateAvailability(unittest.TestCase):
  def setUp(self):
    self.updater = Updater()

  def test_update_ready_when_staged_differs_from_running(self):
    self.updater.branches["release3"] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
    consistent = mock.Mock()
    consistent.is_file.return_value = True

    with mock.patch("openpilot.system.updated.updated.Path", return_value=consistent):
      with mock.patch.object(type(self.updater), "target_branch", new_callable=mock.PropertyMock, return_value="release3"):
        with mock.patch.object(self.updater, "get_commit_hash", side_effect=self._hashes(running="aaaaaaa", staged="bbbbbbb")):
          with mock.patch.object(self.updater, "get_branch", side_effect=self._branches(running="release3", staged="release3")):
            self.assertTrue(self.updater.update_ready)

  def test_update_ready_false_when_staged_matches_running(self):
    self.updater.branches["release3"] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
    consistent = mock.Mock()
    consistent.is_file.return_value = True

    with mock.patch("openpilot.system.updated.updated.Path", return_value=consistent):
      with mock.patch.object(type(self.updater), "target_branch", new_callable=mock.PropertyMock, return_value="release3"):
        with mock.patch.object(self.updater, "get_commit_hash", side_effect=self._hashes(running="aaaaaaa", staged="aaaaaaa")):
          with mock.patch.object(self.updater, "get_branch", side_effect=self._branches(running="release3", staged="release3")):
            self.assertFalse(self.updater.update_ready)

  def test_update_ready_when_staged_ahead_of_running_despite_stale_remote(self):
    """After installing a previously staged build, install should still be offered for a newer download."""
    self.updater.branches["release3"] = "aaaaaaa"  # stale: remote already reflected in running tree
    consistent = mock.Mock()
    consistent.is_file.return_value = True

    with mock.patch("openpilot.system.updated.updated.Path", return_value=consistent):
      with mock.patch.object(type(self.updater), "target_branch", new_callable=mock.PropertyMock, return_value="release3"):
        with mock.patch.object(self.updater, "get_commit_hash", side_effect=self._hashes(running="aaaaaaa", staged="bbbbbbb")):
          with mock.patch.object(self.updater, "get_branch", side_effect=self._branches(running="release3", staged="release3")):
            self.assertTrue(self.updater.update_ready)

  def test_update_available_when_running_behind_remote(self):
    self.updater.branches["release3"] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"

    with mock.patch.object(type(self.updater), "target_branch", new_callable=mock.PropertyMock, return_value="release3"):
      with mock.patch.object(self.updater, "get_commit_hash", side_effect=self._hashes(running="aaaaaaa", overlay="aaaaaaa")):
        with mock.patch.object(self.updater, "get_branch", side_effect=self._branches(running="release3")):
          with mock.patch("openpilot.system.updated.updated.os.path.isdir", return_value=True):
            self.assertTrue(self.updater.update_available)

  def test_update_available_when_overlay_matches_remote_but_running_does_not(self):
    """Overlay can match the remote tip while the running tree is still behind (stale overlay)."""
    self.updater.branches["release3"] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"

    with mock.patch.object(type(self.updater), "target_branch", new_callable=mock.PropertyMock, return_value="release3"):
      with mock.patch.object(self.updater, "get_commit_hash", side_effect=self._hashes(running="aaaaaaa", overlay="bbbbbbb")):
        with mock.patch.object(self.updater, "get_branch", side_effect=self._branches(running="release3")):
          with mock.patch("openpilot.system.updated.updated.os.path.isdir", return_value=True):
            self.assertTrue(self.updater.update_available)

  def _hashes(self, running: str, staged: str | None = None, overlay: str | None = None):
    staged = staged or running
    overlay = overlay or running

    def _hash(path: str) -> str:
      if path == BASEDIR:
        return running
      if path == FINALIZED:
        return staged
      if path == OVERLAY_MERGED:
        return overlay
      return running

    return _hash

  def _branches(self, running: str, staged: str | None = None):
    staged = staged or running

    def _branch(path: str) -> str:
      if path == FINALIZED:
        return staged
      return running

    return _branch


if __name__ == "__main__":
  unittest.main()
