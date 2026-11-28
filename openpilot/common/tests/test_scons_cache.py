import os
import tempfile

from openpilot.common.scons_cache import purge_zero_byte_artifacts
from openpilot.common.test import OpenpilotTestCase


def _touch(path: str, size: int) -> None:
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "wb") as f:
    f.write(b"\0" * size)


class TestSconsCache(OpenpilotTestCase):
  def test_purge_zero_byte_artifacts(self):
    with tempfile.TemporaryDirectory() as source_root, tempfile.TemporaryDirectory() as cache_dir:
      keep = [
        os.path.join(source_root, "good.o"),
        os.path.join(source_root, "empty_source.cc"),  # non-object files are left alone
        os.path.join(source_root, ".git", "objects", "empty.o"),  # vcs dirs are skipped
        os.path.join(cache_dir, "good_cache"),
      ]
      remove = [
        os.path.join(source_root, "nested", "empty.o"),
        os.path.join(source_root, "empty.os"),
        os.path.join(source_root, "work.tmp"),
        os.path.join(cache_dir, "AB", "empty_cache"),
      ]

      for path in keep:
        _touch(path, 0 if "empty" in path else 8)
      for path in remove:
        _touch(path, 0)

      assert purge_zero_byte_artifacts(source_root, cache_dir) == len(remove)
      for path in keep:
        assert os.path.exists(path), path
      for path in remove:
        assert not os.path.exists(path), path

  def test_missing_dirs(self):
    assert purge_zero_byte_artifacts("/nonexistent/source", "/nonexistent/cache") == 0
    assert purge_zero_byte_artifacts("/nonexistent/source", None) == 0
