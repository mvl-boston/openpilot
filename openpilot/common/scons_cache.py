import os

_OBJECT_SUFFIXES = (".o", ".os", ".tmp")
_SKIP_DIRS = frozenset({".git", ".venv", "node_modules", "__pycache__"})


def _unlink_if_empty(path: str) -> bool:
  try:
    if os.path.getsize(path) == 0:
      os.unlink(path)
      return True
  except OSError:
    pass
  return False


def purge_zero_byte_artifacts(source_root: str, cache_dir: str | None = None) -> int:
  """Delete empty object files and cache entries left behind by interrupted builds.

  An interrupted compile can leave a 0-byte .o in the tree, which scons then
  copies into the cache and happily restores on every subsequent build, causing
  persistent link failures. Neither location can legitimately hold empty files.
  """
  removed = 0

  if os.path.isdir(source_root):
    for dirroot, dirs, files in os.walk(source_root):
      dirs[:] = [d for d in dirs if d not in _SKIP_DIRS]
      for name in files:
        if name.endswith(_OBJECT_SUFFIXES) and _unlink_if_empty(os.path.join(dirroot, name)):
          removed += 1

  if cache_dir and os.path.isdir(cache_dir):
    for dirroot, _, files in os.walk(cache_dir):
      for name in files:
        if _unlink_if_empty(os.path.join(dirroot, name)):
          removed += 1

  return removed
