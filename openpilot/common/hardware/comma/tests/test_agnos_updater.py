import json
import os
import tempfile
import requests

from openpilot.system.updated.updated import AGNOS_MANIFEST_PATHS, get_agnos_manifest_path

TEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)))
MANIFEST = os.path.join(TEST_DIR, "../agnos.json")
BASEDIR = os.path.abspath(os.path.join(TEST_DIR, "../../../../.."))

# All paths where an updater (past or present) may look for agnos.json when
# switching to this branch. Kept as symlinks to the real manifest so OTA branch
# switches from older layouts keep working.
COMPAT_MANIFEST_PATHS = [
  "openpilot/system/hardware/comma/agnos.json",
  "openpilot/system/hardware/tici/agnos.json",
  "system/hardware/tici/agnos.json",
]


from openpilot.common.test import OpenpilotTestCase
class TestAgnosUpdater(OpenpilotTestCase):

  def test_compat_manifest_paths(self):
    real_manifest = os.path.realpath(MANIFEST)
    for rel_path in COMPAT_MANIFEST_PATHS:
      assert rel_path in AGNOS_MANIFEST_PATHS, f"{rel_path} is not a path the updater looks at"
      path = os.path.join(BASEDIR, rel_path)
      assert os.path.isfile(path), f"{rel_path} missing or dangling symlink"
      assert os.path.realpath(path) == real_manifest, f"{rel_path} does not resolve to {real_manifest}"
      with open(path) as f:
        json.load(f)

  def test_manifest_resolution_across_layouts(self):
    # this branch's updater must find the manifest in a target checkout of any
    # layout generation, e.g. when switching to a branch that predates the
    # nested openpilot/ directory
    for rel_path in AGNOS_MANIFEST_PATHS:
      with tempfile.TemporaryDirectory() as basedir:
        manifest = os.path.join(basedir, rel_path)
        os.makedirs(os.path.dirname(manifest))
        with open(manifest, "w") as f:
          f.write("[]")
        assert get_agnos_manifest_path(basedir) == manifest

    with tempfile.TemporaryDirectory() as basedir:
      with self.assertRaises(FileNotFoundError):
        get_agnos_manifest_path(basedir)

  def test_manifest(self):
    with open(MANIFEST) as f:
      m = json.load(f)

    for img in m:
      r = requests.head(img['url'], timeout=10)
      r.raise_for_status()
      assert r.headers['Content-Type'] == "application/x-xz"
      if not img['sparse']:
        assert img['hash'] == img['hash_raw']
