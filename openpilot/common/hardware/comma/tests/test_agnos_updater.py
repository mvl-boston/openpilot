import json
import os
import requests

TEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)))
MANIFEST = os.path.join(TEST_DIR, "../agnos.json")
BASEDIR = os.path.abspath(os.path.join(TEST_DIR, "../../../../.."))

# All paths where an updater (past or present) may look for agnos.json when
# switching to this branch. Must stay in sync with AGNOS_MANIFEST_PATHS in
# openpilot/system/updated/updated.py. Kept as symlinks to the real manifest
# so OTA branch switches from older layouts keep working.
COMPAT_MANIFEST_PATHS = [
  "openpilot/system/hardware/comma/agnos.json",
  "openpilot/system/hardware/tici/agnos.json",
  "system/hardware/tici/agnos.json",
  "selfdrive/hardware/tici/agnos.json",
]


from openpilot.common.test import OpenpilotTestCase
class TestAgnosUpdater(OpenpilotTestCase):

  def test_compat_manifest_paths(self):
    real_manifest = os.path.realpath(MANIFEST)
    for rel_path in COMPAT_MANIFEST_PATHS:
      path = os.path.join(BASEDIR, rel_path)
      assert os.path.isfile(path), f"{rel_path} missing or dangling symlink"
      assert os.path.realpath(path) == real_manifest, f"{rel_path} does not resolve to {real_manifest}"
      with open(path) as f:
        json.load(f)

  def test_manifest(self):
    with open(MANIFEST) as f:
      m = json.load(f)

    for img in m:
      r = requests.head(img['url'], timeout=10)
      r.raise_for_status()
      assert r.headers['Content-Type'] == "application/x-xz"
      if not img['sparse']:
        assert img['hash'] == img['hash_raw']
