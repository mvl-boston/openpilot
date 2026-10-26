import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openpilot.common.basedir import BASEDIR
from openpilot.system.updated import branch_switch_compat as compat

LEGACY_LAUNCH = f"""#!/usr/bin/env bash
function agnos_init {{
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
{compat.LEGACY_LAUNCH_OLD}
  fi
}}
"""

LEGACY_AGNOS_PY = """import json
import os

import openpilot.system.updated.casync.casync as casync


def get_partition_path(target_slot_number, partition):
  return "/dev/null"


def extract_casync_image(target_slot_number: int, partition: dict, cloudlog):
  path = get_partition_path(target_slot_number, partition)
  return casync.parse_caibx(partition['casync_caibx'])
"""


def make_legacy_tree(root: str, agnos_version: str = "18.4") -> None:
  tree = Path(root)
  (tree / "system" / "manager").mkdir(parents=True)
  (tree / "system" / "manager" / "manager.py").write_text("")
  (tree / "system" / "hardware" / "tici").mkdir(parents=True)
  (tree / "system" / "hardware" / "tici" / "agnos.json").write_text("[]")
  (tree / "system" / "hardware" / "tici" / "agnos.py").write_text(LEGACY_AGNOS_PY)
  (tree / "common").mkdir()
  (tree / "launch_chffrplus.sh").write_text(LEGACY_LAUNCH)
  (tree / "launch_env.sh").write_text(f'if [ -z "$AGNOS_VERSION" ]; then\n  export AGNOS_VERSION="{agnos_version}"\nfi\n')


class TestBranchSwitchCompat(unittest.TestCase):
  def test_is_agnos_downgrade(self):
    self.assertTrue(compat.is_agnos_downgrade("19.9", "18.4"))
    self.assertTrue(compat.is_agnos_downgrade("19.10", "19.9"))
    self.assertFalse(compat.is_agnos_downgrade("19.9", "19.9"))
    self.assertFalse(compat.is_agnos_downgrade("18.4", "19.9"))

  def test_target_agnos_version_from_launch_env(self):
    with tempfile.TemporaryDirectory() as tmp:
      make_legacy_tree(tmp, "18.4")
      self.assertEqual(compat.get_target_agnos_version(tmp), "18.4")
      self.assertTrue(compat.is_legacy_layout(tmp))
      self.assertFalse(compat.is_legacy_layout(BASEDIR))

  def test_patch_legacy_tree(self):
    with tempfile.TemporaryDirectory() as tmp:
      make_legacy_tree(tmp)
      compat.patch_legacy_tree(tmp)

      # pyserial shim loads common/serial.py copied from the running checkout
      self.assertTrue(os.path.isfile(os.path.join(tmp, "common", "serial.py")))
      shim_root = os.path.join(tmp, "system", "hardware", "tici", "pyserial_compat")
      self.assertTrue(os.path.isfile(os.path.join(shim_root, "serial", "__init__.py")))
      out = subprocess.check_output(
        [sys.executable, "-c", "import serial; print(serial.Serial.__name__, serial.VTIMESerial is serial.Serial)"],
        env={**os.environ, "PYTHONPATH": shim_root}, encoding="utf8",
      ).strip()
      self.assertEqual(out, "Serial True")

      # cross-layout manifest symlink
      comma_manifest = os.path.join(tmp, "system", "hardware", "comma", "agnos.json")
      self.assertTrue(os.path.islink(comma_manifest))
      self.assertEqual(os.path.realpath(comma_manifest), os.path.realpath(os.path.join(tmp, "system", "hardware", "tici", "agnos.json")))

      # casync import is deferred into extract_casync_image, so --verify works without casync deps
      agnos_py = Path(tmp, "system", "hardware", "tici", "agnos.py").read_text()
      self.assertNotIn("\nimport openpilot.system.updated.casync.casync as casync\n", agnos_py)
      self.assertIn(compat.CASNYC_LAZY_PREFIX, agnos_py)
      compile(agnos_py, "agnos.py", "exec")

      # launch loops the updater and swaps itself
      launch = Path(tmp, "launch_chffrplus.sh").read_text()
      self.assertIn(compat.MARKER, launch)
      self.assertIn("--swap", launch)
      self.assertIn("pyserial_compat", launch)
      self.assertNotIn(compat.LEGACY_LAUNCH_OLD, launch)
      subprocess.check_call(["bash", "-n", os.path.join(tmp, "launch_chffrplus.sh")])

      # idempotent
      compat.patch_legacy_tree(tmp)
      self.assertEqual(launch, Path(tmp, "launch_chffrplus.sh").read_text())
      self.assertEqual(agnos_py, Path(tmp, "system", "hardware", "tici", "agnos.py").read_text())

  def test_apply_only_on_agnos_downgrade_to_legacy_layout(self):
    with tempfile.TemporaryDirectory() as tmp, \
         mock.patch.object(compat, "AGNOS", True), \
         mock.patch.object(compat.HARDWARE, "get_os_version", return_value="19.9"):
      make_legacy_tree(tmp, "19.9")
      compat.apply_branch_switch_compat(tmp)
      self.assertNotIn(compat.MARKER, Path(tmp, "launch_chffrplus.sh").read_text())

      Path(tmp, "launch_env.sh").write_text('export AGNOS_VERSION="18.4"\n')
      compat.apply_branch_switch_compat(tmp)
      self.assertIn(compat.MARKER, Path(tmp, "launch_chffrplus.sh").read_text())

  def test_apply_never_raises(self):
    with tempfile.TemporaryDirectory() as tmp, \
         mock.patch.object(compat, "AGNOS", True), \
         mock.patch.object(compat.HARDWARE, "get_os_version", return_value="19.9"):
      compat.apply_branch_switch_compat(tmp)  # no launch_env.sh at all


if __name__ == "__main__":
  unittest.main()
