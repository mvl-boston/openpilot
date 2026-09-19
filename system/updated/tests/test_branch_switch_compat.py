"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import tempfile
from pathlib import Path

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.updated.branch_switch_compat import MARKER, apply_branch_switch_compat, is_agnos_downgrade


class TestBranchSwitchCompat(OpenpilotTestCase):
  def test_is_agnos_downgrade(self):
    assert is_agnos_downgrade("19.7", "18.4")
    assert not is_agnos_downgrade("18.4", "19.7")

  def test_legacy_launch_patch(self, mocker):
    mocker.patch("openpilot.system.updated.branch_switch_compat.AGNOS", True)
    mocker.patch("openpilot.system.updated.branch_switch_compat.HARDWARE.get_os_version", return_value="19.7")

    with tempfile.TemporaryDirectory() as tmp:
      basedir = Path(tmp)
      (basedir / "system" / "manager").mkdir(parents=True)
      (basedir / "system" / "manager" / "manager.py").write_text("# stub\n")
      (basedir / "launch_env.sh").write_text('export AGNOS_VERSION="18.4"\n')

      launch = basedir / "launch_chffrplus.sh"
      launch.write_text("""function agnos_init {
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}
""")

      apply_branch_switch_compat(str(basedir))
      patched = launch.read_text()
      assert MARKER in patched
      assert "--swap" in patched
      assert (basedir / "common" / "serial.py").is_file()
