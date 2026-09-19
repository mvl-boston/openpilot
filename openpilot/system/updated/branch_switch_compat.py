"""Inject AGNOS boot compatibility into target branch checkouts during OTA.

Dev branches (sp-honda-dev-202608, 0111-op-honda-dev) apply these patches when
finalizing an update to a branch that expects a different AGNOS than the device
is currently running, so release branches do not need their own branch-switch
fixes.
"""
import os
import shutil
import subprocess
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import AGNOS, HARDWARE

COMPAT_DATA = Path(__file__).resolve().parent / "branch_switch_compat_data"
MARKER = "sunnypilot branch-switch-compat"

LEGACY_LAUNCH_OLD = """    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST"""

LEGACY_LAUNCH_NEW = f"""    # {MARKER} (injected by dev-branch updater)
    if ! python3 -c "import serial" 2> /dev/null; then
      export PYTHONPATH="$DIR/system/hardware/tici/pyserial_compat${{PYTHONPATH:+:$PYTHONPATH}}"
    fi
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    while true; do
      $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
      if $AGNOS_PY --swap $MANIFEST; then
        sudo reboot
      fi
    done"""

NESTED_SWAP_OLD = """    while true; do
      $DIR/openpilot/common/hardware/comma/updater $AGNOS_PY $MANIFEST
    done"""

NESTED_SWAP_NEW = f"""    while true; do
      $DIR/openpilot/common/hardware/comma/updater $AGNOS_PY $MANIFEST
      if $AGNOS_PY --swap $MANIFEST; then
        sudo reboot
      fi
    done"""

CASNYC_MODULE_IMPORT = "import openpilot.system.updated.casync.casync as casync\n"
CASNYC_LAZY_PREFIX = """  # sunnypilot branch-switch-compat: lazy casync import
  import openpilot.system.updated.casync.casync as casync

"""


def agnos_version_tuple(version: str) -> tuple[int, ...]:
  return tuple(int(part) for part in version.split("."))


def is_agnos_downgrade(current: str, target: str) -> bool:
  return agnos_version_tuple(target) < agnos_version_tuple(current)


def get_target_agnos_version(basedir: str) -> str:
  return subprocess.check_output(
    ["bash", "-c", r"unset AGNOS_VERSION && source launch_env.sh && echo -n $AGNOS_VERSION"],
    cwd=basedir,
    stderr=subprocess.STDOUT,
    encoding="utf8",
  ).strip()


def is_nested_layout(basedir: str) -> bool:
  return os.path.isfile(os.path.join(basedir, "openpilot", "system", "manager", "manager.py"))


def is_legacy_layout(basedir: str) -> bool:
  return os.path.isfile(os.path.join(basedir, "system", "manager", "manager.py"))


def needs_branch_switch_compat(basedir: str) -> bool:
  if not AGNOS:
    return False
  return HARDWARE.get_os_version() != get_target_agnos_version(basedir)


def apply_branch_switch_compat(basedir: str) -> None:
  if not needs_branch_switch_compat(basedir):
    return

  cloudlog.info(f"Applying {MARKER} patches under {basedir}")
  if is_legacy_layout(basedir):
    _apply_legacy_compat(basedir)
  elif is_nested_layout(basedir):
    _apply_nested_compat(basedir)
  else:
    cloudlog.warning(f"{MARKER}: unrecognized layout under {basedir}, skipping patch")


def _apply_legacy_compat(basedir: str) -> None:
  _install_legacy_pyserial_shim(basedir)
  _ensure_comma_agnos_symlink(basedir)
  _patch_legacy_agnos_lazy_casync(basedir)
  _patch_legacy_launch(basedir)


def _apply_nested_compat(basedir: str) -> None:
  _patch_nested_launch(basedir)


def _install_legacy_pyserial_shim(basedir: str) -> None:
  common_serial = os.path.join(basedir, "common", "serial.py")
  if not os.path.isfile(common_serial):
    os.makedirs(os.path.dirname(common_serial), exist_ok=True)
    shutil.copy2(COMPAT_DATA / "legacy_common_serial.py", common_serial)

  dest = os.path.join(basedir, "system", "hardware", "tici", "pyserial_compat")
  if not os.path.isdir(dest):
    shutil.copytree(COMPAT_DATA / "legacy_pyserial_compat", dest)


def _ensure_comma_agnos_symlink(basedir: str) -> None:
  tici_manifest = os.path.join(basedir, "system", "hardware", "tici", "agnos.json")
  comma_manifest = os.path.join(basedir, "system", "hardware", "comma", "agnos.json")
  if os.path.isfile(tici_manifest) and not os.path.exists(comma_manifest):
    os.makedirs(os.path.dirname(comma_manifest), exist_ok=True)
    os.symlink("../tici/agnos.json", comma_manifest)


def _patch_legacy_agnos_lazy_casync(basedir: str) -> None:
  agnos_py = os.path.join(basedir, "system", "hardware", "tici", "agnos.py")
  if not os.path.isfile(agnos_py):
    return

  text = Path(agnos_py).read_text()
  if MARKER in text or CASNYC_MODULE_IMPORT not in text:
    return

  text = text.replace(CASNYC_MODULE_IMPORT, "\n", 1)
  if "def extract_casync_image" not in text:
    return

  text = text.replace(
    "def extract_casync_image(target_slot_number: int, partition: dict, cloudlog):\n  path = get_partition_path",
    "def extract_casync_image(target_slot_number: int, partition: dict, cloudlog):\n" + CASNYC_LAZY_PREFIX + "  path = get_partition_path",
    1,
  )
  Path(agnos_py).write_text(text)


def _patch_legacy_launch(basedir: str) -> None:
  launch = os.path.join(basedir, "launch_chffrplus.sh")
  if not os.path.isfile(launch):
    return

  text = Path(launch).read_text()
  if MARKER in text:
    return
  if LEGACY_LAUNCH_OLD not in text:
    if "while true; do" in text and "--swap" in text:
      return
    cloudlog.warning(f"{MARKER}: legacy launch pattern not found in {launch}")
    return

  Path(launch).write_text(text.replace(LEGACY_LAUNCH_OLD, LEGACY_LAUNCH_NEW, 1))


def _patch_nested_launch(basedir: str) -> None:
  launch = os.path.join(basedir, "launch_chffrplus.sh")
  if not os.path.isfile(launch):
    return

  text = Path(launch).read_text()
  if MARKER in text or "--swap" in text:
    return
  if NESTED_SWAP_OLD not in text:
    return

  text = text.replace(NESTED_SWAP_OLD, NESTED_SWAP_NEW, 1)
  if "export PYTHONPATH=\"$DIR" not in text:
    text = text.replace(
      'MANIFEST="$DIR/openpilot/system/hardware/comma/agnos.json"\n',
      'MANIFEST="$DIR/openpilot/system/hardware/comma/agnos.json"\n'
      f'    # {MARKER} (injected by dev-branch updater)\n'
      '    export PYTHONPATH="$DIR${PYTHONPATH:+:$PYTHONPATH}"\n',
      1,
    )
  Path(launch).write_text(text)
