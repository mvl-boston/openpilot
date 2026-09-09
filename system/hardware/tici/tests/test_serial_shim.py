import importlib
import re
import subprocess
import sys
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
SHIM_DIR = REPO_ROOT / "system/hardware/tici/pyserial_compat"
AGNOS_PY = REPO_ROOT / "system/hardware/tici/agnos.py"
LAUNCH_SCRIPT = REPO_ROOT / "launch_chffrplus.sh"


@pytest.fixture
def shim_on_path(monkeypatch):
  # simulate an AGNOS without pyserial, with the shim dir on PYTHONPATH as launch_chffrplus.sh does
  saved_modules = dict(sys.modules)
  monkeypatch.syspath_prepend(str(SHIM_DIR))
  sys.modules.pop("serial", None)
  yield
  sys.modules.clear()
  sys.modules.update(saved_modules)


class TestSerialShim:
  def test_shim_exports(self, shim_on_path):
    serial = importlib.import_module("serial")
    from openpilot.common.serial import Serial, SerialException

    assert Path(serial.__file__).resolve() == (SHIM_DIR / "serial/__init__.py").resolve()
    assert serial.Serial.__name__ == Serial.__name__
    assert serial.SerialException.__name__ == SerialException.__name__
    assert issubclass(serial.SerialException, OSError)
    assert serial.VTIMESerial is serial.Serial

  def test_shim_with_updater_shadowing_openpilot(self, shim_on_path):
    # The updater zipapp bundles its own openpilot/ tree without common/serial.py,
    # which shadows the live checkout when both are on sys.path.
    fake_openpilot = types.ModuleType("openpilot")
    fake_openpilot.__path__ = []  # type: ignore[attr-defined]
    sys.modules["openpilot"] = fake_openpilot

    serial = importlib.import_module("serial")
    assert callable(serial.Serial)
    assert issubclass(serial.SerialException, OSError)
    assert serial.VTIMESerial is serial.Serial

  def test_launch_script_points_at_shim(self):
    # launch_chffrplus.sh must reference the directory that contains the serial/ package.
    script = LAUNCH_SCRIPT.read_text()
    m = re.search(r'PYTHONPATH="\$DIR/(?P<rel>[^"$:]+)\$\{PYTHONPATH', script)
    assert m is not None, "pyserial shim PYTHONPATH export missing from launch_chffrplus.sh"
    assert (REPO_ROOT / m.group("rel")).resolve() == SHIM_DIR.resolve()
    assert (SHIM_DIR / "serial/__init__.py").is_file()
    # the shim is only meant to fill in for a missing pyserial, never to shadow it
    assert 'if ! python3 -c "import serial"' in script
    # a crashing updater must not fall through to manager on a mismatched AGNOS
    assert re.search(r"while true; do\s+\$DIR/system/hardware/tici/updater \$AGNOS_PY \$MANIFEST\s+if \$AGNOS_PY --swap \$MANIFEST", script)

  def test_agnos_verify_does_not_need_pyserial(self):
    # First boot after a branch switch runs `agnos.py --verify` on the *previous*
    # branch's AGNOS, which may not ship pyserial (dropped in AGNOS 19.x). Loading
    # the script must not drag in openpilot.system.hardware -> tici.lpa -> serial.
    code = f"""
import importlib.util, sys
sys.modules['serial'] = None
sys.modules['openpilot.system.hardware'] = None
spec = importlib.util.spec_from_file_location('agnos_verify', {str(AGNOS_PY)!r})
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
assert callable(mod.verify_agnos_update) and callable(mod.flash_agnos_update)
"""
    subprocess.run([sys.executable, "-c", code], check=True, cwd=REPO_ROOT)
