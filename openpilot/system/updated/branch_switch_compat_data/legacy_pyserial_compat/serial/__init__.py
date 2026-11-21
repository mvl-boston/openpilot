"""Pyserial compatibility shim for running this branch on a newer AGNOS.

AGNOS 19.x no longer ships pyserial (upstream dropped it in commaai/openpilot#38311),
but this branch, and the prebuilt AGNOS updater zipapp it uses, still `import serial`
through openpilot.system.hardware -> tici.lpa. When a device switches to this branch
from a newer branch, the first boot runs this branch's code on the *newer* AGNOS until
agnos.py --verify and the updater have flashed and swapped to the matching AGNOS. If
`import serial` fails there, verify and the updater both crash and the device stays on
the boot logo.

launch_chffrplus.sh puts this directory's parent on PYTHONPATH only when the running
AGNOS has no importable `serial` package, so the real pyserial is never shadowed on a
matching AGNOS.

The updater zipapp bundles a stale openpilot tree that shadows the live checkout, so the
implementation is loaded from common/serial.py by path instead of importing
openpilot.common.serial.
"""
import importlib.util
from pathlib import Path

_SERIAL_PATH = Path(__file__).resolve().parents[5] / "common" / "serial.py"
_spec = importlib.util.spec_from_file_location("_openpilot_common_serial", _SERIAL_PATH)
if _spec is None or _spec.loader is None:
  raise ImportError(f"serial shim: could not load {_SERIAL_PATH}")
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

Serial = _mod.Serial
SerialException = _mod.SerialException

# pigeond and older updater builds reference pyserial's VTIMESerial; our Serial is compatible.
VTIMESerial = Serial

__all__ = ["Serial", "SerialException", "VTIMESerial"]
