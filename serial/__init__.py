"""Pyserial compatibility shim for the prebuilt AGNOS updater zipapp.

Upstream removed pyserial (#38311) in favor of openpilot.common.serial, but the
updater LFS blob was never rebuilt and still does `import serial`. The zipapp
also bundles a stale openpilot tree that shadows the live checkout, so this
shim loads serial.py from disk instead of importing openpilot.common.serial.
"""
import importlib.util
from pathlib import Path

_SERIAL_PATH = Path(__file__).resolve().parent.parent / "openpilot" / "common" / "serial.py"
_spec = importlib.util.spec_from_file_location("_openpilot_common_serial", _SERIAL_PATH)
if _spec is None or _spec.loader is None:
  raise ImportError(f"serial shim: could not load {_SERIAL_PATH}")
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

Serial = _mod.Serial
SerialException = _mod.SerialException

# Old updater code referenced pyserial's VTIMESerial; our Serial is compatible.
VTIMESerial = Serial

__all__ = ["Serial", "SerialException", "VTIMESerial"]
