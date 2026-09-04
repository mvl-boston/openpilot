"""Pyserial compatibility shim for the prebuilt AGNOS updater zipapp.

Upstream removed pyserial (#38311) in favor of openpilot.common.serial, but the
updater LFS blob was never rebuilt and still does `import serial`. Putting this
package on PYTHONPATH lets that stale binary keep working without LFS access.
"""
from openpilot.common.serial import Serial, SerialException

# Old updater code referenced pyserial's VTIMESerial; our Serial is compatible.
VTIMESerial = Serial

__all__ = ["Serial", "SerialException", "VTIMESerial"]
