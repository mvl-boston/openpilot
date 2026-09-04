import importlib
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[5]
UPDATER = REPO_ROOT / "openpilot/common/hardware/comma/updater"


class TestSerialShim(unittest.TestCase):
  def test_serial_shim_exports(self):
    with patch.dict(sys.modules, {"serial": None}):
      sys.modules.pop("serial", None)
      serial = importlib.import_module("serial")
      from openpilot.common.serial import Serial, SerialException

      self.assertEqual(serial.Serial.__name__, Serial.__name__)
      self.assertEqual(serial.SerialException.__name__, SerialException.__name__)
      self.assertIs(serial.VTIMESerial, serial.Serial)

  def test_serial_shim_with_updater_shadowing_openpilot(self):
    if not UPDATER.is_file() or UPDATER.read_bytes()[:2] != b"PK":
      self.skipTest("updater zipapp not available")

    # The zipapp bundles openpilot/ without common/serial.py, which shadows the
    # live checkout when both are on sys.path.
    fake_openpilot = types.ModuleType("openpilot")
    fake_openpilot.__path__ = []  # type: ignore[attr-defined]
    sys.modules["openpilot"] = fake_openpilot

    try:
      sys.modules.pop("serial", None)
      serial = importlib.import_module("serial")
      self.assertTrue(callable(serial.Serial))
      self.assertTrue(issubclass(serial.SerialException, OSError))
      self.assertIs(serial.VTIMESerial, serial.Serial)
    finally:
      sys.modules.pop("serial", None)
      sys.modules.pop("openpilot", None)


if __name__ == "__main__":
  unittest.main()
