import importlib
import sys
import unittest
from unittest.mock import patch


class TestSerialShim(unittest.TestCase):
  def test_serial_shim_exports(self):
    # Simulate device boot: repo root on PYTHONPATH, no pyserial installed.
    with patch.dict(sys.modules, {"serial": None}):
      sys.modules.pop("serial", None)
      serial = importlib.import_module("serial")
      from openpilot.common.serial import Serial, SerialException

      self.assertIs(serial.Serial, Serial)
      self.assertIs(serial.SerialException, SerialException)
      self.assertIs(serial.VTIMESerial, Serial)


if __name__ == "__main__":
  unittest.main()
