"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.updated.updated import agnos_version_tuple, is_agnos_downgrade


class TestAgnosBranchSwitch(OpenpilotTestCase):
  def test_agnos_version_tuple(self):
    assert agnos_version_tuple("18.4") == (18, 4)
    assert agnos_version_tuple("19.7") == (19, 7)

  def test_is_agnos_downgrade(self):
    assert is_agnos_downgrade("19.7", "18.4")
    assert not is_agnos_downgrade("18.4", "19.7")
    assert not is_agnos_downgrade("19.7", "19.7")
