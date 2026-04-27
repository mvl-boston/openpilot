import json
from unittest.mock import Mock

from openpilot.selfdrive.car.persistent_state import HONDA_BRAKE_PID_PARAMS_KEY, cache_honda_brake_pid_state, restore_honda_brake_pid_state


class TestBrakePIDPersistentState:
  def test_restore_honda_brake_pid_state(self):
    params = Mock()
    controller = Mock()
    params.get.return_value = json.dumps({
      "version": 1,
      "carFingerprint": "HONDA_CIVIC",
      "brakePIDFactorNonLowSpeed": 0.62,
    }).encode()

    restore_honda_brake_pid_state(params, controller)

    params.get.assert_called_once_with(HONDA_BRAKE_PID_PARAMS_KEY)
    controller.set_persistent_state.assert_called_once_with({
      "version": 1,
      "carFingerprint": "HONDA_CIVIC",
      "brakePIDFactorNonLowSpeed": 0.62,
    })

  def test_restore_honda_brake_pid_state_removes_corrupt_cache(self):
    params = Mock()
    controller = Mock()
    params.get.return_value = b"{not-json"

    restore_honda_brake_pid_state(params, controller)

    controller.set_persistent_state.assert_not_called()
    params.remove.assert_called_once_with(HONDA_BRAKE_PID_PARAMS_KEY)

  def test_cache_honda_brake_pid_state(self):
    params = Mock()
    controller = Mock()
    controller.get_persistent_state.return_value = {
      "version": 1,
      "carFingerprint": "HONDA_CIVIC",
      "brakePIDFactorNonLowSpeed": 0.62,
    }

    cache_honda_brake_pid_state(params, controller)

    params.put_nonblocking.assert_called_once()
    key, payload = params.put_nonblocking.call_args.args
    assert key == HONDA_BRAKE_PID_PARAMS_KEY
    assert json.loads(payload) == {
      "version": 1,
      "carFingerprint": "HONDA_CIVIC",
      "brakePIDFactorNonLowSpeed": 0.62,
    }

  def test_cache_honda_brake_pid_state_skips_none(self):
    params = Mock()
    controller = Mock()
    controller.get_persistent_state.return_value = None

    cache_honda_brake_pid_state(params, controller)

    params.put_nonblocking.assert_not_called()
