import json
from typing import Any

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

HONDA_BRAKE_PID_PARAMS_KEY = "HondaBrakePIDParams"


def restore_honda_brake_pid_state(params: Params, controller: Any | None) -> None:
  if controller is None or not hasattr(controller, "set_persistent_state"):
    return

  cached_state = params.get(HONDA_BRAKE_PID_PARAMS_KEY)
  if cached_state is None:
    return

  try:
    if isinstance(cached_state, bytes):
      cached_state = cached_state.decode("utf-8")
    controller.set_persistent_state(json.loads(cached_state))
  except Exception:
    cloudlog.exception("failed to restore Honda brake PID params")
    params.remove(HONDA_BRAKE_PID_PARAMS_KEY)


def cache_honda_brake_pid_state(params: Params, controller: Any | None) -> None:
  if controller is None or not hasattr(controller, "get_persistent_state"):
    return

  try:
    persistent_state = controller.get_persistent_state()
    if persistent_state is not None:
      params.put_nonblocking(HONDA_BRAKE_PID_PARAMS_KEY, json.dumps(persistent_state))
  except Exception:
    cloudlog.exception("failed to cache Honda brake PID params")
