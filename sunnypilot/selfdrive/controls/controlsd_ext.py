"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import numpy as np

import cereal.messaging as messaging
from cereal import log, custom

from opendbc.car import structs
from opendbc.sunnypilot.car.honda.lane_path import LANE_LENGTH_MAX_VALUE
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0

# OP lane render for the Honda Bosch radarless dash
DASH_PATH_FIT_MAX = 110.0
DASH_PATH_PROB_ON = 0.25         # lane-line existence prob to start drawing
DASH_PATH_PROB_OFF = 0.10
DASH_HALF_OFFSET = 1.65          # m, half lane width when only one lane available
DASH_PATH_FULL_LEN_SPEED = 27.0  # m/s for full draw length
DASH_PATH_LEAD_FULL_DIST = 70.0  # m lead distance for full draw length
DASH_PATH_MIN_REACH = 0.15       # min draw fraction (short stub when stopped / low speed)


def _line_trusted(prob: float, was_on: bool) -> bool:
  # hysteresis to prevent flicker
  return prob >= (DASH_PATH_PROB_OFF if was_on else DASH_PATH_PROB_ON)


def _fit_cubic(x: np.ndarray, y: np.ndarray) -> list[float] | None:
  # cubic [c0..c3] over x <= DASH_PATH_FIT_MAX; None if too few points in range
  m = x <= DASH_PATH_FIT_MAX
  if m.sum() < 4:
    return None
  return [float(v) for v in np.polyfit(x[m], y[m], 3)[::-1]]


def select_lane_render(model: log.ModelDataV2, prev_left: bool, prev_right: bool) -> tuple[list[float] | None, bool, bool]:
  """Dash center cubic + which ego lines to draw, from per-side model confidence."""
  lls, probs = model.laneLines, model.laneLineProbs
  if len(lls) < 3 or len(probs) < 3 or len(lls[1].x) == 0:
    return None, False, False

  left = _line_trusted(probs[1], prev_left)
  right = _line_trusted(probs[2], prev_right)
  x = np.array(lls[1].x)
  yl, yr = np.array(lls[1].y), np.array(lls[2].y)
  if left and right:
    poly = _fit_cubic(x, (yl + yr) / 2.0)
  elif right:
    poly = _fit_cubic(x, yr - DASH_HALF_OFFSET)
  elif left:
    poly = _fit_cubic(x, yl + DASH_HALF_OFFSET)
  else:
    return None, False, False
  return (poly, left, right) if poly is not None else (None, False, False)


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()
    self._left_on = False   # dash lane-line
    self._right_on = False

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP']
    self.pm_services_ext = ['carControlSP']

  def initialize_lateral_control(self, lac, CI, dt):
    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_versions = self.params.get("TorqueControlTune")
    if not enforce_torque_control:
      if self.CP.lateralTuning.which() == 'torque':
        return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)  # FIXME-SP: revert when upstream fixes tuning issues with v1
      return lac

    if torque_versions == 0.0:  # v0
      return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
    else:
      return lac

  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["liveDelay"].lateralDelay)

      self._param_update_time = time.monotonic()

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    if self.blinker_pause_lateral.update(sm['carState']):
      return False

    ss_sp = sm['selfdriveStateSP']
    if ss_sp.mads.available:
      return bool(ss_sp.mads.active)

    # MADS not available, use stock state to engage
    return bool(sm['selfdriveState'].active)

  @staticmethod
  def get_lead_data(ld: log.RadarState.LeadData) -> dict:
    return {
      "dRel": ld.dRel,
      "yRel": ld.yRel,
      "vRel": ld.vRel,
      "aRel": ld.aRel,
      "vLead": ld.vLead,
      "dPath": ld.dPath,
      "vLat": ld.vLat,
      "vLeadK": ld.vLeadK,
      "aLeadK": ld.aLeadK,
      "fcw": ld.fcw,
      "status": ld.status,
      "aLeadTau": ld.aLeadTau,
      "modelProb": ld.modelProb,
      "radar": ld.radar,
      "radarTrackId": ld.radarTrackId,
    }

  def get_dash_path(self, model: log.ModelDataV2, model_valid: bool, v_ego: float, lead_d: float) -> dict:
    """Build the DashPath payload (lane center + reach + per-side lines) for the radarless cluster.

    Returns a blank (draws nothing) when the model is invalid or no ego line is confident. reach grows
    the drawn length with speed and lead distance.
    """
    blank = {"valid": False, "poly": [], "reach": 0.0, "laneCross": 0, "leftLine": False, "rightLine": False}

    poly, left_on, right_on = (None, False, False)
    if model_valid:
      poly, left_on, right_on = select_lane_render(model, self._left_on, self._right_on)
    if poly is None:
      return blank
    self._left_on, self._right_on = left_on, right_on

    reach = float(np.clip(max(v_ego / DASH_PATH_FULL_LEN_SPEED, lead_d / DASH_PATH_LEAD_FULL_DIST, DASH_PATH_MIN_REACH), 0.0, 1.0))
    if round(reach * LANE_LENGTH_MAX_VALUE) <= 0:
      return blank
    return {"valid": True, "poly": poly, "reach": reach, "laneCross": 0,
            "leftLine": left_on, "rightLine": right_on}

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    CC_SP.leadOne = self.get_lead_data(sm['radarState'].leadOne)
    CC_SP.leadTwo = self.get_lead_data(sm['radarState'].leadTwo)

    # MADS state
    CC_SP.mads = sm['selfdriveStateSP'].mads

    CC_SP.intelligentCruiseButtonManagement = sm['selfdriveStateSP'].intelligentCruiseButtonManagement

    # OP lane for the Honda Bosch radarless dash (LANE_PATH)
    lead = sm['radarState'].leadOne
    lead_d = lead.dRel if lead.status else 0.0   # extend the lane out to the lead (0 = no lead)
    CC_SP.dashPath = self.get_dash_path(sm['modelV2'], sm.valid['modelV2'], sm['carState'].vEgo, lead_d)

    return CC_SP

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)
