import math
import numpy as np

from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.nrdr_long_tune import LongTune

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState

# Bundle D / L2 — hard safety constant (CODE, not a tune field).
# Once true standstill is confirmed the held decel must never be WEAKER (less
# negative) than this, so the car cannot roll back on a grade while the
# pitch-aware floor is still being validated on the road. The pitch-aware floor
# is allowed to go MORE negative than this; this only caps the weak side.
STOPPING_HARD_HOLD_FLOOR = -1.0  # m/s^2

ACCEL_DUE_TO_GRAVITY = 9.81  # m/s^2 (local to keep the pure fn cereal-free)

# Proximity low-pass: rolling minimum over this many frames feeds a 1-pole filter
# so a single noisy dRel sample can't briefly speed the stopping ramp up.
_DREL_ROLLING_FRAMES = 10
_DREL_FILTER_ALPHA = 0.3  # 1-pole blend toward the rolling-min each frame


def compute_stopping_accel(last_output_accel, stop_accel, stopping_decel_rate,
                           v_ego, v_ego_stopping, hold_accel, phase_switch_v,
                           proximity_scale_m, pitch_margin, drel_filtered, pitch):
  """Pure two-phase stopping-ramp shape for Bundle D / L2.

  Returns the next stopping-state output accel. Provably INERT above
  v_ego_stopping (returns the stock monotonic ramp toward stop_accel) so highway
  behavior is bit-identical to stock when the caller only invokes this in the
  stopping state. Every plumbed signal (drel_filtered, pitch) is isfinite-gated
  with a stock fallback so NaN can never reach the commanded accel.

  Phases (only below v_ego_stopping):
    A) vEgo > phase_switch_v  -> ramp toward the gentle hold (hold_accel), removing
       the head-bob of slamming straight to stop_accel at the moment of the stop.
    B) vEgo <= phase_switch_v -> true standstill; ramp toward the pitch-aware hold
       floor, hard-capped so it is never WEAKER than STOPPING_HARD_HOLD_FLOOR and
       never STRONGER than stop_accel.

  Rate shaping (one ramp shape, two complementary terms):
    rate_eff = stopping_decel_rate
               * interp(vEgo, [0, 0.3, v_ego_stopping], [0.3, 0.7, 1.0])      # speed
               * min(1.0, drel_filtered / proximity_scale_m)                  # proximity
  """
  # --- stock fallback values (used both above v_ego_stopping and on bad inputs) ---
  if not math.isfinite(last_output_accel):
    last_output_accel = 0.0

  # INERT above the stopping speed window: reproduce the stock monotonic ramp so
  # highway behavior is provably unchanged (invariant 1).
  if not (math.isfinite(v_ego) and v_ego <= v_ego_stopping):
    out = last_output_accel
    if out > stop_accel:
      out = min(out, 0.0)
      out -= stopping_decel_rate * DT_CTRL
    return out

  # --- speed-scaled rate term ---
  v_clamped = max(0.0, v_ego)
  speed_scale = float(np.interp(v_clamped, [0.0, 0.3, max(0.3 + 1e-6, v_ego_stopping)],
                                [0.3, 0.7, 1.0]))

  # --- proximity rate term (inert == 1.0 when no valid lead) ---
  prox_scale = 1.0
  if math.isfinite(drel_filtered) and proximity_scale_m > 0.0:
    prox_scale = min(1.0, max(0.0, drel_filtered) / proximity_scale_m)

  rate_eff = stopping_decel_rate * speed_scale * prox_scale

  at_standstill = v_ego <= phase_switch_v

  if not at_standstill:
    # Phase A — gentle approach: ramp toward the gentle hold, not stop_accel.
    target = hold_accel
  else:
    # Phase B — true standstill: pitch-aware hold floor.
    pitch_term = 0.0
    if math.isfinite(pitch):
      pitch_term = -ACCEL_DUE_TO_GRAVITY * math.sin(pitch) * pitch_margin
    pitch_aware_floor = hold_accel + pitch_term
    # Allowed to go MORE negative than the hard floor; never WEAKER than it.
    target = min(pitch_aware_floor, STOPPING_HARD_HOLD_FLOOR)
    # Never command stronger braking than stop_accel.
    target = max(target, stop_accel)

  # Monotone ramp from the current output toward the phase target.
  out = last_output_accel
  if out > target:
    out = min(out, 0.0)
    out = max(target, out - rate_eff * DT_CTRL)
  return out


def get_param_float(params, key, default, min_value=None, max_value=None, scale=1.0):
  value = params.get(key)
  if value is None:
    ret = default
  else:
    try:
      # sunnypilot Params.get() auto-casts by declared key type, so an INT param
      # comes back as a python int (not bytes). Handle bytes/str/number uniformly.
      if isinstance(value, bytes):
        value = value.decode("utf-8")
      ret = float(value) / scale
    except (AttributeError, TypeError, ValueError):
      ret = default

  if min_value is not None:
    ret = max(min_value, ret)
  if max_value is not None:
    ret = min(max_value, ret)
  return ret


def long_control_state_trans(CP, CP_SP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill, v_ego_starting=None):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_SP.enableGasInterceptor

  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > (CP.vEgoStarting if v_ego_starting is None else v_ego_starting)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state


class LongControl:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.params = Params()
    self.last_output_accel = 0.0
    self.frame = 0

    # Live-tunable longitudinal params; default to the car's configured values so
    # an unset param means "no change from stock".
    self.long_pid_tune_scale = 1.0
    self.stop_accel = CP.stopAccel
    self.stopping_decel_rate = CP.stoppingDecelRate
    self.v_ego_starting = CP.vEgoStarting
    self.v_ego_stopping = CP.vEgoStopping

    # Bundle D / L2 stopping shape. longcontrol runs in controlsd, a separate process from
    # the planner (plannerd), so it cannot share the planner's mpc.tune instance — construct a
    # local LongTune reading the same /data/nrdr_long_tune.json and refresh it on the existing
    # live-param poll cadence below (no extra IO churn at 100 Hz).
    self.tune = LongTune()
    # dRel proximity low-pass: rolling-min window + 1-pole filter (L2 spec).
    self._drel_window: list[float] = []
    self._drel_filtered = float("inf")

  def reset(self):
    self.pid.reset()

  def _read_live_params(self):
    self.long_pid_tune_scale = get_param_float(self.params, "LongPidTuneScale", 1.0, 0.0, 5.0, scale=100.0)
    self.stop_accel = get_param_float(self.params, "HondaStopAccel", self.CP.stopAccel, -10.0, 0.0)
    self.stopping_decel_rate = get_param_float(self.params, "HondaStoppingDecelRateLong", self.CP.stoppingDecelRate, 0.0, 5.0)
    self.v_ego_starting = get_param_float(self.params, "HondaVEgoStarting", self.CP.vEgoStarting, 0.0, 5.0)
    self.v_ego_stopping = get_param_float(self.params, "HondaVEgoStopping", self.CP.vEgoStopping, 0.0, 5.0)

  def _update_drel_filter(self, drel):
    """Rolling-min over _DREL_ROLLING_FRAMES then a 1-pole low-pass (L2 proximity term).
    Returns the filtered dRel, or +inf when there is no valid lead (proximity term inert)."""
    if drel is None or not math.isfinite(drel):
      # No valid lead this frame: clear history so a stale near value can't linger.
      self._drel_window = []
      self._drel_filtered = float("inf")
      return float("inf")
    self._drel_window.append(float(drel))
    if len(self._drel_window) > _DREL_ROLLING_FRAMES:
      self._drel_window.pop(0)
    rolling_min = min(self._drel_window)
    if not math.isfinite(self._drel_filtered):
      self._drel_filtered = rolling_min  # seed on first valid sample (no startup transient)
    else:
      self._drel_filtered = (_DREL_FILTER_ALPHA * rolling_min
                             + (1.0 - _DREL_FILTER_ALPHA) * self._drel_filtered)
    return self._drel_filtered

  def update(self, active, CS, a_target, should_stop, accel_limits, pitch=None, drel=None):
    """Update longitudinal control. This updates the state machine and runs a PID loop.

    pitch (rad) and drel (m) are optional Bundle D / L2 plumbing from controlsd:
      pitch <- calibrated_pose.orientation.xyz[1] (same source as carControl.orientationNED[1])
      drel  <- longitudinalPlan.leadTrajectoryX0[0] when a lead is present, else None
    Both are isfinite-gated inside compute_stopping_accel; None/NaN => stock fallback, so
    they can never push NaN into a commanded accel (invariant 2)."""
    if self.frame % 300 == 0:
      self._read_live_params()
    self.tune.refresh()
    self.frame += 1

    drel_filtered = self._update_drel_filter(drel)

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill, v_ego_starting=self.v_ego_starting)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.0

    elif self.long_control_state == LongCtrlState.stopping:
      # Bundle D / L2: two-phase, speed/proximity/pitch-aware stopping shape. This only changes
      # the *value* of output_accel; the LongCtrlState stays `stopping`, so the carcontroller's
      # stopping_counter / create_acc_commands path and its stopping-state brake rate-limit
      # (carcontroller.py: brake_rate_up = stopping_decel_rate when longControlState==stopping)
      # are unaffected — they key off the state enum, not the accel value. L2 is provably INERT
      # above v_ego_stopping (returns the stock monotonic ramp), so highway stops are unchanged.
      if self.tune.stopping["l2_enable"] >= 0.5:
        output_accel = compute_stopping_accel(
          self.last_output_accel, self.stop_accel, self.stopping_decel_rate,
          CS.vEgo, self.v_ego_stopping,
          self.tune.stopping["hold_accel"], self.tune.stopping["phase_switch_v"],
          self.tune.stopping["proximity_scale_m"], self.tune.stopping["pitch_margin"],
          drel_filtered, pitch if pitch is not None else float("nan"),
        )
      else:
        # stock monotonic ramp to stop_accel
        output_accel = self.last_output_accel
        if output_accel > self.stop_accel:
          output_accel = min(output_accel, 0.0)
          output_accel -= self.stopping_decel_rate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo

      output_accel = self.pid.update(
        error,
        speed=CS.vEgo,
        feedforward=a_target,
      )

      output_accel *= self.long_pid_tune_scale

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
