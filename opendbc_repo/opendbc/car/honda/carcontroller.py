import json
import math
import os
import threading
from collections import deque
from queue import Empty, Queue

import numpy as np
from openpilot.common.params import Params

from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, rate_limit, make_tester_present_msg, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CAR, CruiseButtons, HONDA_BOSCH, HONDA_BOSCH_CANFD, HONDA_BOSCH_RADARLESS, \
                                     HONDA_BOSCH_TJA_CONTROL, HONDA_NIDEC_ALT_PCM_ACCEL, CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.common.pid import PIDController

from opendbc.sunnypilot.car.honda.mads import MadsCarController
from opendbc.sunnypilot.car.honda.gas_interceptor import GasInterceptorCarController
from opendbc.sunnypilot.car.honda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


def get_param_bool(params, key, default=False):
  value = params.get(key)
  if value is None:
    return default
  return params.get_bool(key)


def get_param_float(params, key, default, min_value=None, max_value=None, scale=1.0):
  value = params.get(key)
  if value is None:
    ret = default
  else:
    try:
      # sunnypilot Params.get() auto-casts by declared key type, so INT/FLOAT params
      # come back as python numbers (not bytes). Handle bytes/str/number uniformly.
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


def compute_gb_honda_bosch(accel, speed):
  # TODO returns 0s, is unused
  return 0.0, 0.0


def compute_gb_honda_nidec(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / 4.8 - creep_brake
  return np.clip(gb, 0.0, 1.0), np.clip(-gb, 0.0, 1.0)


def compute_gas_brake(accel, speed, fingerprint):
  if fingerprint in HONDA_BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  else:
    return compute_gb_honda_nidec(accel, speed)


# TODO not clear this does anything useful
def actuator_hysteresis(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02    # to activate brakes exceed this value
  brake_hyst_off = 0.005  # to deactivate brakes below this value
  brake_hyst_gap = 0.01   # don't change brake command for small oscillations within this value

  # *** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  return brake, braking, brake_steady


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_ts, ts):
  pump_on = False

  # reset pump timer if:
  # - there is an increment in brake request
  # - we are applying steady state brakes and we haven't been running the pump
  #   for more than 20s (to prevent pressure bleeding)
  if apply_brake > apply_brake_last or (ts - last_pump_ts > 20. and apply_brake > 0):
    last_pump_ts = ts

  # once the pump is on, run it for at least 0.2s
  if ts - last_pump_ts < 0.2 and apply_brake > 0:
    pump_on = True

  return pump_on, last_pump_ts


def process_hud_alert(hud_alert):
  alert_fcw = False
  alert_steer_required = False

  # Make sure FCW is prioritized over steering required
  # TODO: implement separate available LDW alert
  if hud_alert == VisualAlert.fcw:
    alert_fcw = True
  elif hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw):
    alert_steer_required = True

  return alert_fcw, alert_steer_required

def get_eps_modified_steering_pressed(raw_pressed, steering_torque, torque_cmd, filter_s, previous_pressed):
  raw_pressed = bool(raw_pressed)
  steering_torque = float(steering_torque)
  torque_cmd = float(torque_cmd)

  if not raw_pressed:
    return 0.0, False

  torque_product = steering_torque * torque_cmd
  torque_cmd_abs = abs(torque_cmd)

  if previous_pressed or torque_cmd_abs < 0.10 or torque_product < 0.0:
    return 1.0, True

  filter_s = min(1.0, filter_s + DT_CTRL)
  return filter_s, filter_s >= 0.28

def torque_lpf_tau(v_ego: float, low_tau: float, standard_tau: float, highway_tau: float) -> float:
  # Speed-banded low-pass filter time constant (seconds), fully tunable from the UI.
  if v_ego < 25.0 * CV.MPH_TO_MS:
    return low_tau
  if v_ego < 50.0 * CV.MPH_TO_MS:
    return standard_tau
  return highway_tau


def notch_biquad_coeffs(f0: float, q: float, fs: float):
  # Standard RBJ notch (band-reject) biquad. Removes a narrow band around f0 (Hz)
  # with width set by q (higher q = narrower). Near-zero phase lag away from f0,
  # so it kills a fixed EPS resonance without the broadband delay a LPF adds.
  q = max(q, 0.1)
  w0 = 2.0 * math.pi * (f0 / fs)
  cos_w0 = math.cos(w0)
  alpha = math.sin(w0) / (2.0 * q)
  b0 = 1.0
  b1 = -2.0 * cos_w0
  b2 = 1.0
  a0 = 1.0 + alpha
  a1 = -2.0 * cos_w0
  a2 = 1.0 - alpha
  return (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)


class NotchFilter:
  # Direct-form-II transposed biquad. Recomputes coeffs only when freq/q change.
  def __init__(self, fs: float):
    self.fs = fs
    self.f0 = 0.0
    self.q = 0.0
    self.b0 = 1.0
    self.b1 = 0.0
    self.b2 = 0.0
    self.a1 = 0.0
    self.a2 = 0.0
    self.z1 = 0.0
    self.z2 = 0.0

  def reset(self):
    self.z1 = 0.0
    self.z2 = 0.0

  def update(self, x: float, f0: float, q: float) -> float:
    if f0 != self.f0 or q != self.q:
      self.f0, self.q = f0, q
      self.b0, self.b1, self.b2, self.a1, self.a2 = notch_biquad_coeffs(f0, q, self.fs)
    y = self.b0 * x + self.z1
    self.z1 = self.b1 * x - self.a1 * y + self.z2
    self.z2 = self.b2 * x - self.a2 * y
    return y


# Sidecar path for fingerprint + version metadata (does NOT touch params_keys.h)
LEARNER_META_PATH = "/data/honda_learner_meta.json"

# Bump when learner semantics change so persisted values are discarded
LEARN_VERSION = 2

# Learner tick cadence: update() runs every 2 controller frames at 100 Hz → 0.02 s per tick
_LEARNER_DT = 2 * DT_CTRL  # 0.02 s

# Lag alignment: typical longitudinalActuatorDelay ~0.5 s → 25 learner ticks
_LAG_TICKS = 25  # 25 * 0.02 s = 0.50 s

# Quasi-steady gate: reject samples where the command is actively changing
# |Δaccel_cmd| / dt must be < 0.3 m/s³ across the deque window
_ACCEL_RATE_THRESH = 0.3  # m/s³

# G4 soft relative clamps (relative to nominal 1.0)
_HARD_LO = 0.6
_HARD_HI = 1.6
_SOFT_LO = 0.8
_SOFT_HI = 1.25
_DECAY_RATE_PER_MIN = 0.01  # fraction/min decayed toward 1.0 while outside soft band
_DECAY_PER_TICK = _DECAY_RATE_PER_MIN / 60.0 * _LEARNER_DT

# Applied-factor first-order filter (rc ~7.5 s nominal)
_FACTOR_FILTER_RC = 7.5
_FACTOR_FILTER_ALPHA = _LEARNER_DT / (_FACTOR_FILTER_RC + _LEARNER_DT)

# Hill/saturation deadband
_PITCH_DEADBAND = 0.02   # rad
_BRAKE_ADDON_DEADBAND = 1.0  # m/s²


class LongGasLearner:
  """
  Lag-aligned gas/wind factor learner with torqued-grade safety rails (G1 + G4).

  Separation of concerns:
  - raw_gasfactor / raw_windfactor: the learned integrators (persisted)
  - gasfactor / windfactor: slow-filtered applied values (initialized from persisted; no startup transient)
  - All param reads/writes handled externally; this class is pure logic.

  Tick cadence:
    Called every 2 controller frames (frame % 2 == 0) at 100 Hz → DT = 0.02 s.
    Deque depth 25 → 25 × 0.02 s = 0.50 s lag alignment (matches longitudinalActuatorDelay).
  """

  def __init__(self, init_gasfactor: float, init_windfactor: float, car_fingerprint: str):
    # Clamp + NaN-guard on load
    init_gasfactor = self._safe_clamp(init_gasfactor)
    init_windfactor = self._safe_clamp(init_windfactor, lo=_HARD_LO, hi=_HARD_HI)

    # Learned integrators (raw, before filter)
    self.raw_gasfactor = init_gasfactor
    self.raw_windfactor = init_windfactor

    # Applied factors (FirstOrderFilter outputs)
    # Initialize at loaded value → no startup transient
    self.gasfactor = init_gasfactor
    self.windfactor = init_windfactor

    self.car_fingerprint = car_fingerprint

    # Deque of accel commands (length = _LAG_TICKS + 1 for rate check)
    self._accel_deque: deque = deque(maxlen=_LAG_TICKS + 1)

    # Anti-windup shadow sentinels (stores raw integrator value before maxgas/brake boundary)
    self.gasfactor_before_maxgas = init_gasfactor
    self.windfactor_before_maxgas = init_windfactor
    self.windfactor_before_brake = init_windfactor

    # Track engagement state for deque reset
    self._was_engaged = False

  @staticmethod
  def _safe_clamp(v: float, lo: float = _HARD_LO, hi: float = _HARD_HI) -> float:
    """NaN/inf guard + absolute hard clamp. Returns 1.0 on non-finite."""
    if not math.isfinite(v):
      return 1.0
    return float(np.clip(v, lo, hi))

  @staticmethod
  def _decay_toward_nominal(v: float) -> float:
    """Decay v toward 1.0 by one tick's worth if outside soft band."""
    if v < _SOFT_LO or v > _SOFT_HI:
      if v < 1.0:
        v = min(1.0, v + _DECAY_PER_TICK)
      else:
        v = max(1.0, v - _DECAY_PER_TICK)
    return v

  def reset_deque(self, accel_cmd: float):
    """Reset lag deque on engagement edge or gasPressed."""
    self._accel_deque.clear()
    # Pre-fill with current command so lagged ref is valid immediately
    for _ in range(_LAG_TICKS + 1):
      self._accel_deque.append(accel_cmd)

  def update(self,
             accel_cmd: float,
             a_ego: float,
             gas_pedal_force: float,
             wind_brake_ms2: float,
             long_active: bool,
             long_pid: bool,
             gas_pressed: bool,
             brake_pressed: bool,
             v_ego: float,
             at_standstill: bool,
             pitch: float,
             brake_addon: float,
             at_accel_max: bool):
    """
    One learner tick (called at the 2-frame cadence, NOT every frame).

    Returns: (gasfactor_applied, windfactor_applied)
    Always returns finite values — NaN cannot propagate.
    """
    engaged = long_active and long_pid

    # Engagement-edge or gasPressed reset
    if (not self._was_engaged and engaged) or gas_pressed:
      self.reset_deque(accel_cmd)
    self._was_engaged = engaged

    # Push current command into deque
    self._accel_deque.append(accel_cmd)

    # Only learn when conditions are right
    should_learn = (
      engaged
      and not gas_pressed
      and not brake_pressed
      and not at_standstill
    )

    if should_learn and len(self._accel_deque) == _LAG_TICKS + 1:
      # Lag-aligned reference: the command that was current ~0.5 s ago
      lagged_accel = self._accel_deque[0]

      # Quasi-steady gate: check that command has not been changing rapidly
      oldest = self._accel_deque[0]
      newest = self._accel_deque[-1]
      accel_rate = abs(newest - oldest) / (_LAG_TICKS * _LEARNER_DT)
      quasi_steady = accel_rate < _ACCEL_RATE_THRESH

      # Hill / saturation deadband (G4 rail 7)
      pitch_ok = abs(pitch) < _PITCH_DEADBAND
      brake_addon_ok = abs(brake_addon) < _BRAKE_ADDON_DEADBAND
      condition_ok = quasi_steady and pitch_ok and brake_addon_ok

      if condition_ok:
        gas_error = lagged_accel - a_ego

        # --- gasfactor update (gas_pedal_force > 0 gate) ---
        if gas_error != 0.0 and gas_pedal_force > 0.0:
          if self.car_fingerprint == "HONDA_INSIGHT":
            learn_speed = 150
          elif self.car_fingerprint in ("ACURA_RDX_3G", "ACURA_RDX_3G_MMR"):
            learn_speed = 300
          else:
            learn_speed = 50
          self.raw_gasfactor = np.clip(
            self.raw_gasfactor + gas_error / learn_speed * gas_pedal_force,
            _HARD_LO, _HARD_HI
          )

        # --- windfactor update ---
        if gas_error != 0.0 and v_ego > 0.0:
          if self.car_fingerprint in ("ACURA_RDX_3G", "ACURA_RDX_3G_MMR"):
            wind_learn_speed = 100
          else:
            wind_learn_speed = 1000
          wind_adjust = 1.0 + wind_brake_ms2 / wind_learn_speed
          self.raw_windfactor = np.clip(
            self.raw_windfactor * (wind_adjust if gas_error > 0.0 else 1.0 / wind_adjust),
            _HARD_LO, _HARD_HI
          )

    # Anti-windup shadows — operate on lagged command as well (G1 requirement)
    # Use gas_pedal_force (computed from lagged-or-current path) for saturation check
    if gas_pedal_force <= 0.0:
      # Braking: don't reduce windfactor, allow increases
      self.raw_windfactor = max(self.raw_windfactor, self.windfactor_before_brake)
    else:
      self.windfactor_before_brake = self.raw_windfactor

    if at_accel_max:
      # Saturation: don't increase gasfactor or windfactor
      self.raw_gasfactor = min(self.raw_gasfactor, self.gasfactor_before_maxgas)
      self.raw_windfactor = min(self.raw_windfactor, self.windfactor_before_maxgas)
      # G4 saturation-decay: slightly decay gasfactor when clipped at BOSCH_ACCEL_MAX
      self.raw_gasfactor = max(_HARD_LO, self.raw_gasfactor - _DECAY_PER_TICK)
    else:
      self.gasfactor_before_maxgas = self.raw_gasfactor
      self.windfactor_before_maxgas = self.raw_windfactor

    # G4 NaN/inf guard on raw integrators
    if not math.isfinite(self.raw_gasfactor):
      self.raw_gasfactor = 1.0
      self.gasfactor_before_maxgas = 1.0
    if not math.isfinite(self.raw_windfactor):
      self.raw_windfactor = 1.0
      self.windfactor_before_maxgas = 1.0
      self.windfactor_before_brake = 1.0

    # G4 decay-back toward nominal while outside soft band
    self.raw_gasfactor = self._decay_toward_nominal(self.raw_gasfactor)
    self.raw_windfactor = self._decay_toward_nominal(self.raw_windfactor)

    # Hard clamp (belt-and-suspenders)
    self.raw_gasfactor = float(np.clip(self.raw_gasfactor, _HARD_LO, _HARD_HI))
    self.raw_windfactor = float(np.clip(self.raw_windfactor, _HARD_LO, _HARD_HI))

    # G4 slow FirstOrderFilter between raw integrator and applied factor
    # Alpha = DT / (RC + DT), ~7.5 s time constant
    self.gasfactor = _FACTOR_FILTER_ALPHA * self.raw_gasfactor + (1.0 - _FACTOR_FILTER_ALPHA) * self.gasfactor
    self.windfactor = _FACTOR_FILTER_ALPHA * self.raw_windfactor + (1.0 - _FACTOR_FILTER_ALPHA) * self.windfactor

    # Final NaN guard on applied factors — safety absolute last resort
    if not math.isfinite(self.gasfactor):
      self.gasfactor = 1.0
    if not math.isfinite(self.windfactor):
      self.windfactor = 1.0

    return self.gasfactor, self.windfactor


def _load_learner_meta(car_fingerprint: str) -> tuple[float, float]:
  """
  Load persisted gasfactor/windfactor from Params, verifying fingerprint + LEARN_VERSION
  from the sidecar JSON. Returns (1.0, 1.0) on any mismatch or error.

  The sidecar JSON is written atomically (temp-then-rename) and carries:
    {"car_fingerprint": "...", "learn_version": 2}

  Note: does NOT touch params_keys.h (boot-brick trap on this fork).
  """
  try:
    params = Params()
    raw_gas = params.get("HondaGasFactorParams")
    raw_wind = params.get("HondaWindFactorParams")

    if raw_gas is None or raw_wind is None:
      return 1.0, 1.0

    # Read sidecar for fingerprint + version check
    try:
      with open(LEARNER_META_PATH, "r", encoding="utf-8") as f:
        meta = json.load(f)
      if meta.get("car_fingerprint") != car_fingerprint:
        return 1.0, 1.0
      if meta.get("learn_version") != LEARN_VERSION:
        return 1.0, 1.0
    except (OSError, json.JSONDecodeError, KeyError):
      # No sidecar or corrupt → treat as fresh (reset to nominal)
      return 1.0, 1.0

    # Parse param values
    if isinstance(raw_gas, bytes):
      raw_gas = raw_gas.decode("utf-8")
    if isinstance(raw_wind, bytes):
      raw_wind = raw_wind.decode("utf-8")

    gas = float(raw_gas)
    wind = float(raw_wind)

    if not math.isfinite(gas) or not math.isfinite(wind):
      return 1.0, 1.0

    gas = float(np.clip(gas, _HARD_LO, _HARD_HI))
    wind = float(np.clip(wind, _HARD_LO, _HARD_HI))
    return gas, wind

  except Exception:
    return 1.0, 1.0


def _write_learner_meta_atomic(car_fingerprint: str):
  """
  Atomically write sidecar JSON (temp-then-rename pattern from nrdr_long_tune.py).
  Only writes the metadata — actual factor values live in Params.
  No-ops silently on /data write failures (device may have read-only fs).
  """
  try:
    meta = {
      "car_fingerprint": car_fingerprint,
      "learn_version": LEARN_VERSION,
    }
    tmp = LEARNER_META_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump(meta, f, indent=2, sort_keys=True)
      f.write("\n")
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, LEARNER_META_PATH)
  except OSError:
    pass


# ---------------------------------------------------------------------------
# G3 — Brake-integrator memory (nrdrbranchdebug-cra.3)
#
# brake_pid.reset() zeroes the integrator at every braking onset (carcontroller resets
# whenever accel >= 0), so every brake application starts cold. G3 caches the *converged*
# integrator value per speed bin on brake RELEASE and preloads it on the next braking
# ENGAGEMENT, removing the "brake amnesia" lag without changing the command-path semantics.
#
# Unit mapping (BINDING, from PIDController + brake_pid config k_i=0.5, neg_limit=-2.0):
#   The brake controller has k_p=0, so its output brake_addon == self.i (the integrator term),
#   already in m/s^2 (it is summed straight into `control` and clamped to [-2.0, 0.0]).
#   error_integral == self.i / k_i == self.i / 0.5 (units m/s^2 * s). We cache and preload the
#   OUTPUT-unit integrator self.i directly, so "integrator units" == m/s^2 and the -0.5 cap is
#   literally "never preload more brake than 0.5 m/s^2 of integrator": _PRELOAD_CAP = -0.5.
# This is BOUNDED integrator state, never a learned multiplier; every cap is conservative
# (no CMBS backstop on this car).
# ---------------------------------------------------------------------------

# Speed bins (m/s) for the brake-integrator memory. Edges define 4 bins:
#   [0,2), [2,5), [5,10), [10,20)  (>=20 m/s never preloads — high-speed braking is rare/varied)
_BRAKE_BIN_EDGES = (0.0, 2.0, 5.0, 10.0, 20.0)
_BRAKE_N_BINS = len(_BRAKE_BIN_EDGES) - 1

# Persisted sidecar (atomic write-temp-then-rename; NO params_keys.h key — boot-brick trap)
BRAKE_PROFILES_PATH = "/data/honda_brake_profiles.json"
# Bump when bin layout / semantics change so stale persisted values are discarded
BRAKE_LEARN_VERSION = 1

_BRAKE_EMA_ALPHA = 0.1          # EMA on brake RELEASE folding the converged integrator into its bin
_BRAKE_PRELOAD_CAP = -0.5       # m/s^2: most negative preload allowed (conservative; pos side capped at 0.0)
_BRAKE_DECAY_PER_MIN = 0.01     # whole map decays toward 0 at x0.99/min while idle
_BRAKE_LEARNER_DT = 2 * DT_CTRL # update() runs on the frame % 2 == 0 cadence -> 0.02 s/tick
_BRAKE_DECAY_PER_TICK = _BRAKE_DECAY_PER_MIN / 60.0 * _BRAKE_LEARNER_DT
# Discard a braking episode whose aEgo variance exceeded this (m/s^2)^2 — noisy/transient stops
# don't represent a stable integrator value worth caching.
_BRAKE_AEGO_VAR_DISCARD = 0.25


def _brake_bin_index(v_ego: float) -> int:
  """Return the bin index for v_ego, or -1 if outside [0, top edge) / non-finite."""
  if not math.isfinite(v_ego) or v_ego < _BRAKE_BIN_EDGES[0] or v_ego >= _BRAKE_BIN_EDGES[-1]:
    return -1
  for i in range(_BRAKE_N_BINS):
    if v_ego < _BRAKE_BIN_EDGES[i + 1]:
      return i
  return -1


class BrakeMemory:
  """Per-speed-bin cache of the converged brake integrator, preloaded at braking onset.

  Pure logic (no Params IO inside): the carcontroller owns load/persist. The cache holds
  OUTPUT-unit integrator values (m/s^2, same units as brake_pid.i), each in [_PRELOAD_CAP, 0.0].

  Lifecycle per call to update():
    - braking ENGAGEMENT edge (was not braking, now braking): return the preload for the
      current speed bin (pitch-scaled, capped). Caller injects it into brake_pid.i.
    - braking sustained: accumulate aEgo samples for the variance discard test.
    - braking RELEASE edge (was braking, now not): EMA-fold the final integrator into the
      bin that was active at engagement, UNLESS the episode's aEgo variance was too high.
    - idle: decay the whole map toward 0 at x0.99/min.
  """

  def __init__(self, bins, car_fingerprint: str):
    # bins: iterable of _BRAKE_N_BINS floats (loaded from sidecar, already validated)
    self.bins = [self._safe_clamp(b) for b in bins]
    if len(self.bins) != _BRAKE_N_BINS:
      self.bins = [0.0] * _BRAKE_N_BINS
    self.car_fingerprint = car_fingerprint

    self._was_braking = False
    self._engage_bin = -1
    # Online variance accumulators (Welford) over the episode's aEgo
    self._n = 0
    self._mean = 0.0
    self._m2 = 0.0

  @staticmethod
  def _safe_clamp(v: float) -> float:
    """NaN/inf guard + cache-value clamp to [_PRELOAD_CAP, 0.0]. Returns 0.0 on non-finite."""
    if not math.isfinite(v):
      return 0.0
    return float(min(0.0, max(_BRAKE_PRELOAD_CAP, v)))

  def _reset_episode_stats(self):
    self._n = 0
    self._mean = 0.0
    self._m2 = 0.0

  def _accumulate(self, a_ego: float):
    if not math.isfinite(a_ego):
      return
    self._n += 1
    delta = a_ego - self._mean
    self._mean += delta / self._n
    self._m2 += delta * (a_ego - self._mean)

  def _episode_variance(self) -> float:
    if self._n < 2:
      return 0.0
    return self._m2 / self._n

  def _preload_for(self, v_ego: float, pitch: float) -> float:
    """Pitch-scaled, capped preload for the current speed bin. 0.0 if no usable bin/value."""
    idx = _brake_bin_index(v_ego)
    if idx < 0:
      return 0.0
    base = self.bins[idx]
    # Scale preload by current pitch: on a downhill (pitch < 0) gravity already brakes for us,
    # so trust LESS preload; on an uphill scale toward full. sin(pitch) in [-1,1]; map the
    # downhill side down to 0. Conservative: never amplify beyond the cached value.
    if math.isfinite(pitch):
      scale = float(np.clip(1.0 + math.sin(pitch), 0.0, 1.0))
    else:
      scale = 1.0
    return self._safe_clamp(base * scale)

  def update(self, braking: bool, integrator: float, v_ego: float, a_ego: float,
             pitch: float) -> float:
    """One learner tick. Returns a preload to inject into brake_pid.i on the engagement edge,
    else 0.0 (caller only applies the value on the engagement edge). Never returns NaN."""
    preload = 0.0

    if braking and not self._was_braking:
      # ENGAGEMENT edge
      self._engage_bin = _brake_bin_index(v_ego)
      self._reset_episode_stats()
      self._accumulate(a_ego)
      preload = self._preload_for(v_ego, pitch)

    elif braking and self._was_braking:
      # sustained braking — accumulate for the variance discard test
      self._accumulate(a_ego)

    elif (not braking) and self._was_braking:
      # RELEASE edge — EMA-fold the converged integrator into the engagement bin, unless the
      # episode was too noisy to trust.
      if self._engage_bin >= 0 and math.isfinite(integrator):
        if self._episode_variance() <= _BRAKE_AEGO_VAR_DISCARD:
          final = self._safe_clamp(integrator)
          old = self.bins[self._engage_bin]
          self.bins[self._engage_bin] = self._safe_clamp(
            (1.0 - _BRAKE_EMA_ALPHA) * old + _BRAKE_EMA_ALPHA * final
          )
      self._engage_bin = -1
      self._reset_episode_stats()

    else:
      # idle (not braking) — decay the whole map toward 0
      for i in range(_BRAKE_N_BINS):
        b = self.bins[i]
        if b < 0.0:
          self.bins[i] = min(0.0, b + _BRAKE_DECAY_PER_TICK * -_BRAKE_PRELOAD_CAP)
        elif b > 0.0:
          self.bins[i] = max(0.0, b - _BRAKE_DECAY_PER_TICK * -_BRAKE_PRELOAD_CAP)

    self._was_braking = braking

    if not math.isfinite(preload):
      return 0.0
    return preload


def _load_brake_profiles(car_fingerprint: str):
  """Load persisted brake-memory bins from BRAKE_PROFILES_PATH, verifying fingerprint +
  BRAKE_LEARN_VERSION (G4 sidecar pattern). Returns a list of _BRAKE_N_BINS floats; all-zero on
  any mismatch/error/corruption (provable no-op == stock cold-start)."""
  zero = [0.0] * _BRAKE_N_BINS
  try:
    with open(BRAKE_PROFILES_PATH, "r", encoding="utf-8") as f:
      data = json.load(f)
    if not isinstance(data, dict):
      return zero
    if data.get("car_fingerprint") != car_fingerprint:
      return zero
    if data.get("learn_version") != BRAKE_LEARN_VERSION:
      return zero
    bins = data.get("bins")
    if not isinstance(bins, list) or len(bins) != _BRAKE_N_BINS:
      return zero
    out = []
    for b in bins:
      fb = float(b)
      if not math.isfinite(fb):
        return zero
      out.append(float(min(0.0, max(_BRAKE_PRELOAD_CAP, fb))))
    return out
  except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError):
    return zero


def _write_brake_profiles_atomic(car_fingerprint: str, bins):
  """Atomic write-temp-then-rename of the brake-memory sidecar. No-ops on read-only /data."""
  try:
    payload = {
      "car_fingerprint": car_fingerprint,
      "learn_version": BRAKE_LEARN_VERSION,
      "bins": [float(b) for b in bins],
    }
    tmp = BRAKE_PROFILES_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump(payload, f, indent=2, sort_keys=True)
      f.write("\n")
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, BRAKE_PROFILES_PATH)
  except OSError:
    pass


class HondaParamWriter:
  def __init__(self):
    self._params = Params()
    self._queue = Queue()
    self._thread = threading.Thread(target=self._run, name="honda-param-writer", daemon=True)
    self._thread.start()

  def put_many(self, values):
    self._queue.put({key: float(value) for key, value in values.items()})

  def _run(self):
    while True:
      pending = self._queue.get()

      # Collapse queued snapshots so delayed writes keep only the newest value per key.
      try:
        while True:
          pending.update(self._queue.get_nowait())
      except Empty:
        pass

      for key, value in pending.items():
        self._params.put_nonblocking(key, value)


class CarController(CarControllerBase, MadsCarController, GasInterceptorCarController, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)
    GasInterceptorCarController.__init__(self, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)
    self.CAN = hondacan.CanBus(CP)
    self.tja_control = CP.carFingerprint in HONDA_BOSCH_TJA_CONTROL
    self.param_reader = Params()
    self.param_writer = HondaParamWriter()

    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.stopping_counter = 0

    self.accel = 0.0
    self.speed = 0.0
    self.gas = 0.0
    self.brake = 0.0
    self.last_torque = 0.0
    self.bosch_last_gas = 0

    # Load persisted factors with fingerprint + version check (G4 rails)
    init_gas, init_wind = _load_learner_meta(CP.carFingerprint)
    self._learner = LongGasLearner(init_gas, init_wind, CP.carFingerprint)

    self.pitch = 0.0

    self.torque_lpf = 0.0
    self.notch_filter = NotchFilter(1.0 / DT_CTRL)
    self.prev_torque_cmd = 0.0
    self.override_ramp = 1.0
    self.lat_active_prev = False
    self.steering_pressed_prev = False
    self.steering_pressed_filter_s = 0.0
    self.steering_pressed_robust_prev = False

    # Bosch extra-brake controller
    self.brake_pid = PIDController(k_p=([0,], [0,]),
                                   k_i=([0.], [0.5]),
                                   pos_limit=0.0,
                                   neg_limit=-2.0,
                                   rate=50)
    self.brake_pid.reset()

    # G3 brake-integrator memory: load per-speed-bin cache (fingerprint + version checked)
    self._brake_memory = BrakeMemory(_load_brake_profiles(CP.carFingerprint), CP.carFingerprint)
    self._brake_active_prev = False

  def _filtered_steering_pressed(self, CS, torque_cmd: float) -> bool:
    raw_pressed = bool(CS.out.steeringPressed)
    steering_torque = float(getattr(CS.out, "steeringTorque", 0.0))
    torque_cmd = float(torque_cmd)

    if not raw_pressed:
      self.steering_pressed_filter_s = 0.0
      self.steering_pressed_robust_prev = False
      return False

    torque_product = steering_torque * torque_cmd
    torque_cmd_abs = abs(torque_cmd)

    if self.steering_pressed_robust_prev or torque_cmd_abs < 0.10 or torque_product < 0.0:
      self.steering_pressed_filter_s = 1.0
      self.steering_pressed_robust_prev = True
      return True

    self.steering_pressed_filter_s = min(1.0, self.steering_pressed_filter_s + DT_CTRL)
    steering_pressed = self.steering_pressed_filter_s >= 0.28

    self.steering_pressed_robust_prev = steering_pressed
    return steering_pressed

  def _get_live_tuning_params(self):
    return {
      "override_fade_down_s": get_param_float(self.param_reader, "HondaOverrideFadeDownSecs", 0.0, 0.0, 10.0),
      "override_fade_up_s": get_param_float(self.param_reader, "HondaOverrideFadeUpSecs", 1.5, 0.0, 10.0),
      "override_torque_scale": get_param_float(self.param_reader, "HondaOverrideTorqueScale", 0.0, 0.0, 100.0, scale=100.0),
      "driver_assist_during_override": get_param_bool(self.param_reader, "HondaDriverAssistDuringOverride", True),
      # Default ON for Bosch (gas learns well), OFF for Nidec; only applies when unset.
      "live_learning_gas": get_param_bool(self.param_reader, "HondaLiveLearningGas", self.CP.carFingerprint in HONDA_BOSCH),
      "torque_lpf_enabled": get_param_bool(self.param_reader, "HondaTorqueLowPassFilter", True),
      "lpf_tau_low": get_param_float(self.param_reader, "HondaLpfTauLowSpeed", 0.1, 0.0, 5.0),
      "lpf_tau_standard": get_param_float(self.param_reader, "HondaLpfTauStandard", 0.1, 0.0, 5.0),
      "lpf_tau_highway": get_param_float(self.param_reader, "HondaLpfTauHighway", 0.1, 0.0, 5.0),
      "notch_enabled": get_param_bool(self.param_reader, "HondaNotchEnabled", False),
      "notch_freq": get_param_float(self.param_reader, "HondaNotchFreq", 7.5, 1.0, 20.0),
      "notch_q": get_param_float(self.param_reader, "HondaNotchQ", 1.5, 0.1, 10.0),
      "steer_delta_limiter_enabled": get_param_bool(self.param_reader, "HondaSteerDeltaLimiter", False),
      "steer_delta_up": get_param_float(self.param_reader, "HondaSteerDeltaUp", 3.0, 0.0, 100.0),
      "steer_delta_down": get_param_float(self.param_reader, "HondaSteerDeltaDown", 3.0, 0.0, 100.0),
      "stopping_decel_rate": get_param_float(self.param_reader, "HondaStoppingDecelRate", 0.3, 0.0, 1.0, scale=100.0),
      "increase_override_tolerance": get_param_bool(self.param_reader, "NrdrIncreaseOverrideTolerance", True),
    }

  def _update_steering_torque(self, CC, CS, live):
    torque_cmd = float(CC.actuators.torque) if CC.latActive else 0.0
    steering_pressed = False

    if CC.latActive:
      if live["increase_override_tolerance"]:
        steering_pressed = self._filtered_steering_pressed(CS, torque_cmd)
      else:
        steering_pressed = bool(CS.out.steeringPressed)

      if not self.lat_active_prev:
        self.override_ramp = 0.0

      if steering_pressed:
        fade_down_s = live["override_fade_down_s"]
        self.override_ramp = live["override_torque_scale"] if fade_down_s <= 0.0 else max(live["override_torque_scale"], self.override_ramp - DT_CTRL / fade_down_s)
      else:
        fade_up_s = live["override_fade_up_s"]
        self.override_ramp = 1.0 if fade_up_s <= 0.0 else min(1.0, self.override_ramp + DT_CTRL / fade_up_s)

      torque_cmd *= self.override_ramp

      if live["torque_lpf_enabled"]:
        tau = torque_lpf_tau(CS.out.vEgo, live["lpf_tau_low"], live["lpf_tau_standard"], live["lpf_tau_highway"])
        alpha = DT_CTRL / (tau + DT_CTRL)
        self.torque_lpf = alpha * torque_cmd + (1.0 - alpha) * self.torque_lpf
        torque_cmd = self.torque_lpf
      else:
        self.torque_lpf = torque_cmd

      # Notch filter (in series, after LPF): removes the narrow EPS chatter band
      # (~7Hz) without the broadband lag a LPF adds. Independently toggleable.
      if live["notch_enabled"]:
        torque_cmd = self.notch_filter.update(torque_cmd, live["notch_freq"], live["notch_q"])
      else:
        self.notch_filter.reset()

      self.prev_torque_cmd = torque_cmd
    else:
      self.override_ramp = 0.0
      self.torque_lpf = 0.0
      self.notch_filter.reset()
      self.prev_torque_cmd = 0.0
      self.steering_pressed_filter_s = 0.0
      self.steering_pressed_robust_prev = False

    if live["steer_delta_limiter_enabled"]:
      limited_torque = rate_limit(torque_cmd, self.last_torque,
                                  -live["steer_delta_down"] * DT_CTRL,
                                  live["steer_delta_up"] * DT_CTRL)
    else:
      limited_torque = torque_cmd

    self.last_torque = limited_torque
    self.lat_active_prev = CC.latActive
    self.steering_pressed_prev = steering_pressed

    # "Driver assist during override" ON  -> openpilot gives way while you steer (LKAS torque drops out).
    # OFF -> openpilot keeps applying torque during override (more resistant).
    lkas_active = CC.latActive and (not live["driver_assist_during_override"] or not steering_pressed)
    return limited_torque, lkas_active

  def update(self, CC, CC_SP, CS, now_nanos):
    MadsCarController.update(self, self.CP, CC, CC_SP)
    live = self._get_live_tuning_params()
    gas_pedal_force = 0.0
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_v_cruise = hud_control.setSpeed / CS.v_cruise_factor if hud_control.speedVisible else 255
    pcm_cancel_cmd = CC.cruiseControl.cancel

    if len(CC.orientationNED) == 3:
      self.pitch = CC.orientationNED[1]
    hill_brake = math.sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY

    if CC.longActive:
      accel = actuators.accel
      if (self.CP.carFingerprint in (CAR.ACURA_MDX_3G, CAR.ACURA_MDX_3G_MMR)) and (accel > max(0, CS.out.aEgo) + 0.1):
        accel = 10000.0 # help with lagged accel until pedal tuning is inserted
      gas, brake = compute_gas_brake(actuators.accel + hill_brake, CS.out.vEgo, self.CP.carFingerprint)
    else:
      accel = 0.0
      gas, brake = 0.0, 0.0

    # *** rate limit / filter steer ***
    limited_torque, lkas_active = self._update_steering_torque(CC, CS, live)

    # *** apply brake hysteresis ***
    pre_limit_brake, self.braking, self.brake_steady = actuator_hysteresis(brake, self.braking, self.brake_steady,
                                                                           CS.out.vEgo, self.CP.carFingerprint)

    # *** rate limit after the enable check ***
    brake_rate_up = live["stopping_decel_rate"] if actuators.longControlState == LongCtrlState.stopping else 3.0
    self.brake_last = rate_limit(pre_limit_brake, self.brake_last, -2., brake_rate_up * DT_CTRL)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    alert_fcw, alert_steer_required = process_hud_alert(hud_control.visualAlert)

    # **** process the car messages ****

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_torque = int(np.interp(-limited_torque * self.params.STEER_MAX,
                                 self.params.STEER_LOOKUP_BP, self.params.STEER_LOOKUP_V))

    speed_control = 1 if ((accel <= 0.0) and (CS.out.vEgo == 0)) else 0

    # Send CAN commands
    can_sends = []

    # tester present - w/ no response (keeps radar disabled)
    if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS) and self.CP.openpilotLongitudinalControl:
      if self.frame % 10 == 0:
        can_sends.append(make_tester_present_msg(0x18DAB0F1, self.CAN.pt, suppress_response=True))

    # Send steering command.
    can_sends.append(hondacan.create_steering_control(self.packer, self.CAN, apply_torque, lkas_active, self.tja_control))

    # wind brake from air resistance decel at high speed
    wind_brake = np.interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15]) * self._learner.windfactor # not in m/s2 units
    wind_brake_ms2 = np.interp(CS.out.vEgo, [0.0, 13.4, 22.4, 31.3, 40.2], [0.000, 0.049, 0.136, 0.267, 0.441]) # in m/s2 units

    # all of this is only relevant for HONDA NIDEC
    speed_control = 0
    max_accel = np.interp(CS.out.vEgo, self.params.NIDEC_MAX_ACCEL_BP, self.params.NIDEC_MAX_ACCEL_V)
    # TODO this 1.44 is just to maintain previous behavior
    pcm_speed_BP = [-wind_brake,
                    -wind_brake * (3 / 4),
                    0.0,
                    0.5]
    # The Honda ODYSSEY seems to have different PCM_ACCEL
    # msgs, is it other cars too?
    if self.CP_SP.enableGasInterceptor or not CC.longActive:
      pcm_speed = 0.0
      pcm_accel = int(0.0)
    elif self.CP.carFingerprint in HONDA_NIDEC_ALT_PCM_ACCEL:
      pcm_speed_V = [0.0,
                     np.clip(CS.out.vEgo - 3.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 0.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
      pcm_speed = float(np.interp(gas - brake, pcm_speed_BP, pcm_speed_V))
      pcm_accel = int(1.0 * self.params.NIDEC_GAS_MAX)
    elif (self.CP.carFingerprint in (CAR.ACURA_MDX_3G, CAR.ACURA_MDX_3G_MMR)):
      pcm_speed_V = [0.0,
                     np.clip(CS.out.vEgo - 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 20.0, 0.0, 100.0)]
      pcm_speed = float(np.interp(gas - brake, pcm_speed_BP, pcm_speed_V))
      pcm_accel = int(np.clip((accel / 1.44) / max_accel, 10.0 / self.params.NIDEC_GAS_MAX, 1.0) * self.params.NIDEC_GAS_MAX)
      if speed_control == 1 and CC.longActive:
        pcm_accel = 198
    else:
      pcm_speed_V = [0.0,
                     np.clip(CS.out.vEgo - 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
      pcm_speed = float(np.interp(gas - brake, pcm_speed_BP, pcm_speed_V))
      pcm_accel = int(np.clip((accel / 1.44) / max_accel, 0.0, 1.0) * self.params.NIDEC_GAS_MAX)

    if not self.CP.openpilotLongitudinalControl:
      if self.frame % 2 == 0 and self.CP.carFingerprint not in HONDA_BOSCH_RADARLESS | HONDA_BOSCH_CANFD:
        can_sends.append(hondacan.create_bosch_supplemental_1(self.packer, self.CAN))
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.CANCEL, self.CP.carFingerprint))
      elif CC.cruiseControl.resume:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.RES_ACCEL, self.CP.carFingerprint))

    else:
      # Send gas and brake commands.
      if self.frame % 2 == 0:
        ts = self.frame * DT_CTRL

        if self.CP.carFingerprint in HONDA_BOSCH:
          brake_active = (accel < 0) and (CS.out.vEgo > 1e-3)

          # G3 brake-integrator memory tick (same 2-frame cadence as the brake controller).
          # On the engagement edge it returns a bounded, pitch-scaled preload; on release it
          # EMA-caches the converged integrator. The command-path semantics below are unchanged
          # except that brake_pid.i is preloaded (instead of starting from a reset 0) at onset.
          preload = self._brake_memory.update(
            braking=brake_active,
            integrator=float(self.brake_pid.i),
            v_ego=CS.out.vEgo,
            a_ego=CS.out.aEgo,
            pitch=self.pitch,
          )

          if brake_active:
            if not self._brake_active_prev:
              # ENGAGEMENT edge: preload the integrator instead of leaving it at the reset 0.
              # Bounded to [_BRAKE_PRELOAD_CAP, 0.0] inside BrakeMemory; clamp again belt-and-suspenders.
              self.brake_pid.i = float(np.clip(preload, _BRAKE_PRELOAD_CAP, 0.0))
            brake_addon = self.brake_pid.update(error = accel - CS.out.aEgo, speed = CS.out.vEgo)
            targetaccel = min(accel,accel + brake_addon)
          else:
            self.brake_pid.reset()
            brake_addon = 0.0
            targetaccel = accel
          self._brake_active_prev = brake_active

          self.accel = float(np.clip(targetaccel, self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX))
          gas_pedal_force = self.accel + wind_brake_ms2 * self._learner.windfactor + hill_brake

          # live-learn gas pedal adjustments when openpilot is controlling gas (G1+G4)
          if live["live_learning_gas"]:
            self._learner.update(
              accel_cmd=self.accel,
              a_ego=CS.out.aEgo,
              gas_pedal_force=gas_pedal_force,
              wind_brake_ms2=wind_brake_ms2,
              long_active=CC.longActive,
              long_pid=(actuators.longControlState == LongCtrlState.pid),
              gas_pressed=CS.out.gasPressed,
              brake_pressed=CS.out.brakePressed,
              v_ego=CS.out.vEgo,
              at_standstill=(CS.out.vEgo <= 0.0),
              pitch=self.pitch,
              brake_addon=float(brake_addon),
              at_accel_max=(gas_pedal_force >= self.params.BOSCH_ACCEL_MAX),
            )
          self.gas = float(np.interp(gas_pedal_force * self._learner.gasfactor, self.params.BOSCH_GAS_LOOKUP_BP, self.params.BOSCH_GAS_LOOKUP_V))

          # limit gas ramp to 60 units per frame, matches stock. Higher sometimes causes powertrain to ignore gas command.
          max_gas = max(60, self.bosch_last_gas + 60)
          self.gas = min(self.gas, max_gas)
          self.bosch_last_gas = self.gas

          stopping = actuators.longControlState == LongCtrlState.stopping
          self.stopping_counter = self.stopping_counter + 1 if stopping else 0
          can_sends.extend(hondacan.create_acc_commands(self.packer, self.CAN, CC.enabled, CC.longActive, self.accel, self.gas,
                                                        self.stopping_counter, self.CP.carFingerprint, gas_pedal_force))
        else:
          apply_brake = np.clip(self.brake_last - wind_brake, 0.0, 1.0)
          apply_brake = int(np.clip(apply_brake * self.params.NIDEC_BRAKE_MAX, 0, self.params.NIDEC_BRAKE_MAX - 1))
          pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)

          pcm_override = True
          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, apply_brake, pump_on,
                                                         pcm_override, pcm_cancel_cmd, alert_fcw,
                                                         self.CP.carFingerprint, CS.stock_brake, self.CP_SP))
          self.apply_brake_last = apply_brake
          self.brake = apply_brake / self.params.NIDEC_BRAKE_MAX

          if live["live_learning_gas"] and self.CP_SP.enableGasInterceptor:
            self._learner.update(
              accel_cmd=actuators.accel,
              a_ego=CS.out.aEgo,
              gas_pedal_force=gas,
              wind_brake_ms2=wind_brake * 4.8,  # convert to consistent units for Nidec path
              long_active=CC.longActive,
              long_pid=(actuators.longControlState == LongCtrlState.pid),
              gas_pressed=CS.out.gasPressed,
              brake_pressed=CS.out.brakePressed,
              v_ego=CS.out.vEgo,
              at_standstill=(CS.out.vEgo <= 0.0),
              pitch=self.pitch,
              brake_addon=0.0,
              at_accel_max=(gas >= 1.0),
            )
          can_sends.extend(GasInterceptorCarController.update(self, CC, CS, gas * self._learner.gasfactor, brake, wind_brake, self.packer, self.frame))

    # Send dashboard UI commands.
    if self.frame % 10 == 0:
      if CC.longActive and (self.CP.carFingerprint in (CAR.ACURA_MDX_3G, CAR.ACURA_MDX_3G_MMR)):
        # standstill disengage
        if (accel >= 0.01) and (CS.out.vEgo < 4.0) and (pcm_speed < 25.0 / 3.6):
          pcm_speed = 25.0 / 3.6

      if self.CP.openpilotLongitudinalControl:
        # On Nidec, this also controls longitudinal positive acceleration
        can_sends.append(hondacan.create_acc_hud(self.packer, self.CAN.pt, self.CP, CC.enabled, pcm_speed, pcm_accel,
                                                 hud_control, hud_v_cruise, CS.is_metric, CS.acc_hud, speed_control))

      steering_available = CS.out.cruiseState.available and CS.out.vEgo > max(self.params.STEER_GLOBAL_MIN_SPEED, self.CP.minSteerSpeed)
      reduced_steering = CS.out.steeringPressed
      steer_maxed = abs(apply_torque) >= self.params.STEER_MAX
      can_sends.extend(hondacan.create_lkas_hud(self.packer, self.CAN.lkas, self.CP, hud_control, CC.latActive,
                                                steering_available, reduced_steering, alert_steer_required, CS.lkas_hud, self.dashed_lanes,
                                                steer_maxed))

      if self.CP.openpilotLongitudinalControl:
        # TODO: combining with create_acc_hud block above will change message order and will need replay logs regenerated
        if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS):
          can_sends.append(hondacan.create_radar_hud(self.packer, self.CAN.pt))
        if self.CP.carFingerprint == CAR.HONDA_CIVIC_BOSCH:
          can_sends.append(hondacan.create_legacy_brake_command(self.packer, self.CAN.pt))
        if self.CP.carFingerprint not in HONDA_BOSCH:
          self.speed = pcm_speed
          if not self.CP_SP.enableGasInterceptor:
            self.gas = pcm_accel / self.params.NIDEC_GAS_MAX

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, self.packer, self.frame,
                                                                       self.last_button_frame, self.CAN))

    new_actuators = actuators.as_builder()
    new_actuators.speed = self.speed
    new_actuators.accel = self.accel
    new_actuators.gas = float(self._learner.gasfactor)
    new_actuators.brake = float(self._learner.windfactor)
    new_actuators.torque = self.last_torque
    new_actuators.torqueOutputCan = apply_torque

    if self.frame % 6000 == 0:
      # Write raw integrator values (not filtered) so next load resumes from actual learned position
      self.param_writer.put_many({
        "HondaGasFactorParams": self._learner.raw_gasfactor,
        "HondaWindFactorParams": self._learner.raw_windfactor,
      })
      # Write sidecar atomically (fingerprint + LEARN_VERSION for G4 fingerprint check)
      _write_learner_meta_atomic(self.CP.carFingerprint)
      # G3: persist the brake-integrator memory (own atomic sidecar, no params_keys.h key)
      _write_brake_profiles_atomic(self.CP.carFingerprint, self._brake_memory.bins)

    self.frame += 1
    return new_actuators, can_sends