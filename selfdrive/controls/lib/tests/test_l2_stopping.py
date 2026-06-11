"""Bundle D / L2 — two-phase stopping-shape tests (nrdrbranchdebug-cra.1).

longcontrol.py imports cereal/capnp (unavailable off-device), so the pure stopping function
compute_stopping_accel + its module constants are AST-extracted from the source and exec'd
against numpy/math (same pattern as selfdrive/controls/tests/test_radard_k1.py). The tune-schema
tests import nrdr_long_tune directly (pure stdlib, importable on the dev box).

Contract under test:
  - two-phase profile: ramp to the gentle hold while rolling, to the pitch-aware floor at standstill
  - speed-scaled rate term + dRel-proximity rate term
  - pitch-aware hold floor with the -1.0 HARD minimum (never weaker)
  - INERT above v_ego_stopping (bit-identical to the stock monotonic ramp) — invariant 1
  - NaN/None fallbacks for every plumbed signal — invariant 2
  - stopping tune-section defaults/clamps
"""
import ast
import math
from pathlib import Path

import numpy as np
import pytest

LIB = Path(__file__).resolve().parent.parent
LONGCONTROL = LIB / "longcontrol.py"

DT_CTRL = 0.01  # real value from common.realtime; inject so the extracted fn is host-runnable


def _extract_ns():
  tree = ast.parse(LONGCONTROL.read_text(encoding="utf-8"))
  wanted_funcs = {"compute_stopping_accel"}
  wanted_assigns = {"STOPPING_HARD_HOLD_FLOOR", "ACCEL_DUE_TO_GRAVITY"}
  nodes = []
  for node in tree.body:
    if isinstance(node, ast.FunctionDef) and node.name in wanted_funcs:
      nodes.append(node)
    elif isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) \
        and node.targets[0].id in wanted_assigns:
      nodes.append(node)
  ns = {"np": np, "math": math, "DT_CTRL": DT_CTRL}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(LONGCONTROL), "exec"), ns)
  assert "compute_stopping_accel" in ns, "compute_stopping_accel missing from longcontrol.py"
  assert "STOPPING_HARD_HOLD_FLOOR" in ns
  return ns


NS = _extract_ns()
CSA = NS["compute_stopping_accel"]
HARD_FLOOR = NS["STOPPING_HARD_HOLD_FLOOR"]
G = NS["ACCEL_DUE_TO_GRAVITY"]

# Bundle D default shape
HOLD = -0.6
PHASE_V = 0.15
PROX_M = 8.0
MARGIN = 1.0
STOP_ACCEL = -2.0
RATE = 0.3
V_STOP = 0.3


def _call(last, v_ego, drel=float("inf"), pitch=0.0, **kw):
  args = dict(stop_accel=STOP_ACCEL, stopping_decel_rate=RATE, v_ego_stopping=V_STOP,
              hold_accel=HOLD, phase_switch_v=PHASE_V, proximity_scale_m=PROX_M,
              pitch_margin=MARGIN)
  args.update(kw)
  return CSA(last, args["stop_accel"], args["stopping_decel_rate"], v_ego, args["v_ego_stopping"],
             args["hold_accel"], args["phase_switch_v"], args["proximity_scale_m"],
             args["pitch_margin"], drel, pitch)


def _stock_ramp(last, stop_accel, rate):
  out = last
  if out > stop_accel:
    out = min(out, 0.0)
    out -= rate * DT_CTRL
  return out


class TestInertAboveStopping:
  """Invariant 1: above v_ego_stopping the shape reproduces the stock monotonic ramp."""

  def test_bit_identical_to_stock_above_window(self):
    for last in (0.5, 0.0, -0.3, -0.6, -1.5, -2.0):
      for v in (0.31, 1.0, 5.0, 30.0):
        got = _call(last, v)
        exp = _stock_ramp(last, STOP_ACCEL, RATE)
        assert got == pytest.approx(exp, abs=1e-12), f"last={last} v={v}: {got} != stock {exp}"

  def test_at_floor_above_window_holds(self):
    # already at stop_accel, above window -> stays (stock behavior: no further ramp)
    assert _call(STOP_ACCEL, 5.0) == pytest.approx(STOP_ACCEL, abs=1e-12)


class TestTwoPhaseProfile:
  def test_phaseA_targets_gentle_hold_not_stop_accel(self):
    # rolling (between phase_switch_v and v_ego_stopping): ramps toward HOLD, never past it
    out = -0.1
    for _ in range(2000):
      out = _call(out, v_ego=0.25)
    assert out == pytest.approx(HOLD, abs=1e-6), "Phase A must settle at the gentle hold"

  def test_phaseA_does_not_exceed_hold(self):
    # starting already below hold (more negative) on a rolling frame -> must not ramp further down
    out = _call(-0.9, v_ego=0.25)
    assert out == pytest.approx(-0.9, abs=1e-12), "Phase A must not push below hold once there"

  def test_phaseB_targets_pitch_floor_flat(self):
    # standstill, flat ground: floor = min(hold, HARD_FLOOR) = HARD_FLOOR (-1.0)
    out = HOLD
    for _ in range(3000):
      out = _call(out, v_ego=0.0, pitch=0.0)
    assert out == pytest.approx(HARD_FLOOR, abs=1e-6), "flat standstill must hold at the -1.0 hard floor"

  def test_head_bob_removed_at_moment_of_stop(self):
    # crossing from rolling into standstill the target deepens smoothly from HOLD toward the floor,
    # never jumping straight to stop_accel
    out = HOLD  # settled gentle hold while rolling
    out = _call(out, v_ego=0.0, pitch=0.0)  # first standstill frame
    assert out < HOLD, "standstill must begin deepening past the gentle hold"
    assert out > HARD_FLOOR - 1e-9, "must not overshoot below the hard floor in one frame"


class TestSpeedScaledRate:
  def test_rate_scales_with_speed(self):
    # speed_scale interp [0,0.3,v_ego_stopping] -> [0.3,0.7,1.0]: a faster (but still sub-stopping)
    # frame ramps MORE per step than a near-zero one. Both rolling frames target hold from -0.1.
    out_lo = _call(-0.1, v_ego=0.0001)   # speed_scale ~0.3
    out_hi = _call(-0.1, v_ego=0.29)     # speed_scale ~ near 1.0
    assert (-0.1 - out_hi) > (-0.1 - out_lo), "higher speed must ramp faster per frame"

  def test_zero_rate_no_motion(self):
    assert _call(-0.1, v_ego=0.0, stopping_decel_rate=0.0) == pytest.approx(-0.1, abs=1e-12)


class TestProximityScaling:
  def test_close_lead_slows_ramp(self):
    far = _call(-0.1, v_ego=0.0, drel=float("inf"))   # prox=1.0
    near = _call(-0.1, v_ego=0.0, drel=2.0)           # prox=2/8=0.25 -> smaller step
    assert (-0.1 - far) > (-0.1 - near), "closer lead must produce a gentler (smaller) ramp step"

  def test_drel_at_scale_saturates(self):
    at = _call(-0.1, v_ego=0.0, drel=PROX_M)          # prox=1.0
    above = _call(-0.1, v_ego=0.0, drel=PROX_M * 3)   # clamped to 1.0
    assert at == pytest.approx(above, abs=1e-12)

  def test_no_lead_is_inert_full_rate(self):
    inf_lead = _call(-0.1, v_ego=0.0, drel=float("inf"))
    none_eq = _call(-0.1, v_ego=0.0, drel=float("nan"))  # NaN -> prox=1.0 fallback
    assert inf_lead == pytest.approx(none_eq, abs=1e-12)


class TestPitchFloor:
  def test_uphill_deepens_floor_below_hard_minimum(self):
    # uphill (positive pitch): pitch term = -g*sin(pitch)*margin < 0 -> floor more negative than -1.0
    pitch = 0.2  # ~11.5 deg uphill
    out = HOLD
    for _ in range(5000):
      out = _call(out, v_ego=0.0, pitch=pitch)
    expected_floor = max(HOLD + (-G * math.sin(pitch) * MARGIN), STOP_ACCEL)
    expected_floor = min(expected_floor, HARD_FLOOR)  # never weaker than hard floor
    expected_floor = max(expected_floor, STOP_ACCEL)
    assert out == pytest.approx(expected_floor, abs=1e-4)
    assert out < HARD_FLOOR, "uphill floor must be stronger (more negative) than -1.0"

  def test_hard_minimum_never_weaker_than_minus_one(self):
    # downhill (negative pitch) would weaken the pitch-aware floor toward hold (-0.6); the hard
    # -1.0 minimum must still clamp the held value at -1.0 (never weaker)
    out = HOLD
    for _ in range(5000):
      out = _call(out, v_ego=0.0, pitch=-0.3)
    assert out <= HARD_FLOOR + 1e-9, "downhill must NOT weaken the hold above -1.0"
    assert out == pytest.approx(HARD_FLOOR, abs=1e-4)

  def test_floor_never_stronger_than_stop_accel(self):
    # extreme uphill with big margin: floor clamped to stop_accel, never beyond
    out = HOLD
    for _ in range(8000):
      out = _call(out, v_ego=0.0, pitch=1.0, pitch_margin=2.0)
    assert out >= STOP_ACCEL - 1e-9, "floor must never command stronger than stop_accel"

  def test_margin_zero_disables_pitch_awareness(self):
    out = HOLD
    for _ in range(5000):
      out = _call(out, v_ego=0.0, pitch=0.2, pitch_margin=0.0)
    # floor = min(hold, HARD_FLOOR) = HARD_FLOOR regardless of pitch
    assert out == pytest.approx(HARD_FLOOR, abs=1e-4)


class TestNaNFallbacks:
  def test_nan_vego_falls_back_to_stock_ramp(self):
    got = CSA(-0.1, STOP_ACCEL, RATE, float("nan"), V_STOP, HOLD, PHASE_V, PROX_M, MARGIN,
              float("inf"), 0.0)
    assert got == pytest.approx(_stock_ramp(-0.1, STOP_ACCEL, RATE), abs=1e-12)

  def test_nan_last_accel_treated_as_zero(self):
    got = _call(float("nan"), v_ego=0.0)
    assert math.isfinite(got)

  def test_nan_pitch_no_grade_term(self):
    out = HOLD
    for _ in range(3000):
      out = _call(out, v_ego=0.0, pitch=float("nan"))
    assert out == pytest.approx(HARD_FLOOR, abs=1e-4), "NaN pitch must behave like flat (no grade term)"

  def test_output_always_finite(self):
    for last in (float("nan"), float("inf"), -2.0, 0.5):
      for v in (float("nan"), 0.0, 0.2, 5.0):
        for d in (float("nan"), float("inf"), 1.0):
          for p in (float("nan"), 0.0, 0.3, -0.3):
            assert math.isfinite(_call(last, v, drel=d, pitch=p))


# ---------------------------------------------------------------------------
# Tune-schema tests (direct import — nrdr_long_tune is pure stdlib)
# ---------------------------------------------------------------------------
from openpilot.selfdrive.controls.lib.nrdr_long_tune import (  # noqa: E402
  LongTune, write_tune, _STOPPING_CLAMPS,
)


def _force(tune):
  tune._frame = 0
  tune.refresh()


class TestStoppingTuneSchema:
  def test_defaults_match_bundle_d_shape(self):
    assert _STOPPING_CLAMPS["l2_enable"][0] == 1.0           # council shipped ON
    assert _STOPPING_CLAMPS["hold_accel"][0] == -0.6
    assert _STOPPING_CLAMPS["phase_switch_v"][0] == 0.15
    assert _STOPPING_CLAMPS["proximity_scale_m"][0] == 8.0
    assert _STOPPING_CLAMPS["pitch_margin"][0] == 1.0

  def test_hold_accel_clamp_envelope(self):
    # hold accel clamp must keep it inside [-1.0, -0.3] (safe envelope; hard -1.0 floor is code)
    assert _STOPPING_CLAMPS["hold_accel"][1] == -1.0
    assert _STOPPING_CLAMPS["hold_accel"][2] == -0.3

  def test_unset_tune_gives_defaults(self, tmp_path):
    tune = LongTune(path=str(tmp_path / "t.json"))
    _force(tune)
    assert tune.stopping["l2_enable"] == 1.0
    assert tune.stopping["hold_accel"] == -0.6
    assert tune.stopping["proximity_scale_m"] == 8.0

  def test_values_clamped(self, tmp_path):
    path = str(tmp_path / "t.json")
    write_tune({"stopping": {"hold_accel": -5.0, "phase_switch_v": 9.0,
                             "proximity_scale_m": 0.1, "pitch_margin": 50.0,
                             "l2_enable": 0.0}}, path=path)
    tune = LongTune(path=path)
    _force(tune)
    assert tune.stopping["hold_accel"] == -1.0       # clamped to lo
    assert tune.stopping["phase_switch_v"] == 0.5    # clamped to hi
    assert tune.stopping["proximity_scale_m"] == 2.0 # clamped to lo
    assert tune.stopping["pitch_margin"] == 2.0      # clamped to hi
    assert tune.stopping["l2_enable"] == 0.0

  def test_nonfinite_rejected_per_field(self, tmp_path):
    path = str(tmp_path / "t.json")
    (tmp_path / "t.json").write_text('{"stopping": {"hold_accel": NaN, "phase_switch_v": 0.2}}',
                                     encoding="utf-8")
    tune = LongTune(path=path)
    _force(tune)
    assert tune.stopping["hold_accel"] == -0.6   # rejected, default
    assert tune.stopping["phase_switch_v"] == 0.2  # good field applied

  def test_l2_disable_roundtrip(self, tmp_path):
    path = str(tmp_path / "t.json")
    write_tune({"stopping": {"l2_enable": 0}}, path=path)
    tune = LongTune(path=path)
    _force(tune)
    assert tune.stopping["l2_enable"] == 0.0


if __name__ == "__main__":
  raise SystemExit(pytest.main([__file__, "-v"]))
