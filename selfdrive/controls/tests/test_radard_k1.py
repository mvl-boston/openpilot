"""Behavioral tests for K1 — honor the measured flag in radard (nrdrbranchdebug-86t.3).

radard.py imports capnp/cereal which are unavailable off-device, so the Track class and its
constants are AST-extracted from the source and exec'd against the real KF1D/FirstOrderFilter.
The K1 contract under test:
  1. KF updates only on measured cycles; placeholder cycles coast (predict-only, bounded).
  2. Published vRel/vLead on unmeasured cycles come from the KF estimate, never the fake
     relative-stationary ingestion sample; after the coast budget, fall back to ingestion.
  3. NaN vRel (Bosch fine ingest placeholder) can never poison the KF or reach radarState.
  4. aLeadTau learning is frozen on unmeasured cycles.
"""
import ast
import importlib.util
import math
from pathlib import Path

import pytest

CONTROLS = Path(__file__).resolve().parent.parent
REPO = CONTROLS.parent.parent
RADARD = CONTROLS / "radard.py"

DT_MDL = 0.05


def _load(name, rel):
  spec = importlib.util.spec_from_file_location(name, REPO / rel)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def _radard_ns():
  tree = ast.parse(RADARD.read_text(encoding="utf-8"))
  wanted_classes = {"Track", "KalmanParams"}
  wanted_assigns = {"SPEED", "ACCEL", "_LEAD_ACCEL_TAU", "UNMEASURED_COAST_CYCLES", "V_EGO_STATIONARY"}
  def _target_names(t):
    if isinstance(t, ast.Name):
      return {t.id}
    if isinstance(t, ast.Tuple):
      return {e.id for e in t.elts if isinstance(e, ast.Name)}
    return set()

  nodes = []
  for node in tree.body:
    if isinstance(node, ast.ClassDef) and node.name in wanted_classes:
      nodes.append(node)
    elif isinstance(node, ast.Assign) and _target_names(node.targets[0]) & wanted_assigns:
      nodes.append(node)
  import numpy as np
  ns = {
    "np": np,
    "math": math,
    "DT_MDL": DT_MDL,
    "KF1D": _load("simple_kalman", "common/simple_kalman.py").KF1D,
    "FirstOrderFilter": _load("filter_simple", "common/filter_simple.py").FirstOrderFilter,
  }
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(RADARD), "exec"), ns)
  assert "UNMEASURED_COAST_CYCLES" in ns, "K1 constant missing from radard.py"
  return ns


NS = _radard_ns()
COAST = NS["UNMEASURED_COAST_CYCLES"]


def make_track(v_seed=20.0, ident=7):
  return NS["Track"](ident, v_seed, NS["KalmanParams"](DT_MDL))


def run_measured(track, n, v_lead, v_ego=25.0, d_rel=40.0):
  for _ in range(n):
    track.update(d_rel, 0.0, v_lead - v_ego, v_lead, True, v_ego)


class TestMeasuredPath:
  def test_stock_behavior_preserved(self):
    track = make_track(v_seed=20.0)
    run_measured(track, 50, v_lead=20.0)
    assert abs(track.vLeadK - 20.0) < 0.05
    assert abs(track.aLeadK) < 0.05
    state = track.get_RadarState(0.9)
    assert state["vLead"] == pytest.approx(20.0, abs=0.05)
    assert state["vRel"] == pytest.approx(-5.0, abs=0.05)  # raw ingestion value on measured cycles

  def test_decelerating_lead_tracks(self):
    track = make_track(v_seed=20.0)
    v = 20.0
    for _ in range(60):
      v -= 2.0 * DT_MDL  # 2 m/s^2 braking lead
      track.update(40.0, 0.0, v - 25.0, v, True, 25.0)
    assert track.aLeadK < -1.0  # KF sees the decel


class TestUnmeasuredCoast:
  def test_kf_not_dragged_to_stationary(self):
    track = make_track(v_seed=20.0)
    run_measured(track, 50, v_lead=20.0)
    # placeholder cycles: ingestion publishes vRel=0 -> v_lead=v_ego=25 (fake stationary-relative)
    for _ in range(3):
      track.update(40.0, 0.0, 0.0, 25.0, False, 25.0)
    assert abs(track.vLeadK - 20.0) < 0.1, "coasting KF must not absorb the fake sample"

  def test_coast_extrapolates_decel(self):
    track = make_track(v_seed=20.0)
    v = 20.0
    for _ in range(60):
      v -= 2.0 * DT_MDL
      track.update(40.0, 0.0, v - 25.0, v, True, 25.0)
    v_before, a_before = track.vLeadK, track.aLeadK
    track.update(40.0, 0.0, float("nan"), float("nan"), False, 25.0)
    assert track.vLeadK == pytest.approx(v_before + a_before * DT_MDL, abs=1e-9), \
      "coast must propagate v along aLeadK (predict-only), not freeze or reset"

  def test_published_vrel_uses_kf_on_unmeasured(self):
    track = make_track(v_seed=20.0)
    run_measured(track, 50, v_lead=20.0)
    track.update(40.0, 0.0, 0.0, 25.0, False, 25.0)
    state = track.get_RadarState(0.9)
    assert state["vRel"] == pytest.approx(track.vLeadK - 25.0, abs=1e-9)
    assert state["vLead"] == pytest.approx(track.vLeadK, abs=1e-9)
    assert abs(state["vRel"] - (-5.0)) < 0.1, "should still read ~-5 m/s, not the fake 0.0"

  def test_fallback_to_ingestion_after_budget(self):
    track = make_track(v_seed=20.0)
    run_measured(track, 50, v_lead=20.0)
    for _ in range(COAST + 1):
      track.update(40.0, 0.0, 0.0, 25.0, False, 25.0)
    state = track.get_RadarState(0.9)
    assert state["vRel"] == 0.0 and state["vLead"] == 25.0, \
      "past the coast budget the raw ingestion values must be published"

  def test_measured_return_resets_budget(self):
    track = make_track(v_seed=20.0)
    run_measured(track, 50, v_lead=20.0)
    for _ in range(COAST):
      track.update(40.0, 0.0, 0.0, 25.0, False, 25.0)
    track.update(40.0, 0.0, -5.0, 20.0, True, 25.0)
    assert track.unmeasured_cnt == 0
    track.update(40.0, 0.0, 0.0, 25.0, False, 25.0)
    state = track.get_RadarState(0.9)
    assert state["vLead"] == pytest.approx(track.vLeadK, abs=1e-9)  # coasting again


class TestNaNSafety:
  def test_nan_vrel_never_reaches_kf_or_state(self):
    track = make_track(v_seed=20.0)
    run_measured(track, 20, v_lead=20.0)
    for _ in range(10):
      track.update(40.0, 0.0, float("nan"), float("nan"), False, 25.0)
    state = track.get_RadarState(0.9)
    assert all(math.isfinite(v) for k, v in state.items() if isinstance(v, float)), \
      f"NaN leaked into radarState: {state}"
    assert math.isfinite(track.vLeadK) and math.isfinite(track.aLeadK)

  def test_nan_seed_does_not_poison(self):
    track = make_track(v_seed=float("nan"))
    track.update(40.0, 0.0, float("nan"), float("nan"), False, 25.0)
    assert math.isfinite(track.vLeadK)
    state = track.get_RadarState(0.9)
    assert all(math.isfinite(v) for k, v in state.items() if isinstance(v, float))

  def test_nan_measured_flag_forced_false(self):
    # interface promises measured == not isnan(vRel), but Track must not rely on it
    track = make_track(v_seed=20.0)
    run_measured(track, 20, v_lead=20.0)
    track.update(40.0, 0.0, float("nan"), float("nan"), True, 25.0)  # lying flag
    assert math.isfinite(track.vLeadK)
    assert not track.measured


class TestALeadTau:
  def test_tau_frozen_on_unmeasured(self):
    track = make_track(v_seed=20.0)
    v = 20.0
    for _ in range(60):
      v -= 2.0 * DT_MDL
      track.update(40.0, 0.0, v - 25.0, v, True, 25.0)
    tau_before = track.aLeadTau.x
    assert tau_before < NS["_LEAD_ACCEL_TAU"]  # decel learned
    for _ in range(3):
      track.update(40.0, 0.0, 0.0, 25.0, False, 25.0)
    assert track.aLeadTau.x == pytest.approx(tau_before, abs=1e-12), \
      "aLeadTau must not learn from placeholder cycles"


class TestWiring:
  """Source-level guards on the RadarD.update side (not extractable as pure code)."""

  def test_vision_seed_wired(self):
    src = RADARD.read_text(encoding="utf-8")
    assert "vision_seed" in src
    assert "self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], self.v_ego_hist[0])" in src

  def test_low_speed_override_path_uses_get_radar_state(self):
    # the low-speed path publishes tracks without vision matching — exactly where the K1
    # publish-side estimate substitution must be active (it calls get_RadarState, which is patched)
    src = RADARD.read_text(encoding="utf-8")
    assert "closest_track.get_RadarState()" in src


if __name__ == "__main__":
  raise SystemExit(pytest.main([__file__, "-v"]))
