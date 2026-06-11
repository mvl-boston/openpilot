"""Behavioral tests for K2 — one-smoother radard retune for Civic Bosch fine ingest.

radard.py imports capnp/cereal which are unavailable off-device, so the Track class,
KalmanParams, and needed constants are AST-extracted from the source and exec'd against
the real KF1D/FirstOrderFilter/get_kalman_gain helpers loaded via importlib.

K2 contract under test:
  1. KalmanParams(dt, civic_bosch_fine=True) computes DARE-solved gains via get_kalman_gain
     instead of the stock interp tables (which bake R=1e3).
  2. RadarD.update has a freshness gate: when sm.recv_frame['liveTracks'] is unchanged, the
     measured flag is forced False for that cycle (routing into the K1 coast path).
  3. Civic gains converge faster on a braking lead than stock gains.
"""
import ast
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

CONTROLS = Path(__file__).resolve().parent.parent
REPO = CONTROLS.parent.parent
RADARD = CONTROLS / "radard.py"

CIVIC_BOSCH_LEAD_KF_DT = 0.06


def _load(name, rel):
  spec = importlib.util.spec_from_file_location(name, REPO / rel)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def _radard_ns():
  tree = ast.parse(RADARD.read_text(encoding="utf-8"))
  wanted_classes = {"Track", "KalmanParams"}
  wanted_assigns = {
    "SPEED", "ACCEL", "_LEAD_ACCEL_TAU", "UNMEASURED_COAST_CYCLES",
    "V_EGO_STATIONARY", "CIVIC_BOSCH_LEAD_KF_Q", "CIVIC_BOSCH_LEAD_KF_R",
    "CIVIC_BOSCH_LEAD_KF_DT",
  }

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

  simple_kalman = _load("simple_kalman", "common/simple_kalman.py")
  ns = {
    "np": np,
    "math": math,
    "DT_MDL": 0.05,
    "KF1D": simple_kalman.KF1D,
    "get_kalman_gain": simple_kalman.get_kalman_gain,
    "FirstOrderFilter": _load("filter_simple", "common/filter_simple.py").FirstOrderFilter,
  }
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(RADARD), "exec"), ns)
  assert "CIVIC_BOSCH_LEAD_KF_Q" in ns, "K2 constant CIVIC_BOSCH_LEAD_KF_Q missing from radard.py"
  assert "CIVIC_BOSCH_LEAD_KF_R" in ns, "K2 constant CIVIC_BOSCH_LEAD_KF_R missing from radard.py"
  return ns


NS = _radard_ns()


def make_track(v_seed=20.0, ident=7, civic=False, dt=CIVIC_BOSCH_LEAD_KF_DT):
  kp = NS["KalmanParams"](dt, civic_bosch_fine=civic)
  return NS["Track"](ident, v_seed, kp)


def run_measured(track, n, v_lead, v_ego=25.0, d_rel=40.0):
  for _ in range(n):
    track.update(d_rel, 0.0, v_lead - v_ego, v_lead, True, v_ego)


# ---------------------------------------------------------------------------
# A. Gain tests
# ---------------------------------------------------------------------------

class TestGains:
  def test_civic_gains_differ_from_stock(self):
    """civic_bosch_fine=True must produce different K than the stock interp table."""
    civic_kp = NS["KalmanParams"](CIVIC_BOSCH_LEAD_KF_DT, civic_bosch_fine=True)
    stock_kp = NS["KalmanParams"](CIVIC_BOSCH_LEAD_KF_DT, civic_bosch_fine=False)
    assert civic_kp.K != stock_kp.K, "civic and stock gains should differ at dt=0.06"

  def test_civic_gains_match_direct_get_kalman_gain(self):
    """Civic gains must match a direct get_kalman_gain call to ~1e-9."""
    dt = CIVIC_BOSCH_LEAD_KF_DT
    civic_kp = NS["KalmanParams"](dt, civic_bosch_fine=True)
    A = np.array([[1.0, dt], [0.0, 1.0]])
    C = np.array([[1.0, 0.0]])
    Q = np.diag(NS["CIVIC_BOSCH_LEAD_KF_Q"])
    R = np.array([[NS["CIVIC_BOSCH_LEAD_KF_R"]]])
    K_direct = NS["get_kalman_gain"](dt, A, C, Q, R)
    assert abs(civic_kp.K[0][0] - float(K_direct[0, 0])) < 1e-9
    assert abs(civic_kp.K[1][0] - float(K_direct[1, 0])) < 1e-9

  def test_civic_gains_approx_expected(self):
    """Civic gains should be approximately [[0.3513],[1.2477]]."""
    civic_kp = NS["KalmanParams"](CIVIC_BOSCH_LEAD_KF_DT, civic_bosch_fine=True)
    assert abs(civic_kp.K[0][0] - 0.3513) < 0.01, f"K[0] = {civic_kp.K[0][0]}"
    assert abs(civic_kp.K[1][0] - 1.2477) < 0.01, f"K[1] = {civic_kp.K[1][0]}"

  def test_stock_gains_from_interp_table_at_dt006(self):
    """Stock gains at dt=0.06 come from the interp tables (not the civic path)."""
    stock_kp = NS["KalmanParams"](CIVIC_BOSCH_LEAD_KF_DT, civic_bosch_fine=False)
    # Compute expected interp values from the radard tables
    dts = [i * 0.01 for i in range(1, 21)]
    K0_table = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
                0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
                0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
                0.35353899, 0.36200124]
    K1_table = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
                0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
                0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
                0.26393339, 0.26278425]
    expected_k0 = float(np.interp(CIVIC_BOSCH_LEAD_KF_DT, dts, K0_table))
    expected_k1 = float(np.interp(CIVIC_BOSCH_LEAD_KF_DT, dts, K1_table))
    assert abs(stock_kp.K[0][0] - expected_k0) < 1e-9
    assert abs(stock_kp.K[1][0] - expected_k1) < 1e-9
    # Verify the stock gains are NOT the civic gains (they are ~[[0.336],[0.291]])
    assert abs(stock_kp.K[0][0] - 0.3513) > 0.005, "stock K[0] should not match civic"

  def test_stock_gains_approx_known_values(self):
    """Stock interp at dt=0.06 should be approx [[0.2137],[0.2834]] (6th entry in dts table)."""
    stock_kp = NS["KalmanParams"](CIVIC_BOSCH_LEAD_KF_DT, civic_bosch_fine=False)
    # Actual interp(0.06, dts, K0_table) = 0.21372394, interp(0.06, dts, K1_table) = 0.28342219
    assert abs(stock_kp.K[0][0] - 0.2137) < 0.01, f"stock K[0] = {stock_kp.K[0][0]}"
    assert abs(stock_kp.K[1][0] - 0.2834) < 0.01, f"stock K[1] = {stock_kp.K[1][0]}"


# ---------------------------------------------------------------------------
# B. One-smoother cascade contract
# ---------------------------------------------------------------------------

class TestOneSmoother:
  """Synthetic braking lead pipeline: civic gains converge faster than stock."""

  DT = CIVIC_BOSCH_LEAD_KF_DT
  V_EGO = 25.0
  V_LEAD_INIT = 20.0
  TRUE_DECEL = -3.0  # m/s^2
  STEADY_CYCLES = 60
  BRAKE_CYCLES = 25
  NOISE_STD = 0.3

  def _run_pipeline(self, civic: bool):
    """Return (aLeadK_history_during_braking, aLeadK_steady_phase) for a Track."""
    rng = np.random.default_rng(42)
    kp = NS["KalmanParams"](self.DT, civic_bosch_fine=civic)
    track = NS["Track"](1, self.V_LEAD_INIT, kp)

    # Steady phase — lead at constant 20 m/s
    steady_aLeadK = []
    v_lead = float(self.V_LEAD_INIT)
    for _ in range(self.STEADY_CYCLES):
      meas = v_lead + rng.normal(0, self.NOISE_STD)
      track.update(40.0, 0.0, meas - self.V_EGO, meas, True, self.V_EGO)
      steady_aLeadK.append(track.aLeadK)

    # Braking phase
    brake_aLeadK = []
    for _ in range(self.BRAKE_CYCLES):
      v_lead += self.TRUE_DECEL * self.DT
      meas = v_lead + rng.normal(0, self.NOISE_STD)
      track.update(40.0, 0.0, meas - self.V_EGO, meas, True, self.V_EGO)
      brake_aLeadK.append(track.aLeadK)

    return brake_aLeadK, steady_aLeadK

  def test_civic_reaches_50pct_decel_within_7_cycles(self):
    """Civic track must reach 50% of true decel in <= 7 cycles from brake onset."""
    brake_aLeadK, _ = self._run_pipeline(civic=True)
    target = 0.5 * self.TRUE_DECEL  # -1.5
    cycles_to_50pct = next(
      (i + 1 for i, a in enumerate(brake_aLeadK) if a <= target),
      None
    )
    assert cycles_to_50pct is not None, (
      f"civic aLeadK never reached {target} m/s^2 in {self.BRAKE_CYCLES} cycles; "
      f"final value={brake_aLeadK[-1]:.3f}"
    )
    assert cycles_to_50pct <= 7, (
      f"civic aLeadK took {cycles_to_50pct} cycles to reach 50% decel, expected <= 7"
    )

  def test_stock_takes_strictly_longer_than_civic(self):
    """Stock gains must take strictly more cycles to reach 50% decel than civic gains."""
    target = 0.5 * self.TRUE_DECEL  # -1.5
    civic_brake, _ = self._run_pipeline(civic=True)
    stock_brake, _ = self._run_pipeline(civic=False)

    civic_lag = next(
      (i + 1 for i, a in enumerate(civic_brake) if a <= target),
      self.BRAKE_CYCLES + 1
    )
    stock_lag = next(
      (i + 1 for i, a in enumerate(stock_brake) if a <= target),
      self.BRAKE_CYCLES + 1
    )
    assert civic_lag < stock_lag, (
      f"civic convergence ({civic_lag} cycles) must be strictly faster than "
      f"stock ({stock_lag} cycles)"
    )

  def test_civic_steady_phase_noise_bounded(self):
    """During steady phase, civic aLeadK pstdev (std) must be < 0.45."""
    _, steady_aLeadK = self._run_pipeline(civic=True)
    # Use second half of steady phase (after KF has settled)
    settled = steady_aLeadK[len(steady_aLeadK) // 2:]
    pstdev = float(np.std(settled))
    assert pstdev < 0.45, (
      f"civic aLeadK steady-phase noise std={pstdev:.4f}, expected < 0.45"
    )


# ---------------------------------------------------------------------------
# C. Wiring (source-level asserts on radard.py text)
# ---------------------------------------------------------------------------

class TestWiring:
  def test_civic_bosch_fine_in_kalman_params_init(self):
    src = RADARD.read_text(encoding="utf-8")
    assert "civic_bosch_fine" in src, \
      "'civic_bosch_fine' parameter not found in radard.py"

  def test_radar_fresh_gate_present_in_radard_update(self):
    src = RADARD.read_text(encoding="utf-8")
    assert "radar_fresh" in src, \
      "'radar_fresh' gate not found in radard.py RadarD.update"

  def test_ar_pts_uses_pt_measured_and_radar_fresh(self):
    src = RADARD.read_text(encoding="utf-8")
    assert "pt.measured and radar_fresh" in src, \
      "'pt.measured and radar_fresh' not found in ar_pts construction"

  def test_civic_bosch_fine_gating_references_honda_civic_bosch(self):
    src = RADARD.read_text(encoding="utf-8")
    assert "HONDA_CIVIC_BOSCH" in src, \
      "'HONDA_CIVIC_BOSCH' not referenced in _civic_bosch_fine gating"

  def test_civic_bosch_fine_gating_references_radar_unavailable(self):
    src = RADARD.read_text(encoding="utf-8")
    assert "radarUnavailable" in src, \
      "'radarUnavailable' not referenced in _civic_bosch_fine gating"

  def test_civic_bosch_fine_attribute_set_in_radard_init(self):
    src = RADARD.read_text(encoding="utf-8")
    assert "_civic_bosch_fine" in src, \
      "'_civic_bosch_fine' attribute not set in RadarD.__init__"


if __name__ == "__main__":
  raise SystemExit(pytest.main([__file__, "-v"]))
