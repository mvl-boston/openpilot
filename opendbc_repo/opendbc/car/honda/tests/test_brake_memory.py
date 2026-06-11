"""Unit tests for G3 BrakeMemory (nrdrbranchdebug-cra.3).

Run from cwd C:\\nrdrbranchdebug\\pyshim:
  PYTHONPATH="C:\\nrdrbranchdebug\\wt-long\\opendbc_repo" py -3.13 -m pytest \
    "C:\\nrdrbranchdebug\\wt-long\\opendbc_repo\\opendbc\\car\\honda\\tests\\test_brake_memory.py" \
    -q -p no:cacheprovider -o addopts="" \
    --confcutdir="C:\\nrdrbranchdebug\\wt-long\\opendbc_repo\\opendbc"

Same stub strategy as test_carcontroller_learners.py: only openpilot.common.params is faked;
everything in opendbc is importable from PYTHONPATH.
"""
import json
import math
import os
import sys
import tempfile
import types
import unittest


class _FakeParams:
  def __init__(self): self._store = {}
  def get(self, k): return self._store.get(k)
  def get_bool(self, k): return bool(self._store.get(k, False))
  def put_nonblocking(self, k, v): self._store[k] = v

_op = types.ModuleType("openpilot")
_op_common = types.ModuleType("openpilot.common")
_op_params = types.ModuleType("openpilot.common.params")
_op_params.Params = _FakeParams
sys.modules.setdefault("openpilot", _op)
sys.modules.setdefault("openpilot.common", _op_common)
sys.modules.setdefault("openpilot.common.params", _op_params)

from opendbc.car.honda.carcontroller import (  # noqa: E402
  BrakeMemory,
  _brake_bin_index,
  _load_brake_profiles,
  _write_brake_profiles_atomic,
  _BRAKE_BIN_EDGES,
  _BRAKE_N_BINS,
  _BRAKE_PRELOAD_CAP,
  _BRAKE_EMA_ALPHA,
  _BRAKE_DECAY_PER_TICK,
  _BRAKE_AEGO_VAR_DISCARD,
  BRAKE_LEARN_VERSION,
  BRAKE_PROFILES_PATH,
)

assert _BRAKE_N_BINS == 4, f"expected 4 speed bins, got {_BRAKE_N_BINS}"
assert _BRAKE_PRELOAD_CAP == -0.5, "preload cap must be -0.5 m/s^2 per spec"


def _zeros():
  return [0.0] * _BRAKE_N_BINS


def _mk(bins=None, fp="HONDA_CIVIC_BOSCH"):
  return BrakeMemory(_zeros() if bins is None else bins, fp)


def _brake_episode(mem, integrator, v_ego, a_ego=None, n_sustain=5, pitch=0.0):
  """Run one full engage->sustain->release episode. Returns the engagement-edge preload."""
  if a_ego is None:
    a_ego = -1.0
  preload = mem.update(braking=True, integrator=0.0, v_ego=v_ego, a_ego=a_ego, pitch=pitch)
  for _ in range(n_sustain):
    mem.update(braking=True, integrator=integrator, v_ego=v_ego, a_ego=a_ego, pitch=pitch)
  # release edge — integrator carries the converged value
  mem.update(braking=False, integrator=integrator, v_ego=v_ego, a_ego=a_ego, pitch=pitch)
  return preload


class TestBinSelection(unittest.TestCase):
  def test_edges(self):
    self.assertEqual(_brake_bin_index(0.0), 0)
    self.assertEqual(_brake_bin_index(1.9), 0)
    self.assertEqual(_brake_bin_index(2.0), 1)
    self.assertEqual(_brake_bin_index(4.9), 1)
    self.assertEqual(_brake_bin_index(5.0), 2)
    self.assertEqual(_brake_bin_index(9.9), 2)
    self.assertEqual(_brake_bin_index(10.0), 3)
    self.assertEqual(_brake_bin_index(19.9), 3)

  def test_out_of_range(self):
    self.assertEqual(_brake_bin_index(20.0), -1)   # >= top edge: never preloads
    self.assertEqual(_brake_bin_index(50.0), -1)
    self.assertEqual(_brake_bin_index(-1.0), -1)
    self.assertEqual(_brake_bin_index(float("nan")), -1)


class TestEMACacheOnRelease(unittest.TestCase):
  def test_release_folds_integrator_into_bin(self):
    mem = _mk()
    idx = _brake_bin_index(7.0)  # bin 2
    _brake_episode(mem, integrator=-0.4, v_ego=7.0)
    # EMA from 0.0: 0.9*0 + 0.1*(-0.4) = -0.04
    self.assertAlmostEqual(mem.bins[idx], (1 - _BRAKE_EMA_ALPHA) * 0.0 + _BRAKE_EMA_ALPHA * -0.4,
                           places=9)

  def test_repeated_episodes_converge_toward_value(self):
    mem = _mk()
    idx = _brake_bin_index(7.0)
    for _ in range(200):
      _brake_episode(mem, integrator=-0.4, v_ego=7.0)
    self.assertAlmostEqual(mem.bins[idx], -0.4, places=3)

  def test_only_engagement_bin_updated(self):
    mem = _mk()
    _brake_episode(mem, integrator=-0.3, v_ego=1.0)  # bin 0
    self.assertLess(mem.bins[0], 0.0)
    self.assertEqual(mem.bins[1], 0.0)
    self.assertEqual(mem.bins[2], 0.0)
    self.assertEqual(mem.bins[3], 0.0)

  def test_release_outside_bins_no_update(self):
    mem = _mk()
    _brake_episode(mem, integrator=-0.4, v_ego=25.0)  # >= top edge
    self.assertEqual(mem.bins, _zeros())


class TestPreloadOnEngagement(unittest.TestCase):
  def test_preload_returned_on_engage_edge(self):
    mem = _mk(bins=[-0.3, 0.0, 0.0, 0.0])
    p = mem.update(braking=True, integrator=0.0, v_ego=1.0, a_ego=-1.0, pitch=0.0)
    self.assertAlmostEqual(p, -0.3, places=9)

  def test_no_preload_when_not_engage_edge(self):
    mem = _mk(bins=[-0.3, 0.0, 0.0, 0.0])
    mem.update(braking=True, integrator=0.0, v_ego=1.0, a_ego=-1.0, pitch=0.0)  # engage
    p = mem.update(braking=True, integrator=-0.2, v_ego=1.0, a_ego=-1.0, pitch=0.0)  # sustain
    self.assertEqual(p, 0.0)

  def test_preload_zero_for_empty_bin(self):
    mem = _mk()
    p = mem.update(braking=True, integrator=0.0, v_ego=7.0, a_ego=-1.0, pitch=0.0)
    self.assertEqual(p, 0.0)


class TestPreloadCap(unittest.TestCase):
  def test_loaded_value_beyond_cap_clamped(self):
    # construction clamps any out-of-range bin to [_BRAKE_PRELOAD_CAP, 0.0]
    mem = _mk(bins=[-5.0, 1.0, 0.0, 0.0])
    self.assertEqual(mem.bins[0], _BRAKE_PRELOAD_CAP)
    self.assertEqual(mem.bins[1], 0.0)  # positive clamped to 0

  def test_ema_result_never_exceeds_cap(self):
    mem = _mk(bins=[_BRAKE_PRELOAD_CAP, 0.0, 0.0, 0.0])
    for _ in range(500):
      _brake_episode(mem, integrator=-2.0, v_ego=1.0)  # try to push way past cap
    self.assertGreaterEqual(mem.bins[0], _BRAKE_PRELOAD_CAP - 1e-12)

  def test_preload_never_more_negative_than_cap(self):
    mem = _mk(bins=[_BRAKE_PRELOAD_CAP, 0.0, 0.0, 0.0])
    p = mem.update(braking=True, integrator=0.0, v_ego=1.0, a_ego=-1.0, pitch=0.0)
    self.assertGreaterEqual(p, _BRAKE_PRELOAD_CAP - 1e-12)


class TestDecay(unittest.TestCase):
  def test_idle_decays_toward_zero(self):
    mem = _mk(bins=[-0.4, -0.4, -0.4, -0.4])
    before = list(mem.bins)
    for _ in range(50):
      mem.update(braking=False, integrator=0.0, v_ego=0.0, a_ego=0.0, pitch=0.0)
    for i in range(_BRAKE_N_BINS):
      self.assertGreater(mem.bins[i], before[i], "idle decay must move bins toward 0")
      self.assertLess(mem.bins[i], 0.0)

  def test_decay_rate_magnitude(self):
    # one idle tick moves a bin toward 0 by _BRAKE_DECAY_PER_TICK * |cap| (x0.99/min cadence)
    mem = _mk(bins=[-0.4, 0.0, 0.0, 0.0])
    mem.update(braking=False, integrator=0.0, v_ego=0.0, a_ego=0.0, pitch=0.0)
    step = _BRAKE_DECAY_PER_TICK * (-_BRAKE_PRELOAD_CAP)
    self.assertAlmostEqual(mem.bins[0], -0.4 + step, places=9)

  def test_decay_does_not_cross_zero(self):
    mem = _mk(bins=[-1e-6, 0.0, 0.0, 0.0])
    for _ in range(100):
      mem.update(braking=False, integrator=0.0, v_ego=0.0, a_ego=0.0, pitch=0.0)
    self.assertEqual(mem.bins[0], 0.0)


class TestVarianceDiscard(unittest.TestCase):
  def test_high_variance_episode_discarded(self):
    mem = _mk()
    idx = _brake_bin_index(7.0)
    # engage, then sustain with wildly varying aEgo -> variance > discard threshold
    mem.update(braking=True, integrator=0.0, v_ego=7.0, a_ego=-3.0, pitch=0.0)
    for a in (2.0, -3.0, 2.0, -3.0, 2.0, -3.0):
      mem.update(braking=True, integrator=-0.4, v_ego=7.0, a_ego=a, pitch=0.0)
    mem.update(braking=False, integrator=-0.4, v_ego=7.0, a_ego=0.0, pitch=0.0)
    self.assertEqual(mem.bins[idx], 0.0, "noisy episode must not be cached")

  def test_low_variance_episode_kept(self):
    mem = _mk()
    idx = _brake_bin_index(7.0)
    mem.update(braking=True, integrator=0.0, v_ego=7.0, a_ego=-1.0, pitch=0.0)
    for _ in range(6):
      mem.update(braking=True, integrator=-0.4, v_ego=7.0, a_ego=-1.0, pitch=0.0)
    mem.update(braking=False, integrator=-0.4, v_ego=7.0, a_ego=-1.0, pitch=0.0)
    self.assertLess(mem.bins[idx], 0.0, "steady episode must be cached")

  def test_threshold_boundary(self):
    self.assertEqual(_BRAKE_AEGO_VAR_DISCARD, 0.25)


class TestPitchScaling(unittest.TestCase):
  def test_downhill_reduces_preload(self):
    mem = _mk(bins=[-0.4, 0.0, 0.0, 0.0])
    flat = mem.update(braking=True, integrator=0.0, v_ego=1.0, a_ego=-1.0, pitch=0.0)
    mem._was_braking = False  # reset edge state
    down = mem.update(braking=True, integrator=0.0, v_ego=1.0, a_ego=-1.0, pitch=-0.3)
    self.assertGreater(down, flat, "downhill (gravity helps) must reduce preload magnitude")

  def test_steep_downhill_zeroes_preload(self):
    mem = _mk(bins=[-0.4, 0.0, 0.0, 0.0])
    p = mem.update(braking=True, integrator=0.0, v_ego=1.0, a_ego=-1.0,
                   pitch=-math.pi / 2)  # sin(pitch) = -1 -> scale clamps to 0
    self.assertAlmostEqual(p, 0.0, places=9)

  def test_uphill_does_not_amplify_beyond_cache(self):
    mem = _mk(bins=[-0.4, 0.0, 0.0, 0.0])
    up = mem.update(braking=True, integrator=0.0, v_ego=1.0, a_ego=-1.0, pitch=0.3)
    self.assertGreaterEqual(up, -0.4 - 1e-12, "uphill scale capped at 1.0 (never amplifies)")

  def test_nan_pitch_full_preload(self):
    mem = _mk(bins=[-0.4, 0.0, 0.0, 0.0])
    p = mem.update(braking=True, integrator=0.0, v_ego=1.0, a_ego=-1.0, pitch=float("nan"))
    self.assertAlmostEqual(p, -0.4, places=9)


class TestNaNSafety(unittest.TestCase):
  def test_nan_integrator_not_cached(self):
    mem = _mk()
    idx = _brake_bin_index(7.0)
    mem.update(braking=True, integrator=0.0, v_ego=7.0, a_ego=-1.0, pitch=0.0)
    mem.update(braking=True, integrator=float("nan"), v_ego=7.0, a_ego=-1.0, pitch=0.0)
    mem.update(braking=False, integrator=float("nan"), v_ego=7.0, a_ego=-1.0, pitch=0.0)
    self.assertTrue(math.isfinite(mem.bins[idx]))
    self.assertEqual(mem.bins[idx], 0.0)

  def test_preload_always_finite(self):
    for bins in ([-0.4, 0, 0, 0], [float("nan"), 0, 0, 0]):
      mem = _mk(bins=bins)
      for v in (0.0, 1.0, 7.0, 25.0, float("nan")):
        for p in (0.0, 0.3, -0.3, float("nan")):
          out = mem.update(braking=True, integrator=0.0, v_ego=v, a_ego=-1.0, pitch=p)
          mem._was_braking = False
          self.assertTrue(math.isfinite(out))


class TestFingerprintReset(unittest.TestCase):
  def _write(self, path, payload):
    with open(path, "w", encoding="utf-8") as f:
      json.dump(payload, f)

  def test_load_roundtrip(self):
    import opendbc.car.honda.carcontroller as cc
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "bp.json")
      orig = cc.BRAKE_PROFILES_PATH
      cc.BRAKE_PROFILES_PATH = path
      try:
        bins = [-0.3, -0.2, -0.1, -0.05]
        cc._write_brake_profiles_atomic("HONDA_CIVIC_BOSCH", bins)
        self.assertFalse(os.path.exists(path + ".tmp"), "atomic write leaves no .tmp")
        loaded = cc._load_brake_profiles("HONDA_CIVIC_BOSCH")
        for a, b in zip(loaded, bins):
          self.assertAlmostEqual(a, b, places=9)
      finally:
        cc.BRAKE_PROFILES_PATH = orig

  def test_fingerprint_mismatch_resets(self):
    import opendbc.car.honda.carcontroller as cc
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "bp.json")
      orig = cc.BRAKE_PROFILES_PATH
      cc.BRAKE_PROFILES_PATH = path
      try:
        cc._write_brake_profiles_atomic("HONDA_CIVIC_BOSCH", [-0.3, -0.2, -0.1, -0.05])
        loaded = cc._load_brake_profiles("ACURA_RDX_3G")
        self.assertEqual(loaded, _zeros(), "fingerprint mismatch -> reset to zeros")
      finally:
        cc.BRAKE_PROFILES_PATH = orig

  def test_version_mismatch_resets(self):
    import opendbc.car.honda.carcontroller as cc
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "bp.json")
      orig = cc.BRAKE_PROFILES_PATH
      cc.BRAKE_PROFILES_PATH = path
      try:
        self._write(path, {"car_fingerprint": "HONDA_CIVIC_BOSCH",
                           "learn_version": BRAKE_LEARN_VERSION + 99,
                           "bins": [-0.3, -0.2, -0.1, -0.05]})
        loaded = cc._load_brake_profiles("HONDA_CIVIC_BOSCH")
        self.assertEqual(loaded, _zeros(), "version mismatch -> reset to zeros")
      finally:
        cc.BRAKE_PROFILES_PATH = orig

  def test_corrupt_file_resets(self):
    import opendbc.car.honda.carcontroller as cc
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "bp.json")
      orig = cc.BRAKE_PROFILES_PATH
      cc.BRAKE_PROFILES_PATH = path
      try:
        with open(path, "w", encoding="utf-8") as f:
          f.write("{ not json")
        self.assertEqual(cc._load_brake_profiles("HONDA_CIVIC_BOSCH"), _zeros())
      finally:
        cc.BRAKE_PROFILES_PATH = orig

  def test_wrong_bin_count_resets(self):
    import opendbc.car.honda.carcontroller as cc
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "bp.json")
      orig = cc.BRAKE_PROFILES_PATH
      cc.BRAKE_PROFILES_PATH = path
      try:
        self._write(path, {"car_fingerprint": "HONDA_CIVIC_BOSCH",
                           "learn_version": BRAKE_LEARN_VERSION, "bins": [-0.3, -0.2]})
        self.assertEqual(cc._load_brake_profiles("HONDA_CIVIC_BOSCH"), _zeros())
      finally:
        cc.BRAKE_PROFILES_PATH = orig

  def test_missing_file_is_noop(self):
    import opendbc.car.honda.carcontroller as cc
    with tempfile.TemporaryDirectory() as d:
      orig = cc.BRAKE_PROFILES_PATH
      cc.BRAKE_PROFILES_PATH = os.path.join(d, "does_not_exist.json")
      try:
        self.assertEqual(cc._load_brake_profiles("HONDA_CIVIC_BOSCH"), _zeros())
      finally:
        cc.BRAKE_PROFILES_PATH = orig


class TestDefaultPath(unittest.TestCase):
  def test_path_is_data_no_params_key(self):
    self.assertEqual(BRAKE_PROFILES_PATH, "/data/honda_brake_profiles.json")


if __name__ == "__main__":
  unittest.main()
