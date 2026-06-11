"""
Unit tests for LongGasLearner (G1 + G4 beads).

Run from cwd C:\\nrdrbranchdebug\\pyshim:
  PYTHONPATH="C:\\nrdrbranchdebug\\wt-long\\opendbc_repo" py -3.13 -m pytest \
    "C:\\nrdrbranchdebug\\wt-long\\opendbc_repo\\opendbc\\car\\honda\\tests\\test_carcontroller_learners.py" \
    -q -p no:cacheprovider -o addopts="" \
    --confcutdir="C:\\nrdrbranchdebug\\wt-long\\opendbc_repo\\opendbc"
"""
import math
import sys
import types
import unittest


# ---------------------------------------------------------------------------
# Minimal stub: only openpilot.common.params is needed; everything else in
# opendbc is importable directly from the PYTHONPATH.
# ---------------------------------------------------------------------------

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

# Now import what we need from the production module
from opendbc.car.honda.carcontroller import (  # noqa: E402
  LongGasLearner,
  _LAG_TICKS,
  _LEARNER_DT,
  _HARD_LO,
  _HARD_HI,
  _SOFT_LO,
  _SOFT_HI,
  _DECAY_PER_TICK,
  _FACTOR_FILTER_ALPHA,
  _PITCH_DEADBAND,
  LEARN_VERSION,
)

# Verify tick cadence assumptions that test design relies on
assert _LAG_TICKS == 25, f"Expected 25 lag ticks (0.5 s), got {_LAG_TICKS}"
assert abs(_LEARNER_DT - 0.02) < 1e-9, f"Expected 0.02 s learner DT, got {_LEARNER_DT}"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_learner(init_gas=1.0, init_wind=1.0, fingerprint="HONDA_CIVIC_BOSCH"):
  return LongGasLearner(init_gas, init_wind, fingerprint)


def _steady_kwargs(**overrides):
  base = dict(
    accel_cmd=0.5,
    a_ego=0.5,
    gas_pedal_force=0.6,
    wind_brake_ms2=0.1,
    long_active=True,
    long_pid=True,
    gas_pressed=False,
    brake_pressed=False,
    v_ego=15.0,
    at_standstill=False,
    pitch=0.0,
    brake_addon=0.0,
    at_accel_max=False,
  )
  base.update(overrides)
  return base


def _tick_n(learner, n, **kwargs):
  """Run n learner ticks; returns (gasfactor, windfactor) after last tick."""
  gf = wf = None
  for _ in range(n):
    gf, wf = learner.update(**_steady_kwargs(**kwargs))
  return gf, wf


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestLagAlignment(unittest.TestCase):
  """G1: gas_error must be attributed against the LAGGED command, not the current one."""

  def test_step_at_t0_not_attributed_immediately(self):
    """
    A command step at t=0 must NOT cause learning until the deque has aged
    through LAG_TICKS ticks. The first tick after a step the deque[0] still
    holds the pre-step value → error = 0 → no learning.
    """
    learner = _make_learner()

    # Pre-fill with zero-error steady state
    for _ in range(_LAG_TICKS + 5):
      learner.update(**_steady_kwargs(accel_cmd=0.5, a_ego=0.5))

    gas_baseline = learner.raw_gasfactor

    # Single tick with big positive error; deque[0] is still 0.5, a_ego=0.5 → error = 0
    learner.update(**_steady_kwargs(accel_cmd=1.5, a_ego=0.5, gas_pedal_force=1.0))
    self.assertAlmostEqual(
      learner.raw_gasfactor, gas_baseline, places=6,
      msg="Must not learn immediately on a step — lagged cmd is still 0.5"
    )

  def test_error_attributed_after_lag_delay(self):
    """
    After LAG_TICKS + 1 ticks (enough to fully displace the deque), the step
    propagates to deque[0] and learning begins.

    Deque maxlen = LAG_TICKS + 1 = 26. After pre-fill, all 26 slots hold 0.5.
    Pushing 1.5 for LAG_TICKS=25 ticks leaves one 0.5 at deque[0].
    One more tick (LAG_TICKS+1 total) fully displaces the pre-fill.
    """
    learner = _make_learner()

    # Steady pre-fill
    for _ in range(_LAG_TICKS + 5):
      learner.update(**_steady_kwargs(accel_cmd=0.5, a_ego=0.5))

    gas_baseline = learner.raw_gasfactor

    # Hold the step for LAG_TICKS+1 ticks so deque[0] is finally = 1.5
    for _ in range(_LAG_TICKS + 1):
      learner.update(**_steady_kwargs(accel_cmd=1.5, a_ego=0.5, gas_pedal_force=1.0))

    self.assertGreater(
      learner.raw_gasfactor, gas_baseline,
      msg="After full lag displacement, lagged error (1.5-0.5=1.0) must drive gasfactor up"
    )

  def test_old_code_footgun_prevented(self):
    """
    Regression guard: old code did gas_error = self.accel - CS.out.aEgo (no lag).
    New code: first tick after a step must NOT update gasfactor.
    """
    learner = _make_learner()
    for _ in range(_LAG_TICKS + 5):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0))

    before = learner.raw_gasfactor
    # Large instantaneous error — old code would learn; new code must not
    learner.update(**_steady_kwargs(accel_cmd=2.0, a_ego=1.0, gas_pedal_force=1.0))
    self.assertAlmostEqual(learner.raw_gasfactor, before, places=6)


class TestQuasiSteadyGate(unittest.TestCase):
  """G1: learning must be gated on quasi-steady (low accel rate) samples."""

  def test_fast_ramp_suppressed(self):
    """A rapidly changing command (rate >> 0.3 m/s³) should not drive learning."""
    learner = _make_learner()
    # Pre-fill deque in quasi-steady state
    for _ in range(_LAG_TICKS + 5):
      learner.update(**_steady_kwargs(accel_cmd=0.5, a_ego=0.5))

    baseline = learner.raw_gasfactor

    # Ramp: 0.5 → 2.5 over LAG_TICKS ticks → rate ~4 m/s³ >> 0.3 threshold
    for i in range(_LAG_TICKS):
      cmd = 0.5 + 2.0 * i / _LAG_TICKS
      learner.update(**_steady_kwargs(accel_cmd=cmd, a_ego=0.0, gas_pedal_force=1.0))

    # Should not have grown meaningfully (only tiny decay-back effects allowed)
    self.assertLessEqual(learner.raw_gasfactor, baseline + 0.05,
                         "Fast ramp must be blocked by quasi-steady gate")

  def test_steady_input_learns(self):
    """Constant command with non-zero error must cause learning."""
    learner = _make_learner()
    for _ in range(2 * _LAG_TICKS):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=0.8, gas_pedal_force=0.8))

    self.assertGreater(learner.raw_gasfactor, 1.0,
                       "Steady positive error must push gasfactor above 1.0")


class TestDequeReset(unittest.TestCase):
  """G1: deque must reset on engagement edge and gasPressed."""

  def test_reset_on_engagement_edge(self):
    """After disengage → re-engage, deque pre-fills with current accel_cmd."""
    learner = _make_learner()
    # Engaged steady state
    for _ in range(_LAG_TICKS + 5):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0,
                                     long_active=True, long_pid=True))

    # Disengage
    for _ in range(5):
      learner.update(**_steady_kwargs(long_active=False, long_pid=False))

    # Re-engage with a completely different command value
    learner.update(**_steady_kwargs(accel_cmd=0.3, a_ego=1.0,
                                   long_active=True, long_pid=True))

    self.assertEqual(len(learner._accel_deque), _LAG_TICKS + 1,
                     "Deque must be full after reset pre-fill")
    for v in learner._accel_deque:
      self.assertAlmostEqual(v, 0.3, places=9,
                             msg="All deque entries must equal reset value 0.3")

  def test_reset_on_gas_pressed(self):
    """gasPressed must trigger deque reset pre-filled with current command."""
    learner = _make_learner()
    for _ in range(_LAG_TICKS + 5):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0))

    learner.update(**_steady_kwargs(accel_cmd=0.5, a_ego=1.0, gas_pressed=True))
    for v in learner._accel_deque:
      self.assertAlmostEqual(v, 0.5, places=9)

  def test_fresh_learner_deque_empty(self):
    """Freshly constructed learner has an empty deque (no spurious learning possible)."""
    learner = _make_learner()
    self.assertEqual(len(learner._accel_deque), 0)


class TestRelativeClampAndDecay(unittest.TestCase):
  """G4: hard clamp + decay-back toward nominal from outside soft band."""

  def test_hard_clamp_upper(self):
    """raw_gasfactor must never exceed _HARD_HI."""
    learner = _make_learner(init_gas=1.0)
    for _ in range(1000):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=0.0, gas_pedal_force=2.0))
    self.assertLessEqual(learner.raw_gasfactor, _HARD_HI + 1e-9)

  def test_hard_clamp_lower(self):
    """raw_gasfactor must never go below _HARD_LO."""
    learner = _make_learner(init_gas=1.0)
    for _ in range(1000):
      # negative error pushes gasfactor down
      learner.update(**_steady_kwargs(accel_cmd=0.0, a_ego=2.0, gas_pedal_force=2.0))
    self.assertGreaterEqual(learner.raw_gasfactor, _HARD_LO - 1e-9)

  def test_decay_back_from_high(self):
    """Factor above SOFT_HI decays toward 1.0 even when learning is frozen."""
    learner = _make_learner(init_gas=_HARD_HI)
    # Freeze learning via brakePressed
    for _ in range(3000):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0, brake_pressed=True))
    self.assertLess(learner.raw_gasfactor, _HARD_HI - 0.01,
                    "Factor above SOFT_HI must decay toward 1.0")

  def test_no_decay_inside_soft_band(self):
    """Factor at 1.0 (inside soft band) must not drift under decay rules."""
    learner = _make_learner(init_gas=1.0)
    for _ in range(500):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0, gas_pedal_force=0.0))
    self.assertAlmostEqual(learner.raw_gasfactor, 1.0, places=4)


class TestNaNReset(unittest.TestCase):
  """G4: NaN/inf in raw integrators → reset to 1.0; never propagates to commands."""

  def test_nan_gasfactor_resets(self):
    learner = _make_learner()
    learner.raw_gasfactor = float("nan")
    gf, _ = learner.update(**_steady_kwargs())
    self.assertTrue(math.isfinite(gf))
    self.assertTrue(math.isfinite(learner.raw_gasfactor))

  def test_inf_windfactor_resets(self):
    learner = _make_learner()
    learner.raw_windfactor = float("inf")
    _, wf = learner.update(**_steady_kwargs())
    self.assertTrue(math.isfinite(wf))
    self.assertTrue(math.isfinite(learner.raw_windfactor))

  def test_nan_applied_factor_healed(self):
    """NaN injected into the applied filter output must be healed on next tick."""
    learner = _make_learner()
    learner.gasfactor = float("nan")
    learner.windfactor = float("nan")
    gf, wf = learner.update(**_steady_kwargs())
    self.assertTrue(math.isfinite(gf))
    self.assertTrue(math.isfinite(wf))

  def test_nan_never_propagates_to_return(self):
    """Even with both raw and applied set to NaN, return values must be finite."""
    learner = _make_learner()
    learner.raw_gasfactor = float("nan")
    learner.raw_windfactor = float("nan")
    learner.gasfactor = float("nan")
    learner.windfactor = float("nan")
    for _ in range(3):
      gf, wf = learner.update(**_steady_kwargs())
      self.assertTrue(math.isfinite(gf))
      self.assertTrue(math.isfinite(wf))


class TestFingerprintLoadInit(unittest.TestCase):
  """G4: applied factor must initialize at loaded value (no startup transient)."""

  def test_no_startup_transient(self):
    """Applied factors must equal raw on construction."""
    init = 1.25
    learner = LongGasLearner(init, init, "HONDA_CIVIC_BOSCH")
    self.assertAlmostEqual(learner.gasfactor, init, places=6)
    self.assertAlmostEqual(learner.windfactor, init, places=6)

  def test_out_of_range_init_clamped(self):
    """init values outside [HARD_LO, HARD_HI] must be clamped."""
    learner = LongGasLearner(99.0, -5.0, "HONDA_CIVIC_BOSCH")
    self.assertLessEqual(learner.raw_gasfactor, _HARD_HI)
    self.assertGreaterEqual(learner.raw_windfactor, _HARD_LO)

  def test_nan_init_resets_to_nominal(self):
    """NaN init must produce a valid learner at 1.0."""
    learner = LongGasLearner(float("nan"), float("nan"), "HONDA_CIVIC_BOSCH")
    self.assertEqual(learner.raw_gasfactor, 1.0)
    self.assertEqual(learner.raw_windfactor, 1.0)


class TestAppliedFactorFilter(unittest.TestCase):
  """G4 rail 4: slow first-order filter between raw integrator and applied factor."""

  def test_applied_lags_raw(self):
    """After a sudden raw integrator jump, applied factor must lag behind."""
    learner = _make_learner(init_gas=1.0)
    # Warm up filter in steady state
    for _ in range(50):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0))

    # Force a sudden raw jump
    learner.raw_gasfactor = 1.5
    gf, _ = learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0))

    self.assertGreater(gf, 1.0)
    self.assertLess(gf, 1.49,
                    "Applied factor must lag raw after sudden jump (RC ~7.5 s)")

  def test_filter_alpha_matches_spec(self):
    """Filter alpha = DT / (RC + DT) for RC = 7.5 s."""
    dt = _LEARNER_DT
    expected = dt / (7.5 + dt)
    self.assertAlmostEqual(_FACTOR_FILTER_ALPHA, expected, places=8)

  def test_init_no_transient(self):
    """After construction with init=1.3, no first-tick jump to a wrong value."""
    init = 1.3
    learner = LongGasLearner(init, init, "HONDA_CIVIC_BOSCH")
    # Don't tick at all — applied must already equal init
    self.assertAlmostEqual(learner.gasfactor, init, places=6)


class TestFreezeDuringBrakeStandstill(unittest.TestCase):
  """G4 rail 5: learning freeze during brakePressed and standstill."""

  def _warm_up(self, learner, ticks=150):
    """Warm up learner in steady state with a small positive error."""
    for _ in range(ticks):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=0.8, gas_pedal_force=0.8))

  def test_brake_pressed_blocks_learning_increase(self):
    """gasfactor must not grow while brakePressed=True."""
    learner = _make_learner()
    self._warm_up(learner)
    before = learner.raw_gasfactor

    # Large error but brake pressed → must not increase
    for _ in range(100):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=0.0,
                                     brake_pressed=True, gas_pedal_force=0.8))
    self.assertLessEqual(learner.raw_gasfactor, before + 1e-9)

  def test_standstill_blocks_learning(self):
    """gasfactor must not grow during standstill."""
    learner = _make_learner()
    self._warm_up(learner)
    before = learner.raw_gasfactor

    for _ in range(100):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=0.0,
                                     at_standstill=True, v_ego=0.0,
                                     gas_pedal_force=0.8))
    self.assertLessEqual(learner.raw_gasfactor, before + 1e-9)


class TestSaturationDecay(unittest.TestCase):
  """G4 rail 6: gasfactor decays when at_accel_max=True."""

  def test_saturation_decays_gasfactor(self):
    learner = _make_learner(init_gas=1.3)
    before = learner.raw_gasfactor
    for _ in range(300):
      learner.update(**_steady_kwargs(at_accel_max=True, gas_pedal_force=2.5,
                                     accel_cmd=1.0, a_ego=1.0))
    self.assertLess(learner.raw_gasfactor, before,
                    "Saturation must decay gasfactor")

  def test_no_saturation_no_forced_decay(self):
    """Without saturation, factor near 1.1 decays only via soft-band decay (slow)."""
    learner = _make_learner(init_gas=1.1)
    before = learner.raw_gasfactor

    # Only soft-band decay applies
    for _ in range(200):
      learner.update(**_steady_kwargs(at_accel_max=False, accel_cmd=1.0, a_ego=1.0,
                                     gas_pedal_force=1.0))
    # Should be closer to 1.0 but not at saturation-decay speed
    after = learner.raw_gasfactor
    self.assertLess(abs(after - 1.0), abs(before - 1.0) + 1e-6)


class TestHillDeadbandGate(unittest.TestCase):
  """G4 rail 7: learning blocked when |pitch| >= _PITCH_DEADBAND."""

  def test_large_pitch_blocks_learning(self):
    """Pitch above deadband must prevent gasfactor update even with non-zero error."""
    learner = _make_learner()
    # Fill deque for quasi-steady
    for _ in range(_LAG_TICKS + 5):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0))

    before = learner.raw_gasfactor
    big_pitch = _PITCH_DEADBAND * 2.0

    for _ in range(300):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=0.0,
                                     gas_pedal_force=1.0, pitch=big_pitch))
    self.assertAlmostEqual(learner.raw_gasfactor, before, delta=0.02,
                           msg="Pitch above deadband must block learning")

  def test_small_pitch_allows_learning(self):
    """Pitch well inside deadband must allow normal learning."""
    learner = _make_learner()
    for _ in range(_LAG_TICKS + 5):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0))
    before = learner.raw_gasfactor

    small_pitch = _PITCH_DEADBAND * 0.3
    for _ in range(300):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=0.0,
                                     gas_pedal_force=1.0, pitch=small_pitch))
    self.assertGreater(learner.raw_gasfactor, before)


class TestAntiWindupShadowParity(unittest.TestCase):
  """
  G1+G4: anti-windup shadows must operate on the lag-aligned command.

  Old code: shadows updated unconditionally each tick regardless of lag state.
  New code: at_accel_max freezes gasfactor AND applies saturation-decay; the
  shadow gasfactor_before_maxgas only advances when NOT at saturation.

  Scenario: sustained saturation + positive error must not ratchet gasfactor up.
  """

  def test_saturation_does_not_ratchet_gasfactor_up(self):
    """Under sustained saturation + positive error, gasfactor must not grow."""
    learner = _make_learner(init_gas=1.0)

    # Warm up in normal state
    for _ in range(_LAG_TICKS + 10):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=1.0))
    pre_sat = learner.raw_gasfactor

    # Sustained saturation with large positive error
    for _ in range(400):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=0.5,
                                     at_accel_max=True, gas_pedal_force=2.0))
    post_sat = learner.raw_gasfactor

    self.assertLessEqual(post_sat, pre_sat + 1e-6,
                         "Anti-windup must prevent gasfactor ratchet under saturation")

  def test_windfactor_brake_shadow_preserved(self):
    """
    windfactor_before_brake shadow: factor must not decrease below its
    value just before gas_pedal_force dropped to zero (braking transition).
    """
    learner = _make_learner(init_wind=1.1)
    # Learn up a bit
    for _ in range(_LAG_TICKS + 50):
      learner.update(**_steady_kwargs(accel_cmd=1.0, a_ego=0.8,
                                     gas_pedal_force=0.8, wind_brake_ms2=0.1))
    pre_brake = learner.raw_windfactor

    # Enter braking: gas_pedal_force <= 0 activates the shadow
    for _ in range(100):
      learner.update(**_steady_kwargs(accel_cmd=0.0, a_ego=0.5,
                                     gas_pedal_force=0.0, brake_pressed=True))
    post_brake = learner.raw_windfactor

    self.assertGreaterEqual(post_brake, pre_brake - 1e-6,
                            "windfactor must not fall below pre-brake shadow")


class TestLearnVersion(unittest.TestCase):
  """LEARN_VERSION must be 2 (G1 changed semantics)."""

  def test_version(self):
    self.assertEqual(LEARN_VERSION, 2)


if __name__ == "__main__":
  unittest.main()
