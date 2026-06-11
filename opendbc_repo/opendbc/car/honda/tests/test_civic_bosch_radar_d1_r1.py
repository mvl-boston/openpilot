#!/usr/bin/env python3
"""D1 (tag-demux assembler) + R1 (per-slot range KF) tests for the Civic Bosch fine ingest.

Companion to test_civic_bosch_radar.py (same real-CANParser end-to-end style, same helpers).
D1: nrdrbranchdebug-86t.1 — vl_all demux so a metadata sub-frame sharing a header ID in one emit
    window can no longer mis-clear a live track (CANParser.vl is last-write-wins).
R1: nrdrbranchdebug-86t.4 — 2-state [range, range_rate] KF replaces d(dRel)/dt + EMA; quantitative
    smoothing/responsiveness contracts + the 6u8 re-seed EMA-halving regression.
"""
import math
import statistics
import unittest

from opendbc.car import structs
from opendbc.car.honda.radar_interface import (
  RadarInterface,
  BOSCH_RADAR_HDR_TAG,
  BOSCH_RADAR_BORN_CYCLES,
  BOSCH_RADAR_VREL_DT_MAX_S,
  BOSCH_RADAR_VREL_MAX,
)
from opendbc.car.honda.values import CAR

RANGE_SCALE = 0.00357  # m/LSB (DBC)
RANGE_OFFSET = -3.0
SWEEP_NS = int(0.06 * 1e9)  # ~real sweep cadence


def _frame(b0, tag, b2, b3, b4, b5, b6, b7):
  return bytes([b0, tag, b2, b3, b4, b5, b6, b7])


def _hdr_frame(range_raw, *, tag=BOSCH_RADAR_HDR_TAG, strength=0x00, lat_raw=0x8000, cntr=0x00):
  return _frame(strength, tag, (range_raw >> 8) & 0xFF, range_raw & 0xFF,
                (lat_raw >> 8) & 0xFF, lat_raw & 0xFF, 0x00, cntr)


def _meta_frame(*, tag=0x75, cntr=0x00):
  # a non-0x74 sub-frame mux'd onto the same header ID (payload bytes irrelevant to the range path)
  return _frame(0x10, tag, 0xAA, 0xBB, 0xCC, 0xDD, 0x00, cntr)


def _raw_for(d_m):
  return int(round((d_m - RANGE_OFFSET) / RANGE_SCALE))


def _can(nanos, frames):
  return [(nanos, frames)]


class BoschCase(unittest.TestCase):
  TRIG = 0x2DC

  def setUp(self):
    CP = structs.CarParams()
    CP.carFingerprint = CAR.HONDA_CIVIC_BOSCH
    CP.radarUnavailable = False
    self.ri = RadarInterface(CP, structs.CarParamsSP())
    self.bus = self.ri.rcp.bus
    self.assertTrue(self.ri.bosch_radar)

  def _f(self, addr, frame):
    return (addr, frame, self.bus)

  def _trig(self, cntr):
    return self._f(self.TRIG, _hdr_frame(0x8000, tag=0xF0, strength=0xFE, cntr=cntr))

  def _emit(self, nanos, frames, cntr):
    return self.ri.update(_can(nanos, list(frames) + [self._trig(cntr)]))

  def _close_sequence(self, d0_m, v_mps, n, *, start_ns=0, noise=None, addr=0x280):
    """Drive a lead closing at constant v; returns list of (t_s, true_d, rr)."""
    out = []
    d = d0_m
    for k in range(n):
      t_ns = start_ns + k * SWEEP_NS
      d_meas = d if noise is None else d + noise[k % len(noise)]
      cntr = (0x10 + k) & 0xFF
      rr = self._emit(t_ns, [self._f(addr, _hdr_frame(_raw_for(d_meas), cntr=cntr))], cntr)
      out.append((t_ns * 1e-9, d, rr))
      d += v_mps * (SWEEP_NS * 1e-9)
    return out


class TestD1TagDemux(BoschCase):
  def test_meta_after_range_no_longer_clears_live_track(self):
    # warm to born with clean sweeps
    self._close_sequence(40.0, 0.0, BOSCH_RADAR_BORN_CYCLES)
    base = BOSCH_RADAR_BORN_CYCLES * SWEEP_NS
    # same window: 0x74 range frame FIRST, meta frame AFTER (old vl read saw only the meta -> mis-clear)
    rr = self.ri.update(_can(base, [
      self._f(0x280, _hdr_frame(_raw_for(40.0), cntr=0x20)),
      self._f(0x280, _meta_frame(cntr=0x20)),
      self._trig(0x20),
    ]))
    self.assertEqual(len(rr.points), 1, "live track must survive a trailing metadata sub-frame")
    self.assertAlmostEqual(rr.points[0].dRel, 40.0, delta=0.01)
    self.assertEqual(self.ri._clobber_recovered, 1)

  def test_meta_only_window_still_decays(self):
    # warm to EXACTLY born (valid_cnt == BORN_CYCLES) so two genuine misses floor the counter
    self._close_sequence(40.0, 0.0, BOSCH_RADAR_BORN_CYCLES)
    base = BOSCH_RADAR_BORN_CYCLES * SWEEP_NS
    # two consecutive meta-only windows -> genuine miss x2 -> point dropped (S2 parity with old gate)
    for k in range(BOSCH_RADAR_BORN_CYCLES):
      rr = self.ri.update(_can(base + k * SWEEP_NS, [
        self._f(0x280, _meta_frame(cntr=0x30 + k)),
        self._trig(0x30 + k),
      ]))
    self.assertEqual(len(rr.points), 0, "meta-only windows are a genuine range miss -> decay to drop")

  def test_multi_batch_harvest_across_update_calls(self):
    # frames arrive in SEPARATE rcp.update batches within one trigger window (the on-device shape:
    # vl_all is wiped per batch; the harvest must have captured the early frame)
    self._close_sequence(40.0, 0.0, BOSCH_RADAR_BORN_CYCLES)
    base = BOSCH_RADAR_BORN_CYCLES * SWEEP_NS
    r = self.ri.update(_can(base, [self._f(0x280, _hdr_frame(_raw_for(39.5), cntr=0x40))]))
    self.assertIsNone(r)  # no trigger yet -> no emit
    rr = self.ri.update(_can(base + int(8e6), [self._f(0x280, _meta_frame(cntr=0x40)), self._trig(0x40)]))
    self.assertEqual(len(rr.points), 1, "range frame from the earlier batch must be recovered")
    self.assertAlmostEqual(rr.points[0].dRel, 39.5, delta=0.01)

  def test_pending_drained_each_window(self):
    self._close_sequence(40.0, 0.0, BOSCH_RADAR_BORN_CYCLES)
    self.assertEqual(sum(len(v) for v in self.ri._pending.values()), 0,
                     "assembler must be stateless across emit windows")


class TestR1RangeKF(BoschCase):
  def test_quantization_chatter_smoothed_vs_raw_derivative(self):
    # steady -5 m/s close with +/-1 LSB measurement chatter; compare published vRel noise to the
    # raw per-cycle derivative noise. The old EMA gave ~sqrt(alpha)=0.71x; the KF must do better.
    noise = [0.0, RANGE_SCALE, -RANGE_SCALE, 0.0, -RANGE_SCALE, RANGE_SCALE]
    seq = self._close_sequence(60.0, -5.0, 30, noise=noise)
    vrels = [rr.points[0].vRel for _, _, rr in seq if rr.points and not math.isnan(rr.points[0].vRel)]
    settled = vrels[10:]
    self.assertGreater(len(settled), 10)
    self.assertLess(statistics.pstdev(settled), 0.25, "KF must suppress LSB chatter to well under raw")
    self.assertAlmostEqual(statistics.fmean(settled), -5.0, delta=0.25)

  def test_hard_brake_onset_never_modeled_optimistic(self):
    # lead steady, then it brakes: closing rate ramps -0 -> -4 m/s over ~0.5 s. After the ramp the
    # published vRel must be within 1 m/s of truth and NEVER lag on the optimistic (less-closing) side
    # by more than 1.5 m/s during the ramp tail (R1 ship gate: responsiveness over smoothness).
    self._close_sequence(50.0, 0.0, 10)
    d = 50.0
    v = 0.0
    t0 = 10 * SWEEP_NS
    worst_optimism = 0.0
    last_vrel = None
    for k in range(20):
      v = max(-4.0, v - 0.5)  # ramp 0.5 m/s per sweep
      d += v * (SWEEP_NS * 1e-9)
      cntr = (0x60 + k) & 0xFF
      rr = self._emit(t0 + k * SWEEP_NS, [self._f(0x280, _hdr_frame(_raw_for(d), cntr=cntr))], cntr)
      if rr.points and not math.isnan(rr.points[0].vRel):
        last_vrel = rr.points[0].vRel
        if k >= 10:  # ramp complete; truth = -4.0
          worst_optimism = max(worst_optimism, rr.points[0].vRel - v)
    self.assertIsNotNone(last_vrel)
    self.assertAlmostEqual(last_vrel, -4.0, delta=1.0)
    self.assertLess(worst_optimism, 1.5, "post-ramp vRel must not read meaningfully less-closing than truth")

  def test_arel_packed_after_convergence(self):
    seq = self._close_sequence(50.0, 0.0, 2)
    self.assertTrue(all(math.isnan(p.aRel) for _, _, rr in seq for p in rr.points),
                    "aRel must stay NaN until the KF has a rate history")
    # decelerating-range scenario: closing accelerates -> aRel goes finite and negative-ish
    d = 50.0
    v = 0.0
    for k in range(12):
      v -= 0.3
      d += v * (SWEEP_NS * 1e-9)
      cntr = (0x80 + k) & 0xFF
      rr = self._emit((2 + k) * SWEEP_NS, [self._f(0x280, _hdr_frame(_raw_for(d), cntr=cntr))], cntr)
    p = rr.points[0]
    self.assertFalse(math.isnan(p.aRel), "converged KF must pack a smoothed range-accel into aRel")
    self.assertLess(p.aRel, 0.0)

  def test_6u8_regression_no_post_gap_vrel_halving(self):
    # The old EMA blended the FIRST post-reseed derivative with the stale pre-gap vRel (measured=True),
    # halving it for one cycle (bug nrdrbranchdebug-6u8). The KF reseeds cleanly: the first published
    # vRel after the gap must be full magnitude.
    self._close_sequence(60.0, -5.0, 8)
    t_gap = int(8 * SWEEP_NS + (BOSCH_RADAR_VREL_DT_MAX_S + 0.2) * 1e9)
    # re-appear closer, still closing at -5 m/s
    d = 50.0
    rr = self._emit(t_gap, [self._f(0x280, _hdr_frame(_raw_for(d), cntr=0xA0))], 0xA0)
    self.assertTrue(all(math.isnan(p.vRel) for p in rr.points))  # reseed cycle: estimate only
    vrels = []
    for k in range(1, 5):
      d += -5.0 * (SWEEP_NS * 1e-9)
      cntr = (0xA0 + k) & 0xFF
      rr = self._emit(t_gap + k * SWEEP_NS, [self._f(0x280, _hdr_frame(_raw_for(d), cntr=cntr))], cntr)
      if rr.points and not math.isnan(rr.points[0].vRel):
        vrels.append(rr.points[0].vRel)
    self.assertTrue(vrels)
    self.assertLess(vrels[0], -4.0, f"first post-gap vRel must be full magnitude, not halved: {vrels}")
    self.assertTrue(all(abs(v) <= BOSCH_RADAR_VREL_MAX for v in vrels))

  def test_panic_brake_optimism_bounded(self):
    # The R1 ship gate, physics-grounded: a lead panic-braking at -6 m/s^2 (closing ramps 0 -> -8 m/s).
    # Published vRel must never read optimistic (less-closing than truth) by >= 1.0 m/s and must land
    # within 0.3 m/s of truth once the ramp completes. Sim-tuned constants (Q_V=4.0) give 0.88 worst.
    self._close_sequence(50.0, 0.0, 10)
    d, v = 50.0, 0.0
    dts = SWEEP_NS * 1e-9
    worst_optimism = 0.0
    for k in range(30):
      v = max(-8.0, v - 6.0 * dts)
      d += v * dts
      cntr = (0xC0 + k) & 0xFF
      rr = self._emit((10 + k) * SWEEP_NS, [self._f(0x280, _hdr_frame(_raw_for(d), cntr=cntr))], cntr)
      if rr.points and not math.isnan(rr.points[0].vRel):
        worst_optimism = max(worst_optimism, rr.points[0].vRel - v)
        last_err = rr.points[0].vRel - v
    self.assertLess(worst_optimism, 1.0, f"panic-brake optimism {worst_optimism:.2f} breaches the gate")
    self.assertLess(abs(last_err), 0.3)


if __name__ == "__main__":
  unittest.main()
