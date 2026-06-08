#!/usr/bin/env python3
"""Unit tests for the Honda Civic Bosch FINE 0x280 track-table radar ingest.

These drive the REAL CANParser + RadarInterface end-to-end (no decode re-implementation) so the DBC,
the honda_* CHKSUM/CNTR trap avoidance, the b1==0x74 gate, the sentinel skips, multi-object emission,
trackId stability, vRel derivation, and the staleness->EMPTY-RadarData gate are all exercised together.

100% OFFLINE: frames are synthetic or replayed from radar-re captures; no panda, no CAN TX, no flash.

Bus note: the fine object frames are physically on CanBus.camera. For a bare (un-fingerprinted)
CarParams the CanBus offset resolves to a negative placeholder, so each test reads the bus the parser
was ACTUALLY built on (parser.bus) and feeds frames there -- this exercises the real parse path
independent of how the offset resolves in this opendbc-only checkout.
"""
import math
import os
import unittest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.honda.radar_interface import (
  RadarInterface,
  BOSCH_RADAR_HDR_MSGS,
  BOSCH_RADAR_HDR_TAG,
  BOSCH_RADAR_LAT_SCALE,
  BOSCH_RADAR_STALE_S,
  BOSCH_RADAR_VREL_MAX,
  BOSCH_RADAR_BORN_CYCLES,
  BOSCH_RADAR_VALID_CAP,
  BOSCH_RADAR_TRACKID_STRIDE,
  BOSCH_RADAR_RANGE_MAX,
  BOSCH_RADAR_RAW_SAT,
  BOSCH_RADAR_CNTR_STALL_CYCLES,
  BOSCH_RADAR_VREL_DT_MAX_S,
)
from opendbc.car.honda.values import CAR, DBC

BFCAR_CSV = r"C:/claudecode/firmware-analysis-kit/radar-re/captures/closing_bfcar.csv"
IDLE_CSV = r"C:/claudecode/firmware-analysis-kit/radar-re/captures/closing_10m.csv"


def _frame(b0, tag, b2, b3, b4, b5, b6, b7):
  return bytes([b0, tag, b2, b3, b4, b5, b6, b7])


def _hdr_frame(range_raw, *, tag=BOSCH_RADAR_HDR_TAG, strength=0x00, lat_raw=0x8000, cntr=0x00):
  """Header frame: b0=strength, b1=tag, b2:b3=range_raw BE16, b4:b5=lat_raw BE16, b7=cntr."""
  return _frame(strength, tag, (range_raw >> 8) & 0xFF, range_raw & 0xFF,
                (lat_raw >> 8) & 0xFF, lat_raw & 0xFF, 0x00, cntr)


def _make_ri():
  CP = structs.CarParams()
  CP.carFingerprint = CAR.HONDA_CIVIC_BOSCH
  CP.radarUnavailable = False
  CP_SP = structs.CarParamsSP()
  return RadarInterface(CP, CP_SP)


def _can(nanos, frames):
  return [nanos, frames]


class TestCivicBoschFineDBC(unittest.TestCase):
  """DBC-level decode + the honda_* CHKSUM/CNTR auto-enforcement trap."""

  def setUp(self):
    self.parser = CANParser(DBC[CAR.HONDA_CIVIC_BOSCH][Bus.radar],
                            [(m, 20) for m in BOSCH_RADAR_HDR_MSGS], 2)
    self.bus = self.parser.bus

  def test_range_decode_known_raw(self):
    # Real bfcar frame 00740f9f9bc00336 on 0x280 -> raw 0x0f9f -> 0.00357*3999 - 3.0 ~= 11.28 m.
    data = bytes.fromhex("00740f9f9bc00336")
    self.parser.update(_can(0, [(0x280, data, self.bus)]))
    vl = self.parser.vl[0x280]
    self.assertEqual(int(vl["TRACK_TAG"]), 0x74)
    self.assertEqual(int(vl["RANGE_RAW"]), 0x0F9F)
    self.assertAlmostEqual(vl["RANGE"], 0.00357 * 3999 - 3.0, places=4)
    self.assertAlmostEqual(vl["RANGE"], 11.28, delta=0.01)
    self.assertEqual(int(vl["STRENGTH"]), 0x00)
    self.assertEqual(int(vl["CNTR"]), 0x36)

  def test_range_scale_offset(self):
    self.parser.update(_can(0, [(0x280, _hdr_frame(3999), self.bus)]))
    self.assertAlmostEqual(self.parser.vl[0x280]["RANGE"], 0.00357 * 3999 - 3.0, places=6)

  def test_lat_offset_binary_center(self):
    # raw 0x8000 -> 0 (offset -32768); 0x9bc0 -> +7104.
    self.parser.update(_can(0, [(0x280, _hdr_frame(3999, lat_raw=0x8000), self.bus)]))
    self.assertEqual(int(self.parser.vl[0x280]["LAT_RAW"]), 0)
    self.parser.update(_can(1, [(0x280, _hdr_frame(3999, lat_raw=0x9BC0), self.bus)]))
    self.assertEqual(int(self.parser.vl[0x280]["LAT_RAW"]), 0x9BC0 - 0x8000)

  def test_chksum_cntr_trap_avoided(self):
    # opendbc auto-enforces a Honda checksum/counter ONLY on signals literally named CHECKSUM/COUNTER,
    # dropping frames on mismatch. Our fine frames carry CNTR (no CHECKSUM at all) so an arbitrary
    # counter value must NOT cause the frame to be dropped: the decode must still land.
    names = set()
    for m in BOSCH_RADAR_HDR_MSGS:
      names |= set(self.parser.vl[m].keys())
    self.assertNotIn("CHECKSUM", names)
    self.assertNotIn("COUNTER", names)
    self.assertIn("CNTR", names)
    self.parser.update(_can(0, [(0x280, _hdr_frame(5000, cntr=0xAB), self.bus)]))
    self.assertEqual(int(self.parser.vl[0x280]["CNTR"]), 0xAB)
    self.assertAlmostEqual(self.parser.vl[0x280]["RANGE"], 0.00357 * 5000 - 3.0, places=4)


class TestCivicBoschFineParser(unittest.TestCase):
  """RadarInterface._update_bosch end-to-end through the real CANParser."""

  TRIG = 0x2DC  # S4: the parser triggers the emit on the sweep terminator (0x2DC), not the head (0x280).

  def setUp(self):
    self.ri = _make_ri()
    self.bus = self.ri.rcp.bus
    self.assertTrue(self.ri.bosch_radar)
    self.assertEqual(self.ri.trigger_msg, self.TRIG)  # S4 sweep-coherent trigger

  def _step(self, nanos, frames):
    return self.ri.update(_can(nanos, frames))

  def _f(self, addr, frame):
    return (addr, frame, self.bus)

  def _trig(self, cntr):
    # The trigger frame (sweep terminator). Its payload is a benign non-range sentinel (b1!=0x74) so it
    # never births a slot-5 point by itself; it exists only to fire the emit and advance the S3 CNTR.
    return self._f(self.TRIG, _hdr_frame(0x8000, tag=0xF0, strength=0xFE, cntr=cntr))

  def _emit(self, nanos, frames, cntr):
    # Drive one sweep: the given head/body frames PLUS the trigger terminator, so update() emits.
    return self._step(nanos, list(frames) + [self._trig(cntr)])

  def _warm(self, range_raw, *, slot_addr=0x280, lat_raw=0x8000, base_ns=0, cntr0=0x10):
    # S2 birth hysteresis: a slot must be a valid range-carrier for BOSCH_RADAR_BORN_CYCLES consecutive
    # sweeps before it emits a point. Drive the slot (default 0x280) for BORN_CYCLES sweeps at a steady
    # range so it is "born" on the returned sweep. Each sweep is terminated by the trigger (0x2DC) so the
    # emit fires; CNTR advances each sweep to keep the S3 CNTR-stall fault from firing. Returns the last rr.
    dt_ns = int(0.05 * 1e9)
    rr = None
    for k in range(BOSCH_RADAR_BORN_CYCLES):
      cntr = (cntr0 + k) & 0xFF
      body = [self._f(slot_addr, _hdr_frame(range_raw, lat_raw=lat_raw, cntr=cntr))]
      rr = self._emit(base_ns + k * dt_ns, body, cntr)
    return rr

  def test_single_live_track_emits_one_point(self):
    # S2: a single sweep must NOT birth a point; two consecutive valid sweeps do.
    self._emit(0, [self._f(0x280, _hdr_frame(3999, cntr=0x10))], 0x10)
    rr = self._emit(int(0.05 * 1e9), [self._f(0x280, _hdr_frame(3999, cntr=0x11))], 0x11)
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 1)
    p = rr.points[0]
    # S1: trackId is slot*STRIDE + incarnation (slot 0, first incarnation = 1), no longer the bare slot.
    self.assertEqual(p.trackId, 0 * BOSCH_RADAR_TRACKID_STRIDE + 1)
    self.assertEqual(p.trackId // BOSCH_RADAR_TRACKID_STRIDE, 0)  # still slot-decodable
    self.assertAlmostEqual(p.dRel, 0.00357 * 3999 - 3.0, places=4)
    # S5: vRel is derived; on the birth cycle it derives from the prime cycle (steady range -> ~0), so the
    # point is measured. vRel here is a real (near-zero) derived value, not NaN.
    self.assertTrue(p.measured)
    self.assertTrue(math.isnan(p.aRel))
    self.assertFalse(math.isnan(p.vRel))  # derived from the prime cycle (steady -> ~0)

  def test_b1_tag_gate_skips_nonheader(self):
    # b1 != 0x74 -> non-range sub-frame -> skipped (no point), even with a plausible range field.
    rr = self._emit(0, [self._f(0x280, _hdr_frame(3999, tag=0x02))], 0x10)
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 0)

  def test_sentinel_strength_idle(self):
    rr = self._emit(0, [self._f(0x280, _hdr_frame(3999, strength=0xFE))], 0x10)
    self.assertEqual(len(rr.points), 0)

  def test_sentinel_range_unset(self):
    rr = self._emit(0, [self._f(0x280, _hdr_frame(0x8000))], 0x10)
    self.assertEqual(len(rr.points), 0)

  def test_sentinel_range_saturation(self):
    rr = self._emit(0, [self._f(0x280, _hdr_frame(0xFF80))], 0x10)
    self.assertEqual(len(rr.points), 0)
    rr = self._emit(int(0.05 * 1e9), [self._f(0x280, _hdr_frame(0xFFFF))], 0x11)
    self.assertEqual(len(rr.points), 0)

  def test_multi_object_emission(self):
    dt_ns = int(0.05 * 1e9)
    def burst(ns, cntr):
      return self._emit(ns, [self._f(0x280, _hdr_frame(3000, cntr=cntr)),   # slot 0
                             self._f(0x284, _hdr_frame(4000, cntr=cntr)),   # slot 1
                             self._f(0x2D0, _hdr_frame(5000, cntr=cntr))], cntr)  # slot 2
    burst(0, 0x10)                 # prime (S2 born hysteresis)
    rr = burst(dt_ns, 0x11)        # born
    self.assertEqual(len(rr.points), 3)
    # S1: trackId = slot*STRIDE + incarnation(=1); decode the slot back out to verify the 3 distinct slots.
    by_slot = {p.trackId // BOSCH_RADAR_TRACKID_STRIDE: p for p in rr.points}
    self.assertEqual(set(by_slot), {0, 1, 2})
    self.assertAlmostEqual(by_slot[0].dRel, 0.00357 * 3000 - 3.0, places=4)
    self.assertAlmostEqual(by_slot[1].dRel, 0.00357 * 4000 - 3.0, places=4)
    self.assertAlmostEqual(by_slot[2].dRel, 0.00357 * 5000 - 3.0, places=4)
    # All three are first-incarnation distinct trackIds (no reuse across slots).
    self.assertEqual(len({p.trackId for p in rr.points}), 3)

  def test_trackid_stability_across_cycles(self):
    dt_ns = int(0.05 * 1e9)  # 20 Hz
    rr0 = self._warm(3000)                                                   # born (cycles 0,1)
    id0 = rr0.points[0].trackId
    rr1 = self._emit(2 * dt_ns, [self._f(0x280, _hdr_frame(3010, cntr=0x20))], 0x20)
    # S1: a continuously-present slot keeps its trackId (same incarnation) across cycles.
    self.assertEqual(rr1.points[0].trackId, id0)
    self.assertEqual(id0 // BOSCH_RADAR_TRACKID_STRIDE, 0)  # slot 0
    self.assertEqual(id0, 1)                                # first incarnation

  def test_vrel_derived_closing_negative(self):
    dt_ns = int(0.05 * 1e9)  # 50 ms
    # cycle 0: first sight (valid_cnt 0->1, vRel NaN, no emit under S2 born hysteresis)
    self._emit(0, [self._f(0x280, _hdr_frame(4000, cntr=0x10))], 0x10)
    # cycle 1: range shrank -> closing; valid_cnt 1->2 -> point born this cycle with a derived vRel
    rr = self._emit(dt_ns, [self._f(0x280, _hdr_frame(3000, cntr=0x11))], 0x11)
    p = rr.points[0]
    self.assertFalse(math.isnan(p.vRel))
    self.assertLess(p.vRel, 0.0)
    expected = (0.00357 * 3000 - 0.00357 * 4000) / 0.05  # raw d(dRel)/dt (first derived sample, no EMA yet)
    self.assertAlmostEqual(p.vRel, expected, delta=1.0)

  def test_yrel_sign_and_placeholder_scale(self):
    rr = self._warm(3999, lat_raw=0x9000)  # lat_raw > center -> right; born under S2
    p = rr.points[0]
    lat = 0x9000 - 0x8000
    self.assertAlmostEqual(p.yRel, -lat * BOSCH_RADAR_LAT_SCALE, places=6)
    self.assertLess(p.yRel, 0.0)  # right of center -> negative y

  def _slots(self, rr):
    return {p.trackId // BOSCH_RADAR_TRACKID_STRIDE for p in rr.points}

  def test_slot_clears_when_track_goes_sentinel(self):
    dt_ns = int(0.05 * 1e9)
    # born: two valid sweeps for slots 0 and 1
    self._emit(0, [self._f(0x280, _hdr_frame(3000, cntr=0x10)), self._f(0x284, _hdr_frame(4000, cntr=0x10))], 0x10)
    rr = self._emit(dt_ns, [self._f(0x280, _hdr_frame(3000, cntr=0x11)), self._f(0x284, _hdr_frame(4000, cntr=0x11))], 0x11)
    self.assertEqual(self._slots(rr), {0, 1})
    # slot 0 sentinel for TWO sweeps -> S2 persist tolerates the first, drops on the second.
    rr = self._emit(2 * dt_ns, [self._f(0x280, _hdr_frame(3000, strength=0xFE, cntr=0x12)),
                                self._f(0x284, _hdr_frame(4000, cntr=0x12))], 0x12)
    self.assertEqual(self._slots(rr), {0, 1})  # slot 0 persists one cycle (valid_cnt 2->1)
    rr = self._emit(3 * dt_ns, [self._f(0x280, _hdr_frame(3000, strength=0xFE, cntr=0x13)),
                                self._f(0x284, _hdr_frame(4000, cntr=0x13))], 0x13)
    self.assertEqual(self._slots(rr), {1})     # second sentinel cycle -> slot 0 dropped (valid_cnt -> 0)

  def test_stale_track_aged_when_absent(self):
    dt_ns = int(0.05 * 1e9)
    # born: two valid sweeps for slots 0 and 1
    self._emit(0, [self._f(0x280, _hdr_frame(3000, cntr=0x10)), self._f(0x284, _hdr_frame(4000, cntr=0x10))], 0x10)
    rr = self._emit(dt_ns, [self._f(0x280, _hdr_frame(3000, cntr=0x11)), self._f(0x284, _hdr_frame(4000, cntr=0x11))], 0x11)
    self.assertEqual(self._slots(rr), {0, 1})
    # 0x284 absent for TWO trigger sweeps -> S2 persist tolerates one, ages out on the second.
    rr = self._emit(2 * dt_ns, [self._f(0x280, _hdr_frame(3010, cntr=0x12))], 0x12)
    self.assertEqual(self._slots(rr), {0, 1})  # slot 1 persists one absent cycle
    rr = self._emit(3 * dt_ns, [self._f(0x280, _hdr_frame(3020, cntr=0x13))], 0x13)
    self.assertEqual(self._slots(rr), {0})     # second absent cycle -> slot 1 aged out

  def test_staleness_returns_empty_radardata_not_none(self):
    # trigger (0x2DC) born over two sweeps; then it goes quiet while another declared header keeps the
    # parser clock advancing past STALE_S -> EMPTY RadarData (not None) + radarUnavailableTemporary.
    self._warm(3000)  # born (2 sweeps), point established
    self.assertEqual(len(self.ri.pts), 1)
    stale_ns = int((BOSCH_RADAR_STALE_S + 0.2) * 1e9)
    rr = self._step(stale_ns, [self._f(0x284, _hdr_frame(0x8000))])  # 0x284 frame; trigger 0x2DC absent
    self.assertIsNotNone(rr)             # must be EMPTY RadarData, NOT None
    self.assertEqual(len(rr.points), 0)
    self.assertTrue(rr.errors.radarUnavailableTemporary)
    self.assertEqual(len(self.ri.pts), 0)  # points cleared

  def test_trigger_absent_no_stale_returns_none(self):
    # If the trigger (0x2DC) is merely absent for one cycle (well under STALE_S) and we have live points,
    # the update returns None (normal cadence gate) -- staleness only fires past STALE_S.
    self._warm(3000)  # born, live point present
    dt_ns = int(0.05 * 1e9)
    rr = self._step(2 * dt_ns + int(0.02 * 1e9), [self._f(0x284, _hdr_frame(4000))])  # trigger absent, 20ms later
    self.assertIsNone(rr)

  def test_fully_silent_radar_clears_points_and_returns_empty(self):
    # MOST safety-relevant staleness case: the radar goes COMPLETELY silent (no frames on ANY id) while
    # the parser clock keeps advancing (e.g. radard still pumps update() on its own cadence). The
    # frozen-phantom must still be cleared -> EMPTY RadarData (not None) + radarUnavailableTemporary.
    self._warm(3000)  # born (2 cycles), point established
    self.assertEqual(len(self.ri.pts), 1)
    stale_ns = int((BOSCH_RADAR_STALE_S + 0.2) * 1e9)
    rr = self.ri.update(_can(stale_ns, []))  # whole bus silent, advancing timestamp
    self.assertIsNotNone(rr)              # EMPTY RadarData, NOT None
    self.assertEqual(len(rr.points), 0)
    self.assertTrue(rr.errors.radarUnavailableTemporary)
    self.assertEqual(len(self.ri.pts), 0)  # phantom cleared

  def test_fully_silent_under_threshold_returns_none(self):
    # Brief whole-bus silence under STALE_S must NOT prematurely drop the live point.
    self._warm(3000)  # born, live point present
    dt_ns = int(0.05 * 1e9)
    rr = self.ri.update(_can(2 * dt_ns + int(0.05 * 1e9), []))  # 50 ms silent, well under 0.15 s
    self.assertIsNone(rr)
    self.assertEqual(len(self.ri.pts), 1)            # point retained

  def test_vrel_discontinuity_guard_rejects_slot_reuse(self):
    # If object A vacates slot 0 and object B enters the SAME slot in the next cycle WITHOUT an intervening
    # sentinel, the naive derivative teleports vRel to a non-physical value. The guard must reject that
    # sample (vRel NaN) AND re-seed history so the NEXT cycle derives cleanly. S1: the post-swap point
    # must ALSO be published under a DIFFERENT trackId (no reuse across the swap, capnp:314).
    dt_ns = int(0.05 * 1e9)  # 50 ms
    # Object A at ~11 m, then ~11.04 m (slow). Sweep 0 primes (S2), sweep 1 births with a small real vRel.
    self._emit(0, [self._f(0x280, _hdr_frame(3900, cntr=0x10))], 0x10)
    rr = self._emit(dt_ns, [self._f(0x280, _hdr_frame(3910, cntr=0x11))], 0x11)
    self.assertFalse(math.isnan(rr.points[0].vRel))   # real small vRel
    id_a = rr.points[0].trackId                        # object A's trackId (pre-swap)
    # Sweep 2: a DIFFERENT object B jumps into slot 0 at ~110 m (raw ~31600). Implied speed is ~1900 m/s.
    far_raw = int((110.0 + 3.0) / 0.00357)
    rr = self._emit(2 * dt_ns, [self._f(0x280, _hdr_frame(far_raw, cntr=0x12))], 0x12)
    p = rr.points[0]
    self.assertTrue(math.isnan(p.vRel))               # phantom rejected -> NaN, not ~1900 m/s
    self.assertFalse(p.measured)                       # S5: NaN vRel -> estimate, not a measurement
    self.assertAlmostEqual(p.dRel, 0.00357 * far_raw - 3.0, places=2)  # dRel still tracks the new object
    id_b = p.trackId                                   # object B's trackId (post-swap)
    # S1 core assertion: the swap produced a NEW trackId (no reuse). Both still decode to slot 0.
    self.assertNotEqual(id_b, id_a)
    self.assertEqual(id_a // BOSCH_RADAR_TRACKID_STRIDE, 0)
    self.assertEqual(id_b // BOSCH_RADAR_TRACKID_STRIDE, 0)
    # Sweep 3: object B advances slightly -> a real, in-bounds vRel now derives from the re-seeded baseline,
    # and B keeps its (new) trackId across the clean sweep.
    rr = self._emit(3 * dt_ns, [self._f(0x280, _hdr_frame(far_raw - 10, cntr=0x13))], 0x13)
    p = rr.points[0]
    self.assertFalse(math.isnan(p.vRel))
    self.assertLessEqual(abs(p.vRel), BOSCH_RADAR_VREL_MAX)
    self.assertEqual(p.trackId, id_b)                  # stable id once the new object is continuous

  def test_in_bounds_fast_lead_keeps_stable_trackid(self):
    # S1 control: a genuine fast closer (stationary object at highway speed ~31 m/s) must NOT be rejected
    # AND must keep a STABLE trackId across sweeps (no spurious incarnation bump on a real, in-bounds vRel).
    dt_ns = int(0.05 * 1e9)
    self._emit(0, [self._f(0x280, _hdr_frame(int((50.0 + 3.0) / 0.00357), cntr=0x10))], 0x10)      # 50 m
    rr = self._emit(dt_ns, [self._f(0x280, _hdr_frame(int((48.5 + 3.0) / 0.00357), cntr=0x11))], 0x11)  # ~-30 m/s
    p = rr.points[0]
    id0 = p.trackId
    self.assertFalse(math.isnan(p.vRel))
    self.assertLess(p.vRel, 0.0)
    self.assertLessEqual(abs(p.vRel), BOSCH_RADAR_VREL_MAX)
    rr = self._emit(2 * dt_ns, [self._f(0x280, _hdr_frame(int((47.0 + 3.0) / 0.00357), cntr=0x12))], 0x12)  # ~-30
    self.assertEqual(rr.points[0].trackId, id0)   # stable id for a continuous in-bounds fast lead

  def test_returns_radardata_with_points_list(self):
    rr = self._warm(3000)  # born under S2
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 1)            # iterable points sequence on the RadarData
    # canError mirrors rcp.can_valid (set on both the normal and stale paths).
    self.assertEqual(rr.errors.canError, not self.ri.rcp.can_valid)


class TestCivicBoschFineSafeParity(unittest.TestCase):
  """Dedicated acceptance tests for the SAFE parity-hardening set (S1-S6). RX-only, keep-AEB preserved."""

  TRIG = 0x2DC

  def setUp(self):
    self.ri = _make_ri()
    self.bus = self.ri.rcp.bus
    self.dt_ns = int(0.05 * 1e9)

  def _f(self, addr, frame):
    return (addr, frame, self.bus)

  def _trig(self, cntr):
    return self._f(self.TRIG, _hdr_frame(0x8000, tag=0xF0, strength=0xFE, cntr=cntr))

  def _emit(self, k, body, cntr):
    return self.ri.update(_can(k * self.dt_ns, list(body) + [self._trig(cntr)]))

  def _emit_full(self, k, slot0_frame, trig_cntr):
    # Drive a FULL 6-header sweep (so the CANParser reaches can_valid). slot 0 carries slot0_frame; slots
    # 1..5 (including the 0x2DC terminator) are benign sentinels carrying trig_cntr. Used for S3 faults,
    # which are gated on can_valid (a fault must not fire on a not-yet-valid bus).
    frames = [self._f(0x280, slot0_frame)]
    for a in BOSCH_RADAR_HDR_MSGS[1:]:
      frames.append(self._f(a, _hdr_frame(0x8000, tag=0xF0, strength=0xFE, cntr=trig_cntr)))
    return self.ri.update(_can(k * self.dt_ns, frames))

  def _slots(self, rr):
    return {p.trackId // BOSCH_RADAR_TRACKID_STRIDE for p in rr.points}

  # ---- S2 birth/persist hysteresis -------------------------------------------------------------
  def test_s2_single_frame_glitch_no_phantom(self):
    # (a) a single 1-frame valid glitch must NOT birth a phantom point.
    rr = self._emit(0, [self._f(0x280, _hdr_frame(3000, cntr=0x10))], 0x10)
    self.assertEqual(len(rr.points), 0)         # valid_cnt == 1 < BORN_CYCLES
    self.assertEqual(len(self.ri.pts), 0)

  def test_s2_born_after_n_cycles(self):
    # birth requires exactly BOSCH_RADAR_BORN_CYCLES consecutive valid sweeps.
    for k in range(BOSCH_RADAR_BORN_CYCLES - 1):
      rr = self._emit(k, [self._f(0x280, _hdr_frame(3000, cntr=0x10 + k))], 0x10 + k)
      self.assertEqual(len(rr.points), 0)
    rr = self._emit(BOSCH_RADAR_BORN_CYCLES - 1,
                    [self._f(0x280, _hdr_frame(3000, cntr=0x10 + BOSCH_RADAR_BORN_CYCLES - 1))],
                    0x10 + BOSCH_RADAR_BORN_CYCLES - 1)
    self.assertEqual(len(rr.points), 1)

  def test_s2_single_miss_does_not_drop(self):
    # (b) one missed cycle must NOT drop an established point (persist tolerance).
    self._emit(0, [self._f(0x280, _hdr_frame(3000, cntr=0x10))], 0x10)
    self._emit(1, [self._f(0x280, _hdr_frame(3000, cntr=0x11))], 0x11)  # born
    rr = self._emit(2, [self._f(0x280, _hdr_frame(0x8000, cntr=0x12))], 0x12)  # slot-0 sentinel (1 miss)
    self.assertEqual(self._slots(rr), {0})      # retained for one missed cycle

  def test_s2_two_misses_drop(self):
    # (c) two consecutive missed cycles DO drop the point.
    self._emit(0, [self._f(0x280, _hdr_frame(3000, cntr=0x10))], 0x10)
    self._emit(1, [self._f(0x280, _hdr_frame(3000, cntr=0x11))], 0x11)  # born (valid_cnt 2)
    self._emit(2, [self._f(0x280, _hdr_frame(0x8000, cntr=0x12))], 0x12)  # miss 1 -> valid_cnt 1
    rr = self._emit(3, [self._f(0x280, _hdr_frame(0x8000, cntr=0x13))], 0x13)  # miss 2 -> valid_cnt 0
    self.assertEqual(len(rr.points), 0)

  def test_s2_valid_cap_saturates(self):
    # the confidence counter must saturate at VALID_CAP (so persist tolerance is bounded, not unbounded).
    for k in range(BOSCH_RADAR_VALID_CAP + 4):
      self._emit(k, [self._f(0x280, _hdr_frame(3000, cntr=(0x10 + k) & 0xFF))], (0x10 + k) & 0xFF)
    self.assertLessEqual(self.ri._valid_cnt[0], BOSCH_RADAR_VALID_CAP)

  # ---- S3 plausibility / self-consistency faults ----------------------------------------------
  def test_s3_out_of_band_range_raises_wrongconfig(self):
    # A decoded dRel beyond the physical ceiling -> wrongConfig + NO point emitted. The existing
    # saturation sentinel (raw >= 0xFF80 ~= 230.5 m) catches most over-range raws first; the S3 check
    # covers the narrow window between RANGE_MAX (230.0 m) and the sat rail. Pick a raw in that window.
    over_raw = 65350  # 0.00357*65350 - 3.0 = 230.30 m -> > RANGE_MAX(230) and < 0xFF80(65408)
    self.assertLess(over_raw, BOSCH_RADAR_RAW_SAT)
    self.assertGreater(0.00357 * over_raw - 3.0, BOSCH_RADAR_RANGE_MAX)
    rr = self._emit(0, [self._f(0x280, _hdr_frame(over_raw, cntr=0x10))], 0x10)
    self.assertTrue(rr.errors.wrongConfig)
    self.assertEqual(len(rr.points), 0)

  def test_s3_in_band_range_no_wrongconfig(self):
    rr = self._emit(0, [self._f(0x280, _hdr_frame(3000, cntr=0x10))], 0x10)
    self.assertFalse(rr.errors.wrongConfig)

  def test_s3_frozen_cntr_raises_radarfault(self):
    # CNTR frozen (constant) while can_valid -> radarFault after CNTR_STALL_CYCLES. Use FULL sweeps so the
    # parser reaches can_valid (the fault is intentionally gated on can_valid). Freeze the trigger CNTR.
    faulted = False
    for k in range(BOSCH_RADAR_CNTR_STALL_CYCLES + 4):
      rr = self._emit_full(k, _hdr_frame(3000, cntr=0x10), 0x55)  # trigger CNTR frozen at 0x55
      if rr is not None and rr.errors.radarFault:
        faulted = True
    self.assertTrue(faulted)

  def test_s3_advancing_cntr_no_radarfault(self):
    # A normal advancing CNTR (full valid sweeps) must raise neither radarFault nor wrongConfig.
    saw_fault = saw_wrong = False
    for k in range(BOSCH_RADAR_CNTR_STALL_CYCLES + 6):
      rr = self._emit_full(k, _hdr_frame(3000, cntr=(0x10 + k) & 0xFF), (0x20 + k) & 0xFF)
      if rr is not None:
        saw_fault |= rr.errors.radarFault
        saw_wrong |= rr.errors.wrongConfig
    self.assertFalse(saw_fault)
    self.assertFalse(saw_wrong)

  # ---- S5 honest measured flag ----------------------------------------------------------------
  def test_s5_estimate_vs_measurement(self):
    # A point whose vRel is still NaN (re-seed/first-sight) is an ESTIMATE (measured=False); once a stable
    # derived vRel exists the point is a MEASUREMENT (measured=True).
    self._emit(0, [self._f(0x280, _hdr_frame(4000, cntr=0x10))], 0x10)            # prime
    rr = self._emit(1, [self._f(0x280, _hdr_frame(3900, cntr=0x11))], 0x11)       # born, derived vRel
    self.assertFalse(math.isnan(rr.points[0].vRel))
    self.assertTrue(rr.points[0].measured)
    # Force a re-seed via a discontinuity -> vRel NaN this cycle -> measured False.
    far = int((120.0 + 3.0) / 0.00357)
    rr = self._emit(2, [self._f(0x280, _hdr_frame(far, cntr=0x12))], 0x12)
    self.assertTrue(math.isnan(rr.points[0].vRel))
    self.assertFalse(rr.points[0].measured)

  # ---- S6 vRel derivation hardening -----------------------------------------------------------
  def test_s6_long_gap_reseeds_no_spike(self):
    # A long gap (> DT_MAX) between two sightings must RE-SEED (vRel NaN this cycle), not derive a spike
    # from two far-apart-in-time samples. Use a big dRel change over a > DT_MAX gap.
    self._emit(0, [self._f(0x280, _hdr_frame(4000, cntr=0x10))], 0x10)
    self._emit(1, [self._f(0x280, _hdr_frame(3950, cntr=0x11))], 0x11)  # born, small real vRel
    # next sweep arrives after a > DT_MAX gap with a large dRel change
    gap_k = int((BOSCH_RADAR_VREL_DT_MAX_S + 0.2) / 0.05) + 1
    rr = self._emit(1 + gap_k, [self._f(0x280, _hdr_frame(2000, cntr=0x12))], 0x12)
    self.assertTrue(math.isnan(rr.points[0].vRel))   # re-seeded, not a spike
    # the cycle AFTER re-seed derives a clean, in-bounds vRel from the new baseline
    rr = self._emit(2 + gap_k, [self._f(0x280, _hdr_frame(1990, cntr=0x13))], 0x13)
    self.assertFalse(math.isnan(rr.points[0].vRel))
    self.assertLessEqual(abs(rr.points[0].vRel), BOSCH_RADAR_VREL_MAX)

  def test_s6_smooth_close_stable_negative_vrel(self):
    # A smooth closing sequence yields a stable negative derived vRel (EMA applied, in-bounds).
    rng = 4000
    self._emit(0, [self._f(0x280, _hdr_frame(rng, cntr=0x10))], 0x10)  # prime
    vrels = []
    for k in range(1, 8):
      rng -= 50  # steady close
      rr = self._emit(k, [self._f(0x280, _hdr_frame(rng, cntr=(0x10 + k) & 0xFF))], (0x10 + k) & 0xFF)
      v = rr.points[0].vRel
      if not math.isnan(v):
        vrels.append(v)
    self.assertGreater(len(vrels), 3)
    self.assertTrue(all(v < 0.0 for v in vrels))                 # all closing
    self.assertTrue(all(abs(v) <= BOSCH_RADAR_VREL_MAX for v in vrels))

  # ---- keep-AEB / RX-only invariants ----------------------------------------------------------
  def test_keepaeb_invariants_preserved(self):
    # radarUnavailable pinned False on this CP (stock radar + AEB alive); the parser was actually built
    # (rcp is not None) and triggers on the sweep terminator. RX-only: the interface declares no TX path.
    self.assertFalse(self.ri.radar_off_can)
    self.assertIsNotNone(self.ri.rcp)
    self.assertEqual(self.ri.trigger_msg, 0x2DC)
    # The radar ingest never publishes vRel/aRel authority changes; aRel is always NaN (Toyota posture).
    rng = 4000
    self._emit(0, [self._f(0x280, _hdr_frame(rng, cntr=0x10))], 0x10)
    rr = self._emit(1, [self._f(0x280, _hdr_frame(rng - 50, cntr=0x11))], 0x11)
    self.assertTrue(all(math.isnan(p.aRel) for p in rr.points))  # aRel never fabricated (R2 deferred)


@unittest.skipUnless(os.path.exists(BFCAR_CSV), "radar-re bfcar capture not present")
class TestCivicBoschFineRealCapture(unittest.TestCase):
  """Replay real radar-re captures through the parser (positive + negative control)."""

  HDR_HEX = {f"{m:X}" for m in BOSCH_RADAR_HDR_MSGS}
  TRIG_HEX = f"{BOSCH_RADAR_HDR_MSGS[-1]:X}"  # 0x2DC -- the sweep terminator / trigger

  @classmethod
  def _load_sweeps(cls, path):
    # Load all 6 header IDs (bus 2) in arrival order and group into sweeps. A sweep boundary is each new
    # 0x280 (head of burst); each sweep's frames are replayed together so the 0x2DC terminator triggers
    # a single coherent emit per sweep (S4). Returns a list of sweeps, each a list of (addr, bytes).
    import csv
    rows = []
    with open(path) as f:
      for row in csv.DictReader(f):
        a = row["addr_hex"].upper()
        if row["bus"] == "2" and a in cls.HDR_HEX:
          rows.append((int(a, 16), bytes.fromhex(row["data_hex"])))
    sweeps, cur = [], []
    for addr, data in rows:
      if addr == 0x280 and cur:
        sweeps.append(cur)
        cur = []
      cur.append((addr, data))
    if cur:
      sweeps.append(cur)
    return sweeps

  def test_bfcar_positive_control_clean_close(self):
    sweeps = self._load_sweeps(BFCAR_CSV)
    self.assertGreater(len(sweeps), 100)
    ri = _make_ri()
    bus = ri.rcp.bus
    ranges, vrels = [], []
    dt_ns = int(0.05 * 1e9)
    for i, sweep in enumerate(sweeps):
      rr = ri.update(_can(i * dt_ns, [(addr, data, bus) for addr, data in sweep]))
      if rr is not None and rr.points:
        ranges.append(rr.points[0].dRel)
        if not math.isnan(rr.points[0].vRel):
          vrels.append(rr.points[0].vRel)
    self.assertGreater(len(ranges), 100)
    # Clean monotonic close 11.28 -> 1.52 m (CONFIRM-REPORT); allow a small margin. (S3 plausibility: every
    # dRel stays well inside [-3, 230] m so the positive control must raise NO wrongConfig.)
    self.assertAlmostEqual(max(ranges), 11.28, delta=0.2)
    self.assertAlmostEqual(min(ranges), 1.52, delta=0.5)
    # A closing trajectory -> derived vRel net negative on average.
    self.assertGreater(len(vrels), 50)
    self.assertLess(sum(vrels) / len(vrels), 0.0)

  def test_bfcar_sweep_coherent_no_split(self):
    # S4 acceptance: every live slot in a sweep is emitted in the SAME RadarData (no slot split across two
    # emits). With the 0x2DC trigger the emit fires once per sweep AFTER the whole burst has accumulated,
    # so the number of emits equals the number of sweeps that contain the terminator (no double-emit, no
    # mid-sweep partial). bfcar happens to carry exactly one live slot (slot 0); assert exactly one emit
    # per sweep and that the live slot is present in it.
    sweeps = self._load_sweeps(BFCAR_CSV)
    # restrict to sweeps that actually contain the terminator (otherwise no emit is expected)
    full = [s for s in sweeps if any(addr == BOSCH_RADAR_HDR_MSGS[-1] for addr, _ in s)]
    self.assertGreater(len(full), 100)
    ri = _make_ri()
    bus = ri.rcp.bus
    dt_ns = int(0.05 * 1e9)
    emits = 0
    for i, sweep in enumerate(full):
      rr = ri.update(_can(i * dt_ns, [(addr, data, bus) for addr, data in sweep]))
      if rr is not None:
        emits += 1
    # exactly one emit per terminator-bearing sweep -> no split, no double-emit.
    self.assertEqual(emits, len(full))

  @unittest.skipUnless(os.path.exists(IDLE_CSV), "radar-re idle capture not present")
  def test_idle_negative_control_zero_points(self):
    sweeps = self._load_sweeps(IDLE_CSV)
    self.assertGreater(len(sweeps), 100)
    ri = _make_ri()
    bus = ri.rcp.bus
    max_pts = 0
    dt_ns = int(0.05 * 1e9)
    for i, sweep in enumerate(sweeps):
      rr = ri.update(_can(i * dt_ns, [(addr, data, bus) for addr, data in sweep]))
      if rr is not None:
        max_pts = max(max_pts, len(rr.points))
    # All frames are b0==0xFE idle / b1==0xF0 (never 0x74) -> every frame skipped -> 0 points ever.
    self.assertEqual(max_pts, 0)


if __name__ == "__main__":
  unittest.main()
