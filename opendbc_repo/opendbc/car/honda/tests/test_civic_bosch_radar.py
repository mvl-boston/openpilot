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

  def setUp(self):
    self.ri = _make_ri()
    self.bus = self.ri.rcp.bus
    self.assertTrue(self.ri.bosch_radar)
    self.assertEqual(self.ri.trigger_msg, 0x280)

  def _step(self, nanos, frames):
    return self.ri.update(_can(nanos, frames))

  def _f(self, addr, frame):
    return (addr, frame, self.bus)

  def test_single_live_track_emits_one_point(self):
    rr = self._step(0, [self._f(0x280, _hdr_frame(3999))])
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 1)
    p = rr.points[0]
    self.assertEqual(p.trackId, 0)
    self.assertAlmostEqual(p.dRel, 0.00357 * 3999 - 3.0, places=4)
    self.assertTrue(p.measured)
    self.assertTrue(math.isnan(p.aRel))
    self.assertTrue(math.isnan(p.vRel))  # NaN on first sight (no prior dRel)

  def test_b1_tag_gate_skips_nonheader(self):
    # b1 != 0x74 -> non-range sub-frame -> skipped (no point), even with a plausible range field.
    rr = self._step(0, [self._f(0x280, _hdr_frame(3999, tag=0x02))])
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 0)

  def test_sentinel_strength_idle(self):
    rr = self._step(0, [self._f(0x280, _hdr_frame(3999, strength=0xFE))])
    self.assertEqual(len(rr.points), 0)

  def test_sentinel_range_unset(self):
    rr = self._step(0, [self._f(0x280, _hdr_frame(0x8000))])
    self.assertEqual(len(rr.points), 0)

  def test_sentinel_range_saturation(self):
    rr = self._step(0, [self._f(0x280, _hdr_frame(0xFF80))])
    self.assertEqual(len(rr.points), 0)
    rr = self._step(1, [self._f(0x280, _hdr_frame(0xFFFF))])
    self.assertEqual(len(rr.points), 0)

  def test_multi_object_emission(self):
    frames = [self._f(0x280, _hdr_frame(3000)),   # slot 0
              self._f(0x284, _hdr_frame(4000)),   # slot 1
              self._f(0x2D0, _hdr_frame(5000))]   # slot 2
    rr = self._step(0, frames)
    self.assertEqual(len(rr.points), 3)
    by_id = {p.trackId: p for p in rr.points}
    self.assertEqual(set(by_id), {0, 1, 2})
    self.assertAlmostEqual(by_id[0].dRel, 0.00357 * 3000 - 3.0, places=4)
    self.assertAlmostEqual(by_id[1].dRel, 0.00357 * 4000 - 3.0, places=4)
    self.assertAlmostEqual(by_id[2].dRel, 0.00357 * 5000 - 3.0, places=4)

  def test_trackid_stability_across_cycles(self):
    dt_ns = int(0.05 * 1e9)  # 20 Hz
    rr0 = self._step(0, [self._f(0x280, _hdr_frame(3000))])
    id0 = rr0.points[0].trackId
    rr1 = self._step(dt_ns, [self._f(0x280, _hdr_frame(3010))])
    self.assertEqual(rr1.points[0].trackId, id0)
    self.assertEqual(id0, 0)

  def test_vrel_derived_closing_negative(self):
    dt_ns = int(0.05 * 1e9)  # 50 ms
    self._step(0, [self._f(0x280, _hdr_frame(4000))])            # first sight: vRel NaN
    rr = self._step(dt_ns, [self._f(0x280, _hdr_frame(3000))])   # range shrank -> closing
    p = rr.points[0]
    self.assertFalse(math.isnan(p.vRel))
    self.assertLess(p.vRel, 0.0)
    expected = (0.00357 * 3000 - 0.00357 * 4000) / 0.05  # raw d(dRel)/dt (first derived sample, no EMA yet)
    self.assertAlmostEqual(p.vRel, expected, delta=1.0)

  def test_yrel_sign_and_placeholder_scale(self):
    rr = self._step(0, [self._f(0x280, _hdr_frame(3999, lat_raw=0x9000))])  # lat_raw > center -> right
    p = rr.points[0]
    lat = 0x9000 - 0x8000
    self.assertAlmostEqual(p.yRel, -lat * BOSCH_RADAR_LAT_SCALE, places=6)
    self.assertLess(p.yRel, 0.0)  # right of center -> negative y

  def test_slot_clears_when_track_goes_sentinel(self):
    dt_ns = int(0.05 * 1e9)
    rr = self._step(0, [self._f(0x280, _hdr_frame(3000)), self._f(0x284, _hdr_frame(4000))])
    self.assertEqual({p.trackId for p in rr.points}, {0, 1})
    rr = self._step(dt_ns, [self._f(0x280, _hdr_frame(3000, strength=0xFE)),
                            self._f(0x284, _hdr_frame(4000))])
    self.assertEqual({p.trackId for p in rr.points}, {1})

  def test_stale_track_aged_when_absent(self):
    dt_ns = int(0.05 * 1e9)
    rr = self._step(0, [self._f(0x280, _hdr_frame(3000)), self._f(0x284, _hdr_frame(4000))])
    self.assertEqual({p.trackId for p in rr.points}, {0, 1})
    rr = self._step(dt_ns, [self._f(0x280, _hdr_frame(3010))])  # 0x284 absent this cycle -> aged out
    self.assertEqual({p.trackId for p in rr.points}, {0})

  def test_staleness_returns_empty_radardata_not_none(self):
    # 0x280 (trigger) seen, building a point; then it goes quiet while another declared header keeps the
    # parser clock advancing past STALE_S -> EMPTY RadarData (not None) + radarUnavailableTemporary.
    self._step(0, [self._f(0x280, _hdr_frame(3000))])
    self.assertEqual(len(self.ri.pts), 1)
    stale_ns = int((BOSCH_RADAR_STALE_S + 0.05) * 1e9)
    rr = self._step(stale_ns, [self._f(0x284, _hdr_frame(0x8000))])  # 0x284 sentinel; 0x280 absent
    self.assertIsNotNone(rr)             # must be EMPTY RadarData, NOT None
    self.assertEqual(len(rr.points), 0)
    self.assertTrue(rr.errors.radarUnavailableTemporary)
    self.assertEqual(len(self.ri.pts), 0)  # points cleared

  def test_trigger_absent_no_stale_returns_none(self):
    # If 0x280 is merely absent for one cycle (well under STALE_S) and we have live points, the update
    # returns None (normal cadence gate) -- staleness only fires past STALE_S.
    self._step(0, [self._f(0x280, _hdr_frame(3000))])
    rr = self._step(int(0.02 * 1e9), [self._f(0x284, _hdr_frame(4000))])  # 0x280 absent, only 20ms later
    self.assertIsNone(rr)

  def test_fully_silent_radar_clears_points_and_returns_empty(self):
    # MOST safety-relevant staleness case: the radar goes COMPLETELY silent (no frames on ANY id) while
    # the parser clock keeps advancing (e.g. radard still pumps update() on its own cadence). The
    # frozen-phantom must still be cleared -> EMPTY RadarData (not None) + radarUnavailableTemporary.
    self._step(0, [self._f(0x280, _hdr_frame(3000))])
    self.assertEqual(len(self.ri.pts), 1)
    stale_ns = int((BOSCH_RADAR_STALE_S + 0.05) * 1e9)
    rr = self.ri.update(_can(stale_ns, []))  # whole bus silent, advancing timestamp
    self.assertIsNotNone(rr)              # EMPTY RadarData, NOT None
    self.assertEqual(len(rr.points), 0)
    self.assertTrue(rr.errors.radarUnavailableTemporary)
    self.assertEqual(len(self.ri.pts), 0)  # phantom cleared

  def test_fully_silent_under_threshold_returns_none(self):
    # Brief whole-bus silence under STALE_S must NOT prematurely drop the live point.
    self._step(0, [self._f(0x280, _hdr_frame(3000))])
    rr = self.ri.update(_can(int(0.05 * 1e9), []))  # 50 ms silent, well under 0.15 s
    self.assertIsNone(rr)
    self.assertEqual(len(self.ri.pts), 1)            # point retained

  def test_vrel_discontinuity_guard_rejects_slot_reuse(self):
    # trackId == slot index. If object A vacates slot 0 and object B enters the SAME slot in the next
    # cycle WITHOUT an intervening sentinel, the naive derivative teleports vRel to a non-physical value.
    # The guard must reject that sample (vRel NaN) AND re-seed history so the NEXT cycle derives cleanly.
    dt_ns = int(0.05 * 1e9)  # 50 ms
    # Object A at ~11 m, then ~11.04 m (slow), gives a small, real vRel.
    self._step(0, [self._f(0x280, _hdr_frame(3900))])
    rr = self._step(dt_ns, [self._f(0x280, _hdr_frame(3910))])
    self.assertFalse(math.isnan(rr.points[0].vRel))   # real small vRel
    # Cycle 3: a DIFFERENT object B jumps into slot 0 at ~110 m (raw ~31600). Implied speed is ~1900 m/s.
    far_raw = int((110.0 + 3.0) / 0.00357)
    rr = self._step(2 * dt_ns, [self._f(0x280, _hdr_frame(far_raw))])
    p = rr.points[0]
    self.assertTrue(math.isnan(p.vRel))               # phantom rejected -> NaN, not ~1900 m/s
    self.assertAlmostEqual(p.dRel, 0.00357 * far_raw - 3.0, places=2)  # dRel still tracks the new object
    # Cycle 4: object B advances slightly -> a real, in-bounds vRel now derives from the re-seeded baseline.
    rr = self._step(3 * dt_ns, [self._f(0x280, _hdr_frame(far_raw - 10))])
    p = rr.points[0]
    self.assertFalse(math.isnan(p.vRel))
    self.assertLessEqual(abs(p.vRel), BOSCH_RADAR_VREL_MAX)

  def test_vrel_in_bounds_fast_lead_not_rejected(self):
    # A genuine fast closer (e.g. stationary object at highway speed ~31 m/s) must NOT be rejected.
    dt_ns = int(0.05 * 1e9)
    self._step(0, [self._f(0x280, _hdr_frame(int((50.0 + 3.0) / 0.00357)))])      # 50 m
    rr = self._step(dt_ns, [self._f(0x280, _hdr_frame(int((48.5 + 3.0) / 0.00357)))])  # 48.5 m -> ~-30 m/s
    p = rr.points[0]
    self.assertFalse(math.isnan(p.vRel))
    self.assertLess(p.vRel, 0.0)
    self.assertLessEqual(abs(p.vRel), BOSCH_RADAR_VREL_MAX)

  def test_returns_radardata_with_points_list(self):
    rr = self._step(0, [self._f(0x280, _hdr_frame(3000))])
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 1)            # iterable points sequence on the RadarData
    # canError mirrors rcp.can_valid; on the very first frame the parser is typically not yet valid.
    self.assertEqual(rr.errors.canError, not self.ri.rcp.can_valid)


@unittest.skipUnless(os.path.exists(BFCAR_CSV), "radar-re bfcar capture not present")
class TestCivicBoschFineRealCapture(unittest.TestCase):
  """Replay real radar-re captures through the parser (positive + negative control)."""

  @staticmethod
  def _load_280(path):
    import csv
    out = []
    with open(path) as f:
      for row in csv.DictReader(f):
        if row["bus"] == "2" and row["addr_hex"].upper().endswith("280"):
          out.append(bytes.fromhex(row["data_hex"]))
    return out

  def test_bfcar_positive_control_clean_close(self):
    frames = self._load_280(BFCAR_CSV)
    self.assertGreater(len(frames), 100)
    ri = _make_ri()
    bus = ri.rcp.bus
    ranges, vrels = [], []
    dt_ns = int(0.05 * 1e9)
    for i, data in enumerate(frames):
      rr = ri.update(_can(i * dt_ns, [(0x280, data, bus)]))
      if rr is not None and rr.points:
        ranges.append(rr.points[0].dRel)
        if not math.isnan(rr.points[0].vRel):
          vrels.append(rr.points[0].vRel)
    self.assertGreater(len(ranges), 100)
    # Clean monotonic close 11.28 -> 1.52 m (CONFIRM-REPORT); allow a small margin.
    self.assertAlmostEqual(max(ranges), 11.28, delta=0.2)
    self.assertAlmostEqual(min(ranges), 1.52, delta=0.5)
    # A closing trajectory -> derived vRel net negative on average.
    self.assertGreater(len(vrels), 50)
    self.assertLess(sum(vrels) / len(vrels), 0.0)

  @unittest.skipUnless(os.path.exists(IDLE_CSV), "radar-re idle capture not present")
  def test_idle_negative_control_zero_points(self):
    frames = self._load_280(IDLE_CSV)
    self.assertGreater(len(frames), 100)
    ri = _make_ri()
    bus = ri.rcp.bus
    max_pts = 0
    dt_ns = int(0.05 * 1e9)
    for i, data in enumerate(frames):
      rr = ri.update(_can(i * dt_ns, [(0x280, data, bus)]))
      if rr is not None:
        max_pts = max(max_pts, len(rr.points))
    # All frames are b0==0xFE idle / b1==0xF0 (never 0x74) -> every frame skipped -> 0 points ever.
    self.assertEqual(max_pts, 0)


if __name__ == "__main__":
  unittest.main()
