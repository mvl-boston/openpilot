#!/usr/bin/env python3
"""73z: Bosch fine radar stationary-lead gate -- unit tests.

Tests the allow-list fix: BOSCH_RADAR_HDR_TAG_SET = {0x74, 0x94}.

Background: the Bosch radar broadcasts a stopped/decelerating lead's RANGE under TRACK_TAG=0x94
(stationary motion class), using the SAME byte layout as moving-class 0x74. The old full-byte
gate (== 0x74) dropped every 0x94 frame as metadata -> no RadarPoint -> vision-only during stops.
Fix: gate on membership in BOSCH_RADAR_HDR_TAG_SET. 0x34 and all other tags STAY REJECTED.

Coverage:
  (a) 0x94-tagged sub-frame with a valid range now produces a RadarPoint (was dropped before)
  (b) 0x74 frame still works identically (regression)
  (c) 0x34/0x54/0xFE/other tags are still rejected
  (d) 0x94 frame flows through the demux as range_frame not meta_frame
  (e) mixed 0x74+0x94 in one window demuxes correctly (last wins; both are range-class)

Test approach: real CANParser + RadarInterface end-to-end (same style as test_civic_bosch_radar.py
and test_civic_bosch_radar_d1_r1.py). No decode re-implementation; all constants pulled directly
from the module under test.

100% OFFLINE: synthetic frames only.
"""
import math
import unittest

from opendbc.car import structs
from opendbc.car.honda.radar_interface import (
  RadarInterface,
  BOSCH_RADAR_HDR_MSGS,
  BOSCH_RADAR_HDR_TAG,        # 0x74 -- kept for DBC decode asserts and backward compat
  BOSCH_RADAR_HDR_TAG_SET,    # {0x74, 0x94} -- the allow-list
  BOSCH_RADAR_BORN_CYCLES,
  BOSCH_RADAR_TRACKID_STRIDE,
)
from opendbc.car.honda.values import CAR

RANGE_SCALE = 0.00357    # m/LSB (DBC)
RANGE_OFFSET = -3.0
SWEEP_NS = int(0.05 * 1e9)
TRIG = 0x2DC


# ---------------------------------------------------------------------------
# Low-level frame helpers (mirrors test_civic_bosch_radar.py)
# ---------------------------------------------------------------------------

def _frame(b0, tag, b2, b3, b4, b5, b6, b7):
  return bytes([b0, tag, b2, b3, b4, b5, b6, b7])


def _hdr_frame(range_raw, *, tag=BOSCH_RADAR_HDR_TAG, strength=0x00, lat_raw=0x8000, cntr=0x00):
  """8-byte header frame: b0=strength, b1=tag, b2:b3=range_raw BE16, b4:b5=lat_raw BE16, b7=cntr."""
  return _frame(strength, tag, (range_raw >> 8) & 0xFF, range_raw & 0xFF,
                (lat_raw >> 8) & 0xFF, lat_raw & 0xFF, 0x00, cntr)


def _raw_for(d_m):
  return int(round((d_m - RANGE_OFFSET) / RANGE_SCALE))


def _can(nanos, frames):
  return [nanos, frames]


# ---------------------------------------------------------------------------
# Base test case (mirrors BoschCase in test_civic_bosch_radar_d1_r1.py)
# ---------------------------------------------------------------------------

class BoschCase(unittest.TestCase):
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
    return self._f(TRIG, _hdr_frame(0x8000, tag=0xF0, strength=0xFE, cntr=cntr))

  def _emit(self, nanos, frames, cntr):
    return self.ri.update(_can(nanos, list(frames) + [self._trig(cntr)]))

  def _warm(self, range_raw, *, tag=BOSCH_RADAR_HDR_TAG, slot_addr=0x280,
            lat_raw=0x8000, base_ns=0, cntr0=0x10):
    """Drive BORN_CYCLES sweeps so the slot is born on the last returned rr."""
    rr = None
    for k in range(BOSCH_RADAR_BORN_CYCLES):
      cntr = (cntr0 + k) & 0xFF
      body = [self._f(slot_addr, _hdr_frame(range_raw, tag=tag, lat_raw=lat_raw, cntr=cntr))]
      rr = self._emit(base_ns + k * SWEEP_NS, body, cntr)
    return rr

  def _slots(self, rr):
    return {p.trackId // BOSCH_RADAR_TRACKID_STRIDE for p in rr.points}


# ---------------------------------------------------------------------------
# (a) + (b) Core allow-list acceptance / regression
# ---------------------------------------------------------------------------

class TestStationaryGateAllowList(BoschCase):
  """Cases (a) and (b): 0x94 now admitted; 0x74 still works."""

  def test_a_0x94_range_frame_produces_radar_point(self):
    # (a) A 0x94-tagged frame with a valid range must now produce a RadarPoint.
    # This was the exact failure mode: every 0x94 frame was sent to meta_frames,
    # so rec.range_frame stayed None and the slot was cleared each sweep.
    rr = self._warm(_raw_for(15.0), tag=0x94)
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 1, "0x94 stop-class frame must produce a RadarPoint")
    p = rr.points[0]
    self.assertAlmostEqual(p.dRel, 15.0, delta=0.05)
    self.assertEqual(self._slots(rr), {0})

  def test_a_0x94_range_and_vrel_pipeline_complete(self):
    # (a) full pipeline: two consecutive 0x94 sweeps -> born point with a derived vRel
    rng0 = _raw_for(20.0)
    rng1 = _raw_for(18.5)
    self._emit(0, [self._f(0x280, _hdr_frame(rng0, tag=0x94, cntr=0x10))], 0x10)  # prime
    rr = self._emit(SWEEP_NS, [self._f(0x280, _hdr_frame(rng1, tag=0x94, cntr=0x11))], 0x11)  # born
    self.assertEqual(len(rr.points), 1)
    p = rr.points[0]
    self.assertAlmostEqual(p.dRel, 18.5, delta=0.05)
    self.assertFalse(math.isnan(p.vRel))
    self.assertLess(p.vRel, 0.0)   # closing

  def test_b_0x74_regression_unchanged(self):
    # (b) 0x74 (moving class) still works exactly as before. Guard against regressions.
    rr = self._warm(_raw_for(12.0), tag=BOSCH_RADAR_HDR_TAG)
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 1, "0x74 moving-class frame must still produce a RadarPoint")
    self.assertAlmostEqual(rr.points[0].dRel, 12.0, delta=0.05)

  def test_b_0x74_vrel_regression(self):
    # (b) closing sequence under 0x74 still yields negative vRel (R1 pipeline unaffected)
    self._emit(0, [self._f(0x280, _hdr_frame(_raw_for(25.0), tag=0x74, cntr=0x10))], 0x10)
    rr = self._emit(SWEEP_NS, [self._f(0x280, _hdr_frame(_raw_for(23.5), tag=0x74, cntr=0x11))], 0x11)
    self.assertEqual(len(rr.points), 1)
    self.assertFalse(math.isnan(rr.points[0].vRel))
    self.assertLess(rr.points[0].vRel, 0.0)

  def test_a_0x94_trackid_no_reuse(self):
    # S1 parity: a slot born under 0x94 must carry slot*STRIDE + incarnation as its trackId.
    rr = self._warm(_raw_for(10.0), tag=0x94)
    p = rr.points[0]
    self.assertEqual(p.trackId // BOSCH_RADAR_TRACKID_STRIDE, 0)   # slot 0
    self.assertEqual(p.trackId % BOSCH_RADAR_TRACKID_STRIDE, 1)    # first incarnation


# ---------------------------------------------------------------------------
# (c) Rejected tags -- 0x34, 0x54, 0xFE, 0x02, 0xF0 still produce zero points
# ---------------------------------------------------------------------------

class TestRejectedTags(BoschCase):
  """Case (c): tags outside {0x74, 0x94} STAY REJECTED."""

  def _assert_no_point_for_tag(self, tag):
    # Even BORN_CYCLES sweeps of a given non-allow-list tag must never birth a point.
    for k in range(BOSCH_RADAR_BORN_CYCLES + 2):
      cntr = (0x10 + k) & 0xFF
      rr = self._emit(k * SWEEP_NS, [self._f(0x280, _hdr_frame(_raw_for(10.0), tag=tag, cntr=cntr))], cntr)
      self.assertIsNotNone(rr)
      self.assertEqual(len(rr.points), 0,
                       f"tag 0x{tag:02X} must be rejected; got {len(rr.points)} point(s)")

  def test_c_0x34_rejected(self):
    # 0x34 -- corpus r weak/unvalidated; documented coverage gap, NOT in allow-list.
    self._assert_no_point_for_tag(0x34)

  def test_c_0x54_rejected(self):
    self._assert_no_point_for_tag(0x54)

  def test_c_0xFE_rejected(self):
    # 0xFE is the idle STRENGTH sentinel value; using it as a tag must still be rejected.
    self._assert_no_point_for_tag(0xFE)

  def test_c_0x02_rejected(self):
    self._assert_no_point_for_tag(0x02)

  def test_c_0xF0_rejected(self):
    # 0xF0 is the tag used by the trigger sentinel frame -- must not birth points.
    self._assert_no_point_for_tag(0xF0)

  def test_c_allow_list_exactly_0x74_and_0x94(self):
    # Confirm the allow-list constant itself is exactly {0x74, 0x94} -- no surprise admissions.
    self.assertEqual(BOSCH_RADAR_HDR_TAG_SET, frozenset({0x74, 0x94}))
    self.assertIn(0x74, BOSCH_RADAR_HDR_TAG_SET)
    self.assertIn(0x94, BOSCH_RADAR_HDR_TAG_SET)
    self.assertNotIn(0x34, BOSCH_RADAR_HDR_TAG_SET)
    self.assertNotIn(0x54, BOSCH_RADAR_HDR_TAG_SET)
    self.assertNotIn(0x64, BOSCH_RADAR_HDR_TAG_SET)
    self.assertNotIn(0xB4, BOSCH_RADAR_HDR_TAG_SET)
    self.assertNotIn(0xFE, BOSCH_RADAR_HDR_TAG_SET)
    self.assertNotIn(0xF0, BOSCH_RADAR_HDR_TAG_SET)


# ---------------------------------------------------------------------------
# (d) Demux-path classification: 0x94 must land in range_frame, NOT meta_frames
# ---------------------------------------------------------------------------

class TestDemuxClassification(BoschCase):
  """Case (d): direct _bosch_assemble_record method confirms demux routing.

  _bosch_assemble_record is a method on RadarInterface. We call it directly by populating
  ri._pending (the frame buffer it reads) and invoking ri._bosch_assemble_record(slot).
  This is the AST-extraction-style approach described in the bead spec for testing demux
  internals offline without full CANParser round-tripping.
  """

  def _pending_rec(self, slot, frame_dicts):
    """Directly populate ri._pending for slot and call _bosch_assemble_record."""
    self.ri._pending[slot] = frame_dicts
    return self.ri._bosch_assemble_record(slot)

  def test_d_0x94_lands_in_range_frame_not_meta(self):
    # (d) A 0x94 frame dict must be classified as range_frame, NOT stored in meta_frames.
    raw = _raw_for(15.0)
    frame_dict = {
      'TRACK_TAG': 0x94, 'STRENGTH': 0x00,
      'RANGE_RAW': raw, 'RANGE': raw * RANGE_SCALE + RANGE_OFFSET,
      'LAT_RAW': 0.0, 'CNTR': 0x20,
    }
    rec = self._pending_rec(0, [frame_dict])
    self.assertIsNotNone(rec.range_frame, "0x94 frame must classify as range_frame")
    self.assertNotIn(0x94, rec.meta_frames, "0x94 must NOT be stored in meta_frames")
    self.assertEqual(int(rec.range_frame['TRACK_TAG']), 0x94)

  def test_d_0x74_still_lands_in_range_frame(self):
    # (d) regression: 0x74 still classifies as range_frame.
    raw = _raw_for(10.0)
    frame_dict = {
      'TRACK_TAG': 0x74, 'STRENGTH': 0x00,
      'RANGE_RAW': raw, 'RANGE': raw * RANGE_SCALE + RANGE_OFFSET,
      'LAT_RAW': 0.0, 'CNTR': 0x10,
    }
    rec = self._pending_rec(0, [frame_dict])
    self.assertIsNotNone(rec.range_frame)
    self.assertNotIn(0x74, rec.meta_frames)

  def test_d_0x34_lands_in_meta_frames_not_range(self):
    # (d) 0x34 must still be classified as a meta_frame (coverage gap, not admitted).
    raw = _raw_for(12.0)
    frame_dict = {
      'TRACK_TAG': 0x34, 'STRENGTH': 0x00,
      'RANGE_RAW': raw, 'RANGE': raw * RANGE_SCALE + RANGE_OFFSET,
      'LAT_RAW': 0.0, 'CNTR': 0x30,
    }
    rec = self._pending_rec(0, [frame_dict])
    self.assertIsNone(rec.range_frame, "0x34 must NOT classify as range_frame")
    self.assertIn(0x34, rec.meta_frames, "0x34 must land in meta_frames")

  def test_d_0x94_single_sweep_no_clobber_flag(self):
    # (d) A 0x94 range_frame with no trailing meta should NOT set recovered_clobber.
    raw = _raw_for(20.0)
    frame_dict = {
      'TRACK_TAG': 0x94, 'STRENGTH': 0x00,
      'RANGE_RAW': raw, 'RANGE': raw * RANGE_SCALE + RANGE_OFFSET,
      'LAT_RAW': 0.0, 'CNTR': 0x40,
    }
    rec = self._pending_rec(0, [frame_dict])
    self.assertFalse(rec.recovered_clobber)


# ---------------------------------------------------------------------------
# (e) Mixed 0x74 + 0x94 in one window
# ---------------------------------------------------------------------------

class TestMixedWindow(BoschCase):
  """Case (e): mixed 0x74 + 0x94 in one emit window demuxes correctly."""

  def _raw_frame_dict(self, tag, d_m):
    raw = _raw_for(d_m)
    return {
      'TRACK_TAG': tag, 'STRENGTH': 0x00,
      'RANGE_RAW': raw, 'RANGE': raw * RANGE_SCALE + RANGE_OFFSET,
      'LAT_RAW': 0.0, 'CNTR': 0x50,
    }

  def test_e_0x74_then_0x94_last_wins_range_frame(self):
    # (e) 0x74 arrives first, then 0x94 in the same window. Both are range-class;
    # the LAST one should be range_frame (freshest kinematics rule from D1).
    frames = [
      self._raw_frame_dict(0x74, 10.0),  # first
      self._raw_frame_dict(0x94, 8.0),   # second (last wins)
    ]
    self.ri._pending[0] = frames
    rec = self.ri._bosch_assemble_record(0)
    self.assertIsNotNone(rec.range_frame)
    self.assertAlmostEqual(rec.range_frame['RANGE'], 8.0, delta=0.05)
    self.assertEqual(int(rec.range_frame['TRACK_TAG']), 0x94)
    # Neither 0x74 nor 0x94 should be in meta_frames (both are range-class)
    self.assertNotIn(0x74, rec.meta_frames)
    self.assertNotIn(0x94, rec.meta_frames)

  def test_e_0x94_then_0x74_last_wins_range_frame(self):
    # (e) 0x94 arrives first, then 0x74. Last still wins.
    frames = [
      self._raw_frame_dict(0x94, 8.0),   # first
      self._raw_frame_dict(0x74, 10.0),  # second (last wins)
    ]
    self.ri._pending[0] = frames
    rec = self.ri._bosch_assemble_record(0)
    self.assertIsNotNone(rec.range_frame)
    self.assertAlmostEqual(rec.range_frame['RANGE'], 10.0, delta=0.05)
    self.assertEqual(int(rec.range_frame['TRACK_TAG']), 0x74)

  def test_e_0x74_then_0x94_then_meta_sets_clobber(self):
    # (e) range-carrier (0x74), then another range-carrier (0x94), then a meta (0x75).
    # The meta frame after the last range-carrier triggers recovered_clobber.
    frames = [
      self._raw_frame_dict(0x74, 10.0),
      self._raw_frame_dict(0x94, 8.0),
      {'TRACK_TAG': 0x75, 'STRENGTH': 0x10, 'RANGE_RAW': 0, 'RANGE': 0.0, 'LAT_RAW': 0.0, 'CNTR': 0x50},
    ]
    self.ri._pending[0] = frames
    rec = self.ri._bosch_assemble_record(0)
    self.assertIsNotNone(rec.range_frame)  # range_frame was set before the meta landed
    self.assertTrue(rec.recovered_clobber, "trailing meta after range_frame must set recovered_clobber")
    self.assertIn(0x75, rec.meta_frames)

  def test_e_mixed_end_to_end_0x94_slot_emits_point(self):
    # (e) end-to-end: two sweeps with 0x94 tags, interleaved with meta frames in the same window.
    # The stop-class range carrier must survive the demux and produce a born point.
    meta_bytes = bytes([0x10, 0x75, 0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x10])  # tag=0x75
    rng0 = _raw_for(18.0)
    rng1 = _raw_for(17.0)
    # sweep 0: 0x94 range frame + a trailing 0x75 meta
    self.ri.update(_can(0, [
      self._f(0x280, _hdr_frame(rng0, tag=0x94, cntr=0x10)),
      self._f(0x280, meta_bytes),
      self._trig(0x10),
    ]))
    # sweep 1: same pattern; should be born (2nd valid range cycle)
    rr = self.ri.update(_can(SWEEP_NS, [
      self._f(0x280, _hdr_frame(rng1, tag=0x94, cntr=0x11)),
      self._f(0x280, meta_bytes),
      self._trig(0x11),
    ]))
    self.assertIsNotNone(rr)
    self.assertEqual(len(rr.points), 1, "0x94 stop sweep with trailing meta must emit a RadarPoint")
    self.assertAlmostEqual(rr.points[0].dRel, 17.0, delta=0.05)


# ---------------------------------------------------------------------------
# S2/S3 parity: hysteresis + sentinel behavior is identical for 0x94 tracks
# ---------------------------------------------------------------------------

class TestStationaryGateParity(BoschCase):
  """0x94-based tracks must satisfy the same S2/S3 contracts as 0x74-based tracks."""

  def test_s2_parity_single_0x94_sweep_no_phantom(self):
    # S2: one sweep of 0x94 must NOT birth a point.
    rr = self._emit(0, [self._f(0x280, _hdr_frame(_raw_for(10.0), tag=0x94, cntr=0x10))], 0x10)
    self.assertEqual(len(rr.points), 0, "single 0x94 sweep must not birth a phantom (S2)")

  def test_s2_parity_0x94_born_after_n_cycles(self):
    # S2: 0x94 track born after exactly BORN_CYCLES.
    for k in range(BOSCH_RADAR_BORN_CYCLES - 1):
      cntr = (0x10 + k) & 0xFF
      rr = self._emit(k * SWEEP_NS, [self._f(0x280, _hdr_frame(_raw_for(10.0), tag=0x94, cntr=cntr))], cntr)
      self.assertEqual(len(rr.points), 0)
    cntr = (0x10 + BOSCH_RADAR_BORN_CYCLES - 1) & 0xFF
    rr = self._emit((BOSCH_RADAR_BORN_CYCLES - 1) * SWEEP_NS,
                    [self._f(0x280, _hdr_frame(_raw_for(10.0), tag=0x94, cntr=cntr))], cntr)
    self.assertEqual(len(rr.points), 1)

  def test_s2_parity_0x94_to_0x74_transition_stable(self):
    # A slot that transitions from 0x94 (stop) to 0x74 (moving) should keep its trackId
    # (same slot, same incarnation -- the car accelerated; no vacancy between).
    rr0 = self._warm(_raw_for(5.0), tag=0x94)
    self.assertEqual(len(rr0.points), 1)
    id_stop = rr0.points[0].trackId
    # next sweep: same range, 0x74 tag (car starts moving again)
    rr1 = self._emit(BOSCH_RADAR_BORN_CYCLES * SWEEP_NS,
                     [self._f(0x280, _hdr_frame(_raw_for(5.0), tag=0x74, cntr=0x30))], 0x30)
    self.assertEqual(len(rr1.points), 1)
    self.assertEqual(rr1.points[0].trackId, id_stop,
                     "0x94->0x74 tag change (same slot, no vacancy) must NOT bump incarnation")

  def test_s3_sentinel_strength_0x94_cleared(self):
    # S3: a 0x94 frame with idle strength (0xFE) is a sentinel -> slot cleared, no point.
    rr = self._emit(0, [self._f(0x280, _hdr_frame(_raw_for(10.0), tag=0x94, strength=0xFE, cntr=0x10))], 0x10)
    self.assertEqual(len(rr.points), 0)


if __name__ == "__main__":
  unittest.main()
