#!/usr/bin/env python3
import math
from dataclasses import dataclass, field
from math import pi, sin

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.honda.values import DBC, CAR


def _create_nidec_can_parser(car_fingerprint):
  radar_messages = [0x400] + list(range(0x430, 0x43A)) + list(range(0x440, 0x446))
  messages = [(m, 20) for m in radar_messages]
  return CANParser(DBC[car_fingerprint][Bus.radar], messages, 1)


# 36802-TBA Bosch radar FINE per-track object table (0x280 block).
# Cross-car CONFIRMED 2026-06-07 (3 cars / 6 routes; 8905 tracks fused leadOne.dRel at R^2=0.975).
# This SUPERSEDES the coarse 0x2C8/0x2C9 selected-lead as the range source: the radar broadcasts up to
# 6 track records, each a 4-frame burst on consecutive IDs. Only the HEADER ID of each record carries
# RANGE, and only on the sub-frame tagged b1==0x74. The parser gates on that tag, skips idle/saturation
# sentinels, emits up to 6 RadarPoints (stable trackId per slot), and lets radard select the lead.
# IMPORTANT: these object frames are physically on openpilot CanBus.camera (rlog src=2, confirmed
# across 6 routes), NOT CanBus.radar (bus 0). The Bus.radar key below is only the DBC-name lookup;
# the parser's CAN bus is CanBus(CP).camera. Reconfirm the bus with a read-only sniff before trusting dRel.
BOSCH_RADAR_HDR_MSGS = [0x280, 0x284, 0x2D0, 0x2D4, 0x2D8, 0x2DC]

# Range-carrier header tag: only sub-frames with b1==0x74 carry RANGE in b2:b3. Any other tag is a
# non-range sub-frame mux'd onto the header ID -> skip (do not read RANGE).
BOSCH_RADAR_HDR_TAG = 0x74

# Idle / unset / saturation sentinels (any one -> not an active track). STRENGTH is b0 raw (idle 0xFE),
# RANGE_RAW is b2:b3 raw int (0x8000 ~114 m unset; >=0xFF80 ~230 m saturation rail).
BOSCH_RADAR_STRENGTH_IDLE = 0xFE
BOSCH_RADAR_RAW_UNSET = 0x8000
BOSCH_RADAR_RAW_SAT = 0xFF80

# b4:b5 FIELD IDENTITY = AZIMUTH ANGLE (offset-binary, center 0x8000), NOT range-rate.
# SETTLED 2026-06-08 by a three-source rlog regression against the vision-model lead (no flash, no TX,
# no Ghidra, no controlled/tape capture) -- radar-re/latscale/_rlog/{peter-3d,peter-49,joey}.md and
# radar-re/latscale/LATSCALE-SHIPPED.md. ~20,880 matched (radar-track <-> model-lead) frames across
# three cars:
#   * AZIMUTH wins every discriminating (high-lateral-motion) segment; richest segment (peter-3d seg168,
#     lateral -4.14..+5.38 m) full-sin R^2=0.946, Pearson r(off, asin(y0/rng)) = -0.965.
#   * RANGE-RATE is FALSIFIED: off vs relative velocity is flat to noise everywhere (R^2 0.01-0.08 on the
#     discriminating segments; pooled Pearson r ~= +0.045). b4:b5 is NOT a free vRel -- vRel stays DERIVED.
# So yRel is the lateral PROJECTION of the (range, azimuth) polar measurement, computed trigonometrically
# from dRel and the offset-binary angle -- NOT a linear m/LSB on the raw field:
#     yRel = -dRel * sin((b4b5 - 0x8000) * LAT_SCALE_DEG_PER_LSB * pi/180)
# The negative sign (right-of-center b4b5 -> negative yRel) is the rlog-confirmed convention; LAT_RAW is
# already (b4b5 - 0x8000) per the DBC offset -32768, so it feeds the sin directly.
#
# SCALE: ~0.0009-0.001 deg/LSB (per-source free fits: joey 0.000902, peter-3d 0.0007-0.00086,
# peter-49 0.0006). The firmware-static candidate 0.001462 deg/LSB is REJECTED by all three sources
# (joey nonlinear RMS 1.31 m vs 1.06 m at 0.001; peter-3d pooled R^2 0.215; peter-49 meters-R^2 -29).
# 0.001 deg/LSB is the shipped value: a clean round figure at the top of the converged band, the explicit
# recommendation of the two highest-n sources, and it beats 0.001462 decisively. MEDIUM confidence on the
# third significant figure (0.0007-0.001 residual ambiguity) and the ABSOLUTE boresight zero-offset is
# still unpinned (a per-mount bias the regression absorbed as a per-segment fixed effect) -- one parked
# tape-measure read, or the on-road read-only x31 SRAM cross-check in radar-re/azimuth_capture.py, would
# tighten both. The FIELD IDENTITY (azimuth, not range-rate) is HIGH confidence and settled.
BOSCH_RADAR_LAT_SCALE_DEG_PER_LSB = 0.001  # deg/LSB; rlog-regressed band 0.0007-0.001, 0.001462 rejected

# Staleness gate: if no fresh 0x280 header is seen for this long, clear all points and return an EMPTY
# RadarData (not None) so radard drops the lead within a cycle (no frozen phantom). RadarPoints carry no
# per-point monotime, so radard trusts points unconditionally -- a stalled source must be cleared here.
BOSCH_RADAR_STALE_S = 0.15  # ~3 missed 20 Hz frames

# vRel discontinuity guard (slot-reuse defense). trackId == slot index, so if object A vacates a slot and
# object B enters the SAME slot in the next cycle WITHOUT an intervening empty/sentinel cycle, the naive
# d(dRel)/dt derivative would teleport vRel to a physically impossible value (e.g. an observed ~1928 m/s)
# for what radard then treats as one continuous object. Any implied |vRel| above this bound is therefore
# NOT a real relative speed -- it is a track swap. We REJECT that sample (vRel -> NaN, and re-seed the
# per-track history baseline to the NEW object's position) so the next clean cycle derives a real vRel
# from the new object instead of carrying a phantom. The bound is deliberately generous: a genuine
# fast-closing lead (stationary object at highway speed ~31 m/s, or a head-on ~60 m/s) is far below it,
# so no real lead is ever rejected; only the swap artifact (which is ~20x over) is.
BOSCH_RADAR_VREL_MAX = 100.0  # m/s; |derived vRel| above this == slot-reuse artifact, not a real speed

# --- SAFE parity hardening constants (PARITY-MATRIX §3/§4; RX-only, keep-AEB preserved) ----------
# S1 -- trackId no-reuse (capnp car.capnp:314 "no trackId reuse"). trackId is no longer the bare slot
# index; it is slot*1000 + incarnation, where incarnation is bumped every time a slot is (re)born after
# being vacated (sentinel/absent/wrong-tag) OR on a detected slot-reuse discontinuity. This way a slot
# that hands off from object A to object B presents radard a DIFFERENT trackId, so radard's Kalman
# tracker sees a clean birth instead of one continuous object teleporting. Still UInt64; still
# slot-decodable as trackId // BOSCH_RADAR_TRACKID_STRIDE; never reused for a distinct physical object.
BOSCH_RADAR_TRACKID_STRIDE = 1000  # trackId = slot * STRIDE + incarnation

# S2 -- birth/persist hysteresis (Toyota valid_cnt pattern, toyota/radar_interface.py:66-77). A per-slot
# confidence counter: +1 on a valid range-carrier frame (capped), -1 (floored at 0) on absent/sentinel/
# wrong-tag. A point is only EMITTED once the counter reaches BORN_CYCLES (debounces a 1-frame glitch
# into a phantom), and only DROPPED once the counter floors at 0 (tolerates a single missed cycle).
# This governs per-slot birth/death WITHIN a live trigger stream only; the global STALE_S clear in
# update() and the whole-bus-silent path are UNCHANGED and still win (a whole-bus silence wipes all).
BOSCH_RADAR_BORN_CYCLES = 2  # consecutive valid cycles required before a slot emits a point
BOSCH_RADAR_VALID_CAP = 5    # max value the per-slot confidence counter saturates at

# S3 -- plausibility / self-consistency fault annotations (RX-only; error bits on RadarData, NOT
# authority changes; NO new frame decoded -- this is the SAFE subset of matrix #18). (a) a decoded dRel
# outside the physical band [RANGE_MIN, RANGE_MAX] for the declared scale is a decode/scale inconsistency
# -> wrongConfig + skip the point. (b) if can_valid but the trigger slot's CNTR has not advanced for
# CNTR_STALL_CYCLES cycles, the source is frozen-but-present -> radarFault.
BOSCH_RADAR_RANGE_MIN = -3.0   # m; DBC offset floor (raw 0 -> -3.0). Below this == decode inconsistency.
BOSCH_RADAR_RANGE_MAX = 230.0  # m; saturation rail ceiling. Above this == decode inconsistency.
BOSCH_RADAR_CNTR_STALL_CYCLES = 3  # frozen-CNTR cycles (while can_valid) before radarFault

# S6 -- vRel derivation hardening. Reject a non-positive dt (already guarded) AND clamp the dt upper
# bound: after a long gap the tiny-denominator-free path is fine but a stale baseline across a long gap
# would derive a spurious vRel from two far-apart-in-time samples -> re-seed (drop the derivative this
# cycle) when dt exceeds this.
BOSCH_RADAR_VREL_DT_MAX_S = 0.5  # s; baseline older than this -> re-seed, do not derive a vRel

# D1 (nrdrbranchdebug-86t.1) -- tag-demux frame assembler. CANParser.vl is last-write-wins per signal;
# when a 0x74 range-carrier and a non-0x74 metadata sub-frame share a header ID in one emit window, the
# old vl read saw only the LAST frame and spuriously decayed a live track. Frames are harvested from
# vl_all per rcp.update() batch (vl_all is cleared on every update call, while updated_messages
# accumulates until the 0x2DC trigger) and demuxed by TRACK_TAG at emit time.
BOSCH_RADAR_PENDING_CAP = 8  # max harvested frames retained per slot between trigger emits

# R1 (nrdrbranchdebug-86t.4) -- per-slot constant-velocity [range, range_rate] Kalman filter on the raw
# DBC-scaled RANGE, replacing the d(dRel)/dt + alpha=0.5 EMA derivation (~50-75 ms lag, LSB quantization
# chatter, and the S6 re-seed EMA-halving bug nrdrbranchdebug-6u8, which dies with the EMA).
BOSCH_RADAR_KF_R = 0.012          # m^2; range measurement variance (0.00357 m/LSB quantization + jitter)
BOSCH_RADAR_KF_Q_R = 0.05         # m^2/s; range process noise density
BOSCH_RADAR_KF_Q_V = 4.0          # (m/s)^2/s; range-rate process noise density -- err RESPONSIVE so a
                                  # hard-braking lead is never modeled optimistic (R1 ship gate).
                                  # Sim-tuned 2026-06-10: at -6 m/s^2 lead braking, worst optimism
                                  # 0.88 m/s (0.8 gave 1.77); realistic-noise vRel std 0.35 m/s vs the
                                  # old d/dt+EMA's 1.00 -- still ~3x quieter while 2x more responsive.
BOSCH_RADAR_KF_P0_V = 1.0e4       # (m/s)^2; (re)seed range-rate variance. Large on purpose: the second
                                  # sample then reproduces the raw derivative (old-behavior parity),
                                  # smoothing only engages from the third sample on.
BOSCH_RADAR_KF_NIS_ADAPT = 4.0    # 2-sigma normalized innovation; above this Q_v is inflated this step
                                  # (noise-floor NIS is ~1e-3 at the quantization level -- huge margin)
BOSCH_RADAR_KF_NIS_INFLATE = 10.0 # Q_v inflation factor on an adaptive step (maneuver onset)
BOSCH_RADAR_KF_CONV_UPDATES = 2   # measurements absorbed before range_rate is published (S5 parity:
                                  # the old path also had a derived vRel on the 2nd sighted cycle)
BOSCH_RADAR_ACCEL_EMA_ALPHA = 0.25  # smoothing on the packed aRel (consumer is K5, radard-side, later)

_KF_OK, _KF_BREAK, _KF_RESEED = 0, 1, 2


@dataclass
class BoschTrackRecord:
  """D1: one slot's sub-frames for one emit window, demuxed by TRACK_TAG (stateless per window)."""
  slot: int
  range_frame: dict[str, float] | None = None             # latest TRACK_TAG==0x74 frame (kinematics)
  meta_frames: dict[int, dict[str, float]] = field(default_factory=dict)  # latest frame per other tag
  recovered_clobber: bool = False  # a 0x74 frame was present but a meta frame arrived after it (the
                                   # exact window the old last-write-wins read would have mis-cleared)


class _SlotRangeKF:
  """R1: scalar 2-state CV Kalman filter, full P propagation (sweep dt jitters ~50-70 ms).

  Owns the S6 contracts: dt<=0 -> skip (no double-absorb), dt>VREL_DT_MAX -> in-place reseed,
  implied jump speed > VREL_MAX -> caller-visible BREAK (S1 incarnation bump). range_rate is
  published only after CONV_UPDATES measurements (S5: estimate vs measurement honesty).
  """
  __slots__ = ("r", "v", "a", "p00", "p01", "p11", "t", "n")

  def __init__(self, r0: float, t_nanos: int):
    self._seed(r0, t_nanos)

  def _seed(self, r0: float, t_nanos: int):
    self.r = r0
    self.v = 0.0
    self.a = float('nan')
    self.p00 = BOSCH_RADAR_KF_R
    self.p01 = 0.0
    self.p11 = BOSCH_RADAR_KF_P0_V
    self.t = t_nanos
    self.n = 1

  @property
  def converged(self) -> bool:
    return self.n >= BOSCH_RADAR_KF_CONV_UPDATES

  def update(self, z: float, t_nanos: int) -> int:
    dt = (t_nanos - self.t) * 1e-9
    if dt <= 0:
      return _KF_OK  # non-advancing clock: keep the posterior, never absorb the same instant twice
    if dt > BOSCH_RADAR_VREL_DT_MAX_S:
      self._seed(z, t_nanos)  # long gap: a stale state would alias into a spurious rate
      return _KF_RESEED
    if abs((z - self.r) / dt) > BOSCH_RADAR_VREL_MAX:
      return _KF_BREAK  # slot-reuse discontinuity; caller bumps incarnation and reseeds

    # predict
    r_pred = self.r + self.v * dt
    q_v = BOSCH_RADAR_KF_Q_V * dt
    p00 = self.p00 + 2.0 * dt * self.p01 + dt * dt * self.p11 + BOSCH_RADAR_KF_Q_R * dt
    p01 = self.p01 + dt * self.p11
    p11 = self.p11 + q_v
    y = z - r_pred
    s = p00 + BOSCH_RADAR_KF_R
    if y * y / s > BOSCH_RADAR_KF_NIS_ADAPT:
      # innovation-adaptive Q: maneuver onset (e.g. lead brakes hard) -> trust the model less so the
      # rate snaps to the data instead of lagging optimistic
      extra = BOSCH_RADAR_KF_Q_V * (BOSCH_RADAR_KF_NIS_INFLATE - 1.0) * dt
      p11 += extra
      p01 += extra * dt
      p00 += extra * dt * dt
      s = p00 + BOSCH_RADAR_KF_R

    k0 = p00 / s
    k1 = p01 / s
    v_prev = self.v
    self.r = r_pred + k0 * y
    self.v = self.v + k1 * y
    self.p00 = (1.0 - k0) * p00
    self.p01 = (1.0 - k0) * p01
    self.p11 = p11 - k1 * p01

    # packed aRel: EMA-smoothed posterior rate delta once converged (NaN before). Payoff needs the
    # radard-side consumer (K5); packed now so logs carry it.
    if self.n >= BOSCH_RADAR_KF_CONV_UPDATES:
      a_inst = (self.v - v_prev) / dt
      self.a = a_inst if math.isnan(self.a) else (1.0 - BOSCH_RADAR_ACCEL_EMA_ALPHA) * self.a + BOSCH_RADAR_ACCEL_EMA_ALPHA * a_inst
    self.t = t_nanos
    self.n += 1
    return _KF_OK


def _create_bosch_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None
  messages = [(m, 20) for m in BOSCH_RADAR_HDR_MSGS]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, CanBus(CP).camera)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.track_id = 0
    self.radar_fault = False
    self.radar_wrong_config = False
    self.radar_off_can = CP.radarUnavailable

    # Bosch fine 0x280 track-table (Honda Civic Bosch, 36802-TBA) vs the legacy Nidec path
    self.bosch_radar = CP.carFingerprint == CAR.HONDA_CIVIC_BOSCH and Bus.radar in DBC[CP.carFingerprint]

    # R1: per-SLOT [range, range_rate] KF (replaces the (last_dRel, nanos) derivative baseline).
    # NOTE: keyed by SLOT (0..5), not trackId. trackId now carries an incarnation (S1) so it changes on
    # every (re)birth; the kinematic state must persist across that change, hence the stable slot key.
    self._kf: dict[int, _SlotRangeKF] = {}
    # D1: frames harvested from vl_all per rcp.update() batch (vl_all is wiped each call while
    # updated_messages accumulates until the trigger), demuxed by TRACK_TAG at emit time.
    self._pending: dict[int, list[dict[str, float]]] = {}
    self._clobber_recovered = 0  # emit windows where a meta frame would have mis-cleared a live track
    # S2 birth/persist hysteresis: slot index -> confidence counter (Toyota valid_cnt pattern).
    self._valid_cnt: dict[int, int] = {}
    # S1 trackId no-reuse: slot index -> current incarnation (bumped on (re)birth / slot-reuse break).
    self._incarnation: dict[int, int] = {}
    # S3 CNTR-stall fault: last seen trigger-slot CNTR and how many cycles it has been frozen.
    self._last_cntr: int | None = None
    self._cntr_stall = 0
    # Parser-clock nanos of the last cycle the trigger header (sweep terminator 0x2DC) was emitted on;
    # -1 = never. Used by the staleness gate (compared against the parser's last-update clock). Tracked
    # here rather than reading rcp.ts_nanos so a frame at absolute t=0 (synthetic/replay start) isn't
    # mistaken for "never".
    self._last_trigger_nanos = -1

    if self.radar_off_can:
      self.rcp = None
      self.trigger_msg = 0x445
    elif self.bosch_radar:
      self.rcp = _create_bosch_can_parser(CP)
      # S4 sweep-coherent trigger: the radar emits a 6-slot sweep as a short burst on consecutive header
      # IDs, HEAD-first (0x280 leads, 0x2DC terminates -- confirmed across the bfcar capture: 234 sweeps,
      # 0x280->0x2DC intra-sweep span mean 8.4 ms / max 20.2 ms, well under the ~60 ms inter-sweep
      # cadence and the 150 ms STALE_S). Triggering on the HEAD (0x280) would emit a snapshot in which
      # slot 0 is from sweep N but slots 1..5 are still from sweep N-1 (a 1-sweep time skew, demonstrated
      # with a multi-slot replay). Trigger on the sweep TERMINATOR (0x2DC) instead -- like Toyota
      # (RADAR_B_MSGS[-1]) and Hyundai (0x51F) -- so the emit fires only after the whole sweep has
      # accumulated, giving a time-coherent snapshot. 0x2DC is as reliably present as 0x280 (also 234/234
      # bursts; the trigger keys on FRAME ARRIVAL, not payload validity, so it fires even when 0x2DC's
      # payload is a sentinel). The staleness gate (update()) and per-slot logic are otherwise unchanged.
      self.trigger_msg = 0x2DC
    else:
      self.rcp = _create_nidec_can_parser(CP.carFingerprint)
      self.trigger_msg = 0x445
    self.updated_messages = set()

  def update(self, can_strings):
    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)
    if self.bosch_radar:
      # D1: harvest NOW -- vl_all is cleared on the next rcp.update() call, but the emit window
      # (trigger-gated) can span several update batches.
      self._bosch_harvest_frames(vls)

    if self.trigger_msg not in self.updated_messages:
      # Staleness fallback (Bosch fine only): the trigger header (sweep terminator 0x2DC) drives the
      # normal 20 Hz emit, but if it goes quiet while the parser keeps running we must still publish an
      # EMPTY RadarData so radard drops the stale lead (no frozen phantom). Compare the parser's
      # last-update clock to the last cycle the trigger was emitted -- both from the (replay-safe) clock.
      if self.bosch_radar and self.pts and self._last_trigger_nanos >= 0:
        now = self.rcp._last_update_nanos
        if (now - self._last_trigger_nanos) * 1e-9 > BOSCH_RADAR_STALE_S:
          return self._bosch_stale_radardata()
      return None

    rr = self._update_bosch(self.updated_messages) if self.bosch_radar else self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr

  def _bosch_stale_radardata(self):
    # Clear all tracks + vRel history and return an EMPTY RadarData (NOT None) so liveTracks keeps
    # publishing at 20 Hz with zero points -> radard drops the lead within a cycle. Reset the trigger
    # clock so we emit the empty data exactly once until the trigger (0x2DC) returns.
    self.pts.clear()
    self._kf.clear()
    self._pending.clear()
    self._valid_cnt.clear()
    # Do NOT reset _incarnation here: trackId no-reuse (S1) must hold across a staleness clear too, so a
    # slot that revives after going stale gets a fresh trackId rather than reusing the pre-stale one.
    self._last_trigger_nanos = -1
    self._last_cntr = None
    self._cntr_stall = 0
    stale = structs.RadarData()
    if not self.rcp.can_valid:
      stale.errors.canError = True
    stale.errors.radarUnavailableTemporary = True
    return stale

  def _bosch_trackid(self, slot):
    # S1 trackId no-reuse: slot index -> stable-but-unique id (slot*STRIDE + incarnation). The
    # incarnation is bumped on every (re)birth / slot-reuse break, so a slot that hands off from object A
    # to object B presents radard a DIFFERENT trackId (capnp car.capnp:314 "no trackId reuse").
    return slot * BOSCH_RADAR_TRACKID_STRIDE + self._incarnation.get(slot, 0)

  def _bosch_clear_slot(self, slot):
    # S2 persist hysteresis on an absent/sentinel/wrong-tag/out-of-band cycle: decay the per-slot
    # confidence counter by 1 (floored at 0). The point + vRel history are RETAINED while the counter is
    # still above 0 (tolerates a single missed cycle without dropping a real lead); they are only dropped
    # once the counter floors at 0 (two clean missed cycles from a born point). The incarnation is NOT
    # touched here; it is bumped at (re)birth so the NEXT object to occupy this slot gets a fresh trackId.
    # pts/_hist are keyed by SLOT internally; the wire trackId lives on the point object's .trackId field.
    cnt = max(self._valid_cnt.get(slot, 0) - 1, 0)
    self._valid_cnt[slot] = cnt
    if cnt == 0:
      self.pts.pop(slot, None)
      self._kf.pop(slot, None)

  def _bosch_harvest_frames(self, updated_addrs):
    # D1: explode this batch's vl_all (per-signal aligned lists, one entry per parsed frame) into
    # per-slot frame dicts, in arrival order. Stateless per emit window: drained by _update_bosch.
    for ii in updated_addrs:
      if ii not in BOSCH_RADAR_HDR_MSGS:
        continue
      vl_all = self.rcp.vl_all[ii]
      names = list(vl_all)
      if not names or not vl_all[names[0]]:
        continue
      bucket = self._pending.setdefault(BOSCH_RADAR_HDR_MSGS.index(ii), [])
      for f in range(len(vl_all[names[0]])):
        bucket.append({name: vl_all[name][f] for name in names})
      del bucket[:-BOSCH_RADAR_PENDING_CAP]

  def _bosch_assemble_record(self, slot) -> BoschTrackRecord:
    # D1: demux the window's harvested frames by TRACK_TAG. The LAST 0x74 frame carries the freshest
    # kinematics; non-0x74 frames are retained as metadata (sub-frame decode is D2's RE campaign).
    rec = BoschTrackRecord(slot)
    for fr in self._pending.get(slot, ()):
      if int(fr['TRACK_TAG']) == BOSCH_RADAR_HDR_TAG:
        rec.range_frame = fr
        rec.recovered_clobber = False
      else:
        rec.meta_frames[int(fr['TRACK_TAG'])] = fr
        if rec.range_frame is not None:
          rec.recovered_clobber = True  # old last-write-wins read would have seen only this meta frame
    return rec

  def _update_bosch(self, updated_messages):
    # FINE per-track object table (0x280 block). Fixed slot map (0x280->slot0 ... 0x2DC->slot5).
    # RX-parse only; never takes 0x1DF / longitudinal authority, so factory AEB/CMBS stays fully live.
    # Up to 6 RadarPoints are emitted; radard selects leadOne/leadTwo (we do NOT pre-select).
    #
    # trackId is slot*STRIDE + incarnation (S1, no-reuse). vRel is NOT published on these frames (rlog-
    # confirmed b4:b5 is azimuth, not range-rate), so it is DERIVED per-SLOT as d(dRel)/dt across cycles
    # (per-slot history below). yRel is the trig projection of (dRel, azimuth) -- b4:b5 = azimuth, scale
    # rlog-regressed to ~0.001 deg/LSB (MEDIUM on the exact value, HIGH on the identity; absolute
    # boresight zero still unpinned). Range scale/offset live in the DBC.
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    # Clock from the CANParser frame timestamps (replay-safe: advances with rlog/replay time, not wall
    # clock). Used for the per-track vRel dt. The hard staleness gate (trigger 0x2DC stops arriving) lives
    # in update() above; here we only handle per-slot presence/sentinel within an emit cycle.
    now = self.rcp._last_update_nanos
    # This method only runs when the trigger header (sweep terminator 0x2DC) was present this cycle -> mark
    # it seen so the staleness fallback in update() can detect when the trigger later goes quiet.
    self._last_trigger_nanos = now

    # S3(b) CNTR-stall plausibility: track the trigger slot's CNTR. If it freezes while can_valid the
    # source is present-but-frozen -> radarFault. The trigger (0x2DC) is guaranteed present this cycle (it
    # is what gated us into _update_bosch), and every header carries the same 8-bit CNTR (DBC bit 63).
    trig_cpt = self.rcp.vl[self.trigger_msg]
    cur_cntr = int(trig_cpt['CNTR'])
    if self._last_cntr is not None and cur_cntr == self._last_cntr:
      self._cntr_stall += 1
    else:
      self._cntr_stall = 0
    self._last_cntr = cur_cntr
    if self.rcp.can_valid and self._cntr_stall >= BOSCH_RADAR_CNTR_STALL_CYCLES:
      ret.errors.radarFault = True

    for ii in BOSCH_RADAR_HDR_MSGS:
      slot = BOSCH_RADAR_HDR_MSGS.index(ii)

      # Stale-track aging: a header absent this trigger cycle is not a live track -> decay confidence and
      # drop the point/history so a long-absent slot cannot linger as a frozen point (S2 tolerates a
      # single missed cycle: the point is only fully dropped once the counter floors at 0).
      if ii not in updated_messages:
        self._bosch_clear_slot(slot)
        continue

      # D1: tag-demux over ALL frames this window (vl is last-write-wins; a metadata sub-frame landing
      # after the 0x74 range-carrier used to mis-clear a LIVE track here). Only a window with NO
      # range-carrier at all is a genuine miss.
      rec = self._bosch_assemble_record(slot)
      if rec.recovered_clobber:
        self._clobber_recovered += 1
      cpt = rec.range_frame
      if cpt is None:
        self._bosch_clear_slot(slot)
        continue

      # Sentinel skip (broadened): idle strength, unset range, or saturation rail -> not an active track.
      range_raw = int(cpt['RANGE_RAW'])
      if (int(cpt['STRENGTH']) == BOSCH_RADAR_STRENGTH_IDLE
          or range_raw == BOSCH_RADAR_RAW_UNSET
          or range_raw >= BOSCH_RADAR_RAW_SAT):
        self._bosch_clear_slot(slot)
        continue

      dRel = cpt['RANGE']  # meters, DBC-scaled (0.00357*raw - 3.0). Calibrated cross-car.

      # S3(a) plausibility: a tag-passing, non-sentinel frame whose decoded dRel lands outside the
      # physical band is a decode/scale inconsistency, not a real object -> flag wrongConfig and skip the
      # point (do NOT emit a bad lead). RX-only error annotation; no authority change.
      if not (BOSCH_RADAR_RANGE_MIN <= dRel <= BOSCH_RADAR_RANGE_MAX):
        ret.errors.wrongConfig = True
        self._bosch_clear_slot(slot)
        continue

      # S1 (re)birth detection: a slot whose confidence counter was floored at 0 BEFORE this cycle is a
      # fresh occupant -> bump its incarnation so the trackId it will be published under does not reuse
      # the prior occupant's, and clear its vRel baseline so the first derived sample is clean. Detect
      # rebirth on the 0->1 confidence transition (first SIGHTING), NOT on point absence -- with S2 a slot
      # is sighted for BORN_CYCLES-1 cycles before it is ever published, so point-absence would mis-fire.
      was_vacant = self._valid_cnt.get(slot, 0) == 0
      if was_vacant:
        self._incarnation[slot] = self._incarnation.get(slot, 0) + 1
        self._kf.pop(slot, None)

      # S2 birth/persist hysteresis: this is a valid range-carrier frame -> +1 (saturating). A point is
      # only emitted once the counter reaches BORN_CYCLES (debounces a 1-frame glitch into a phantom).
      self._valid_cnt[slot] = min(self._valid_cnt.get(slot, 0) + 1, BOSCH_RADAR_VALID_CAP)

      # R1: per-slot [range, range_rate] KF on the raw range (replaces d(dRel)/dt + EMA). The filter
      # owns the S6 contracts (dt<=0 skip, long-gap reseed); the BREAK return keeps the S1 slot-reuse
      # semantics: bump the incarnation so radard sees a NEW trackId, drop the stale point so it is
      # re-created under that id this cycle, and reseed the filter at the new object's range.
      kf = self._kf.get(slot)
      if kf is None:
        kf = self._kf[slot] = _SlotRangeKF(dRel, now)
      else:
        status = kf.update(dRel, now)
        if status == _KF_BREAK:
          self._incarnation[slot] = self._incarnation.get(slot, 0) + 1
          self.pts.pop(slot, None)
          kf = self._kf[slot] = _SlotRangeKF(dRel, now)
      # S5: range_rate is an estimate until the filter has absorbed enough measurements -> NaN before
      vRel = kf.v if kf.converged else float('nan')

      # S2 gate: do not emit until the slot is confidently born.
      if self._valid_cnt[slot] < BOSCH_RADAR_BORN_CYCLES:
        # Not yet confident enough to publish; keep accumulating. Drop any stale point (there should be
        # none pre-birth) but retain the counter + baseline so the next valid cycle can promote it.
        self.pts.pop(slot, None)
        continue

      if slot not in self.pts:
        self.pts[slot] = structs.RadarData.RadarPoint()
        self.pts[slot].trackId = self._bosch_trackid(slot)
        self.pts[slot].yvRel = float('nan')

      self.pts[slot].dRel = dRel
      # yRel = lateral projection of the polar (range, azimuth) measurement. b4:b5 is AZIMUTH ANGLE
      # (offset-binary, center 0x8000), settled by the 2026-06-08 three-source rlog regression (see the
      # LAT_SCALE block above); LAT_RAW is already (b4b5 - 0x8000) per the DBC offset. left-positive:
      # right-of-center (LAT_RAW > 0) -> negative yRel (rlog-confirmed sign). Scale MEDIUM-confidence
      # (0.0009-0.001 deg/LSB band; absolute boresight zero still unpinned) but field identity is HIGH.
      az_deg = cpt['LAT_RAW'] * BOSCH_RADAR_LAT_SCALE_DEG_PER_LSB
      self.pts[slot].yRel = -dRel * sin(az_deg * pi / 180.0)
      self.pts[slot].vRel = vRel
      # R1: pack the KF's smoothed range-accel into aRel (NaN until the filter is converged AND has a
      # rate history). RX-only telemetry; the radard-side consumer is K5 (deferred).
      self.pts[slot].aRel = kf.a
      # S5 honest measured flag: vRel is a DERIVED estimate, so flag the point as an estimate (measured=
      # False) whenever vRel is not yet a valid derived value (first-sight/re-seed NaN). dRel/yRel are
      # real measurements, but the capnp measured bit is about point-as-measurement-vs-estimate, and our
      # headline kinematic (vRel) is derived -- True only once a stable derived vRel exists.
      self.pts[slot].measured = not math.isnan(vRel)

    # D1: the emit window is consumed; the next window's frames are harvested fresh from vl_all.
    self._pending.clear()
    ret.points = list(self.pts.values())
    return ret

  def _update(self, updated_messages):
    ret = structs.RadarData()

    for ii in sorted(updated_messages):
      cpt = self.rcp.vl[ii]
      if ii == 0x400:
        # check for radar faults
        self.radar_fault = cpt['RADAR_STATE'] != 0x79
        self.radar_wrong_config = cpt['RADAR_STATE'] == 0x69
      elif cpt['LONG_DIST'] < 255:
        if ii not in self.pts or cpt['NEW_TRACK']:
          self.pts[ii] = structs.RadarData.RadarPoint()
          self.pts[ii].trackId = self.track_id
          self.track_id += 1
        self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
        self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
        self.pts[ii].vRel = cpt['REL_SPEED']
        self.pts[ii].aRel = float('nan')
        self.pts[ii].yvRel = float('nan')
        self.pts[ii].measured = True
      else:
        if ii in self.pts:
          del self.pts[ii]

    if not self.rcp.can_valid:
      ret.errors.canError = True
    if self.radar_fault:
      ret.errors.radarFault = True
    if self.radar_wrong_config:
      ret.errors.wrongConfig = True

    ret.points = list(self.pts.values())

    return ret
