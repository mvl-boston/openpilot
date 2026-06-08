#!/usr/bin/env python3
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

# Lateral scale is NOT pinned (CONFIRM-REPORT §1). yRel sign + offset-binary center are correct; the
# magnitude is a placeholder. TODO: DO_NOT_TRUST yRel magnitude until a controlled lateral cal pins this.
BOSCH_RADAR_LAT_SCALE = 0.001  # PLACEHOLDER m/LSB -- exercises sign/center plumbing only

# Staleness gate: if no fresh 0x280 header is seen for this long, clear all points and return an EMPTY
# RadarData (not None) so radard drops the lead within a cycle (no frozen phantom). RadarPoints carry no
# per-point monotime, so radard trusts points unconditionally -- a stalled source must be cleared here.
BOSCH_RADAR_STALE_S = 0.15  # ~3 missed 20 Hz frames


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

    # Per-track history for vRel derivation: trackId -> (last_dRel_m, last_seen_nanos).
    self._hist: dict[int, tuple[float, int]] = {}
    # Parser-clock nanos of the last cycle the trigger header (0x280) was emitted on; -1 = never. Used
    # by the staleness gate (compared against the parser's last-update clock). Tracked here rather than
    # reading rcp.ts_nanos so a frame at absolute t=0 (synthetic/replay start) isn't mistaken for "never".
    self._last_trigger_nanos = -1

    if self.radar_off_can:
      self.rcp = None
      self.trigger_msg = 0x445
    elif self.bosch_radar:
      self.rcp = _create_bosch_can_parser(CP)
      # Slot-0 header (0x280) is the most reliably-present track -> use it as the cadence trigger.
      self.trigger_msg = 0x280
    else:
      self.rcp = _create_nidec_can_parser(CP.carFingerprint)
      self.trigger_msg = 0x445
    self.updated_messages = set()

  def update(self, can_strings):
    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      # Staleness fallback (Bosch fine only): the trigger header (0x280) drives the normal 20 Hz emit,
      # but if it goes quiet while the parser keeps running we must still publish an EMPTY RadarData so
      # radard drops the stale lead (no frozen phantom). Compare the parser's last-update clock to the
      # last cycle 0x280 was emitted -- both from the (replay-safe) CAN frame clock.
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
    # clock so we emit the empty data exactly once until 0x280 returns.
    self.pts.clear()
    self._hist.clear()
    self._last_trigger_nanos = -1
    stale = structs.RadarData()
    if not self.rcp.can_valid:
      stale.errors.canError = True
    stale.errors.radarUnavailableTemporary = True
    return stale

  def _update_bosch(self, updated_messages):
    # FINE per-track object table (0x280 block). Fixed slot->trackId map (0x280->0 ... 0x2DC->5).
    # RX-parse only; never takes 0x1DF / longitudinal authority, so factory AEB/CMBS stays fully live.
    # Up to 6 RadarPoints are emitted; radard selects leadOne/leadTwo (we do NOT pre-select).
    #
    # vRel is NOT published on these frames, so it is DERIVED per-track as d(dRel)/dt across cycles
    # (per-slot history below). yRel sign/center are correct but its magnitude is a placeholder
    # (LAT_SCALE NOT pinned). Range scale/offset live in the DBC (0.00357 m/unit, offset -3.0).
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    # Clock from the CANParser frame timestamps (replay-safe: advances with rlog/replay time, not wall
    # clock). Used for the per-track vRel dt. The hard staleness gate (0x280 stops arriving) lives in
    # update() above; here we only handle per-slot presence/sentinel within an emit cycle.
    now = self.rcp._last_update_nanos
    # This method only runs when the trigger header (0x280) was present this cycle -> mark it seen so the
    # staleness fallback in update() can detect when 0x280 later goes quiet.
    self._last_trigger_nanos = now

    for ii in BOSCH_RADAR_HDR_MSGS:
      trackId = BOSCH_RADAR_HDR_MSGS.index(ii)

      # Stale-track aging: a header absent this trigger cycle is not a live track -> drop it (and its
      # vRel history) so a long-absent slot cannot linger as a frozen point.
      if ii not in updated_messages:
        self.pts.pop(trackId, None)
        self._hist.pop(trackId, None)
        continue

      cpt = self.rcp.vl[ii]

      # Gate b1==0x74: only the range-carrier header sub-frame carries RANGE. Any other tag is a
      # non-range sub-frame mux'd onto this ID -> skip (clear the slot; do not read RANGE).
      if int(cpt['TRACK_TAG']) != BOSCH_RADAR_HDR_TAG:
        self.pts.pop(trackId, None)
        self._hist.pop(trackId, None)
        continue

      # Sentinel skip (broadened): idle strength, unset range, or saturation rail -> not an active track.
      range_raw = int(cpt['RANGE_RAW'])
      if (int(cpt['STRENGTH']) == BOSCH_RADAR_STRENGTH_IDLE
          or range_raw == BOSCH_RADAR_RAW_UNSET
          or range_raw >= BOSCH_RADAR_RAW_SAT):
        self.pts.pop(trackId, None)
        self._hist.pop(trackId, None)
        continue

      dRel = cpt['RANGE']  # meters, DBC-scaled (0.00357*raw - 3.0). Calibrated cross-car.

      # vRel derived as d(dRel)/dt per track (closing negative). NaN on first sight or non-advancing
      # clock. Light EMA smoothing (alpha=0.5) tames LSB quantization noise at the cost of a little lag.
      vRel = float('nan')
      prev = self._hist.get(trackId)
      if prev is not None:
        last_dRel, last_nanos = prev
        dt = (now - last_nanos) * 1e-9
        if dt > 0:
          raw_vRel = (dRel - last_dRel) / dt
          # If we already have a prior vRel on the point, blend; else seed with the raw estimate.
          prev_pt = self.pts.get(trackId)
          if prev_pt is not None and prev_pt.vRel == prev_pt.vRel:  # not NaN
            vRel = 0.5 * prev_pt.vRel + 0.5 * raw_vRel
          else:
            vRel = raw_vRel
      self._hist[trackId] = (dRel, now)

      if trackId not in self.pts:
        self.pts[trackId] = structs.RadarData.RadarPoint()
        self.pts[trackId].trackId = trackId
        self.pts[trackId].aRel = float('nan')
        self.pts[trackId].yvRel = float('nan')

      self.pts[trackId].dRel = dRel
      # yRel left-positive; offset-binary already centered in the DBC (raw 0x8000 -> 0). LAT_SCALE is a
      # PLACEHOLDER -- DO_NOT_TRUST yRel magnitude until a controlled lateral cal pins m/LSB.
      self.pts[trackId].yRel = -cpt['LAT_RAW'] * BOSCH_RADAR_LAT_SCALE
      self.pts[trackId].vRel = vRel
      self.pts[trackId].measured = True

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
