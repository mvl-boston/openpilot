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


# 36802-TBA Bosch radar coarse selected-lead (0x2C8/0x2C9).
# IMPORTANT: these object frames are physically on openpilot CanBus.camera (rlog src=2, confirmed
# across 6 routes), NOT CanBus.radar (bus 0). The Bus.radar key below is only the DBC-name lookup;
# the parser's CAN bus is CanBus(CP).camera. Reconfirm the bus with a read-only sniff before trusting dRel.
BOSCH_RADAR_MSGS = [0x2C8, 0x2C9]


def _create_bosch_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None
  messages = [(m, 20) for m in BOSCH_RADAR_MSGS]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, CanBus(CP).camera)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.track_id = 0
    self.radar_fault = False
    self.radar_wrong_config = False
    self.radar_off_can = CP.radarUnavailable

    # Bosch coarse selected-lead (Honda Civic Bosch, 36802-TBA) vs the legacy Nidec path
    self.bosch_radar = CP.carFingerprint == CAR.HONDA_CIVIC_BOSCH and Bus.radar in DBC[CP.carFingerprint]

    if self.radar_off_can:
      self.rcp = None
      self.trigger_msg = 0x445
    elif self.bosch_radar:
      self.rcp = _create_bosch_can_parser(CP)
      self.trigger_msg = 0x2C9
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
      return None

    rr = self._update_bosch(self.updated_messages) if self.bosch_radar else self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr

  def _update_bosch(self, updated_messages):
    # Chrysler-style fixed address->trackId map (0x2C8 -> 0, 0x2C9 -> 1). RX-parse only; never takes
    # 0x1DF / longitudinal authority, so factory AEB/CMBS stays fully live. Always returns a RadarData
    # (canError set on can_valid loss) -- it returns None only on the trigger short-circuit above.
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for ii in BOSCH_RADAR_MSGS:
      trackId = BOSCH_RADAR_MSGS.index(ii)

      # Stale-track aging (A7): a frame absent this trigger cycle is not a live track -> drop it,
      # so a long-absent 0x2C8 cannot linger as a frozen point (no conflate-replay dependence).
      if ii not in updated_messages:
        self.pts.pop(trackId, None)
        continue

      cpt = self.rcp.vl[ii]

      # No-target sentinel: B0 == 0xFF (raw, scale-independent). Clear the track.
      if cpt['LONG_DIST_HI'] == 0xFF:
        self.pts.pop(trackId, None)
        continue

      if trackId not in self.pts:
        self.pts[trackId] = structs.RadarData.RadarPoint()
        self.pts[trackId].trackId = trackId
        self.pts[trackId].aRel = float('nan')
        self.pts[trackId].yvRel = float('nan')

      self.pts[trackId].dRel = cpt['LONG_DIST']  # m from front of car (PLACEHOLDER scale until R0 cal A4)
      self.pts[trackId].yRel = -cpt['LAT_DIST']  # car-frame y, left positive (PLACEHOLDER scale until A4)
      self.pts[trackId].vRel = cpt['REL_SPEED']  # m/s, closing negative; 0.25 LSB locked, SIGN pending A4 verify
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
