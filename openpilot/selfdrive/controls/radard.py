#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from openpilot.cereal import messaging, log, custom
from opendbc.car.structs import car

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D
from opendbc.car.honda.radar_interface import BOSCH_A_FREQ_HZ
from opendbc.car.honda.values import HONDA_BOSCH_A


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame
HONDA_BOSCH_A_RADAR_TS = 1.0 / BOSCH_A_FREQ_HZ
HONDA_BOSCH_A_LOW_SPEED_MIN_COUNT = 3
HONDA_BOSCH_A_CHALLENGER_STALE_CYCLES = 2
HONDA_BOSCH_A_GROSS_DISTANCE_STALE_CYCLES = 3
HONDA_BOSCH_A_GROSS_DISTANCE_M = 25.0


def is_bosch_a_radar_car(CP) -> bool:
  return CP.brand == "honda" and CP.carFingerprint in HONDA_BOSCH_A and not CP.radarUnavailable


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: bool = True,
             measurement_update: bool | None = None):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # `measurement_update` is separate from the published measured bit so legacy radar sources keep
    # their existing behaviour. Civic Bosch emits real measurements at ~15 Hz while radard is driven
    # at the ~20 Hz model rate; duplicate liveTracks payloads must not be absorbed twice.
    if measurement_update is None:
      # Preserve the historical Track.update behaviour for direct/legacy callers. The radar source
      # adapter supplies an explicit False only for a duplicate Civic Bosch payload.
      measurement_update = True

    # computed velocity and accelerations
    if measurement_update and self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    if measurement_update:
      # Learn if constant acceleration
      if abs(self.aLeadK) < 0.5:
        self.aLeadTau.x = _LEAD_ACCEL_TAU
      else:
        self.aLeadTau.update(0.0)

      self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "present": True,
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def vision_track_probability(track: Track, lead: capnp._DynamicStructReader, v_ego: float) -> float:
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  prob_d = laplacian_pdf(track.dRel, offset_vision_dist, lead.xStd[0])
  prob_y = laplacian_pdf(track.yRel, -lead.y[0], lead.yStd[0])
  prob_v = laplacian_pdf(track.vRel + v_ego, lead.v[0], lead.vStd[0])
  return prob_d * prob_y * prob_v


def honda_bosch_a_low_speed_radar_lead_sane(track: Track, v_ego: float) -> bool:
  """Require a few real Bosch sweeps before a radar-only low-speed takeover."""
  return track.cnt >= HONDA_BOSCH_A_LOW_SPEED_MIN_COUNT and track.potential_low_speed_lead(v_ego)


def track_matches_vision(track: Track, lead: capnp._DynamicStructReader, v_ego: float, *,
                         dist_scale: float, dist_floor: float, vel_limit: float,
                         y_std_scale: float, y_floor: float) -> bool:
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  dist_sane = abs(track.dRel - offset_vision_dist) < max(abs(offset_vision_dist) * dist_scale, dist_floor)
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < vel_limit) or (v_ego + track.vRel > 3)
  lat_sane = abs(track.yRel + lead.y[0]) < max(y_floor, y_std_scale * max(float(lead.yStd[0]), 0.2))
  return dist_sane and vel_sane and lat_sane


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track]):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track
  else:
    return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "modelProb": float(lead_prob),
    "present": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_custom_yrel(CP: structs.CarParams, CP_SP: structs.CarParamsSP, lead_dict: dict[str, Any],
                    lead_msg: capnp._DynamicStructReader) -> dict[str, Any]:
  if CP.brand == "hyundai" and (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC or
                                CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
    lead_dict['yRel'] = float(-lead_msg.y[0])

  return lead_dict


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, lead_prob: float, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
             low_speed_override: bool = True, preferred_track_id: int = -1,
             honda_bosch_a_radar: bool = False) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  lead_dict = {'present': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_prob)
  elif (track is None) and ready and (lead_prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)

  if low_speed_override:
    if honda_bosch_a_radar:
      low_speed_tracks = [c for c in tracks.values() if honda_bosch_a_low_speed_radar_lead_sane(c, v_ego)]
    else:
      low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]

    model_lead_available = ready and lead_prob > .5

    # Keep a previously selected Bosch radar track through ordinary model-probability fluctuations
    # when it is still coherent. If the model has a valid lead, the old track must still agree with
    # that lead; no model lead leaves the mature radar track eligible for continuity.
    if honda_bosch_a_radar:
      preferred_track = tracks.get(preferred_track_id)
      if (preferred_track is not None and honda_bosch_a_low_speed_radar_lead_sane(preferred_track, v_ego)):
        preferred_matches_model = (not model_lead_available or
                                   track_matches_vision(preferred_track, lead_msg, v_ego,
                                                        dist_scale=0.25, dist_floor=5.0,
                                                        vel_limit=10.0, y_std_scale=1.0, y_floor=1.0))
        preferred_is_current = (not lead_dict.get('present', False) or
                                lead_dict.get('radarTrackId', -1) == preferred_track_id or
                                (lead_dict.get('present', False) and not lead_dict.get('radar', False)))
        if preferred_is_current and preferred_matches_model:
          lead_dict = preferred_track.get_RadarState(lead_prob)

    def candidate_is_established(candidate: Track) -> bool:
      if not honda_bosch_a_radar:
        return True
      if candidate.cnt < HONDA_BOSCH_A_LOW_SPEED_MIN_COUNT:
        return False
      if not lead_dict.get('present', False):
        # A mature centered Bosch point may provide the radar-only low-speed lead.
        return True
      if lead_dict.get('radarTrackId', -1) == candidate.identifier:
        return True
      # Do not replace an established lead with an unrelated closer point when there is no model
      # evidence to arbitrate them. A different candidate may take over only after it agrees with
      # the available model lead; radar-only takeover remains possible when lead_dict is invalid.
      return (model_lead_available and
              track_matches_vision(candidate, lead_msg, v_ego,
                                   dist_scale=0.25, dist_floor=5.0,
                                   vel_limit=10.0, y_std_scale=1.0, y_floor=1.0))

    low_speed_tracks = [c for c in low_speed_tracks if candidate_is_established(c)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


class RadarD:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0,
               honda_bosch_a_radar: bool = False):
    self.CP = CP
    self.CP_SP = CP_SP

    self.current_time = 0.0
    self.honda_bosch_a_radar = honda_bosch_a_radar
    self.tracks: dict[int, Track] = {}
    # The lead KF consumes Bosch measurements at the physical radar cadence. Lead probability
    # filters, however, consume modelV2 leads every model cycle and must retain model-loop timing.
    kf_dt = HONDA_BOSCH_A_RADAR_TS if self.honda_bosch_a_radar else DT_MDL
    self.kalman_params = KalmanParams(kf_dt)
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]
    self.prev_lead_track_ids = [-1, -1]
    self.preferred_stale_track_ids = [-1, -1]
    self.preferred_challenger_stale_counts = [0, 0]
    self.preferred_gross_distance_stale_counts = [0, 0]

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1
    self._last_tracks_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

  def _reset_preferred_stale_evidence(self, lead_index: int, track_id: int = -1) -> None:
    self.preferred_stale_track_ids[lead_index] = track_id
    self.preferred_challenger_stale_counts[lead_index] = 0
    self.preferred_gross_distance_stale_counts[lead_index] = 0

  def _update_honda_bosch_a_preferred_staleness(self, lead_index: int, lead: capnp._DynamicStructReader,
                                                 lead_prob: float) -> None:
    if not self.honda_bosch_a_radar:
      return

    preferred_id = self.prev_lead_track_ids[lead_index]
    if self.preferred_stale_track_ids[lead_index] != preferred_id:
      self._reset_preferred_stale_evidence(lead_index, preferred_id)

    preferred_track = self.tracks.get(preferred_id)
    if preferred_id < 0 or preferred_track is None or not self.ready or lead_prob <= .5:
      self._reset_preferred_stale_evidence(lead_index, preferred_id)
      return

    strict_match = track_matches_vision(preferred_track, lead, self.v_ego,
                                        dist_scale=0.25, dist_floor=5.0,
                                        vel_limit=10.0, y_std_scale=1.0, y_floor=1.0)
    relaxed_match = track_matches_vision(preferred_track, lead, self.v_ego,
                                         dist_scale=0.40, dist_floor=8.0,
                                         vel_limit=13.0, y_std_scale=2.0, y_floor=1.5)

    # Arm A: a preferred track that no longer passes continuity may be stale when another live
    # track has a better association score. Clearing preference never selects that challenger;
    # the unchanged strict match path below remains the only way it can become a radar lead.
    if relaxed_match:
      self.preferred_challenger_stale_counts[lead_index] = 0
    else:
      best_track = max(self.tracks.values(), key=lambda candidate: vision_track_probability(candidate, lead, self.v_ego))
      preferred_score = vision_track_probability(preferred_track, lead, self.v_ego)
      best_score = vision_track_probability(best_track, lead, self.v_ego)
      if best_track.identifier != preferred_id and best_score > preferred_score:
        self.preferred_challenger_stale_counts[lead_index] += 1
      else:
        self.preferred_challenger_stale_counts[lead_index] = 0

    # Arm B: gross absolute range disagreement is independent evidence of staleness, but a strict
    # match is authoritative and resets the streak even when model uncertainty permits >25 m error.
    distance_mismatch = abs(preferred_track.dRel - (lead.x[0] - RADAR_TO_CAMERA))
    if strict_match:
      self.preferred_gross_distance_stale_counts[lead_index] = 0
    elif distance_mismatch > HONDA_BOSCH_A_GROSS_DISTANCE_M:
      self.preferred_gross_distance_stale_counts[lead_index] += 1
    else:
      self.preferred_gross_distance_stale_counts[lead_index] = 0

    challenger_stale = self.preferred_challenger_stale_counts[lead_index] >= HONDA_BOSCH_A_CHALLENGER_STALE_CYCLES
    distance_stale = self.preferred_gross_distance_stale_counts[lead_index] >= HONDA_BOSCH_A_GROSS_DISTANCE_STALE_CYCLES
    if challenger_stale or distance_stale:
      self.prev_lead_track_ids[lead_index] = -1
      self._reset_preferred_stale_evidence(lead_index)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    radar_fresh = True
    if self.honda_bosch_a_radar:
      radar_fresh = sm.recv_frame['radarTracks'] != self._last_tracks_frame
      self._last_tracks_frame = sm.recv_frame['radarTracks']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      measured = rpt[3] if not self.honda_bosch_a_radar else bool(rpt[3] and radar_fresh)
      # Non-Bosch sources retain the historical per-model-cycle update semantics. Only Honda Bosch-A
      # suppresses duplicate measurement updates when radarTracks has not advanced.
      measurement_update = True if not self.honda_bosch_a_radar else measured
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, measured, measurement_update)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        # Asymmetric filter on lead prob to keep lead when uncertain
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

        self._update_honda_bosch_a_preferred_staleness(i, leads_v3[i], self.lead_prob_filters[i].x)

      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, self.lead_prob_filters[0].x,
                                          self.CP, self.CP_SP, low_speed_override=True,
                                          preferred_track_id=self.prev_lead_track_ids[0],
                                          honda_bosch_a_radar=self.honda_bosch_a_radar)
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, self.lead_prob_filters[1].x,
                                          self.CP, self.CP_SP, low_speed_override=False,
                                          preferred_track_id=self.prev_lead_track_ids[1],
                                          honda_bosch_a_radar=self.honda_bosch_a_radar)

      for i, lead in enumerate((self.radar_state.leadOne, self.radar_state.leadTwo)):
        if lead.present and getattr(lead, "radar", False):
          track_id = int(getattr(lead, "radarTrackId", -1))
          if track_id != self.prev_lead_track_ids[i]:
            self._reset_preferred_stale_evidence(i, track_id)
          self.prev_lead_track_ids[i] = track_id
        elif (not lead.present) or (self.prev_lead_track_ids[i] not in self.tracks):
          self.prev_lead_track_ids[i] = -1
          self._reset_preferred_stale_evidence(i)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  cloudlog.info("radard is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(Params().get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("radard got CarParamsSP")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  honda_bosch_a_radar = is_bosch_a_radar_car(CP)
  RD = RadarD(CP, CP_SP, CP.radarDelay, honda_bosch_a_radar=honda_bosch_a_radar)

  while 1:
    sm.update()

    RD.update(sm, sm['radarTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
