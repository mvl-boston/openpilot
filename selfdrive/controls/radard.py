#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car, custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D, get_kalman_gain

from opendbc.car import structs
from opendbc.car.honda.values import CAR as HONDA_CAR
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

# K1 (nrdrbranchdebug-86t.3): max consecutive unmeasured (placeholder-vRel) cycles the lead KF
# is allowed to coast (predict-only) before publishing falls back to raw ingestion values
UNMEASURED_COAST_CYCLES = 5

# K2 (nrdrbranchdebug-86t.5): one-smoother architecture for the Civic Bosch fine ingest. The R1
# source KF already delivers a low-noise vRel posterior; the stock K0/K1 gain tables bake in a
# generic R=1e3 and would silently re-create cascade smoothing on top of it (sim: 720 ms to see
# 50% of a -3 m/s^2 lead decel in aLeadK). DARE-solved gains for the actual noise instead.
# Sim receipts (2026-06-10, synthetic braking lead through R1 source KF -> this KF):
#   stock gains: aLeadK noise 0.10, 50%-decel-lag 12 sweeps (720 ms)
#   Q=[0,4.0] R=0.1: aLeadK noise 0.33, 50%-decel-lag 5 sweeps (300 ms)  <- shipped
CIVIC_BOSCH_LEAD_KF_Q = [0.0, 4.0]   # [v, a] process noise; accel-driven model
CIVIC_BOSCH_LEAD_KF_R = 0.1          # (m/s)^2; R1 posterior rate noise + v_ego noise
CIVIC_BOSCH_LEAD_KF_DT = 0.06        # s; true radar sweep cadence (updates are freshness-gated)

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float, civic_bosch_fine: bool = False):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    if civic_bosch_fine:
      # K2: gains conditioned on the R1 source-KF posterior noise (see constants above); the
      # stock tables below assume a raw, noisy measurement and would cascade-smooth.
      K = get_kalman_gain(dt, np.array(self.A), np.array([self.C]),
                          np.diag(CIVIC_BOSCH_LEAD_KF_Q), np.array([[CIVIC_BOSCH_LEAD_KF_R]]))
      self.K = [[float(K[0, 0])], [float(K[1, 0])]]
      return
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
    self.unmeasured_cnt = 0
    self.v_ego = 0.0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    # K1: never seed the KF with NaN (Bosch fine ingest publishes vRel=NaN on placeholder cycles;
    # a NaN-seeded KF1D is poisoned for the track's whole life — the e72fa4e failure class)
    if math.isnan(v_lead):
      v_lead = 0.0
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float, v_ego: float = 0.0):
    # K1: placeholder cycles on the Bosch fine ingest carry vRel=NaN, measured=False. Sanitize so
    # NaN can never reach the KF, the vision matcher, or the published radarState.
    measured = bool(measured) and not math.isnan(v_rel)
    if math.isnan(v_rel):
      v_rel = 0.0
      v_lead = v_ego

    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate
    self.v_ego = v_ego

    # computed velocity and accelerations
    # K1: only feed the KF real measurements; on placeholder cycles coast (predict-only) so a
    # braking lead's estimate keeps evolving instead of being dragged toward relative-stationary
    if self.cnt > 0:
      if measured:
        self.kf.update(self.vLead)
        self.unmeasured_cnt = 0
      else:
        self.unmeasured_cnt += 1
        if self.unmeasured_cnt <= UNMEASURED_COAST_CYCLES:
          v, a = float(self.kf.x[SPEED][0]), float(self.kf.x[ACCEL][0])
          self.kf.set_x([[v + self.K_A[0][1] * a], [a]])
    elif not measured:
      self.unmeasured_cnt += 1

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration (K1: only from measured cycles — placeholder vRel carries
    # no acceleration information)
    if measured:
      if abs(self.aLeadK) < 0.5:
        self.aLeadTau.x = _LEAD_ACCEL_TAU
      else:
        self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    # K1: on unmeasured cycles publish the KF estimate instead of the fake relative-stationary
    # sample (the MPC anchors on raw vLead in process_lead). After UNMEASURED_COAST_CYCLES
    # consecutive placeholder cycles the coasted estimate is stale — fall back to ingestion.
    if not self.measured and self.unmeasured_cnt <= UNMEASURED_COAST_CYCLES:
      v_lead_pub = self.vLeadK
      v_rel_pub = self.vLeadK - self.v_ego
    else:
      v_lead_pub = self.vLead
      v_rel_pub = self.vRel
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(v_rel_pub),
      "vLead": float(v_lead_pub),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


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


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, CP: structs.CarParams, CP_SP: structs.CarParamsSP, low_speed_override: bool = True) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)
  elif (track is None) and ready and (lead_msg.prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


def get_custom_yrel(CP: structs.CarParams, CP_SP: structs.CarParamsSP, lead_dict: dict[str, Any],
                    lead_msg: capnp._DynamicStructReader) -> dict[str, Any]:
  if CP.brand == "hyundai" and (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC or
                                CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
    lead_dict['yRel'] = float(-lead_msg.y[0])

  return lead_dict


class RadarD:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    self.CP = CP
    self.CP_SP = CP_SP

    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    # K2: Civic-Bosch-fine-conditioned gains + freshness gating; every other brand keeps the
    # stock table gains and stock cadence behavior (their tuning assumes both).
    self._civic_bosch_fine = (CP.brand == "honda" and not CP.radarUnavailable
                              and CP.carFingerprint == HONDA_CAR.HONDA_CIVIC_BOSCH)
    if self._civic_bosch_fine:
      self.kalman_params = KalmanParams(CIVIC_BOSCH_LEAD_KF_DT, civic_bosch_fine=True)
    else:
      self.kalman_params = KalmanParams(DT_MDL)
    self._last_tracks_frame = -1

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    # K2: freshness gate -- radard polls at 20 Hz (modelV2) but the radar sweeps at ~16.7 Hz, so the
    # same liveTracks sample can be consumed twice, double-absorbing one measurement into the lead KF.
    # Repeats are treated as unmeasured cycles (the K1 coast path). Civic-Bosch-fine only.
    radar_fresh = True
    if self._civic_bosch_fine:
      radar_fresh = sm.recv_frame['liveTracks'] != self._last_tracks_frame
      self._last_tracks_frame = sm.recv_frame['liveTracks']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured and radar_fresh] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3

    # K1: velocity seed for tracks born on an unmeasured (placeholder-vRel) cycle. Without this
    # the KF seeds at v_ego (relative-stationary) and the seed bias survives the measured gate.
    vision_seed = None
    if self.ready and len(leads_v3) and leads_v3[0].prob > .5 and len(leads_v3[0].v):
      vision_seed = (leads_v3[0].x[0] - RADAR_TO_CAMERA, self.v_ego + (leads_v3[0].v[0] - model_v_ego))

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        v_seed = v_lead
        unmeasured_birth = not bool(rpt[3]) or math.isnan(rpt[2])
        if unmeasured_birth and vision_seed is not None and abs(rpt[0] - vision_seed[0]) < max(0.25 * vision_seed[0], 5.0):
          v_seed = vision_seed[1]
        elif unmeasured_birth:
          v_seed = self.v_ego_hist[0]  # finite relative-stationary fallback (never NaN)
        self.tracks[ids] = Track(ids, v_seed, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], self.v_ego_hist[0])

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(leads_v3) > 1:
      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, self.CP, self.CP_SP, low_speed_override=True)
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, self.CP, self.CP_SP, low_speed_override=False)

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
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP, CP_SP, CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
