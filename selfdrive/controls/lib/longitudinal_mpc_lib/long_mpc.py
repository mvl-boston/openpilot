#!/usr/bin/env python3
import os
import math
import time
import numpy as np
from cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function, ModelConstants
from openpilot.selfdrive.controls.lib.nrdr_long_tune import LongTune, LOW_SPEED_JERK_BP

LEAD_T_IDXS_MODEL = np.array(ModelConstants.LEAD_T_IDXS)  # [0, 2, 4, 6, 8, 10]s

if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource
MPC_SOURCES = (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1, LongitudinalPlanSource.cruise)

X_DIM = 3
U_DIM = 1
PARAM_DIM = 6
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0
CRUISE_MIN_ACCEL = -1.2
CRUISE_MAX_ACCEL = 2.0
MIN_X_LEAD_FACTOR = 0.5

# --- M1/M2/M3 lead-consumption constants (radard Kalman state -> MPC) ---------------------
# All tunables route through self.tune.lead_consumption (see nrdr_long_tune.py). The constants
# here are fixed mechanism parameters, not live knobs.
M1_CROSSFADE_FRAMES = 5      # frames after a track-id change to crossfade vLeadK anchor in
M2_ALEAD_LP_TAU = 0.4        # s; ~0.4 Hz low-pass on aLeadK before the M2 ramp uses it
M2_TAU_DECAYED = 1.0         # aLeadTau below this => radard learned a sustained accel
B_EFF_MAX_BRAKE = 4.5        # hard ceiling on inflated effective braking (m/s^2)
M3_PERSIST_S = 0.8           # aLeadK must stay below the gate this long before inflating
M3_ENGAGE_HYST_S = 0.3       # extra engage delay (hysteresis) once persistence is met
M3_RELEASE_DECAY_S = 0.5     # b_eff decays to comfort_brake over this long after release
M3_TRACK_AGE_S = 0.5         # same radarTrackId must persist this long before inflating
M3_TAU_NORM = 1.5            # aLeadK_gated = aLeadK * (1 - aLeadTau / M3_TAU_NORM)


def lead_brake_gate(aLeadK, aLeadTau, alead_threshold):
  """Shared M2/M3 engagement signal off the KF accel (council invariant: one gate family).

  Returns (active, aLeadK_gated) where aLeadK_gated discounts the raw KF accel by how *fresh*
  the learned-accel filter still is (aLeadTau high => just-detected => trust less). `active` is
  True only when the discounted decel is below alead_threshold (a negative number).
  Non-finite inputs -> inactive, gated 0.0 (caller falls back to stock behavior).
  """
  if not (math.isfinite(aLeadK) and math.isfinite(aLeadTau)):
    return False, 0.0
  aLeadK_gated = aLeadK * (1.0 - min(max(aLeadTau, 0.0), M3_TAU_NORM) / M3_TAU_NORM)
  return (aLeadK_gated < alead_threshold), aLeadK_gated

def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.standard:
    return 1.0
  elif personality==log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.75
  elif personality==log.LongitudinalPersonality.standard:
    return 1.45
  elif personality==log.LongitudinalPersonality.aggressive:
    return 1.00
  else:
    raise NotImplementedError("Longitudinal personality not supported")

# NRDR live tune: int -> /data tune-file personality name, robust to enum reordering
PERSONALITY_NAMES = {
  int(log.LongitudinalPersonality.aggressive): "aggressive",
  int(log.LongitudinalPersonality.standard): "standard",
  int(log.LongitudinalPersonality.relaxed): "relaxed",
}

# NOTE: gen_long_ocp() bakes the default comfort_brake/stop_distance into the compiled OCP
# cost (desired_dist_comfort). The tune-file overrides below act on the runtime obstacle
# placement only (x_obstacle is a live solver parameter), so codegen output is unchanged.
def get_stopped_equivalence_factor(v_lead, comfort_brake=COMFORT_BRAKE):
  return (v_lead**2) / (2 * comfort_brake)

def get_safe_obstacle_distance(v_ego, t_follow, comfort_brake=COMFORT_BRAKE, stop_distance=STOP_DISTANCE):
  return (v_ego**2) / (2 * comfort_brake) + t_follow * v_ego + stop_distance

def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # states
  x_ego, v_ego, a_ego = SX.sym('x_ego'), SX.sym('v_ego'), SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  a_prev = SX.sym('a_prev')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  model.p = vertcat(a_min, a_max, x_obstacle, a_prev, lead_t_follow, lead_danger_factor)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model

def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  a_prev = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - a_prev,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, get_T_FOLLOW(), LEAD_DANGER_FACTOR])


  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.tune = LongTune(log_fn=cloudlog.warning)  # before reset(): reset -> set_weights reads it
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = LongitudinalPlanSource.cruise

  def reset(self):
    self.solver.reset()

    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N, 1))
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.j_solution = np.zeros(N)
    self.a_prev = np.array(self.a_solution)
    self.yref = np.zeros((N+1, COST_DIM))

    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))

    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.lead_xv_0 = np.zeros((N+1, 2))
    self.lead_xv_1 = np.zeros((N+1, 2))
    # Per-lead-slot state for M1/M2/M3 (slot 0 == leadOne, slot 1 == leadTwo).
    #   _m1_track_id   : last seen radarTrackId (detect track-birth/change)
    #   _m1_birth_cnt  : frames since the current track id appeared (crossfade vLeadK in)
    #   _m2_alead_lp   : low-pass filtered aLeadK
    #   _m3_below_t    : seconds aLeadK_gated has stayed below the gate (persistence)
    #   _m3_b_eff      : currently-applied effective braking (latched, decays on release)
    #   _m3_track_age  : seconds the current radarTrackId has persisted
    self._m1_track_id = [None, None]
    self._m1_birth_cnt = [0, 0]
    self._m2_alead_lp = [0.0, 0.0]
    self._m3_track_id = [None, None]
    self._m3_below_t = [0.0, 0.0]
    self._m3_b_eff = [0.0, 0.0]
    self._m3_track_age = [0.0, 0.0]
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # TODO don't hardcode A_CHANGE_COST idx
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):
    jerk_factor = get_jerk_factor(personality)
    # M5: per-personality split of the single jerk factor into (a_change, j_ego) cost
    # multipliers, live-tunable via /data/nrdr_long_tune.json; defaults reproduce stock.
    name = PERSONALITY_NAMES.get(int(personality))
    a_change_factor, j_ego_factor = self.tune.jerk_factors(name, (jerk_factor, jerk_factor))
    if self.tune.low_speed_jerk_scale > 1.0:
      low_speed_scale = float(np.interp(self.x0[1], LOW_SPEED_JERK_BP, [self.tune.low_speed_jerk_scale, 1.0]))
      a_change_factor *= low_speed_scale
      j_ego_factor *= low_speed_scale
    a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0
    cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, a_change_factor * a_change_cost, j_ego_factor * J_EGO_COST]
    constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:  # probably only helps if v < v_prev
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  def process_lead(self, model_lead, radar_lead, lead_idx=0):
    v_ego = self.x0[1]
    if model_lead.prob > 0.5 and radar_lead.status:
      lc = self.tune.lead_consumption
      d_rel = float(radar_lead.dRel)
      v_lead_raw = float(radar_lead.vLead)
      model_x = np.asarray(model_lead.x, dtype=np.float64)
      model_v = np.asarray(model_lead.v, dtype=np.float64)
      model_x_delta = model_x - model_x[0]
      model_v_delta = model_v - model_v[0]

      # ---- M1: anchor the lead-velocity trajectory at the KF estimate vLeadK -------------
      # vLeadK is the radard Kalman filter's h=0 lead speed (honest, not the double-derived,
      # lagged, quantized raw vLead). On radarless leads radarState is synthesized with
      # vLeadK==vLead (radard.get_RadarState_from_vision) so anchoring there is a provable
      # no-op. Falls back to raw vLead when the knob is off, on any non-finite KF field, or
      # via the |aLeadK| escape hatch (the KF speed can lag under hard transients).
      vLeadK = float(getattr(radar_lead, "vLeadK", v_lead_raw))
      aLeadK = float(getattr(radar_lead, "aLeadK", 0.0))
      aLeadTau = float(getattr(radar_lead, "aLeadTau", 1.0))
      track_id = getattr(radar_lead, "radarTrackId", -1)

      # track-birth / track-change handling: a fresh radarTrackId means the KF just seeded,
      # so its speed estimate is unsettled — use raw vLead for the first M1_CROSSFADE_FRAMES
      # then linearly crossfade the vLeadK anchor in. Tracked per lead slot.
      if track_id != self._m1_track_id[lead_idx]:
        self._m1_track_id[lead_idx] = track_id
        self._m1_birth_cnt[lead_idx] = 0
      else:
        self._m1_birth_cnt[lead_idx] = min(self._m1_birth_cnt[lead_idx] + 1, M1_CROSSFADE_FRAMES)
      birth_frac = self._m1_birth_cnt[lead_idx] / float(M1_CROSSFADE_FRAMES)  # 0 at birth -> 1

      kf_finite = math.isfinite(vLeadK) and math.isfinite(aLeadK) and math.isfinite(aLeadTau)
      anchor_on = lc["m1_anchor"] >= 0.5 and kf_finite
      if anchor_on:
        # escape hatch: |aLeadK| above the live threshold blends raw vLead back in (0..1).
        escape = lc["m1_alead_escape"]
        esc_blend = min(max((abs(aLeadK) - escape) / max(escape, 1e-3), 0.0), 1.0)
        # combined weight on vLeadK: 0 -> use raw vLead; 1 -> use vLeadK.
        kf_weight = birth_frac * (1.0 - esc_blend)
        anchor_v = kf_weight * vLeadK + (1.0 - kf_weight) * v_lead_raw
      else:
        anchor_v = v_lead_raw

      # ---- M2: aLeadK*aLeadTau exponential-decay velocity extrapolation -------------------
      # v(t) = anchor_v + model_v_delta*(1-w) + w*aLeadK_lp*tau*(1-exp(-t/tau))
      # x(t) gets the exact integral of that accel-velocity profile (not a t^2/2 stand-in):
      #   INT[0..t] tau*(1-exp(-s/tau)) ds = tau*(t - tau*(1-exp(-t/tau)))
      # w ramps 0->m2_w_max only when |aLeadK_lp|>deadband, aLeadTau decayed (sustained accel),
      # and dRel within the long-range ghost gate.
      t = LEAD_T_IDXS_MODEL
      w = 0.0
      x_accel_term = np.zeros_like(t)
      v_accel_term = np.zeros_like(t)
      if lc["m2_w_max"] > 0.0 and kf_finite:
        # ~0.4 Hz low-pass on aLeadK before use (per-slot state, advanced once per cycle).
        alpha = self.dt / (M2_ALEAD_LP_TAU + self.dt)
        self._m2_alead_lp[lead_idx] += alpha * (aLeadK - self._m2_alead_lp[lead_idx])
        aLeadK_lp = self._m2_alead_lp[lead_idx]
        # Shared gate (council invariant: M2 & M3 use one helper). The |aLeadK|>deadband
        # condition is the gate's `active` flag with threshold -deadband; M2 also extrapolates
        # accelerating leads, so accept either sign past the deadband.
        brake_active, _ = lead_brake_gate(aLeadK_lp, aLeadTau, -lc["m2_alead_deadband"])
        deadband_ok = brake_active or aLeadK_lp > lc["m2_alead_deadband"]
        tau_ok = aLeadTau < M2_TAU_DECAYED
        drel_ok = d_rel < lc["m2_drel_gate"]
        if deadband_ok and tau_ok and drel_ok:
          w = lc["m2_w_max"]
          tau = max(aLeadTau, 1e-3)
          decay = 1.0 - np.exp(-t / tau)
          v_accel_term = aLeadK_lp * tau * decay
          x_accel_term = aLeadK_lp * tau * (t - tau * decay)
      else:
        # keep the filter primed even when M2 is off so it has no engage transient.
        self._m2_alead_lp[lead_idx] = aLeadK if kf_finite else 0.0

      v_lead_traj = anchor_v + model_v_delta * (1.0 - w) + w * v_accel_term
      x_lead_traj = d_rel + model_x_delta * (1.0 - w) + w * x_accel_term
    else:
      # Fake a fast lead so MPC stays in the same mode.
      x_lead_traj = 50.0 + (v_ego + 10.0) * LEAD_T_IDXS_MODEL
      v_lead_traj = np.full_like(LEAD_T_IDXS_MODEL, v_ego + 10.0)

    # MPC won't converge on immediate crashes; lift h=0 to the minimum braking distance.
    v_lead_0 = v_lead_traj[0]
    min_x_lead = MIN_X_LEAD_FACTOR * (v_ego + v_lead_0) * (v_ego - v_lead_0) / (-ACCEL_MIN * 2)
    x_lead_traj[0] = max(x_lead_traj[0], min_x_lead)
    v_lead_traj = np.clip(v_lead_traj, 0.0, 1e8)

    x_lead_mpc = np.maximum.accumulate(np.interp(T_IDXS, LEAD_T_IDXS_MODEL, x_lead_traj))
    v_lead_mpc = np.interp(T_IDXS, LEAD_T_IDXS_MODEL, v_lead_traj)
    return np.column_stack((x_lead_mpc, v_lead_mpc))

  def lead_b_eff(self, radar_lead, comfort_brake, lead_idx):
    """M3: lead-decel-aware effective braking for the obstacle inflation (one-sided).

    Stock uses comfort_brake for every lead's stopping distance. When the KF says a tracked
    lead is braking harder than that, inflate the obstacle by using a larger b_eff so we react
    earlier. Returns b_eff in [comfort_brake, m3_b_eff_max(<=4.5)] — never below comfort_brake,
    so the worst case is slightly conservative. Pure pre-processing; the compiled OCP only ever
    sees x_obstacle. Stateful per lead slot (persistence + engage hysteresis + release decay +
    track-age gate), advanced once per 20 Hz cycle.
    """
    lc = self.tune.lead_consumption
    b_eff_max = lc["m3_b_eff_max"]
    dt = self.dt
    # M3 off (default) or no live radar lead -> reset slot state, no inflation.
    if b_eff_max <= comfort_brake or not getattr(radar_lead, "status", False):
      self._m3_below_t[lead_idx] = 0.0
      self._m3_b_eff[lead_idx] = 0.0
      self._m3_track_age[lead_idx] = 0.0
      return comfort_brake

    aLeadK = float(getattr(radar_lead, "aLeadK", 0.0))
    aLeadTau = float(getattr(radar_lead, "aLeadTau", 1.0))
    track_id = getattr(radar_lead, "radarTrackId", -1)

    # track-age gate: same radarTrackId must persist M3_TRACK_AGE_S before we trust its decel.
    if track_id == self._m3_track_id[lead_idx] and track_id not in (None, -1):
      self._m3_track_age[lead_idx] = min(self._m3_track_age[lead_idx] + dt, 1e6)
    else:
      self._m3_track_id[lead_idx] = track_id
      self._m3_track_age[lead_idx] = 0.0
    age_ok = self._m3_track_age[lead_idx] >= M3_TRACK_AGE_S

    active, aLeadK_gated = lead_brake_gate(aLeadK, aLeadTau, lc["m3_alead_gate"])

    # persistence: aLeadK_gated must stay below the gate for M3_PERSIST_S + engage hysteresis.
    if active and age_ok:
      self._m3_below_t[lead_idx] = min(self._m3_below_t[lead_idx] + dt, 1e6)
    else:
      self._m3_below_t[lead_idx] = 0.0
    engaged = self._m3_below_t[lead_idx] >= (M3_PERSIST_S + M3_ENGAGE_HYST_S)

    if engaged:
      # b_eff target from the gated decel, one-sided and capped.
      target = min(max(comfort_brake, -aLeadK_gated), b_eff_max, B_EFF_MAX_BRAKE)
      self._m3_b_eff[lead_idx] = max(self._m3_b_eff[lead_idx], target)
    else:
      # release: decay the latched b_eff back toward comfort_brake over M3_RELEASE_DECAY_S.
      if self._m3_b_eff[lead_idx] > comfort_brake:
        step = (b_eff_max - comfort_brake) * (dt / M3_RELEASE_DECAY_S)
        self._m3_b_eff[lead_idx] = max(comfort_brake, self._m3_b_eff[lead_idx] - step)
      else:
        self._m3_b_eff[lead_idx] = 0.0

    return max(comfort_brake, self._m3_b_eff[lead_idx])

  def update(self, v_cruise, modelV2, radarstate, personality=log.LongitudinalPersonality.standard):
    self.tune.refresh()
    t_follow = max(0.9, get_T_FOLLOW(personality) + self.tune.t_follow_offset(PERSONALITY_NAMES.get(int(personality))))
    comfort_brake = self.tune.comfort_brake
    v_ego = self.x0[1]
    model_leads = modelV2.leadsV3
    self.status = model_leads[0].prob > 0.5 or model_leads[1].prob > 0.5

    lead_xv_0 = self.process_lead(model_leads[0], radarstate.leadOne, 0)
    lead_xv_1 = self.process_lead(model_leads[1], radarstate.leadTwo, 1)
    self.lead_xv_0 = lead_xv_0
    self.lead_xv_1 = lead_xv_1

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    # M3: when the KF says a lead is braking harder than comfort_brake, use a larger effective
    # braking (one-sided inflation) so the stopped-equivalence pushes the obstacle nearer/earlier.
    b_eff_0 = self.lead_b_eff(radarstate.leadOne, comfort_brake, 0)
    b_eff_1 = self.lead_b_eff(radarstate.leadTwo, comfort_brake, 1)
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1], b_eff_0)
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1], b_eff_1)

    # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
    # when the leads are no factor.
    v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05)
    # TODO does this make sense when max_a is negative?
    v_upper = v_ego + (T_IDXS * CRUISE_MAX_ACCEL * 1.05)
    v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)
    cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow, comfort_brake, self.tune.stop_distance)

    x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
    self.source = MPC_SOURCES[np.argmin(x_obstacles[0])]

    self.yref[:,:] = 0.0
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,0] = ACCEL_MIN
    self.params[:,1] = ACCEL_MAX
    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.a_prev)
    self.params[:,4] = t_follow
    self.params[:,5] = LEAD_DANGER_FACTOR

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            model_leads[0].prob > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

  def run(self):
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.a_prev = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
