#!/usr/bin/env python3
"""NRDR live longitudinal tune — /data JSON sidecar (no params_keys.h key, no recompile).

Transport layer for live-tuning hardcoded planner/MPC constants (beads M9, nrdrbranchdebug-7u2.1).
A JSON file at /data/nrdr_long_tune.json is polled on the existing live-param cadence (~1 Hz).
Missing or corrupt file == compiled defaults (provable no-op). Every field is hard-clamped in
code; non-finite values are rejected. Writers must use write_tune() (write-temp-then-rename) so
the 20 Hz planner can never read a half-written file.

Consumers:
  selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py  (comfort_brake, stop_distance,
      t_follow_offsets, jerk_factors, low_speed_jerk_scale, lead_consumption)
  selfdrive/controls/lib/longitudinal_planner.py           (a_cruise_max_scale)

lead_consumption (M1/M2/M3 — long_mpc consuming radard's Kalman-filtered lead state). This
section supersedes the old reserved obstacle_inflation_gains block (M3's b_eff lives here now).
Every field below is a PROVABLE NO-OP at its default EXCEPT m1_anchor, which defaults ON: the
radard KF (K1) landed, so vLeadK is the honest h=0 lead-speed signal and anchoring there is the
correct baseline rather than the double-derived raw vLead.
  m1_anchor          [0,1] default 1 — anchor the MPC lead-velocity trajectory at vLeadK
                     instead of raw vLead. Default ON (honest KF signal). Set 0 to revert to
                     stock raw-vLead anchoring. On radarless leads vLeadK==vLead so it is a
                     no-op there regardless.
  m1_alead_escape    [0.5, 2.0] default 1.0 — |aLeadK| escape hatch: above this the anchor
                     blends back toward raw vLead (during hard transients the KF speed can lag,
                     so trust the raw double-derivation more).
  m2_w_max           [0.0, 1.0] default 0.0 (M2 OFF) — peak weight of the aLeadK*aLeadTau
                     exponential-decay velocity extrapolation blended into v_lead_traj. 0 keeps
                     the trajectory shape purely model-derived (stock).
  m2_alead_deadband  [0.3, 1.0] default 0.5 — |aLeadK| below this contributes no M2 ramp.
  m2_drel_gate       [20, 80] default 40 — leads farther than this (m) get no M2 extrapolation
                     (guards against long-range radar ghosts).
  m3_b_eff_max       [2.5, 4.5] default 2.5 (M3 OFF) — max effective braking (m/s^2) used to
                     inflate a decelerating lead's obstacle. Default 2.5 == compiled
                     comfort_brake, so clip(b_eff, comfort_brake, 2.5) can never exceed
                     comfort_brake => zero inflation.
  m3_alead_gate      [-2.0, -0.3] default -0.5 — aLeadK must fall below this (m/s^2) before M3
                     considers the lead "braking" and eligible for obstacle inflation.

stopping (Bundle D / L2 - longcontrol two-phase stopping shape; consumed by
selfdrive/controls/lib/longcontrol.py). Every field is live-tunable; the hard -1.0
minimum-hold floor is CODE (STOPPING_HARD_HOLD_FLOOR in longcontrol.py), NOT a tune field. The
whole section is INERT above v_ego_stopping (highway bit-identity), so these knobs only move
sub-walking-pace stop feel. Defaults are the council-shipped Bundle D shape; l2_enable defaults ON.
  l2_enable          [0,1] default 1 - >=0.5 routes longcontrol's stopping state through the
                     two-phase shape; <0.5 reverts to the stock monotonic ramp to stop_accel.
  hold_accel         [-1.0, -0.3] default -0.6 - gentle Phase-A hold the ramp targets while
                     still rolling (vEgo > phase_switch_v); removes head-bob at the stop. Also
                     the base of the Phase-B pitch-aware floor.
  phase_switch_v     [0.05, 0.5] default 0.15 - vEgo (m/s) below which true standstill is
                     declared and the ramp switches from hold_accel to the pitch-aware floor.
  proximity_scale_m  [2.0, 20.0] default 8.0 - dRel (m) at/above which the proximity rate term
                     saturates to 1.0; below it the ramp slows proportionally. Inert (==1.0)
                     when there is no valid lead.
  pitch_margin       [0.0, 2.0] default 1.0 - multiplier on the grade-compensation term
                     (-g*sin(pitch)) added to hold_accel for the Phase-B floor. 0 disables
                     pitch awareness (floor == hold_accel, still hard-capped at -1.0).

Field semantics (what each knob actually moves — the OCP cost keeps the compiled
COMFORT_BRAKE=2.5 / STOP_DISTANCE=6.0 inside desired_dist_comfort; these knobs act on the
runtime obstacle placement, which is a live solver parameter):
  comfort_brake   [2.0, 3.0]  default 2.5 — divisor of the lead stopped-equivalence factor.
                  LOWER => bigger stopped-equivalence credit for the lead's own braking =>
                  tighter steady-state gap to the actual lead
                  (gap_ss = t_follow*v + 6.0 + v^2*(1/5 - 1/(2*cb))). Also used for the
                  cruise fake obstacle so Python-side math stays self-consistent.
  stop_distance   [4.5, 7.5]  default 6.0 — cruise-obstacle standoff only (the lead-follow
                  stop distance term is compiled into the OCP cost; do not expect this field
                  alone to move stopping gap behind a lead).
  a_cruise_max_scale  4 floats [0.5, 1.5] default 1.0 — per-breakpoint scale on
                  A_CRUISE_MAX_VALS [2.0,1.6,0.8,0.6] @ [0,10,25,40] m/s; result is capped
                  at ACCEL_MAX by the consumer.
  t_follow_offsets    {aggressive|standard|relaxed: [-0.35, 0.5]} default 0.0 — added to
                  get_T_FOLLOW(); consumer floors final t_follow at 0.9 s.
  jerk_factors        {personality: {a_change: [0.2,3.0], j_ego: [0.2,3.0]}} — M5 split of
                  the single jerk factor into (a_change_cost, j_ego_cost) multipliers.
                  Unset side falls back to the compiled get_jerk_factor() value.
  low_speed_jerk_scale [1.0, 2.0] default 1.0 — extra comfort multiplier on both jerk
                  weights, interpolated to 1.0 over LOW_SPEED_JERK_BP (full effect at/below
                  2 m/s, none at/above 5 m/s).

Suggested first tuning session (M5 council values — apply LIVE via CLI, not as code defaults):
  python3 nrdr_long_tune.py set jerk_factors.relaxed.a_change=1.4 jerk_factors.relaxed.j_ego=1.5 \
      jerk_factors.aggressive.a_change=0.6 jerk_factors.aggressive.j_ego=0.5 low_speed_jerk_scale=1.3
"""
import json
import math
import os

NRDR_LONG_TUNE_PATH = "/data/nrdr_long_tune.json"

PERSONALITIES = ("aggressive", "standard", "relaxed")
LOW_SPEED_JERK_BP = (2.0, 5.0)  # m/s; consumer interpolates scale -> 1.0 across this range

# scalar fields: name -> (default, lo, hi)
_SCALARS = {
  "comfort_brake": (2.5, 2.0, 3.0),
  "stop_distance": (6.0, 4.5, 7.5),
  "low_speed_jerk_scale": (1.0, 1.0, 2.0),
}
_A_CRUISE_SCALE_N = 4
_A_CRUISE_SCALE_CLAMP = (0.5, 1.5)
_T_FOLLOW_OFFSET_CLAMP = (-0.35, 0.5)
_JERK_FACTOR_CLAMP = (0.2, 3.0)
_JERK_KEYS = ("a_change", "j_ego")
# M1/M2/M3 lead-consumption knobs (supersedes the old reserved obstacle_inflation_gains block).
# Every entry is (default, lo, hi). Defaults are provable no-ops EXCEPT m1_anchor (default ON).
_LEAD_CONSUMPTION_CLAMPS = {
  "m1_anchor": (1.0, 0.0, 1.0),         # bool-ish; >=0.5 => anchor at vLeadK
  "m1_alead_escape": (1.0, 0.5, 2.0),   # |aLeadK| above this blends back to raw vLead
  "m2_w_max": (0.0, 0.0, 1.0),          # 0 => M2 off (no accel-extrapolation in v_lead_traj)
  "m2_alead_deadband": (0.5, 0.3, 1.0),
  "m2_drel_gate": (40.0, 20.0, 80.0),
  "m3_b_eff_max": (2.5, 2.5, 4.5),      # 2.5 == comfort_brake default => M3 off (no inflation)
  "m3_alead_gate": (-0.5, -2.0, -0.3),
}

# Bundle D / L2 stopping-shape knobs. Every entry is (default, lo, hi). Defaults are the
# council-shipped Bundle D shape (l2_enable ON), NOT stock — but the section is INERT above
# v_ego_stopping, so highway behavior stays bit-identical regardless. The hard -1.0 minimum
# hold floor is a CODE constant (longcontrol.STOPPING_HARD_HOLD_FLOOR), never a tune field.
_STOPPING_CLAMPS = {
  "l2_enable": (1.0, 0.0, 1.0),          # >=0.5 => two-phase shape; <0.5 => stock monotonic ramp
  "hold_accel": (-0.6, -1.0, -0.3),      # gentle Phase-A hold; safe envelope keeps it >= -1.0
  "phase_switch_v": (0.15, 0.05, 0.5),   # vEgo below which standstill is declared
  "proximity_scale_m": (8.0, 2.0, 20.0), # dRel at which the proximity rate term saturates to 1.0
  "pitch_margin": (1.0, 0.0, 2.0),       # multiplier on the grade-compensation term
}


def _clampf(value, lo, hi):
  v = float(value)
  if not math.isfinite(v):
    raise ValueError("non-finite value")
  return min(max(v, lo), hi)


class LongTune:
  """Polls the tune file and exposes clamped values; falls back to compiled defaults on any error."""

  REFRESH_INTERVAL = 20  # refresh() calls between stat() checks (~1 s at the 20 Hz planner rate)

  def __init__(self, path=NRDR_LONG_TUNE_PATH, log_fn=None):
    self._path = path
    self._log_fn = log_fn
    self._frame = 0
    self._file_sig = ()  # () = never checked; None = file absent; (mtime_ns, size) = loaded
    self._last_log = None
    self._set_defaults()

  # -- consumer accessors ----------------------------------------------------
  def t_follow_offset(self, personality_name):
    return self.t_follow_offsets.get(personality_name, 0.0)

  def jerk_factors(self, personality_name, default):
    """(a_change_factor, j_ego_factor); each side independently falls back to `default`."""
    ov = self._jerk_overrides.get(personality_name)
    if not ov:
      return default
    return (ov.get("a_change", default[0]), ov.get("j_ego", default[1]))

  # -- lifecycle ---------------------------------------------------------------
  def _set_defaults(self):
    for name, (default, _, _) in _SCALARS.items():
      setattr(self, name, default)
    self.a_cruise_max_scale = (1.0,) * _A_CRUISE_SCALE_N
    self.t_follow_offsets = {}
    self._jerk_overrides = {}
    # always fully populated with defaults so consumers read scalars directly (no .get fallbacks)
    self.lead_consumption = {k: d for k, (d, _, _) in _LEAD_CONSUMPTION_CLAMPS.items()}
    self.stopping = {k: d for k, (d, _, _) in _STOPPING_CLAMPS.items()}
    self.active = False

  def refresh(self):
    """Call every frame; stats the file ~1 Hz and re-reads only when (mtime, size) changes."""
    if self._frame % self.REFRESH_INTERVAL == 0:
      self._check_file()
    self._frame += 1

  def _check_file(self):
    try:
      st = os.stat(self._path)
      sig = (st.st_mtime_ns, st.st_size)
    except OSError:
      sig = None
    if sig == self._file_sig:
      return
    was_loaded = self._file_sig not in ((), None)
    self._file_sig = sig
    if sig is None:
      self._set_defaults()
      if was_loaded:
        self._warn("tune file removed; reverted to compiled defaults")
      return
    try:
      with open(self._path, encoding="utf-8") as f:
        data = json.load(f)
      if not isinstance(data, dict):
        raise ValueError("top level is not an object")
    except (OSError, ValueError) as e:
      self._set_defaults()
      self._warn(f"unreadable tune file ({e}); using compiled defaults")
      return
    self._apply(data)

  def _apply(self, data):
    self._set_defaults()
    bad = []
    for name, (default, lo, hi) in _SCALARS.items():
      if name in data:
        try:
          setattr(self, name, _clampf(data[name], lo, hi))
        except (TypeError, ValueError):
          bad.append(name)
    if "a_cruise_max_scale" in data:
      try:
        raw = data["a_cruise_max_scale"]
        if not isinstance(raw, (list, tuple)) or len(raw) != _A_CRUISE_SCALE_N:
          raise ValueError("needs exactly 4 floats")
        self.a_cruise_max_scale = tuple(_clampf(v, *_A_CRUISE_SCALE_CLAMP) for v in raw)
      except (TypeError, ValueError):
        bad.append("a_cruise_max_scale")
    if "t_follow_offsets" in data:
      raw = data["t_follow_offsets"]
      if isinstance(raw, dict):
        for p in PERSONALITIES:
          if p in raw:
            try:
              self.t_follow_offsets[p] = _clampf(raw[p], *_T_FOLLOW_OFFSET_CLAMP)
            except (TypeError, ValueError):
              bad.append(f"t_follow_offsets.{p}")
      else:
        bad.append("t_follow_offsets")
    if "jerk_factors" in data:
      raw = data["jerk_factors"]
      if isinstance(raw, dict):
        for p in PERSONALITIES:
          if p in raw and isinstance(raw[p], dict):
            ov = {}
            for k in _JERK_KEYS:
              if k in raw[p]:
                try:
                  ov[k] = _clampf(raw[p][k], *_JERK_FACTOR_CLAMP)
                except (TypeError, ValueError):
                  bad.append(f"jerk_factors.{p}.{k}")
            if ov:
              self._jerk_overrides[p] = ov
          elif p in raw:
            bad.append(f"jerk_factors.{p}")
      else:
        bad.append("jerk_factors")
    if "lead_consumption" in data:
      raw = data["lead_consumption"]
      if isinstance(raw, dict):
        for k, (_, lo, hi) in _LEAD_CONSUMPTION_CLAMPS.items():
          if k in raw:
            try:
              self.lead_consumption[k] = _clampf(raw[k], lo, hi)
            except (TypeError, ValueError):
              bad.append(f"lead_consumption.{k}")
      else:
        bad.append("lead_consumption")
    if "stopping" in data:
      raw = data["stopping"]
      if isinstance(raw, dict):
        for k, (_, lo, hi) in _STOPPING_CLAMPS.items():
          if k in raw:
            try:
              self.stopping[k] = _clampf(raw[k], lo, hi)
            except (TypeError, ValueError):
              bad.append(f"stopping.{k}")
      else:
        bad.append("stopping")

    self.active = True
    summary = self.describe()
    if bad:
      summary += f" (rejected: {', '.join(bad)})"
    self._warn(f"loaded {summary}")

  def describe(self):
    parts = []
    for name, (default, _, _) in _SCALARS.items():
      v = getattr(self, name)
      if v != default:
        parts.append(f"{name}={v:g}")
    if self.a_cruise_max_scale != (1.0,) * _A_CRUISE_SCALE_N:
      parts.append(f"a_cruise_max_scale={list(self.a_cruise_max_scale)}")
    if self.t_follow_offsets:
      parts.append(f"t_follow_offsets={self.t_follow_offsets}")
    if self._jerk_overrides:
      parts.append(f"jerk_factors={self._jerk_overrides}")
    lc_changed = {k: v for k, (d, _, _) in _LEAD_CONSUMPTION_CLAMPS.items()
                  if (v := self.lead_consumption[k]) != d}
    if lc_changed:
      parts.append(f"lead_consumption={lc_changed}")
    st_changed = {k: v for k, (d, _, _) in _STOPPING_CLAMPS.items()
                  if (v := self.stopping[k]) != d}
    if st_changed:
      parts.append(f"stopping={st_changed}")
    return "all-defaults" if not parts else "; ".join(parts)

  def _warn(self, msg):
    if self._log_fn is not None and msg != self._last_log:
      self._last_log = msg
      self._log_fn(f"nrdr_long_tune: {msg}")


def write_tune(values, path=NRDR_LONG_TUNE_PATH):
  """Atomic write (temp file + rename) so the planner can never read a partial file."""
  tmp = path + ".tmp"
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump(values, f, indent=2, sort_keys=True)
    f.write("\n")
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, path)


def _set_dotted(d, dotted_key, value):
  keys = dotted_key.split(".")
  for k in keys[:-1]:
    d = d.setdefault(k, {})
    if not isinstance(d, dict):
      raise ValueError(f"{dotted_key}: {k} is not an object")
  d[keys[-1]] = value


def main():
  import argparse
  parser = argparse.ArgumentParser(description="NRDR live longitudinal tune file editor")
  parser.add_argument("--path", default=NRDR_LONG_TUNE_PATH)
  sub = parser.add_subparsers(dest="cmd", required=True)
  sub.add_parser("show", help="print parsed+clamped state as the planner would see it")
  p_set = sub.add_parser("set", help="set dotted keys, e.g. comfort_brake=2.3 jerk_factors.relaxed.j_ego=1.5")
  p_set.add_argument("assignments", nargs="+")
  sub.add_parser("reset", help="delete the tune file (planner reverts to compiled defaults)")
  args = parser.parse_args()

  if args.cmd == "show":
    tune = LongTune(path=args.path, log_fn=print)
    tune.refresh()
    print(f"file: {args.path} active={tune.active}")
    print(tune.describe())
  elif args.cmd == "set":
    try:
      with open(args.path, encoding="utf-8") as f:
        data = json.load(f)
      if not isinstance(data, dict):
        data = {}
    except (OSError, ValueError):
      data = {}
    for assignment in args.assignments:
      key, _, raw = assignment.partition("=")
      if not _ or not key:
        raise SystemExit(f"bad assignment: {assignment!r} (want key=value)")
      try:
        value = json.loads(raw)
      except ValueError:
        value = raw
      _set_dotted(data, key, value)
    write_tune(data, path=args.path)
    tune = LongTune(path=args.path, log_fn=print)
    tune.refresh()
    print(f"wrote {args.path}; planner will see: {tune.describe()}")
  elif args.cmd == "reset":
    try:
      os.remove(args.path)
      print(f"removed {args.path}")
    except FileNotFoundError:
      print("no tune file; already at compiled defaults")


if __name__ == "__main__":
  main()
