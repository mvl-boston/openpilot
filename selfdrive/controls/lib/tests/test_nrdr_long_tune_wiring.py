"""Wiring/codegen-identity guards for the M9/M5 live-tune integration (AST-based).

long_mpc.py cannot be imported on a non-device host (aarch64 acados .so), so these
tests parse the source instead. They protect the two invariants that keep the live
tune safe on the prebuilt branch:
  1. gen_long_ocp() still builds the OCP from the compiled defaults (codegen identity:
     the committed acados artifacts remain valid for this source).
  2. The runtime tune hooks stay wired (refresh called, defaults mirror the constants).
"""
import ast
from pathlib import Path

LIB = Path(__file__).resolve().parent.parent
LONG_MPC = LIB / "longitudinal_mpc_lib" / "long_mpc.py"
PLANNER = LIB / "longitudinal_planner.py"


def _tree(path):
  return ast.parse(path.read_text(encoding="utf-8"))


def _func(tree, name):
  for node in ast.walk(tree):
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
      return node
  raise AssertionError(f"function {name} not found")


def _module_constants(tree):
  out = {}
  for node in tree.body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      try:
        out[node.targets[0].id] = ast.literal_eval(node.value)
      except ValueError:
        pass
  return out


class TestCodegenIdentity:
  """If these fail, the generated OCP would differ from the committed c_generated_code."""

  def test_gen_long_ocp_uses_compiled_defaults(self):
    tree = _tree(LONG_MPC)
    gen = _func(tree, "gen_long_ocp")
    calls = [n for n in ast.walk(gen) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == "get_safe_obstacle_distance"]
    assert len(calls) == 1, "expected exactly one get_safe_obstacle_distance call in gen_long_ocp"
    call = calls[0]
    assert len(call.args) == 2 and not call.keywords, \
      "gen_long_ocp must call get_safe_obstacle_distance(v_ego, lead_t_follow) with defaults only — " \
      "passing tune values here would change the compiled OCP vs the committed solver artifacts"

  def test_helper_defaults_mirror_module_constants(self):
    tree = _tree(LONG_MPC)
    consts = _module_constants(tree)
    assert consts["COMFORT_BRAKE"] == 2.5
    assert consts["STOP_DISTANCE"] == 6.0

    sef = _func(tree, "get_stopped_equivalence_factor")
    assert [d.id for d in sef.args.defaults if isinstance(d, ast.Name)] == ["COMFORT_BRAKE"]

    sod = _func(tree, "get_safe_obstacle_distance")
    assert [d.id for d in sod.args.defaults if isinstance(d, ast.Name)] == ["COMFORT_BRAKE", "STOP_DISTANCE"]

  def test_tune_defaults_match_compiled_constants(self):
    # the tune module's fail-safe defaults must equal the compiled constants (provable no-op)
    from openpilot.selfdrive.controls.lib.nrdr_long_tune import _SCALARS
    assert _SCALARS["comfort_brake"][0] == 2.5
    assert _SCALARS["stop_distance"][0] == 6.0
    assert _SCALARS["low_speed_jerk_scale"][0] == 1.0


class TestLeadConsumptionSchema:
  """M1/M2/M3 tune-schema invariants: defaults are provable no-ops except m1_anchor (ON)."""

  def test_defaults_are_noops_except_m1_anchor(self):
    from openpilot.selfdrive.controls.lib.nrdr_long_tune import _LEAD_CONSUMPTION_CLAMPS as C
    assert C["m1_anchor"][0] == 1.0          # honest KF signal -> default ON (only non-no-op)
    assert C["m1_alead_escape"][0] == 1.0
    assert C["m2_w_max"][0] == 0.0           # M2 off by default
    assert C["m3_b_eff_max"][0] == 2.5       # == comfort_brake default -> M3 cannot inflate
    # m3_b_eff_max lower clamp == comfort_brake so it can never deflate below it (one-sided)
    assert C["m3_b_eff_max"][1] == 2.5
    assert C["m3_alead_gate"][0] == -0.5

  def test_old_reserved_block_superseded(self):
    # obstacle_inflation_gains was repurposed into lead_consumption; the old clamp dict is gone
    import openpilot.selfdrive.controls.lib.nrdr_long_tune as m
    assert not hasattr(m, "_OBSTACLE_INFLATION_CLAMPS")

  def test_shared_brake_gate_used_by_m2_and_m3(self):
    # council invariant: M2 and M3 engage off ONE helper (lead_brake_gate)
    tree = _tree(LONG_MPC)
    assert any(isinstance(n, ast.FunctionDef) and n.name == "lead_brake_gate"
               for n in tree.body), "lead_brake_gate helper must exist at module scope"
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LongitudinalMpc")
    pl = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "process_lead")
    be = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "lead_b_eff")
    assert "lead_brake_gate(" in ast.unparse(pl)  # M2 path
    assert "lead_brake_gate(" in ast.unparse(be)  # M3 path

  def test_m3_obstacle_uses_b_eff_not_comfort_brake(self):
    tree = _tree(LONG_MPC)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LongitudinalMpc")
    update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update")
    src = ast.unparse(update)
    assert "self.lead_b_eff(" in src
    assert "get_stopped_equivalence_factor(lead_xv_0[:, 1], b_eff_0)" in src
    assert "get_stopped_equivalence_factor(lead_xv_1[:, 1], b_eff_1)" in src

  def test_nan_gates_present(self):
    # every consumed KF field must pass an isfinite gate before use
    tree = _tree(LONG_MPC)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LongitudinalMpc")
    pl = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "process_lead")
    src = ast.unparse(pl)
    assert "math.isfinite(vLeadK)" in src
    assert "math.isfinite(aLeadK)" in src and "math.isfinite(aLeadTau)" in src


class TestRuntimeWiring:
  def test_update_refreshes_tune(self):
    tree = _tree(LONG_MPC)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LongitudinalMpc")
    update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update")
    src = ast.unparse(update)
    assert "self.tune.refresh()" in src
    assert "self.tune.comfort_brake" in src
    assert "self.tune.stop_distance" in src
    assert "t_follow_offset" in src

  def test_tune_created_before_reset(self):
    tree = _tree(LONG_MPC)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LongitudinalMpc")
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    src = ast.unparse(init)
    assert src.index("self.tune") < src.index("self.reset()"), \
      "reset() -> set_weights() reads self.tune; it must exist first"

  def test_set_weights_uses_split_factors(self):
    tree = _tree(LONG_MPC)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LongitudinalMpc")
    sw = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "set_weights")
    src = ast.unparse(sw)
    assert "jerk_factors" in src and "a_change_factor * a_change_cost" in src and "j_ego_factor * J_EGO_COST" in src

  def test_planner_scales_max_accel(self):
    tree = _tree(PLANNER)
    src = ast.unparse(tree)
    assert "get_max_accel(v_ego, self.mpc.tune.a_cruise_max_scale)" in src
    gma = _func(tree, "get_max_accel")
    assert "ACCEL_MAX" in ast.unparse(gma), "scaled accel values must stay capped at ACCEL_MAX"


class TestGapMath:
  """Directionality checks on the runtime obstacle math (pure functions, evaluated in isolation)."""

  def _funcs(self):
    tree = _tree(LONG_MPC)
    ns = {"COMFORT_BRAKE": 2.5, "STOP_DISTANCE": 6.0}
    for name in ("get_stopped_equivalence_factor", "get_safe_obstacle_distance"):
      exec(compile(ast.Module(body=[_func(tree, name)], type_ignores=[]), "<long_mpc>", "exec"), ns)
    return ns

  def test_defaults_unchanged(self):
    ns = self._funcs()
    # stock values: v_lead=20 -> 80 m equivalence; v=30, t_follow=1.45 -> 229.5 m
    assert ns["get_stopped_equivalence_factor"](20.0) == 80.0
    assert abs(ns["get_safe_obstacle_distance"](30.0, 1.45) - 229.5) < 1e-9

  def test_lower_comfort_brake_tightens_gap(self):
    ns = self._funcs()
    v = 25.0
    # gap to actual lead = desired_dist_comfort(compiled) - stopped_equivalence(runtime)
    desired = ns["get_safe_obstacle_distance"](v, 1.45)  # compiled-default side
    gap_stock = desired - ns["get_stopped_equivalence_factor"](v)
    gap_tight = desired - ns["get_stopped_equivalence_factor"](v, 2.2)
    assert gap_tight < gap_stock
    assert gap_tight > 1.45 * v * 0.5, "tightened gap must stay far above half the time-gap floor"


if __name__ == "__main__":
  import pytest
  raise SystemExit(pytest.main([__file__, "-v"]))
