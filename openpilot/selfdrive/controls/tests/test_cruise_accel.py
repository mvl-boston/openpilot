import pytest

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS, get_cruise_accel, get_max_accel

V_CRUISE = 45.


def settle_cruise_accel(e2e, v_ego):
  a_cruise = 0.
  for _ in range(int(5. / DT_MDL)):
    a_cruise = get_cruise_accel(e2e, V_CRUISE, v_ego, a_cruise, DT_MDL)
  return a_cruise


class TestCruiseAccel:
  @pytest.mark.parametrize("v_ego, max_accel", list(zip(A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS, strict=True)))
  def test_max_accel_breakpoints(self, v_ego, max_accel):
    assert get_max_accel(v_ego) == pytest.approx(max_accel)

  @pytest.mark.parametrize("v_ego", [0., 5., 10., 17.5, 25., 32.5, 40.])
  def test_limited_without_experimental_mode(self, v_ego):
    assert settle_cruise_accel(False, v_ego) == pytest.approx(get_max_accel(v_ego))

  @pytest.mark.parametrize("v_ego", [0., 5., 10., 17.5, 25., 32.5, 40.])
  def test_unlimited_in_experimental_mode(self, v_ego):
    assert settle_cruise_accel(True, v_ego) == pytest.approx(min(V_CRUISE - v_ego, ACCEL_MAX))

  def test_limit_applied_gradually(self):
    # switching Experimental Mode off mid-acceleration must not step the target
    v_ego = 20.
    a_cruise = settle_cruise_accel(True, v_ego)
    a_next = get_cruise_accel(False, V_CRUISE, v_ego, a_cruise, DT_MDL)
    assert get_max_accel(v_ego) < a_next < a_cruise
