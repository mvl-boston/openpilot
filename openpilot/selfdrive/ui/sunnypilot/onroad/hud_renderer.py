import math
import os
import subprocess
import time
from pathlib import Path

import pyray as rl

import openpilot.cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiRenderer, DeveloperUiState, get_bottom_dev_ui_offset
from openpilot.selfdrive.ui.sunnypilot.onroad.road_name import RoadNameRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.rocket_fuel import RocketFuel
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_limit import SpeedLimitRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.smart_cruise_control import SmartCruiseControlRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.turn_signal import TurnSignalController
from openpilot.selfdrive.ui.sunnypilot.onroad.circular_alerts import CircularAlertsRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_renderer import SpeedRenderer
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer, UI_CONFIG, FONT_SIZES, COLORS, CRUISE_DISABLED_CHAR
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.sunnypilot.models.model_name import DEFAULT_MODEL
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


SLA_ACTIVE_COLOR = rl.Color(0x91, 0x9b, 0x95, 0xff)
# Large speedometer tuning. The font is calculated from the full HUD height.
CURRENT_SPEED_HUD_HEIGHT_RATIO = 0.65
CURRENT_SPEED_CENTER_X_OFFSET = 0  # Positive moves right; negative moves left.
CURRENT_SPEED_CENTER_Y_RATIO = 0.50
CURRENT_SPEED_UNIT_FONT_SIZE = 66
CURRENT_SPEED_UNIT_Y_RATIO = 0.94
CPU_TEMP_MEDIUM_C = 75.0
CPU_TEMP_EXTREME_C = 90.0
CHESTNUT_TEMP_MEDIUM_C = 75.0
CHESTNUT_TEMP_EXTREME_C = 90.0
CAR_BATTERY_LOW_V = 11.8
CAR_BATTERY_WARNING_V = 12.4
GPU_TEMP_OK_COLOR = rl.Color(0, 255, 0, 255)
GPU_TEMP_MEDIUM_COLOR = rl.Color(255, 165, 0, 255)
GPU_TEMP_EXTREME_COLOR = rl.Color(255, 0, 0, 255)
RIGHT_STATUS_TOP_GAP = 40
MODEL_NAME_FONT_SIZE = 42
MODEL_NAME_MAX_CHARS = 30
EGPU_BUTTON_SIZE = 192
EGPU_ICON_WIDTH = 144
EGPU_ICON_HEIGHT = 107
EGPU_ICON_SPACING = 20
EGPU_DONE_HOLD_SECONDS = 2.5
CAMERA_OFFSET_DISPLAY_SECONDS = 3.0
CAMERA_OFFSET_CONTROLS_TIMEOUT_SECONDS = 3.0
CAMERA_OFFSET_FONT_SIZE = 300
CAMERA_OFFSET_STATUS_FONT_SIZE = 48
CAMERA_OFFSET_STATUS_DIAMETER = 180
STATUS_CIRCLE_GAP = 20
STEER_RATIO_BUTTON_DIAMETER = 150
STEER_RATIO_BUTTON_GAP = 70
STEER_RATIO_STEP = 0.1
STEER_RATIO_MIN_FACTOR = 0.75
STEER_RATIO_MAX_FACTOR = 1.25
STEER_RATIO_MAX_ADJUST_SPEED = 0.1
FIXED_STEER_RATIO_PATH = Path("/data/params/FixedSteerRatio")
CAMERA_OFFSET_BUTTON_WIDTH = 360
CAMERA_OFFSET_BUTTON_HEIGHT = 320
CAMERA_OFFSET_BUTTON_MARGIN = 40
CAMERA_OFFSET_BUTTON_COLOR = rl.Color(0, 220, 80, 255)
CAMERA_OFFSET_BUTTON_PRESSED_COLOR = rl.Color(0, 170, 60, 255)
CAMERA_OFFSET_SCRIPTS = {
  "+": "/data/openpilot/scripts/plus.sh",
  "-": "/data/openpilot/scripts/minus.sh",
}


class CameraOffsetTriangleButton(Widget):
  def __init__(self, label: str, points_left: bool, click_callback):
    super().__init__()
    self._label = label
    self._points_left = points_left
    self._click_callback = click_callback
    self._font = gui_app.font(FontWeight.SEMI_BOLD)

  def _render(self, rect: rl.Rectangle) -> None:
    if self._points_left:
      tip = rl.Vector2(rect.x, rect.y + rect.height / 2)
      top = rl.Vector2(rect.x + rect.width, rect.y)
      bottom = rl.Vector2(rect.x + rect.width, rect.y + rect.height)
      text_center_x = rect.x + rect.width * 0.62
      triangle_points = (tip, bottom, top)
    else:
      tip = rl.Vector2(rect.x + rect.width, rect.y + rect.height / 2)
      top = rl.Vector2(rect.x, rect.y)
      bottom = rl.Vector2(rect.x, rect.y + rect.height)
      text_center_x = rect.x + rect.width * 0.38
      triangle_points = (tip, top, bottom)

    color = CAMERA_OFFSET_BUTTON_PRESSED_COLOR if self.is_pressed else CAMERA_OFFSET_BUTTON_COLOR
    rl.draw_triangle(*triangle_points, color)

    font_size = 160
    text_size = measure_text_cached(self._font, self._label, font_size)
    text_pos = rl.Vector2(text_center_x - text_size.x / 2, rect.y + (rect.height - text_size.y) / 2)
    rl.draw_text_ex(self._font, self._label, text_pos, font_size, 0, rl.WHITE)


class SteerRatioCircleButton(Widget):
  def __init__(self, label: str, click_callback):
    super().__init__()
    self._label = label
    self._click_callback = click_callback
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self.set_enabled(False)

  def _render(self, rect: rl.Rectangle) -> None:
    center = rl.Vector2(rect.x + rect.width / 2, rect.y + rect.height / 2)
    radius = min(rect.width, rect.height) / 2
    fill = CAMERA_OFFSET_BUTTON_COLOR if self.enabled else COLORS.DARK_GREY
    if self.enabled and self.is_pressed:
      fill = CAMERA_OFFSET_BUTTON_PRESSED_COLOR
    rl.draw_circle(int(center.x), int(center.y), radius, fill)
    rl.draw_circle_lines(int(center.x), int(center.y), radius, COLORS.WHITE_TRANSLUCENT)

    text_size = measure_text_cached(self._font, self._label, 90)
    text_pos = rl.Vector2(center.x - text_size.x / 2, center.y - text_size.y / 2)
    rl.draw_text_ex(self._font, self._label, text_pos, 90, 0, rl.WHITE if self.enabled else COLORS.GREY)


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.developer_ui = DeveloperUiRenderer()
    self.road_name_renderer = RoadNameRenderer()
    self.rocket_fuel = RocketFuel()
    self.speed_limit_renderer = SpeedLimitRenderer()
    self.smart_cruise_control_renderer = SmartCruiseControlRenderer()
    self.turn_signal_controller = TurnSignalController()
    self.circular_alerts_renderer = CircularAlertsRenderer()
    self.speed_renderer = SpeedRenderer()
    self._torque_bar = TorqueBar(scale=3.0, always=True)

    self.pcm_cruise_speed: bool = True
    self.show_icbm_status: bool = False
    self.icbm_active_counter: int = 0
    self.speed_cluster: float = 0.0
    self.speed_conv: float = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.cpu_temp: float | None = None
    self.chestnut_temp: float | None = None
    self.car_battery_voltage: float | None = None
    self.rpy: tuple[float, float, float] | None = None
    self.model_name: str = self._get_model_name()
    self._chestnut_sm = messaging.SubMaster(["chestnutState"])
    self._battery_sm = messaging.SubMaster(["peripheralState"])
    self._egpu_icon_white = gui_app.texture("icons_mici/egpu.png", EGPU_ICON_WIDTH, EGPU_ICON_HEIGHT)
    self._egpu_icon_green = gui_app.texture("icons_mici/egpu_green.png", EGPU_ICON_WIDTH, EGPU_ICON_HEIGHT)
    self._egpu_active_prev = ui_state.usbgpu_active is True
    self._egpu_fade_time = rl.get_time() if self._egpu_active_prev else 0.0
    self._egpu_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

    self._params = Params()
    self._camera_offset = self._read_camera_offset()
    self._camera_offset_display_until = 0.0
    self._camera_offset_controls_visible_frames = 0
    self._camera_offset_processes: list[subprocess.Popen] = []
    self._camera_offset_buttons = {
      label: self._child(CameraOffsetTriangleButton(
        label,
        points_left=label == "+",
        click_callback=lambda label=label: self._adjust_camera_offset(label),
      ))
      for label in CAMERA_OFFSET_SCRIPTS
    }
    self._stock_steer_ratio = float(ui_state.sm['carParams'].steerRatio) if ui_state.sm['carParams'].steerRatio > 0 else 15.0
    self._fixed_steer_ratio = self._read_fixed_steer_ratio()
    self._steer_ratio_buttons = {
      label: self._child(SteerRatioCircleButton(label, lambda label=label: self._adjust_steer_ratio(label)))
      for label in ("-", "+")
    }

  def _read_camera_offset(self) -> float:
    value = self._params.get("CameraOffset", return_default=True)
    return float(value) if value is not None else 0.0

  def _read_fixed_steer_ratio(self) -> float:
    try:
      value = FIXED_STEER_RATIO_PATH.read_text().strip()
    except OSError:
      value = None
    try:
      ratio = float(value) if value is not None else self._stock_steer_ratio
    except (TypeError, ValueError):
      ratio = self._stock_steer_ratio
    return self._clamp_steer_ratio(ratio)

  def _clamp_steer_ratio(self, ratio: float) -> float:
    return max(self._stock_steer_ratio * STEER_RATIO_MIN_FACTOR,
               min(ratio, self._stock_steer_ratio * STEER_RATIO_MAX_FACTOR))

  def _steer_ratio_adjustment_allowed(self) -> bool:
    return ui_state.sm['carState'].vEgo <= STEER_RATIO_MAX_ADJUST_SPEED and not ui_state.sm['carControl'].enabled

  def _adjust_steer_ratio(self, direction: str) -> None:
    if not self._steer_ratio_adjustment_allowed():
      return
    delta = STEER_RATIO_STEP if direction == "+" else -STEER_RATIO_STEP
    self._fixed_steer_ratio = self._clamp_steer_ratio(round(self._fixed_steer_ratio + delta, 1))
    self._write_fixed_steer_ratio()

  def _write_fixed_steer_ratio(self) -> None:
    value = f"{self._fixed_steer_ratio:.1f}"
    temp_path = FIXED_STEER_RATIO_PATH.with_name(f".{FIXED_STEER_RATIO_PATH.name}.tmp")
    try:
      temp_path.write_text(value)
      os.replace(temp_path, FIXED_STEER_RATIO_PATH)
    except OSError:
      temp_path.unlink(missing_ok=True)

    # Also use Params when the native key registry includes FixedSteerRatio.
    try:
      self._params.put("FixedSteerRatio", value)
    except Exception:
      pass

  def _adjust_camera_offset(self, direction: str) -> None:
    try:
      self._camera_offset_processes.append(subprocess.Popen(
        [CAMERA_OFFSET_SCRIPTS[direction]],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
      ))
    except OSError:
      pass

    self._camera_offset = self._read_camera_offset()
    now = time.monotonic()
    self._camera_offset_display_until = now + CAMERA_OFFSET_DISPLAY_SECONDS
    self._camera_offset_controls_visible_frames = round(CAMERA_OFFSET_CONTROLS_TIMEOUT_SECONDS * gui_app.target_fps)

  def _update_camera_offset(self) -> None:
    self._camera_offset_processes = [process for process in self._camera_offset_processes if process.poll() is None]
    if time.monotonic() < self._camera_offset_display_until:
      self._camera_offset = self._read_camera_offset()

  @staticmethod
  def _get_model_name() -> str:
    try:
      active_bundle = get_active_bundle()
    except Exception:
      return DEFAULT_MODEL

    if active_bundle is not None and active_bundle.displayName:
      return active_bundle.displayName
    return DEFAULT_MODEL

  def _update_state(self) -> None:
    if ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      return

    if ui_state.CP_SP is not None:
      self.pcm_cruise_speed = ui_state.CP_SP.pcmCruiseSpeed
    self.speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed_cluster = ui_state.sm['carState'].cruiseState.speedCluster * self.speed_conv

    cpu_temps = [temp for temp in ui_state.sm['deviceState'].cpuTempC if temp > 0.0]
    self.cpu_temp = max(cpu_temps) if cpu_temps else None

    self._chestnut_sm.update(0)
    if self._chestnut_sm.valid['chestnutState'] and self._chestnut_sm.alive['chestnutState']:
      chestnut_temp = self._chestnut_sm['chestnutState'].tempC
      self.chestnut_temp = chestnut_temp if chestnut_temp > 0.0 else None
    else:
      self.chestnut_temp = None

    self._battery_sm.update(0)
    peripheral_voltage = self._battery_sm['peripheralState'].voltage
    if self._battery_sm.valid['peripheralState'] and self._battery_sm.alive['peripheralState'] and peripheral_voltage > 0:
      self.car_battery_voltage = peripheral_voltage / 1000.0
    else:
      battery_voltages = [panda.voltage / 1000.0 for panda in ui_state.sm['pandaStates'] if panda.voltage > 0]
      self.car_battery_voltage = max(battery_voltages) if battery_voltages else None

    egpu_active = ui_state.usbgpu_active is True
    if egpu_active and not self._egpu_active_prev:
      self._egpu_fade_time = rl.get_time()
    self._egpu_active_prev = egpu_active

    rpy_calib = ui_state.sm['extrinsicsCalibration'].rpyCalib
    if len(rpy_calib) == 3:
      self.rpy = tuple(math.degrees(value) for value in rpy_calib)

    super()._update_state()
    self.road_name_renderer.update()
    self.speed_limit_renderer.update()
    self.smart_cruise_control_renderer.update()
    self.turn_signal_controller.update()
    self.circular_alerts_renderer.update()
    self.speed_renderer.update()

  def _get_icbm_status(self):
    if not self.pcm_cruise_speed and ui_state.sm['carControl'].enabled:
      if round(self.set_speed) != round(self.speed_cluster):
        self.icbm_active_counter = 3 * gui_app.target_fps  # 3 seconds usually
      elif self.icbm_active_counter > 0:
        self.icbm_active_counter -= 1
    else:
      self.icbm_active_counter = 0

    self.show_icbm_status = self.icbm_active_counter > 0

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    long_plan_sp = ui_state.sm['longitudinalPlanSP']
    long_override = ui_state.sm['carControl'].cruiseControl.override
    self._get_icbm_status()

    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if long_plan_sp.speedLimit.assist.active:
        set_speed_color = SLA_ACTIVE_COLOR if long_override else rl.Color(0, 0xff, 0, 0xff)
        max_color = SLA_ACTIVE_COLOR if long_override else rl.Color(0x80, 0xd8, 0xa6, 0xff)
      else:
        if ui_state.status == UIStatus.ENGAGED:
          max_color = COLORS.ENGAGED
        elif ui_state.status == UIStatus.DISENGAGED:
          max_color = COLORS.DISENGAGED
        elif ui_state.status == UIStatus.OVERRIDE:
          max_color = COLORS.OVERRIDE

    max_str_size = 60 if self.show_icbm_status else 40
    max_str_y = 15 if self.show_icbm_status else 27

    max_text = str(round(self.speed_cluster)) if self.show_icbm_status else tr("MAX")
    max_text_width = measure_text_cached(self._font_semi_bold, max_text, max_str_size).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + max_str_y),
      max_str_size,
      0,
      max_color,
    )

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    speed_center_x = rect.x + rect.width / 2 + CURRENT_SPEED_CENTER_X_OFFSET
    speed_font_size = max(1, round(rect.height * CURRENT_SPEED_HUD_HEIGHT_RATIO))

    showing_camera_offset = time.monotonic() < self._camera_offset_display_until
    if not showing_camera_offset:
      speed_text = str(round(self.speed))
      speed_text_size = measure_text_cached(self._font_bold, speed_text, speed_font_size)
      speed_pos = rl.Vector2(
        speed_center_x - speed_text_size.x / 2,
        rect.y + rect.height * CURRENT_SPEED_CENTER_Y_RATIO - speed_text_size.y / 2,
      )
      rl.draw_text_ex(
        self._font_bold,
        speed_text,
        speed_pos,
        speed_font_size,
        0,
        COLORS.WHITE,
      )

      unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
      unit_text_size = measure_text_cached(self._font_medium, unit_text, CURRENT_SPEED_UNIT_FONT_SIZE)
      unit_pos = rl.Vector2(
        speed_center_x - unit_text_size.x / 2,
        rect.y + rect.height * CURRENT_SPEED_UNIT_Y_RATIO - unit_text_size.y / 2,
      )
      rl.draw_text_ex(
        self._font_medium,
        unit_text,
        unit_pos,
        CURRENT_SPEED_UNIT_FONT_SIZE,
        0,
        COLORS.WHITE_TRANSLUCENT,
      )
    else:
      offset_text = f"{self._camera_offset:.2f}"
      offset_size = measure_text_cached(self._font_bold, offset_text, CAMERA_OFFSET_FONT_SIZE)
      offset_pos = rl.Vector2(
        speed_center_x - offset_size.x / 2,
        rect.y + rect.height * CURRENT_SPEED_CENTER_Y_RATIO - offset_size.y / 2,
      )
      rl.draw_text_ex(self._font_bold, offset_text, offset_pos, CAMERA_OFFSET_FONT_SIZE, 0, COLORS.WHITE)

      offset_label = "Camera Offset"
      label_size = measure_text_cached(self._font_medium, offset_label, CURRENT_SPEED_UNIT_FONT_SIZE)
      label_pos = rl.Vector2(
        speed_center_x - label_size.x / 2,
        rect.y + rect.height * CURRENT_SPEED_UNIT_Y_RATIO - label_size.y / 2,
      )
      rl.draw_text_ex(self._font_medium, offset_label, label_pos, CURRENT_SPEED_UNIT_FONT_SIZE, 0, COLORS.WHITE_TRANSLUCENT)

  def user_interacting(self) -> bool:
    return (super().user_interacting() or
            any(button.is_pressed for button in self._camera_offset_buttons.values()) or
            any(button.is_pressed for button in self._steer_ratio_buttons.values()))

  @staticmethod
  def _steer_ratio_button_rects(rect: rl.Rectangle) -> dict[str, rl.Rectangle]:
    center_x = rect.x + rect.width / 2
    center_y = rect.y + rect.height * 0.78
    offset = STEER_RATIO_BUTTON_DIAMETER / 2 + STEER_RATIO_BUTTON_GAP + 100
    return {
      "-": rl.Rectangle(center_x - offset - STEER_RATIO_BUTTON_DIAMETER / 2,
                        center_y - STEER_RATIO_BUTTON_DIAMETER / 2,
                        STEER_RATIO_BUTTON_DIAMETER, STEER_RATIO_BUTTON_DIAMETER),
      "+": rl.Rectangle(center_x + offset - STEER_RATIO_BUTTON_DIAMETER / 2,
                        center_y - STEER_RATIO_BUTTON_DIAMETER / 2,
                        STEER_RATIO_BUTTON_DIAMETER, STEER_RATIO_BUTTON_DIAMETER),
    }

  @staticmethod
  def _camera_offset_button_rects(rect: rl.Rectangle) -> dict[str, rl.Rectangle]:
    button_y = rect.y + (rect.height - CAMERA_OFFSET_BUTTON_HEIGHT) / 2
    return {
      "+": rl.Rectangle(rect.x + CAMERA_OFFSET_BUTTON_MARGIN, button_y,
                        CAMERA_OFFSET_BUTTON_WIDTH, CAMERA_OFFSET_BUTTON_HEIGHT),
      "-": rl.Rectangle(rect.x + rect.width - CAMERA_OFFSET_BUTTON_MARGIN - CAMERA_OFFSET_BUTTON_WIDTH, button_y,
                        CAMERA_OFFSET_BUTTON_WIDTH, CAMERA_OFFSET_BUTTON_HEIGHT),
    }

  def _handle_mouse_press(self, mouse_pos) -> None:
    touching_triangle = (
      self._camera_offset_controls_visible_frames > 0 and
      any(rl.check_collision_point_rec(mouse_pos, button_rect)
          for button_rect in self._camera_offset_button_rects(self.rect).values())
    )

    self._camera_offset_controls_visible_frames = round(CAMERA_OFFSET_CONTROLS_TIMEOUT_SECONDS * gui_app.target_fps)
    if not touching_triangle:
      self._camera_offset = self._read_camera_offset()
      self._camera_offset_display_until = time.monotonic() + CAMERA_OFFSET_DISPLAY_SECONDS

  def _render(self, rect: rl.Rectangle) -> None:
    self._update_camera_offset()
    if self._camera_offset_controls_visible_frames > 0:
      self._camera_offset_controls_visible_frames -= 1
      for label, button_rect in self._camera_offset_button_rects(rect).items():
        self._camera_offset_buttons[label].render(button_rect)

    adjustment_allowed = self._steer_ratio_adjustment_allowed()
    for label, button_rect in self._steer_ratio_button_rects(rect).items():
      self._steer_ratio_buttons[label].set_enabled(adjustment_allowed)
      self._steer_ratio_buttons[label].render(button_rect)
    self._draw_steer_ratio_value(rect, adjustment_allowed)

    super()._render(rect)

    self._draw_model_name(rect)
    self._draw_right_status(rect)
    self._draw_egpu_status_icon(rect)

    if ui_state.torque_bar:
      torque_rect = rect
      if ui_state.developer_ui in (DeveloperUiState.BOTTOM, DeveloperUiState.BOTH):
        torque_rect = rl.Rectangle(rect.x, rect.y, rect.width, rect.height - get_bottom_dev_ui_offset())
      self._torque_bar.render(torque_rect)

    self.developer_ui.render(rect)
    self.road_name_renderer.render(rect)
    self.speed_limit_renderer.render(rect)
    self.smart_cruise_control_renderer.render(rect)
    self.turn_signal_controller.render(rect)
    self.circular_alerts_renderer.render(rect)
    self.rocket_fuel.render(rect, ui_state.sm)

  def _draw_steer_ratio_value(self, rect: rl.Rectangle, adjustment_allowed: bool) -> None:
    center_x = rect.x + rect.width / 2
    center_y = rect.y + rect.height * 0.78
    applied_ratio = float(ui_state.sm['vehicleParameters'].steerRatio)
    ratio_applied = applied_ratio > 0 and abs(applied_ratio - self._fixed_steer_ratio) < STEER_RATIO_STEP / 2
    value_text = f"{self._fixed_steer_ratio:.1f}"
    value_size = measure_text_cached(self._font_bold, value_text, 76)
    rl.draw_text_ex(self._font_bold, value_text,
                    rl.Vector2(center_x - value_size.x / 2, center_y - value_size.y / 2 - 12),
                    76, 0, COLORS.WHITE)
    if ratio_applied:
      label = "STEER RATIO • FIXED"
    elif adjustment_allowed:
      label = "STEER RATIO • RESTART TO APPLY"
    else:
      label = "STEER RATIO • PARK, THEN RESTART"
    label_size = measure_text_cached(self._font_medium, label, 26)
    rl.draw_text_ex(self._font_medium, label,
                    rl.Vector2(center_x - label_size.x / 2, center_y + 48),
                    26, 0, COLORS.WHITE_TRANSLUCENT)

  def _draw_model_name(self, rect: rl.Rectangle) -> None:
    model_name = self.model_name
    if len(model_name) > MODEL_NAME_MAX_CHARS:
      model_name = f"{model_name[:MODEL_NAME_MAX_CHARS - 1]}…"

    model_text = f"MODEL: {model_name}"
    model_text_size = measure_text_cached(self._font_medium, model_text, MODEL_NAME_FONT_SIZE)
    model_pos = rl.Vector2(
      rect.x + rect.width - UI_CONFIG.border_size - model_text_size.x,
      rect.y + UI_CONFIG.border_size + UI_CONFIG.button_size + RIGHT_STATUS_TOP_GAP,
    )
    rl.draw_text_ex(
      self._font_medium,
      model_text,
      model_pos,
      MODEL_NAME_FONT_SIZE,
      0,
      COLORS.WHITE_TRANSLUCENT,
    )

  def _draw_right_status(self, rect: rl.Rectangle) -> None:
    right_x = rect.x + rect.width - UI_CONFIG.border_size
    radius = CAMERA_OFFSET_STATUS_DIAMETER / 2
    first_center_x = right_x - UI_CONFIG.button_size - 30 - radius
    center_y = rect.y + UI_CONFIG.border_size + radius

    rpy_values = (
      (f"R {self.rpy[0]:+.1f}°", f"P {self.rpy[1]:+.1f}°", f"Y {self.rpy[2]:+.1f}°")
      if self.rpy is not None else ("--",)
    )
    statuses = (
      ("OFFSET", f"{self._camera_offset:.2f}", COLORS.WHITE),
      ("R/P/Y", rpy_values, COLORS.WHITE),
      ("CPU", f"{round(self.cpu_temp)}°C" if self.cpu_temp is not None else "--",
       self._temperature_color(self.cpu_temp, CPU_TEMP_MEDIUM_C, CPU_TEMP_EXTREME_C)),
      ("eGPU", f"{round(self.chestnut_temp)}°C" if self.chestnut_temp is not None else "--",
       self._temperature_color(self.chestnut_temp, CHESTNUT_TEMP_MEDIUM_C, CHESTNUT_TEMP_EXTREME_C)),
      ("BAT", f"{self.car_battery_voltage:.1f}V" if self.car_battery_voltage is not None else "--",
       self._battery_color(self.car_battery_voltage)),
    )

    for index, (label, value, value_color) in enumerate(statuses):
      center = rl.Vector2(first_center_x - index * (CAMERA_OFFSET_STATUS_DIAMETER + STATUS_CIRCLE_GAP), center_y)
      self._draw_status_circle(center, label, value, value_color)

  @staticmethod
  def _temperature_color(value: float | None, medium: float, extreme: float) -> rl.Color:
    if value is None:
      return COLORS.WHITE_TRANSLUCENT
    if value >= extreme:
      return GPU_TEMP_EXTREME_COLOR
    if value >= medium:
      return GPU_TEMP_MEDIUM_COLOR
    return GPU_TEMP_OK_COLOR

  @staticmethod
  def _battery_color(voltage: float | None) -> rl.Color:
    if voltage is None:
      return COLORS.WHITE_TRANSLUCENT
    if voltage < CAR_BATTERY_LOW_V:
      return GPU_TEMP_EXTREME_COLOR
    if voltage < CAR_BATTERY_WARNING_V:
      return GPU_TEMP_MEDIUM_COLOR
    return GPU_TEMP_OK_COLOR

  def _draw_status_circle(self, center: rl.Vector2, label: str, value: str | tuple[str, ...], value_color: rl.Color) -> None:
    radius = CAMERA_OFFSET_STATUS_DIAMETER / 2
    rl.draw_circle(int(center.x), int(center.y), radius, COLORS.BLACK_TRANSLUCENT)
    rl.draw_circle_lines(int(center.x), int(center.y), radius, CAMERA_OFFSET_BUTTON_COLOR)

    label_size = measure_text_cached(self._font_semi_bold, label, 30)
    label_y = center.y - (72 if isinstance(value, tuple) else 55)
    label_pos = rl.Vector2(center.x - label_size.x / 2, label_y)
    rl.draw_text_ex(self._font_semi_bold, label, label_pos, 30, 0, COLORS.WHITE_TRANSLUCENT)

    if isinstance(value, tuple):
      value_font_size = 25
      line_spacing = 29
      first_y = center.y - 32
      for index, line in enumerate(value):
        line_size = measure_text_cached(self._font_bold, line, value_font_size)
        line_pos = rl.Vector2(center.x - line_size.x / 2, first_y + index * line_spacing)
        rl.draw_text_ex(self._font_bold, line, line_pos, value_font_size, 0, value_color)
    else:
      value_size = measure_text_cached(self._font_bold, value, CAMERA_OFFSET_STATUS_FONT_SIZE)
      value_pos = rl.Vector2(center.x - value_size.x / 2, center.y - value_size.y / 2 + 18)
      rl.draw_text_ex(self._font_bold, value, value_pos, CAMERA_OFFSET_STATUS_FONT_SIZE, 0, value_color)

  def _draw_egpu_status_icon(self, rect: rl.Rectangle) -> None:
    loading = ui_state.usbgpu_loading
    active = ui_state.usbgpu_active is True
    show_done = active and 0 < rl.get_time() - self._egpu_fade_time < EGPU_DONE_HOLD_SECONDS
    alpha = self._egpu_alpha_filter.update(loading or show_done)
    if alpha < 1e-2:
      return

    is_rhd = ui_state.sm['driverMonitoringState'].isRHD
    dm_offset = UI_BORDER_SIZE + EGPU_BUTTON_SIZE / 2
    dm_x = rect.x + (rect.width - dm_offset if is_rhd else dm_offset)
    center_x = dm_x + (-1 if is_rhd else 1) * (EGPU_BUTTON_SIZE + EGPU_ICON_SPACING)
    center_y = rect.y + rect.height - dm_offset

    rl.draw_circle(int(center_x), int(center_y), EGPU_BUTTON_SIZE // 2, rl.Color(0, 0, 0, 70))

    if loading:
      icon = self._egpu_icon_white
      pulse = 0.5 - 0.5 * math.cos(rl.get_time() * 6.0)
      opacity = 0.35 + 0.65 * pulse
    else:
      icon = self._egpu_icon_green if active else self._egpu_icon_white
      opacity = 1.0

    icon_pos = rl.Vector2(center_x - icon.width / 2, center_y - icon.height / 2)
    rl.draw_texture_v(icon, icon_pos, rl.Color(255, 255, 255, int(255 * opacity * alpha)))
