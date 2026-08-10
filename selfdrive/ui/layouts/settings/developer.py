from openpilot.common.params import Params
from openpilot.selfdrive.ui.layouts.settings.common import (LANE_CENTER_OFFSET_LABELS, LANE_CENTER_OFFSET_VALUES,
                                                            LANE_CENTERING_E2E_AUTHORITY_LABELS, LANE_CENTERING_E2E_AUTHORITY_VALUES,
                                                            closest_value_index)
from openpilot.selfdrive.ui.widgets.ssh_key import ssh_key_item
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult

# Description constants
DESCRIPTIONS = {
  'enable_adb': tr_noop(
    "ADB (Android Debug Bridge) allows connecting to your device over USB or over the network. " +
    "See https://docs.comma.ai/how-to/connect-to-comma for more info."
  ),
  'ssh_key': tr_noop(
    "Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username " +
    "other than your own. A comma employee will NEVER ask you to add their GitHub username."
  ),
  'alpha_longitudinal': tr_noop(
    "<b>WARNING: openpilot longitudinal control is in alpha for this car and may disable Automatic Emergency Braking (AEB).</b><br><br>" +
    "On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. " +
    "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha. " +
    "Changing this setting will restart openpilot if the car is powered on."
  ),
  'lane_centering': tr_noop(
    "StarPilot Lane Centering (SLC): experimentally bias the model command toward the detected lane center. " +
    "Requires two confident lane lines and remains subject to normal curvature and jerk limits. Ported from StarPilot."
  ),
  'lane_centering_pause_on_signal': tr_noop(
    "Fade the lane-centering correction out when a turn signal is active so it does not fight a lane change or turn."
  ),
  'lane_center_offset': tr_noop(
    "Shift the lane-centering target left or right of the lane center. " +
    "The controller automatically reduces the offset when the detected lane is narrow."
  ),
  'lane_centering_e2e_authority': tr_noop(
    "How strongly a confident end-to-end model path can override lane centering when it deliberately departs the lane center. " +
    "100% gives the model full authority; 0% disables break-in."
  ),
}


class DeveloperLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._is_release = self._params.get_bool("IsReleaseBranch")

    # Build items and keep references for callbacks/state updates
    self._adb_toggle = toggle_item(
      lambda: tr("Enable ADB"),
      description=lambda: tr(DESCRIPTIONS["enable_adb"]),
      initial_state=self._params.get_bool("AdbEnabled"),
      callback=self._on_enable_adb,
      enabled=ui_state.is_offroad,
    )

    # SSH enable toggle + SSH key management
    self._ssh_toggle = toggle_item(
      lambda: tr("Enable SSH"),
      description="",
      initial_state=self._params.get_bool("SshEnabled"),
      callback=self._on_enable_ssh,
    )
    self._ssh_keys = ssh_key_item(lambda: tr("SSH Keys"), description=lambda: tr(DESCRIPTIONS["ssh_key"]))

    self._joystick_toggle = toggle_item(
      lambda: tr("Joystick Debug Mode"),
      description="",
      initial_state=self._params.get_bool("JoystickDebugMode"),
      callback=self._on_joystick_debug_mode,
      enabled=ui_state.is_offroad,
    )

    self._long_maneuver_toggle = toggle_item(
      lambda: tr("Longitudinal Maneuver Mode"),
      description="",
      initial_state=self._params.get_bool("LongitudinalManeuverMode"),
      callback=self._on_long_maneuver_mode,
    )

    self._lat_maneuver_toggle = toggle_item(
      lambda: tr("Lateral Maneuver Mode"),
      description="",
      initial_state=self._params.get_bool("LateralManeuverMode"),
      callback=self._on_lat_maneuver_mode,
    )

    self._alpha_long_toggle = toggle_item(
      lambda: tr("openpilot Longitudinal Control (Alpha)"),
      description=lambda: tr(DESCRIPTIONS["alpha_longitudinal"]),
      initial_state=self._params.get_bool("AlphaLongitudinalEnabled"),
      callback=self._on_alpha_long_enabled,
      enabled=lambda: not ui_state.engaged,
    )

    self._ui_debug_toggle = toggle_item(
      lambda: tr("UI Debug Mode"),
      description="",
      initial_state=self._params.get_bool("ShowDebugInfo"),
      callback=self._on_enable_ui_debug,
    )
    self._on_enable_ui_debug(self._params.get_bool("ShowDebugInfo"))

    self._lane_centering_toggle = toggle_item(
      lambda: tr("SLC (StarPilot Lane Centering)"),
      description=lambda: tr(DESCRIPTIONS["lane_centering"]),
      initial_state=self._params.get_bool("LaneCentering"),
      callback=self._on_lane_centering,
    )

    self._lane_centering_pause_toggle = toggle_item(
      lambda: tr("SLC Pause on Turn Signal"),
      description=lambda: tr(DESCRIPTIONS["lane_centering_pause_on_signal"]),
      initial_state=bool(self._params.get("LaneCenteringPauseOnSignal", return_default=True)),
      callback=self._on_lane_centering_pause_on_signal,
    )

    # button_width sized so the five presets fit beside the longest title ("SLC E2E Override Strength")
    self._lane_center_offset_setting = multiple_button_item(
      lambda: tr("SLC Center Offset"),
      lambda: tr(DESCRIPTIONS["lane_center_offset"]),
      buttons=list(LANE_CENTER_OFFSET_LABELS),
      selected_index=closest_value_index(LANE_CENTER_OFFSET_VALUES, self._params.get("LaneCenterOffset", return_default=True)),
      button_width=155,
      callback=self._on_lane_center_offset,
    )

    self._lane_centering_e2e_authority_setting = multiple_button_item(
      lambda: tr("SLC E2E Override Strength"),
      lambda: tr(DESCRIPTIONS["lane_centering_e2e_authority"]),
      buttons=list(LANE_CENTERING_E2E_AUTHORITY_LABELS),
      selected_index=closest_value_index(LANE_CENTERING_E2E_AUTHORITY_VALUES, self._params.get("LaneCenteringE2EAuthority", return_default=True)),
      button_width=155,
      callback=self._on_lane_centering_e2e_authority,
    )

    self._scroller = Scroller([
      self._adb_toggle,
      self._ssh_toggle,
      self._ssh_keys,
      self._joystick_toggle,
      self._long_maneuver_toggle,
      self._lat_maneuver_toggle,
      self._alpha_long_toggle,
      self._ui_debug_toggle,
      self._lane_centering_toggle,
      self._lane_centering_pause_toggle,
      self._lane_center_offset_setting,
      self._lane_centering_e2e_authority_setting,
    ], line_separator=True, spacing=0)

    # Toggles should be not available to change in onroad state
    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # Hide non-release toggles on release builds
    # TODO: we can do an onroad cycle, but alpha long toggle requires a deinit function to re-enable radar and not fault
    for item in (self._joystick_toggle, self._long_maneuver_toggle, self._lat_maneuver_toggle, self._alpha_long_toggle):
      item.set_visible(not self._is_release)

    # CP gating
    if ui_state.CP is not None:
      alpha_avail = ui_state.CP.alphaLongitudinalAvailable
      if not alpha_avail or self._is_release:
        self._alpha_long_toggle.set_visible(False)
        self._params.remove("AlphaLongitudinalEnabled")
      else:
        self._alpha_long_toggle.set_visible(True)

      long_man_enabled = ui_state.has_longitudinal_control and ui_state.is_offroad()
      self._long_maneuver_toggle.action_item.set_enabled(long_man_enabled)
    else:
      self._long_maneuver_toggle.action_item.set_enabled(False)
      self._lat_maneuver_toggle.action_item.set_enabled(False)
      self._alpha_long_toggle.set_visible(False)

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for key, item in (
      ("AdbEnabled", self._adb_toggle),
      ("SshEnabled", self._ssh_toggle),
      ("JoystickDebugMode", self._joystick_toggle),
      ("LongitudinalManeuverMode", self._long_maneuver_toggle),
      ("LateralManeuverMode", self._lat_maneuver_toggle),
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      ("ShowDebugInfo", self._ui_debug_toggle),
      ("LaneCentering", self._lane_centering_toggle),
    ):
      item.action_item.set_state(self._params.get_bool(key))

    # this param defaults to enabled, so read it with its declared default
    self._lane_centering_pause_toggle.action_item.set_state(bool(self._params.get("LaneCenteringPauseOnSignal", return_default=True)))
    self._lane_center_offset_setting.action_item.set_selected_button(
      closest_value_index(LANE_CENTER_OFFSET_VALUES, self._params.get("LaneCenterOffset", return_default=True)))
    self._lane_centering_e2e_authority_setting.action_item.set_selected_button(
      closest_value_index(LANE_CENTERING_E2E_AUTHORITY_VALUES, self._params.get("LaneCenteringE2EAuthority", return_default=True)))
    self._update_lane_centering_settings_enabled(self._params.get_bool("LaneCentering"))

  def _on_enable_ui_debug(self, state: bool):
    self._params.put_bool("ShowDebugInfo", state, block=True)
    gui_app.set_show_touches(state)
    gui_app.set_show_fps(state)

  def _on_enable_adb(self, state: bool):
    self._params.put_bool("AdbEnabled", state, block=True)

  def _on_enable_ssh(self, state: bool):
    self._params.put_bool("SshEnabled", state, block=True)

  def _update_lane_centering_settings_enabled(self, enabled: bool):
    self._lane_centering_pause_toggle.action_item.set_enabled(enabled)
    self._lane_center_offset_setting.action_item.set_enabled(enabled)
    self._lane_centering_e2e_authority_setting.action_item.set_enabled(enabled)

  def _on_lane_centering(self, state: bool):
    self._params.put_bool("LaneCentering", state, block=True)
    self._update_lane_centering_settings_enabled(state)

  def _on_lane_centering_pause_on_signal(self, state: bool):
    self._params.put_bool("LaneCenteringPauseOnSignal", state, block=True)

  def _on_lane_center_offset(self, index: int):
    self._params.put("LaneCenterOffset", LANE_CENTER_OFFSET_VALUES[index], block=True)

  def _on_lane_centering_e2e_authority(self, index: int):
    self._params.put("LaneCenteringE2EAuthority", LANE_CENTERING_E2E_AUTHORITY_VALUES[index], block=True)

  def _on_joystick_debug_mode(self, state: bool):
    self._params.put_bool("JoystickDebugMode", state, block=True)
    self._params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.action_item.set_state(False)
    self._params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.action_item.set_state(False)

  def _on_long_maneuver_mode(self, state: bool):
    self._params.put_bool("LongitudinalManeuverMode", state, block=True)
    self._params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.action_item.set_state(False)
    self._params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.action_item.set_state(False)

  def _on_lat_maneuver_mode(self, state: bool):
    self._params.put_bool("LateralManeuverMode", state, block=True)
    self._params.put_bool("ExperimentalMode", False, block=True)
    self._params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.action_item.set_state(False)
    self._params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.action_item.set_state(False)

  def _on_alpha_long_enabled(self, state: bool):
    if state:
      def confirm_callback(result: DialogResult):
        if result == DialogResult.CONFIRM:
          self._params.put_bool("AlphaLongitudinalEnabled", True, block=True)
          self._params.put_bool("OnroadCycleRequested", True, block=True)
          self._update_toggles()
        else:
          self._alpha_long_toggle.action_item.set_state(False)

      # show confirmation dialog
      content = (f"<h1>{self._alpha_long_toggle.title}</h1><br>" +
                 f"<p>{self._alpha_long_toggle.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      self._params.put_bool("AlphaLongitudinalEnabled", False, block=True)
      self._params.put_bool("OnroadCycleRequested", True, block=True)
      self._update_toggles()
