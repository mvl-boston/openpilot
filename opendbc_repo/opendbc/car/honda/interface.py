#!/usr/bin/env python3
import numpy as np
from opendbc.car import get_safety_config, structs, uds
from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.honda.values import CarControllerParams, HondaFlags, CAR, HONDA_BOSCH, HONDA_BOSCH_CANFD, \
                                                 HONDA_NIDEC_ALT_SCM_MESSAGES, HONDA_BOSCH_RADARLESS, HondaSafetyFlags, \
                                                 RADAR_FW_0X280_INGEST
from opendbc.car.honda.carcontroller import CarController
from opendbc.car.honda.carstate import CarState
from opendbc.car.honda.radar_interface import RadarInterface
from opendbc.car.interfaces import CarInterfaceBase

from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP, HondaSafetyFlagsSP

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  DRIVABLE_GEARS = (structs.CarState.GearShifter.sport,)

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed):
    if CP.carFingerprint in HONDA_BOSCH:
      return CarControllerParams.BOSCH_ACCEL_MIN, CarControllerParams.BOSCH_ACCEL_MAX
    elif CP_SP.enableGasInterceptor:
      return CarControllerParams.NIDEC_ACCEL_MIN, CarControllerParams.NIDEC_ACCEL_MAX
    else:
      # NIDECs don't allow acceleration near cruise_speed,
      # so limit limits of pid to prevent windup
      ACCEL_MAX_VALS = [CarControllerParams.NIDEC_ACCEL_MAX, 0.2]
      ACCEL_MAX_BP = [cruise_speed - 2., cruise_speed - .2]
      return CarControllerParams.NIDEC_ACCEL_MIN, np.interp(current_speed, ACCEL_MAX_BP, ACCEL_MAX_VALS)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "honda"

    CAN = CanBus(ret, fingerprint)

    if candidate in HONDA_BOSCH:
      cfgs = [get_safety_config(structs.CarParams.SafetyModel.hondaBosch)]
      if candidate in HONDA_BOSCH_CANFD and CAN.pt >= 4:
        cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
      ret.safetyConfigs = cfgs

      ret.radarUnavailable = True
      # Disable the radar and let openpilot control longitudinal
      # WARNING: THIS DISABLES AEB!
      # If Bosch radarless, this blocks ACC messages from the camera
      ret.alphaLongitudinalAvailable = True
      ret.openpilotLongitudinalControl = alpha_long
      ret.pcmCruise = not ret.openpilotLongitudinalControl

      # 0x280 fine-range radar ingest is GATED on the confirmed radar firmware (RADAR_FW_0X280_INGEST in
      # values.py). Only that radar keeps streaming 0x280 objects under op-long, so only it is kept
      # radar-live (radarUnavailable=False); any other Civic Bosch radar fw keeps the standard HONDA_BOSCH
      # path above (radarUnavailable=True) as the safe default. alpha-long/op-long are standard for all
      # Bosch (set above). Factory AEB does NOT stay live under op-long (accepted). Fail-safe: if car_fw
      # is empty/unknown the radar stays off.
      #
      # "Try-out" toggle (HondaCivicRadarTryout): lets a user enable the 0x280 radar on a Civic Bosch whose
      # radar fw is NOT in the verified fingerprint list, without hand-adding firmware strings. The 0x280
      # fine-range decode is cross-car validated (R2~0.99 across Civic Bosch radars), so a try-out radar is
      # treated IDENTICALLY to a fingerprint-matched one: radar-live, and usable by op-long when the user
      # enables experimental/alpha longitudinal (op-long stays = alpha_long, same as a matched car).
      # EXPERIMENTAL: the decode is reverse-engineered and not validated on every individual car — the user
      # must confirm lead distance/closing-rate before relying on it for longitudinal control.
      _radar_tryout = candidate == CAR.HONDA_CIVIC_BOSCH and not docs and Params().get_bool("HondaCivicRadarTryout")
      _radar_fw_match = candidate == CAR.HONDA_CIVIC_BOSCH and \
        any(fw.ecu == structs.CarParams.Ecu.fwdRadar and RADAR_FW_0X280_INGEST in fw.fwVersion for fw in car_fw)
      if _radar_fw_match or _radar_tryout:
        ret.radarUnavailable = False
    else:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hondaNidec)]
      ret.openpilotLongitudinalControl = True

      ret.pcmCruise = True

    if candidate == CAR.HONDA_CRV_5G:
      ret.enableBsm = 0x12f8bfa7 in fingerprint[CAN.radar]

    # Detect Bosch cars with new HUD msgs
    if any(0x33DA in f for f in fingerprint.values()):
      ret.flags |= HondaFlags.BOSCH_EXT_HUD.value

    if 0x184 in fingerprint[CAN.pt]:
      ret.flags |= HondaFlags.HYBRID.value

    if all(msg not in fingerprint[CAN.pt] for msg in (0x191, 0x1A3)):
      ret.transmissionType = TransmissionType.manual
    elif 0x191 in fingerprint[CAN.pt] and candidate != CAR.ACURA_RDX:
      # Traditional CVTs, gearshift position in GEARBOX_CVT
      ret.transmissionType = TransmissionType.cvt
    else:
      # Traditional autos, direct-drive EVs and eCVTs, gearshift position in GEARBOX_AUTO
      ret.transmissionType = TransmissionType.automatic

    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0], [0]]
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kf = 0.00006  # conservative feed-forward
    ret.steerActuatorDelay = 0.1

    ret.stoppingDecelRate = 0.3  # smooth out harsh braking before standstill
    if candidate in HONDA_BOSCH:
      ret.longitudinalActuatorDelay = 0.5 # s
      # longitudinal gas-only tuning for Bosch hondas is in carcontroller
      if candidate in HONDA_BOSCH_RADARLESS:
        ret.stopAccel = CarControllerParams.BOSCH_ACCEL_MIN  # stock uses -4.0 m/s^2 once stopped but limited by safety model
    else:
      # default longitudinal tuning for all Nidec hondas
      ret.longitudinalTuning.kiBP = [0., 5., 35.]
      ret.longitudinalTuning.kiV = [1.2, 0.8, 0.5]

    if candidate == CAR.HONDA_CITY_7G:
      ret.vEgoStopping = 2.0
      ret.vEgoStarting = ret.vEgoStopping
      ret.stoppingDecelRate = 0.3

    # Disable control if EPS mod detected
    for fw in car_fw:
      if fw.ecu == "eps" and b"," in fw.fwVersion:
        ret.dashcamOnly = True

    if candidate == CAR.HONDA_CIVIC:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 2560], [0, 2560]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[1.1], [0.33]]

    elif candidate in (CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL, CAR.ACURA_INTEGRA):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]
      if candidate == CAR.HONDA_CIVIC_BOSCH:
        CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 750]

    elif candidate == CAR.HONDA_CIVIC_2022:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 5120], [0, 5120]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    elif candidate == CAR.HONDA_ACCORD:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]
      if ret.transmissionType == TransmissionType.manual:
        CarControllerParams.BOSCH_GAS_LOOKUP_BP = [-0.2, 2.0]

    elif candidate == CAR.HONDA_ACCORD_11G:
      ret.steerActuatorDelay = 0.22
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 2560, 5200], [0, 2560, 12747]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.18]]

    elif candidate == CAR.ACURA_ILX:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 3840], [0, 3840]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]

    elif candidate in (CAR.HONDA_CRV, CAR.HONDA_CRV_EU, CAR.HONDA_CRV_SA):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 1000], [0, 1000]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]
      ret.wheelSpeedFactor = 1.025

    elif candidate == CAR.HONDA_CRV_HYBRID:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]
      ret.wheelSpeedFactor = 1.025

    elif candidate in (CAR.HONDA_CRV_6G):
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 5100], [0, 5100]]
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]
      if (ret.flags & HondaFlags.HYBRID):
        CarControllerParams.BOSCH_GAS_LOOKUP_BP = [-0.3, 2.0]

    elif candidate == CAR.HONDA_FIT:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.05]]

    elif candidate == CAR.HONDA_FREED:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.05]]

    elif candidate in (CAR.HONDA_HRV, CAR.HONDA_HRV_3G):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]
      if candidate == CAR.HONDA_HRV:
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.16], [0.025]]
        ret.wheelSpeedFactor = 1.025
      else:
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]  # TODO: can probably use some tuning

    elif candidate == CAR.ACURA_RDX:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 1000], [0, 1000]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]

    elif candidate == CAR.ACURA_RDX_3G:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4095], [0, 4095]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.06]]
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2200]

    elif candidate == CAR.ACURA_RDX_3G_MMR:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4076], [0, 4076]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.06]]
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2000]
      if not ret.openpilotLongitudinalControl:
        # When using stock ACC, the radar intercepts and filters steering commands the EPS would otherwise accept
        ret.minSteerSpeed = 70. * CV.KPH_TO_MS

    elif candidate == CAR.HONDA_ODYSSEY:
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.28], [0.08]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end

    elif candidate == CAR.HONDA_ODYSSEY_TWN:
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.28], [0.08]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 32767], [0, 32767]]  # TODO: determine if there is a dead zone at the top end

    elif candidate in (CAR.HONDA_PILOT, CAR.HONDA_PILOT_4G):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      # ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.38], [0.11]] replace w Marco tune below
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    elif candidate == CAR.ACURA_MDX_4G_MMR:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 2560, 4920], [0, 2560, 12000]]
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    elif candidate == CAR.HONDA_RIDGELINE:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.38], [0.11]]

    elif candidate in (CAR.HONDA_INSIGHT, CAR.HONDA_NBOX_2G):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]

    elif candidate in (CAR.HONDA_E, CAR.HONDA_E_ADVANCE):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]] # TODO: can probably use some tuning

    elif candidate == CAR.HONDA_ODYSSEY_5G_MMR:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.06]]
      ret.steerActuatorDelay = 0.15
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2000]
      if not ret.openpilotLongitudinalControl:
        # When using stock ACC, the radar intercepts and filters steering commands the EPS would otherwise accept
        ret.minSteerSpeed = 70. * CV.KPH_TO_MS

    elif candidate == CAR.ACURA_MDX_4G:
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 2560, 4209], [0, 2560, 9150]]
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    elif candidate == CAR.ACURA_TLX_2G_MMR:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]
      ret.steerActuatorDelay = 0.15
      # try Marco tune below
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    # TODO-SP: remove when https://github.com/commaai/opendbc/pull/2687 is merged
    elif candidate in (
        CAR.HONDA_CLARITY,
        CAR.HONDA_ACCORD_9G,
        CAR.ACURA_MDX_3G,
        CAR.ACURA_MDX_3G_MMR,
        CAR.ACURA_TLX_1G,
      ):
      pass

    elif candidate in CAR.HONDA_FIT_4G:
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.05]]

    elif candidate == CAR.ACURA_ADX:
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 5000], [0, 5000]]
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    elif candidate == CAR.HONDA_PASSPORT_4G:
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 2560, 5120], [0, 2560, 12789]]
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    else:
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 3840], [0, 3840]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]

    if candidate == CAR.HONDA_PILOT_4G:
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2200]

    # These cars use alternate user brake msg (0x1BE)
    if 0x1BE in fingerprint[CAN.pt] and candidate in (CAR.HONDA_ACCORD, CAR.HONDA_HRV_3G, CAR.ACURA_ADX, CAR.ACURA_MDX_4G,
                                                      CAR.ACURA_RDX_3G, *HONDA_BOSCH_CANFD):
      ret.flags |= HondaFlags.BOSCH_ALT_BRAKE.value

    if ret.flags & HondaFlags.BOSCH_ALT_BRAKE:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.ALT_BRAKE.value
    if candidate in HONDA_NIDEC_ALT_SCM_MESSAGES:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.NIDEC_ALT.value
    if ret.openpilotLongitudinalControl and candidate in HONDA_BOSCH:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.BOSCH_LONG.value
    if candidate in HONDA_BOSCH_RADARLESS:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.RADARLESS.value
    if candidate in HONDA_BOSCH_CANFD:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.BOSCH_CANFD.value

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter. Otherwise, add 0.5 mph margin to not
    # conflict with PCM acc
    if (candidate == CAR.HONDA_FIT_4G) and (not ret.openpilotLongitudinalControl):
      ret.autoResumeSng = False
    elif (ret.transmissionType == TransmissionType.manual) and (not ret.openpilotLongitudinalControl):
      ret.autoResumeSng = False
    else:
      ret.autoResumeSng = candidate in (HONDA_BOSCH | {CAR.HONDA_CIVIC})
    if ret.autoResumeSng:
      ret.minEnableSpeed = -1.
    elif candidate == CAR.HONDA_ODYSSEY_TWN:
      ret.minEnableSpeed = 19. * CV.MPH_TO_MS
    elif candidate == CAR.HONDA_FIT_4G:
      ret.minEnableSpeed = 30. * CV.KPH_TO_MS
    else:
      ret.minEnableSpeed = 25.51 * CV.MPH_TO_MS

    ret.steerLimitTimer = 0.8
    ret.radarDelay = 0.1

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    CAN = CanBus(stock_cp, fingerprint)

    for fw in car_fw:
      if fw.ecu == "eps" and b"," in fw.fwVersion:
        ret.flags |= HondaFlagsSP.EPS_MODIFIED.value
        stock_cp.dashcamOnly = False

    if bool(stock_cp.flags & HondaFlags.NIDEC) and bool(stock_cp.flags & HondaFlags.HYBRID):
      ret.flags |= HondaFlagsSP.NIDEC_HYBRID.value
      ret.safetyParam |= HondaSafetyFlagsSP.NIDEC_HYBRID
      # some hybrids use a different brake hold
      if 0x223 in fingerprint[CAN.pt]:
        ret.flags |= HondaFlagsSP.HYBRID_ALT_BRAKEHOLD.value

    if 0x35E in fingerprint[CAN.pt]:
      ret.flags |= HondaFlagsSP.HAS_CAMERA_MESSAGES.value

    if candidate == CAR.HONDA_CLARITY:
      stock_cp.lateralParams.torqueBP, stock_cp.lateralParams.torqueV = [[0, 1663], [0, 1663]]
      stock_cp.lateralTuning.pid.kpBP, stock_cp.lateralTuning.pid.kpV = [[0., 12.], [0.05, 0.03]]
      stock_cp.lateralTuning.pid.kiBP, stock_cp.lateralTuning.pid.kiV = [[0., 12.], [0.03, 0.01]]
      stock_cp.lateralTuning.pid.kf = 0.000012
      stock_cp.steerAtStandstill, stock_cp.autoResumeSng = True, True
      stock_cp.minEnableSpeed, stock_cp.minSteerSpeed = -1.0, -1.0

    elif candidate == CAR.HONDA_CIVIC:
      stock_cp.lateralParams.torqueBP, stock_cp.lateralParams.torqueV = [[0, 3840], [0, 3840]]
      stock_cp.lateralTuning.pid.kpV, stock_cp.lateralTuning.pid.kiV = [[0.06], [0.02]]
      stock_cp.lateralTuning.pid.kf = 0.000024
      stock_cp.steerAtStandstill, stock_cp.autoResumeSng = True, True
      stock_cp.minEnableSpeed, stock_cp.minSteerSpeed = -1.0, -1.0

    elif candidate in (CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL):
      stock_cp.lateralParams.torqueBP, stock_cp.lateralParams.torqueV = [[0, 4096], [0, 4096]]
      stock_cp.lateralTuning.pid.kpV, stock_cp.lateralTuning.pid.kiV = [[0.06], [0.02]]
      stock_cp.lateralTuning.pid.kf = 0.000024
      stock_cp.steerAtStandstill, stock_cp.autoResumeSng = True, True
      stock_cp.minEnableSpeed, stock_cp.minSteerSpeed = -1.0, -1.0

    elif candidate in (CAR.HONDA_INSIGHT, CAR.HONDA_NBOX_2G):
      stock_cp.lateralParams.torqueBP, stock_cp.lateralParams.torqueV = [[0, 3840], [0, 3840]]
      stock_cp.lateralTuning.pid.kpV, stock_cp.lateralTuning.pid.kiV = [[0.06], [0.02]]
      stock_cp.lateralTuning.pid.kf = 0.000024
      stock_cp.steerAtStandstill, stock_cp.autoResumeSng = True, True
      stock_cp.minEnableSpeed, stock_cp.minSteerSpeed = -1.0, -1.0

    elif candidate == CAR.HONDA_CRV_5G:
      stock_cp.lateralParams.torqueBP, stock_cp.lateralParams.torqueV = [[0, 3840], [0, 3840]]
      stock_cp.lateralTuning.pid.kpV, stock_cp.lateralTuning.pid.kiV = [[0.06], [0.02]]
      stock_cp.lateralTuning.pid.kf = 0.000024
      stock_cp.steerAtStandstill, stock_cp.autoResumeSng = True, True
      stock_cp.minEnableSpeed, stock_cp.minSteerSpeed = -1.0, -1.0

    elif candidate == CAR.HONDA_CIVIC_2022:
      if ret.flags & HondaFlagsSP.EPS_MODIFIED:
        stock_cp.lateralParams.torqueBP, stock_cp.lateralParams.torqueV = [[0, 2564, 8000], [0, 2564, 3840]]
        stock_cp.lateralTuning.pid.kpV, stock_cp.lateralTuning.pid.kiV = [[0.3], [0.09]]  # 2.5x Modded EPS

    elif candidate == CAR.HONDA_ACCORD:
      if ret.flags & HondaFlagsSP.EPS_MODIFIED:
        stock_cp.lateralTuning.pid.kpV, stock_cp.lateralTuning.pid.kiV = [[0.3], [0.09]]

    elif candidate in (CAR.ACURA_MDX_3G, CAR.ACURA_MDX_3G_MMR): # source mlocoteta
      stock_cp.autoResumeSng = True
      stock_cp.minEnableSpeed = -1
      stock_cp.steerActuatorDelay = 0.3
      stock_cp.lateralParams.torqueBP, stock_cp.lateralParams.torqueV = [[0, 433], [0, 433]]
      stock_cp.lateralTuning.pid.kf = 0.000035
      stock_cp.lateralTuning.pid.kpV, stock_cp.lateralTuning.pid.kiV = [[0.115], [0.052]]

    elif candidate == CAR.ACURA_TLX_1G:
      stock_cp.autoResumeSng = True
      stock_cp.minEnableSpeed = -1
      stock_cp.steerActuatorDelay = 0.15
      stock_cp.lateralParams.torqueBP, stock_cp.lateralParams.torqueV = [[0, 830], [0, 830]]
      stock_cp.lateralTuning.pid.kf = 0.000035
      stock_cp.lateralTuning.pid.kpV, stock_cp.lateralTuning.pid.kiV = [[0.115], [0.052]]

    elif candidate == CAR.HONDA_ACCORD_9G:
      stock_cp.steerActuatorDelay = 0.3
      stock_cp.lateralParams.torqueBP, stock_cp.lateralParams.torqueV = [[0, 239], [0, 239]]
      stock_cp.lateralTuning.pid.kiBP, stock_cp.lateralTuning.pid.kpBP = [[0., 20], [0., 20]]
      stock_cp.lateralTuning.pid.kpV, stock_cp.lateralTuning.pid.kiV = [[0.4, 0.3], [0, 0]]

    if candidate in HONDA_BOSCH:
      pass
    else:
      ret.enableGasInterceptor = 0x201 in fingerprint[CAN.pt]
      stock_cp.pcmCruise = not ret.enableGasInterceptor

    if ret.enableGasInterceptor and candidate not in HONDA_BOSCH:
      ret.safetyParam |= HondaSafetyFlagsSP.GAS_INTERCEPTOR

    stock_cp.autoResumeSng = stock_cp.autoResumeSng or ret.enableGasInterceptor
    stock_cp.minEnableSpeed = -1. if ret.enableGasInterceptor else stock_cp.minEnableSpeed

    ret.intelligentCruiseButtonManagementAvailable = candidate in (HONDA_BOSCH - HONDA_BOSCH_CANFD) or \
                                                     (candidate in HONDA_BOSCH_CANFD and not is_release_sp)

    return ret

  @staticmethod
  def init(CP, CP_SP, can_recv, can_send, communication_control=None):
    if CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS) and CP.openpilotLongitudinalControl:
      # 0x80 silences response
      if communication_control is None:
        communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.DISABLE_RX_DISABLE_TX,
                                       uds.MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT])
      disable_ecu(can_recv, can_send, bus=CanBus(CP).pt, addr=0x18DAB0F1, com_cont_req=communication_control)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX,
                                   uds.MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT])
    CarInterface.init(CP, can_recv, can_send, communication_control)
