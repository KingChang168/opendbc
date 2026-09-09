import random
import re
import unittest
from types import SimpleNamespace

from opendbc.car import DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import CarController, HCAMitigation
from opendbc.car.volkswagen.values import CAR, CarControllerParams as CCP, FW_QUERY_CONFIG, VolkswagenFlags, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenHCAMitigation(unittest.TestCase):
  STUCK_TORQUE_FRAMES = round(CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP))

  def test_same_torque_mitigation(self):
    """Same-torque nudge fires at the threshold, in the correct direction, and resets cleanly."""
    hca_mitigation = HCAMitigation(CCP)

    for actuator_value in (-CCP.STEER_MAX, -1, 0, 1, CCP.STEER_MAX):
      hca_mitigation.update(0, 0)  # Reset mitigation state
      for frame in range(self.STUCK_TORQUE_FRAMES + 2):
        should_nudge = actuator_value != 0 and frame == self.STUCK_TORQUE_FRAMES
        expected_torque = actuator_value - (1, -1)[actuator_value < 0] if should_nudge else actuator_value
        assert hca_mitigation.update(actuator_value, actuator_value) == expected_torque, f"{frame=}"


class TestVolkswagenEmergencyAssist(unittest.TestCase):
  def test_reuses_torque_between_steering_frames(self):
    """Emergency Assist must use the last command on odd, non-HCA send frames."""
    controller = CarController.__new__(CarController)
    controller.frame = 1
    controller.apply_torque_last = 42
    controller.apply_curvature_last = 0.
    controller.accel_last = 0.
    controller.lead_distance_bars_last = 0
    controller.gra_acc_counter_last = 0
    controller.CP = SimpleNamespace(flags=VolkswagenFlags.STOCK_HCA_PRESENT,
                                    openpilotLongitudinalControl=False, pcmCruise=False)
    controller.CCP = SimpleNamespace(STEER_STEP=2, STEER_MAX=300, ACC_CONTROL_STEP=2, LDW_STEP=10, ACC_HUD_STEP=2)
    controller.CAN = SimpleNamespace(cam=2)
    controller.packer_pt = object()

    sent_torques = []
    controller.CCS = SimpleNamespace(create_eps_update=lambda packer, bus, values, torque: sent_torques.append(torque))

    actuators = SimpleNamespace(speed=0., as_builder=lambda: SimpleNamespace())
    hud_control = SimpleNamespace(leadDistanceBars=0)
    CC = SimpleNamespace(actuators=actuators, hudControl=hud_control)
    CC_IC = SimpleNamespace(forceRHDForBSM=False, cruiseSpeedLimitPredicative=False,
                            cruiseSpeedLimitPredReactToSL=False, cruiseSpeedLimitPredReactToCurves=False)
    CS = SimpleNamespace(out=SimpleNamespace(steeringTorque=0.), eps_stock_values={}, gra_stock_values={"COUNTER": 0})

    controller.update(CC, SimpleNamespace(), CC_IC, CS, 0)

    assert sent_torques == [84.]

class TestVolkswagenPlatformConfigs(unittest.TestCase):
  def test_caddy_fwd_camera_radar(self):
    assert CAR.VOLKSWAGEN_CADDY_MK5.config.flags & VolkswagenFlags.FWD_CAMERA_RADAR

  def test_spare_part_fw_pattern(self):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with self.subTest(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self):
    platforms = list(CAR)
    for i, platform in enumerate(platforms):
      with self.subTest(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # Platforms may share chassis codes when their model years disambiguate the VIN.
        for comp in platforms[i + 1:]:
          if not (platform.config.wmis & comp.config.wmis and
                  platform.config.chassis_codes & comp.config.chassis_codes):
            continue

          model_years_overlap = (not platform.config.model_years or not comp.config.model_years or
                                 bool(platform.config.model_years & comp.config.model_years))
          assert not model_years_overlap, f"Ambiguous VIN attributes: {platform} and {comp}"

  def test_custom_fuzzy_fingerprinting(self):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      for wmi in WMI:
        for chassis_code in platform.config.chassis_codes | {"00"}:
          for model_year in platform.config.model_years | {"0"}:
            with self.subTest(platform=platform.name, wmi=wmi, chassis_code=chassis_code, model_year=model_year):
              vin = ["0"] * 17
              vin[0:3] = wmi
              vin[6:8] = chassis_code
              vin[9] = model_year
              vin = "".join(vin)

              # Check a few FW cases - expected, unexpected
              for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
                should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                                (not platform.config.model_years or model_year in platform.config.model_years) and
                                radar_fw in all_radar_fw)

                live_fws = {(0x757, None): [radar_fw]}
                matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

                expected_matches = {platform} if should_match else set()
                assert expected_matches == matches, "Bad match"
