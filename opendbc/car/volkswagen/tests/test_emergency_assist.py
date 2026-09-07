import unittest

from opendbc.car import Bus
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, VolkswagenFlags


class TestEmergencyAssist(unittest.TestCase):
  @staticmethod
  def make_state(platform, present):
    cp = CarInterface.get_non_essential_params(platform)
    if present:
      cp.flags |= VolkswagenFlags.STOCK_EA_PRESENT.value
    else:
      cp.flags &= ~VolkswagenFlags.STOCK_EA_PRESENT.value
    sp = CarInterface.get_non_essential_params_sp(cp, platform)
    ic = CarInterface.get_non_essential_params_ic(cp, platform)
    return CarState(cp, sp, ic), CarState.get_can_parsers(cp, sp)

  def test_absent_ea_is_not_registered(self):
    for platform in (CAR.VOLKSWAGEN_GOLF_MK8, CAR.VOLKSWAGEN_CADDY_MK5, CAR.VOLKSWAGEN_ID3_MK1, CAR.SKODA_KODIAQ_MK1):
      with self.subTest(platform=platform):
        state, parsers = self.make_state(platform, False)
        ret, _, _ = state.update(parsers)
        self.assertFalse(ret.carFaultedNonCritical)
        self.assertNotIn("EA_01", parsers[Bus.cam].vl)
        self.assertNotIn("EA_02", parsers[Bus.cam].vl)
        self.assertNotIn(0x1A4, parsers[Bus.cam].message_states)

  def test_present_ea_fault_detection(self):
    for status in range(8):
      with self.subTest(status=status):
        state, parsers = self.make_state(CAR.VOLKSWAGEN_GOLF_MK8, True)
        parsers[Bus.cam].vl["EA_01"]["EA_Funktionsstatus"] = status
        ret, _, _ = state.update(parsers)
        self.assertEqual(ret.carFaultedNonCritical, status in (3, 4, 5, 6))
        self.assertIn("EA_02", parsers[Bus.cam].vl)


if __name__ == "__main__":
  unittest.main()
