#!/usr/bin/env python3
import unittest
from collections import deque
from types import SimpleNamespace

from cereal import car
from openpilot.selfdrive.car.hyundai.carstate import PREV_BUTTON_SAMPLES
from openpilot.selfdrive.car.hyundai.interface import CarInterface
from openpilot.selfdrive.car.hyundai.values import Buttons, CAR

EventName = car.CarEvent.EventName


class TestHyundaiCancelLateral(unittest.TestCase):
  def make_interface(self, toggle=True, pcm_cruise=True):
    # Exercise the real interface and common events without requiring CAN parser fixtures.
    ci = CarInterface.__new__(CarInterface)
    ci.CP = car.CarParams.new_message(carFingerprint=CAR.ELANTRA_GT_I30, pcmCruise=pcm_cruise,
                                     openpilotLongitudinalControl=not pcm_cruise)
    ci.CS = SimpleNamespace(CP=ci.CP, out=car.CarState.new_message(), prev_cruise_buttons=Buttons.NONE,
                            cruise_buttons=deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES),
                            main_buttons=deque([0] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES))
    ci.cp = ci.cp_cam = None
    ci.disengage_lateral_on_cancel = toggle
    ci.cancel_lateral_latched = False
    ci.low_speed_alert = False
    ci.steering_unpressed = 0
    ci.no_steer_warning = False
    ci.silent_steer_warning = True
    return ci

  def step(self, ci, button=Buttons.NONE, cruise_enabled=True, main=0):
    def update(_cp, _cp_cam):
      ci.CS.prev_cruise_buttons = ci.CS.cruise_buttons[-1]
      ci.CS.cruise_buttons.append(button)
      ci.CS.main_buttons.append(main)
      return car.CarState.new_message(gearShifter=car.CarState.GearShifter.drive,
                                     cruiseState={"available": True, "enabled": cruise_enabled}, vEgo=20.)

    ci.CS.update = update
    ret = ci._update(car.CarControl.new_message())
    ci.CS.out = ret.as_reader()
    return ret

  def assert_event(self, ret, event, present=True):
    self.assertEqual(any(e.name == event for e in ret.events), present)

  def test_stock_acc_cancel_disables_before_pcm(self):
    ci = self.make_interface()
    self.step(ci, button=Buttons.SET_DECEL)
    ret = self.step(ci, button=Buttons.CANCEL, cruise_enabled=True)
    self.assertTrue(ret.cruiseState.enabled)  # stock ACC has not reported its cancellation yet
    self.assert_event(ret, EventName.buttonCancel)

  def test_toggle_off_preserves_stock_acc_behavior(self):
    ci = self.make_interface(toggle=False)
    ret = self.step(ci, button=Buttons.CANCEL)
    self.assertEqual(len(ret.buttonEvents), 0)
    self.assert_event(ret, EventName.buttonCancel, present=False)
    ret = self.step(ci, cruise_enabled=False)
    self.assert_event(ret, EventName.pcmDisable)

  def test_toggle_off_preserves_openpilot_cancel(self):
    ci = self.make_interface(toggle=False, pcm_cruise=False)
    self.assert_event(self.step(ci, button=Buttons.CANCEL), EventName.buttonCancel)

  def test_held_and_released_cancel_blocks_delayed_pcm_enable(self):
    ci = self.make_interface()
    self.step(ci, button=Buttons.CANCEL)
    for button in (Buttons.CANCEL, Buttons.NONE):
      for i in range(20):
        # Even a late cruise status rising edge must not re-enable steering after cancellation.
        ret = self.step(ci, button=button, cruise_enabled=bool(i % 2))
        self.assert_event(ret, EventName.buttonCancel)
        self.assert_event(ret, EventName.pcmEnable, present=False)

  def test_new_set_or_resume_can_reenable(self):
    for button in (Buttons.SET_DECEL, Buttons.RES_ACCEL):
      with self.subTest(button=button):
        ci = self.make_interface()
        self.step(ci, button=Buttons.CANCEL)
        self.step(ci, cruise_enabled=False)
        self.step(ci, button=button, cruise_enabled=False)
        ret = self.step(ci, cruise_enabled=True)
        self.assert_event(ret, EventName.buttonCancel, present=False)
        self.assert_event(ret, EventName.pcmEnable)

  def test_new_main_button_can_reenable(self):
    ci = self.make_interface()
    self.step(ci, button=Buttons.CANCEL, main=1)
    self.step(ci, cruise_enabled=False, main=1)
    self.assertTrue(ci.cancel_lateral_latched)  # an already held Main button cannot override Cancel
    self.step(ci, cruise_enabled=False, main=0)
    ret = self.step(ci, cruise_enabled=True, main=1)
    self.assert_event(ret, EventName.buttonCancel, present=False)
    self.assert_event(ret, EventName.pcmEnable)

  def test_cancel_wins_over_enable_button_release(self):
    ci = self.make_interface(pcm_cruise=False)
    self.step(ci, button=Buttons.SET_DECEL)
    ret = self.step(ci, button=Buttons.CANCEL)
    self.assert_event(ret, EventName.buttonEnable)
    self.assert_event(ret, EventName.buttonCancel)  # USER_DISABLE and NO_ENTRY override ENABLE
    self.assertTrue(ci.cancel_lateral_latched)

  def test_openpilot_long_can_reenable_after_cancel(self):
    ci = self.make_interface(pcm_cruise=False)
    self.step(ci, button=Buttons.CANCEL)
    self.step(ci, cruise_enabled=False)
    self.step(ci, button=Buttons.SET_DECEL, cruise_enabled=False)
    ret = self.step(ci, cruise_enabled=False)
    self.assert_event(ret, EventName.buttonCancel, present=False)
    self.assert_event(ret, EventName.buttonEnable)


if __name__ == "__main__":
  unittest.main()
