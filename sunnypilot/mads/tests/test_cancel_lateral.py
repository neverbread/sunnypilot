import unittest
from types import SimpleNamespace
from unittest.mock import patch

from cereal import car, custom, log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP


ButtonType = car.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
State = log.SelfdriveState.OpenpilotState
MadsState = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState


class TestCancelLateral(unittest.TestCase):
  def make_system(self, toggle=True, mads=True, brand="hyundai"):
    values = {"Mads": mads, "MadsMainCruiseAllowed": True, "MadsUnifiedEngagementMode": False,
              "DisengageLateralOnCancel": toggle, "MadsSteeringMode": 0}
    params = SimpleNamespace(get_bool=lambda key: bool(values.get(key, False)), get=lambda key, **_: values.get(key))
    sd = SimpleNamespace(CP=car.CarParams.new_message(brand=brand), params=params, initialized=True,
                         events=Events(), events_sp=EventsSP(), state_machine=StateMachine(), enabled=True, active=True,
                         CS_prev=self.car_state())
    sd.state_machine.state = State.enabled
    with patch('openpilot.sunnypilot.mads.mads.Params', return_value=params):
      sd.mads = ModularAssistiveDrivingSystem(sd)
    sd.mads.enabled = sd.mads.active = mads
    sd.mads.state_machine.state = MadsState.enabled if mads else MadsState.disabled
    return sd

  @staticmethod
  def car_state(buttons=(), cruise=True, available=True, valid=True):
    return car.CarState.new_message(canValid=valid, vEgo=20., gearShifter=car.CarState.GearShifter.drive,
                                    cruiseState={"enabled": cruise, "available": available},
                                    buttonEvents=[{"type": button, "pressed": pressed} for button, pressed in buttons])

  def step(self, sd, CS=None, events=(), events_sp=()):
    CS = CS if CS is not None else self.car_state()
    sd.events.clear()
    sd.events_sp.clear()
    for event in events:
      sd.events.add(event)
    for event in events_sp:
      sd.events_sp.add(event)
    # Same ordering as SelfdriveD.step, using both production state machines.
    sd.mads.prepare_cancel(CS)
    sd.enabled, sd.active = sd.state_machine.update(sd.events)
    sd.mads.update(CS)
    sd.CS_prev = CS.as_reader()

  def cancel(self, sd):
    self.step(sd, self.car_state([(ButtonType.cancel, True)]))
    self.assertFalse(sd.enabled)
    self.assertFalse(sd.mads.enabled)
    self.assertTrue(sd.mads.cancel_lateral_latched)

  def test_press_and_release_disable_before_stock_cruise_changes(self):
    for pressed in (True, False):
      with self.subTest(pressed=pressed):
        sd = self.make_system()
        self.step(sd, self.car_state([(ButtonType.cancel, pressed)], cruise=True))
        self.assertTrue(sd.CS_prev.cruiseState.enabled)
        self.assertFalse(sd.enabled)
        self.assertFalse(sd.mads.enabled)
        self.assertEqual(sd.mads.state_machine.state, MadsState.disabled)

  def test_stale_pcm_enable_and_main_status_cannot_reengage(self):
    sd = self.make_system()
    self.cancel(sd)
    for i in range(20):
      self.step(sd, self.car_state(cruise=bool(i % 2), available=bool(i % 2)),
                events=(EventName.pcmEnable, EventName.buttonEnable),
                events_sp=(EventNameSP.lkasEnable, EventNameSP.silentLkasEnable))
      self.assertFalse(sd.enabled)
      self.assertFalse(sd.mads.enabled)

  def test_release_or_other_button_does_not_clear_latch(self):
    sd = self.make_system()
    self.cancel(sd)
    for button, pressed in ((ButtonType.cancel, False), (ButtonType.decelCruise, False),
                            (ButtonType.gapAdjustCruise, True), (ButtonType.mainCruise, False)):
      self.step(sd, self.car_state([(button, pressed)]), events=(EventName.pcmEnable,))
      self.assertTrue(sd.mads.cancel_lateral_latched)
      self.assertFalse(sd.enabled)
      self.assertFalse(sd.mads.enabled)

  def test_new_set_or_resume_can_reengage_with_unified_mode_off(self):
    for button in (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise):
      with self.subTest(button=button):
        sd = self.make_system()
        self.cancel(sd)
        self.step(sd, self.car_state([(button, True)]), events=(EventName.pcmEnable,))
        self.assertFalse(sd.mads.cancel_lateral_latched)
        self.assertTrue(sd.enabled)
        self.assertTrue(sd.mads.enabled)

  def test_new_main_action_then_available_edge_allows_lateral(self):
    sd = self.make_system()
    self.cancel(sd)
    self.step(sd, self.car_state([(ButtonType.mainCruise, True)], cruise=False, available=False))
    self.assertFalse(sd.mads.cancel_lateral_latched)
    self.assertFalse(sd.mads.enabled)
    self.step(sd, self.car_state(cruise=False, available=True))
    self.assertTrue(sd.mads.enabled)
    self.assertFalse(sd.enabled)

  def test_new_lkas_press_reengages_steering_only(self):
    sd = self.make_system()
    self.cancel(sd)
    self.step(sd, self.car_state([(ButtonType.lkas, True)], cruise=False))
    self.assertFalse(sd.mads.cancel_lateral_latched)
    self.assertTrue(sd.mads.enabled)
    self.assertFalse(sd.enabled)

  def test_cancel_wins_over_other_engagement_in_same_frame(self):
    sd = self.make_system()
    self.cancel(sd)
    self.step(sd, self.car_state([(ButtonType.cancel, False), (ButtonType.lkas, True), (ButtonType.setCruise, True)]),
              events=(EventName.buttonEnable,))
    self.assertTrue(sd.mads.cancel_lateral_latched)
    self.assertFalse(sd.enabled)
    self.assertFalse(sd.mads.enabled)

  def test_cancel_from_paused_never_resumes_on_brake_release(self):
    sd = self.make_system()
    sd.mads.state_machine.state = MadsState.paused
    sd.mads.active = False
    self.step(sd, self.car_state([(ButtonType.cancel, True)]), events_sp=(EventNameSP.silentLkasDisable,))
    self.assertEqual(sd.mads.state_machine.state, MadsState.disabled)
    for _ in range(5):
      self.step(sd)
      self.assertFalse(sd.mads.enabled)

  def test_toggle_off_keeps_existing_mads_cancel_behavior(self):
    sd = self.make_system(toggle=False)
    self.step(sd, self.car_state([(ButtonType.cancel, True)], cruise=False), events=(EventName.pcmDisable,))
    self.assertFalse(sd.enabled)
    self.assertTrue(sd.mads.enabled)
    self.assertFalse(sd.mads.cancel_lateral_latched)

  def test_option_applies_with_mads_disabled(self):
    sd = self.make_system(mads=False)
    self.cancel(sd)
    self.step(sd, events=(EventName.pcmEnable,))
    self.assertFalse(sd.enabled)

  def test_other_brands_keep_existing_behavior(self):
    sd = self.make_system(brand="toyota")
    self.step(sd, self.car_state([(ButtonType.cancel, True)]), events=(EventName.buttonCancel,))
    self.assertFalse(sd.enabled)
    self.assertTrue(sd.mads.enabled)
    self.assertFalse(sd.mads.cancel_lateral_latched)

  def test_invalid_can_does_not_clear_latch(self):
    sd = self.make_system()
    self.cancel(sd)
    self.step(sd, self.car_state([(ButtonType.lkas, True)], valid=False), events=(EventName.pcmEnable,))
    self.assertTrue(sd.mads.cancel_lateral_latched)
    self.assertFalse(sd.enabled)
    self.assertFalse(sd.mads.enabled)

  def test_reengagement_still_respects_no_entry_conditions(self):
    sd = self.make_system()
    self.cancel(sd)
    self.step(sd, self.car_state([(ButtonType.setCruise, True)]), events=(EventName.pcmEnable, EventName.doorOpen))
    self.assertFalse(sd.enabled)
    self.assertFalse(sd.mads.active)

  def test_brake_blocks_set_resume_reengagement_in_disengage_mode(self):
    for button in (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise):
      with self.subTest(button=button):
        sd = self.make_system()
        sd.mads.steering_mode_on_brake = MadsSteeringModeOnBrake.DISENGAGE
        self.cancel(sd)
        braking = self.car_state(cruise=False)
        braking.brakePressed = True
        self.step(sd, braking, events=(EventName.pedalPressed,))
        braking = self.car_state([(button, True)], cruise=False)
        braking.brakePressed = True
        self.step(sd, braking, events=(EventName.pcmEnable, EventName.pedalPressed))
        self.assertFalse(sd.enabled)
        self.assertFalse(sd.mads.enabled)
        self.assertFalse(sd.mads.active)
        self.assertFalse(sd.events_sp.has(EventNameSP.lkasEnable))
        self.assertTrue(sd.events_sp.has(EventNameSP.pedalPressedAlertOnly))
        self.step(sd, self.car_state(cruise=False))
        self.assertFalse(sd.mads.enabled)


if __name__ == '__main__':
  unittest.main()
