import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.selfdrive.controls.lib.events import ET, EVENTS, Events
from openpilot.selfdrive.controls.lib.steering_speed_alert import SteeringSpeedAlert


EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
State = log.ControlsState.OpenpilotState


class FakeSubMaster(dict):
  def __init__(self, lead):
    super().__init__(radarState=SimpleNamespace(leadOne=lead, radarErrors=[]),
                     modelV2=SimpleNamespace(position=SimpleNamespace(x=[5.0] * 33)))
    self.frame = 0
    self.rcv_frame = {'modelV2': 0, 'radarState': 0}
    self.healthy = True
    self.frozen_services = set()

  def all_checks(self, services):
    return self.healthy

  def advance(self):
    self.frame += 5
    for service in self.rcv_frame:
      if service not in self.frozen_services:
        self.rcv_frame[service] = self.frame


class TestDriverNotifications(unittest.TestCase):
  def setUp(self):
    # Exercise the production wiring without opening sockets or initializing a car.
    self.controls = Controls.__new__(Controls)
    self.controls.initialized = True
    self.controls.read_only = False
    self.controls.joystick_mode = False
    self.controls.CP = SimpleNamespace(notCar=False, openpilotLongitudinalControl=False)
    self.controls.enabled = False
    self.controls.active = False
    self.controls.state = State.disabled
    self.controls.events = Events()
    self.controls.e2e_alerts = E2EAlertsHelper()
    self.controls.steering_speed_alert = SteeringSpeedAlert(32 * CV.MPH_TO_MS, CV.MPH_TO_MS)
    self.toggles = dict(SteeringSpeedChime=True, LeadDepartAlert=True, GreenLightAlert=True)
    self.controls.params = SimpleNamespace(get_bool=lambda key: self.toggles.get(key, False))
    self.lead = SimpleNamespace(status=True, dRel=6.0, vRel=0.0, modelProb=0.99, radar=True, radarTrackId=1)
    self.controls.sm = FakeSubMaster(self.lead)
    self.CS = SimpleNamespace(vEgo=0.0, standstill=True, gasPressed=False, canValid=True, canTimeout=False,
                              gearShifter=GearShifter.drive, cruiseState=SimpleNamespace(enabled=False))

  def update(self):
    self.controls.sm.advance()
    self.controls.events.clear()
    self.controls.update_driver_notifications(self.CS)
    return list(self.controls.events.names)

  def speed_crossing(self):
    self.CS.standstill = False
    self.CS.vEgo = 40 * CV.MPH_TO_MS
    self.assertEqual(self.update(), [])
    self.CS.vEgo = 32 * CV.MPH_TO_MS
    return self.update()

  def wait_at_stop(self):
    for _ in range(60):
      self.assertEqual(self.update(), [])

  def lead_departure(self):
    self.lead.vRel = 1.5
    events = []
    for _ in range(40):
      self.lead.dRel += 0.075
      events.extend(self.update())
    return events

  def model_go(self):
    self.controls.sm['modelV2'].position.x = [40.0] * 33
    return [event for _ in range(40) for event in self.update()]

  def test_speed_requires_enabled(self):
    self.assertEqual(self.speed_crossing(), [])
    self.controls.enabled = True
    self.assertEqual(self.speed_crossing(), [EventName.steeringSpeedChime])
    self.CS.vEgo = 40 * CV.MPH_TO_MS
    self.assertEqual(self.update(), [])
    self.controls.enabled = False
    self.CS.vEgo = 32 * CV.MPH_TO_MS
    self.assertEqual(self.update(), [])
    self.controls.enabled = True
    self.assertEqual(self.update(), [])

  def test_common_notification_gates(self):
    gates = (('controls', 'initialized', False), ('controls', 'read_only', True), ('controls', 'joystick_mode', True),
             ('CP', 'notCar', True), ('CS', 'canValid', False), ('CS', 'canTimeout', True),
             ('CS', 'gearShifter', GearShifter.park), ('CS', 'gearShifter', GearShifter.reverse),
             ('CS', 'gearShifter', GearShifter.neutral), ('CS', 'gearShifter', GearShifter.unknown))
    for owner, attribute, value in gates:
      for notification in ('speed', 'lead', 'light'):
        with self.subTest(owner=owner, attribute=attribute, value=value, notification=notification):
          self.setUp()
          target = {'controls': self.controls, 'CP': self.controls.CP, 'CS': self.CS}[owner]
          setattr(target, attribute, value)
          if notification == 'speed':
            self.controls.enabled = True
            self.assertEqual(self.speed_crossing(), [])
          else:
            self.lead.status = notification == 'lead'
            self.wait_at_stop()
            self.assertEqual(self.lead_departure() if notification == 'lead' else self.model_go(), [])

  def test_forward_gears_allow_speed_notification(self):
    for gear in (GearShifter.drive, GearShifter.sport, GearShifter.low, GearShifter.manumatic):
      with self.subTest(gear=gear):
        self.setUp()
        self.CS.gearShifter = gear
        self.controls.enabled = True
        self.assertEqual(self.speed_crossing(), [EventName.steeringSpeedChime])

  def test_departure_with_stock_cruise_or_manual_stopped_driving(self):
    for cruise_enabled in (False, True):
      with self.subTest(cruise_enabled=cruise_enabled):
        self.setUp()
        self.CS.cruiseState.enabled = cruise_enabled
        self.wait_at_stop()
        self.assertEqual(self.lead_departure(), [EventName.e2eLongStart])
        self.assertFalse(self.controls.enabled)
        self.assertEqual(self.controls.state, State.disabled)

  def test_model_stop_to_go_without_cruise(self):
    self.lead.status = False
    self.wait_at_stop()
    self.assertEqual(self.model_go(), [EventName.e2eLongStart])
    self.assertFalse(self.controls.enabled)
    self.assertEqual(self.controls.state, State.disabled)

  def test_disabling_each_parameter_clears_armed_notification(self):
    self.controls.enabled = True
    self.CS.vEgo = 40 * CV.MPH_TO_MS
    self.assertEqual(self.update(), [])
    self.toggles['SteeringSpeedChime'] = False
    self.CS.vEgo = 32 * CV.MPH_TO_MS
    self.assertEqual(self.update(), [])

    for param in ('LeadDepartAlert', 'GreenLightAlert'):
      with self.subTest(param=param):
        self.setUp()
        self.lead.status = param == 'LeadDepartAlert'
        self.wait_at_stop()
        self.toggles[param] = False
        self.assertEqual(self.lead_departure() if self.lead.status else self.model_go(), [])

  def test_traffic_toggles_are_independent(self):
    self.toggles['GreenLightAlert'] = False
    self.wait_at_stop()
    self.assertEqual(self.lead_departure(), [EventName.e2eLongStart])
    self.setUp()
    self.toggles['LeadDepartAlert'] = False
    self.lead.status = False
    self.wait_at_stop()
    self.assertEqual(self.model_go(), [EventName.e2eLongStart])

  def test_bad_or_frozen_perception_suppresses_traffic_notification(self):
    for fault in ('checks', 'radarErrors', 'modelV2', 'radarState'):
      for notification in ('lead', 'light'):
        with self.subTest(fault=fault, notification=notification):
          self.setUp()
          self.lead.status = notification == 'lead'
          self.wait_at_stop()
          if fault == 'checks':
            self.controls.sm.healthy = False
          elif fault == 'radarErrors':
            self.controls.sm['radarState'].radarErrors = ['fault']
          else:
            self.controls.sm.frozen_services.add(fault)
          self.assertEqual(self.lead_departure() if notification == 'lead' else self.model_go(), [])

  def test_notification_events_do_not_change_control_state(self):
    self.controls.v_cruise_helper = Mock()
    self.controls.is_metric = False
    self.controls.reverse_acc_change = False
    self.controls.soft_disable_timer = 0
    self.controls.experimental_mode = False
    for event in (EventName.e2eLongStart, EventName.steeringSpeedChime):
      self.assertEqual(set(EVENTS[event]), {ET.PERMANENT})
      for state in (State.disabled, State.enabled):
        with self.subTest(event=event, state=state):
          self.controls.events.clear()
          self.controls.events.add(event)
          self.controls.state = state
          self.controls.enabled = self.controls.active = state == State.enabled
          self.controls.state_transition(self.CS)
          self.assertEqual(self.controls.state, state)
          self.assertEqual(self.controls.enabled, state == State.enabled)
          self.assertEqual(self.controls.active, state == State.enabled)
    self.controls.v_cruise_helper.initialize_v_cruise.assert_not_called()


if __name__ == '__main__':
  unittest.main()
