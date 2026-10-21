import unittest
from types import SimpleNamespace

from cereal import car, custom
from opendbc.car.common.conversions import Conversions as CV
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.state import StateMachine as SelfdriveStateMachine
from openpilot.sunnypilot.mads.state import StateMachine as MadsStateMachine
from openpilot.sunnypilot.selfdrive.selfdrived.driver_notifications import DriverNotifications
from openpilot.sunnypilot.selfdrive.selfdrived.events import ET, EVENTS_SP, EventsSP


EventName = custom.OnroadEventSP.EventName
GearShifter = car.CarState.GearShifter


class FakeSubMaster(dict):
  def __init__(self, lead):
    super().__init__(radarState=SimpleNamespace(leadOne=lead, radarErrors=SimpleNamespace(canError=False, radarFault=False, wrongConfig=False, radarUnavailableTemporary=False)),
                     modelV2=SimpleNamespace(position=SimpleNamespace(x=[5.0] * 33)))
    self.frame = 0
    self.recv_frame = {'modelV2': 0, 'radarState': 0}
    self.healthy = True
    self.frozen_services = set()

  def all_checks(self, services):
    return self.healthy

  def advance(self):
    self.frame += 5
    for service in self.recv_frame:
      if service not in self.frozen_services:
        self.recv_frame[service] = self.frame


class TestDriverNotifications(unittest.TestCase):
  def setUp(self):
    self.allowed = True
    self.engaged = False
    self.mads_enabled = False
    self.car_state_fresh = True
    # Matches the shipped defaults, so the suite exercises the rearm mode cars actually run.
    self.toggles = dict(SteeringSpeedChime=True, LeadDepartAlert=True, GreenLightAlert=True,
                        LeadDepartInstantRearm=True)
    params = SimpleNamespace(get_bool=lambda key: self.toggles.get(key, False))
    self.notifications = DriverNotifications(params)
    self.lead = SimpleNamespace(status=True, dRel=6.0, vRel=0.0, modelProb=0.99, radar=True, radarTrackId=1)
    self.sm = FakeSubMaster(self.lead)
    self.CS = SimpleNamespace(vEgo=0.0, standstill=True, gasPressed=False, canValid=True, canTimeout=False,
                              gearShifter=GearShifter.drive, cruiseState=SimpleNamespace(enabled=False))

  def update(self):
    self.sm.advance()
    self.notifications.read_params()
    return self.notifications.update(self.CS, self.sm, allowed=self.allowed,
                                     selfdrive_enabled=self.engaged, mads_enabled=self.mads_enabled,
                                     car_state_fresh=self.car_state_fresh)

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
    self.sm['modelV2'].position.x = [40.0] * 33
    return [event for _ in range(40) for event in self.update()]

  def test_speed_requires_enabled(self):
    self.assertEqual(self.speed_crossing(), [])
    self.engaged = True
    self.assertEqual(self.speed_crossing(), [EventName.steeringSpeedChime])
    self.CS.vEgo = 40 * CV.MPH_TO_MS
    self.assertEqual(self.update(), [])
    self.engaged = False
    self.CS.vEgo = 32 * CV.MPH_TO_MS
    self.assertEqual(self.update(), [])
    self.engaged = True
    self.assertEqual(self.update(), [])

  def test_common_notification_gates(self):
    gates = (("self", "allowed", False), ("self", "car_state_fresh", False),
             ("CS", "canValid", False), ("CS", "canTimeout", True),
             ("CS", "gearShifter", GearShifter.park), ("CS", "gearShifter", GearShifter.reverse),
             ("CS", "gearShifter", GearShifter.neutral), ("CS", "gearShifter", GearShifter.unknown))
    for owner, attribute, value in gates:
      for notification in ('speed', 'lead', 'light'):
        with self.subTest(owner=owner, attribute=attribute, value=value, notification=notification):
          self.setUp()
          setattr(self if owner == "self" else self.CS, attribute, value)
          if notification == 'speed':
            self.engaged = True
            self.assertEqual(self.speed_crossing(), [])
          else:
            self.lead.status = notification == 'lead'
            self.wait_at_stop()
            self.assertEqual(self.lead_departure() if notification == 'lead' else self.model_go(), [])

  def test_paused_mads_remains_enabled_for_downward_speed_reminder(self):
    selfdrive = SimpleNamespace(enabled=False, events=Events(), events_sp=EventsSP(), state_machine=SelfdriveStateMachine())
    state_machine = MadsStateMachine(SimpleNamespace(selfdrive=selfdrive))
    state_machine.state = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState.paused
    self.mads_enabled, mads_active = state_machine.update()
    self.assertTrue(self.mads_enabled)
    self.assertFalse(mads_active)
    self.assertFalse(self.engaged)
    self.assertEqual(self.speed_crossing(), [EventName.steeringSpeedChime])
    self.assertEqual(self.update(), [])
    self.CS.vEgo = 40 * CV.MPH_TO_MS
    self.assertEqual(self.update(), [])
    self.car_state_fresh = False
    self.CS.vEgo = 32 * CV.MPH_TO_MS
    self.assertEqual(self.update(), [])

  def test_forward_gears_allow_speed_notification(self):
    for gear in (GearShifter.drive, GearShifter.sport, GearShifter.low, GearShifter.manumatic):
      with self.subTest(gear=gear):
        self.setUp()
        self.CS.gearShifter = gear
        self.engaged = True
        self.assertEqual(self.speed_crossing(), [EventName.steeringSpeedChime])

  def test_departure_with_stock_cruise_or_manual_stopped_driving(self):
    for cruise_enabled in (False, True):
      with self.subTest(cruise_enabled=cruise_enabled):
        self.setUp()
        self.CS.cruiseState.enabled = cruise_enabled
        self.wait_at_stop()
        self.assertEqual(self.lead_departure(), [EventName.trafficReadyReminder])
        self.assertFalse(self.engaged)

  def test_rearm_toggle_decides_whether_one_stop_chimes_twice(self):
    for instant in (False, True):
      with self.subTest(instant=instant):
        self.setUp()
        self.toggles['LeadDepartInstantRearm'] = instant
        self.wait_at_stop()
        self.assertEqual(self.lead_departure(), [EventName.trafficReadyReminder])
        self.lead.vRel, self.lead.dRel = 0.0, 6.0
        self.wait_at_stop()
        self.assertEqual(self.lead_departure(), [EventName.trafficReadyReminder] if instant else [])

  def test_model_stop_to_go_without_cruise(self):
    self.lead.status = False
    self.wait_at_stop()
    self.assertEqual(self.model_go(), [EventName.trafficReadyReminder])
    self.assertFalse(self.engaged)

  def test_disabling_each_parameter_clears_armed_notification(self):
    self.engaged = True
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
    self.assertEqual(self.lead_departure(), [EventName.trafficReadyReminder])
    self.setUp()
    self.toggles['LeadDepartAlert'] = False
    self.lead.status = False
    self.wait_at_stop()
    self.assertEqual(self.model_go(), [EventName.trafficReadyReminder])

  def test_bad_or_frozen_perception_suppresses_traffic_notification(self):
    for fault in ('checks', 'radarErrors', 'modelV2', 'radarState'):
      for notification in ('lead', 'light'):
        with self.subTest(fault=fault, notification=notification):
          self.setUp()
          self.lead.status = notification == 'lead'
          self.wait_at_stop()
          if fault == 'checks':
            self.sm.healthy = False
          elif fault == 'radarErrors':
            self.sm['radarState'].radarErrors.radarFault = True
          else:
            self.sm.frozen_services.add(fault)
          self.assertEqual(self.lead_departure() if notification == 'lead' else self.model_go(), [])

  def test_notifications_have_no_control_state_event_types(self):
    for event in (EventName.trafficReadyReminder, EventName.steeringSpeedChime):
      self.assertEqual(set(EVENTS_SP[event]), {ET.PERMANENT})

  def test_each_radar_fault_suppresses_traffic_notification(self):
    for fault in ('canError', 'radarFault', 'wrongConfig', 'radarUnavailableTemporary'):
      with self.subTest(fault=fault):
        self.setUp()
        self.wait_at_stop()
        setattr(self.sm['radarState'].radarErrors, fault, True)
        self.assertEqual(self.lead_departure(), [])


if __name__ == '__main__':
  unittest.main()
