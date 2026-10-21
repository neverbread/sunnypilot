"""Optional informational reminders, independent of engagement state transitions."""

from cereal import car, custom
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.steering_speed_alert import SteeringSpeedAlert


EventNameSP = custom.OnroadEventSP.EventName
LeadDepartState = custom.SelfdriveStateSP.LeadDepartState
GearShifter = car.CarState.GearShifter
# Indexes the helper's LEAD_BOX_* codes into the message's enum; the two lists must stay in step.
LEAD_BOX_STATES = (LeadDepartState.none, LeadDepartState.acquiring, LeadDepartState.armed, LeadDepartState.departing)
FORWARD_GEARS = (GearShifter.drive, GearShifter.sport, GearShifter.low, GearShifter.manumatic)


class DriverNotifications:
  def __init__(self, params):
    self.params = params
    self.e2e_alerts = E2EAlertsHelper()
    self.steering_speed_alert = SteeringSpeedAlert(32 * CV.MPH_TO_MS, CV.MPH_TO_MS)
    self.read_params()

  def read_params(self):
    self.speed_enabled = self.params.get_bool("SteeringSpeedChime")
    self.lead_enabled = self.params.get_bool("LeadDepartAlert")
    self.green_light_enabled = self.params.get_bool("GreenLightAlert")
    # Suppression is shared by both stop reminders, so this speeds up the green light reminder
    # as well. It only ever fires with no lead in frame, where the lead chime cannot compete.
    self.lead_rearm_instant = self.params.get_bool("LeadDepartInstantRearm")

  @property
  def lead_box_state(self):
    """How far the reminder's lead tracker has got, as the message's own enum.

    Display-only: the overlay draws what the reminder decided, so the box and the chime can
    never disagree about which lead they are watching.
    """
    return LEAD_BOX_STATES[self.e2e_alerts.lead_box_state]

  def update(self, CS, sm, *, allowed: bool, selfdrive_enabled: bool, mads_enabled: bool, car_state_fresh: bool) -> list[int]:
    events = []
    valid_car = car_state_fresh and CS.canValid and not CS.canTimeout
    forward_gear = CS.gearShifter in FORWARD_GEARS
    allowed = allowed and forward_gear
    # Paused MADS remains enabled: this reminder concerns crossing the steering
    # speed threshold even when steering is temporarily paused by the brake.
    engaged = selfdrive_enabled or mads_enabled
    if self.steering_speed_alert.update(CS.vEgo, allowed and valid_car and engaged and self.speed_enabled):
      events.append(EventNameSP.steeringSpeedChime)

    radar_errors = sm['radarState'].radarErrors
    radar_valid = not (radar_errors.canError or radar_errors.radarFault or radar_errors.wrongConfig or
                       radar_errors.radarUnavailableTemporary)
    if self.e2e_alerts.update(now=sm.frame * DT_CTRL, CS=CS, lead=sm['radarState'].leadOne,
                              model_x=sm['modelV2'].position.x,
                              model_time=sm.recv_frame['modelV2'] * DT_CTRL,
                              radar_time=sm.recv_frame['radarState'] * DT_CTRL,
                              valid=valid_car and sm.all_checks(['modelV2', 'radarState']) and radar_valid,
                              lead_enabled=self.lead_enabled, green_light_enabled=self.green_light_enabled,
                              allowed=allowed, rearm_instant=self.lead_rearm_instant):
      events.append(EventNameSP.trafficReadyReminder)
    return events
