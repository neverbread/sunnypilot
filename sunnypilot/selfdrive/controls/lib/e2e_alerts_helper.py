import math


MAX_DATA_AGE = 0.25
STOPPED_TIME = 2.0
CONFIRM_TIME = 0.3
LEAD_STOPPED_TIME = 1.0
LEAD_DEPART_DISTANCE = 1.0
GREEN_LIGHT_STOP_DISTANCE = 10.0
GREEN_LIGHT_GO_DISTANCE = 30.0

# Display-only view of the lead tracker, mirroring cereal's SelfdriveStateSP.LeadDepartState.
# Kept as plain numbers so this module stays free of cereal; DriverNotifications maps them.
LEAD_BOX_NONE, LEAD_BOX_ACQUIRING, LEAD_BOX_ARMED, LEAD_BOX_DEPARTING = 0, 1, 2, 3


class E2EAlertsHelper:
  """One reminder per stop, based on a departing lead or the model's stop-to-go prediction.

  The model trajectory is a driving prediction, not a traffic-light color detector.
  Timestamps are monotonic seconds; repeated messages never advance confirmation timers.
  """

  def __init__(self):
    self.green_light_alert = False
    self.lead_depart_alert = False
    self.lead_box_state = LEAD_BOX_NONE
    self._last_time = None
    self._alerted = False
    self._saw_lead = False
    self._clear_tracking()

  def _clear_lead(self):
    self._lead_identity = None
    self._lead_previous_distance = None
    self._lead_previous_time = None
    self._lead_stopped_since = None
    self._lead_distance = None
    self._lead_armed = False
    self._lead_depart_since = None

  def _clear_green_light(self):
    self._model_stopped_since = None
    self._green_light_armed = False
    self._green_light_go_since = None

  def _clear_tracking(self):
    self._stopped_since = None
    self._last_radar_time = None
    self._last_model_time = None
    self._clear_lead()
    self._clear_green_light()

  def _update_lead(self, lead, timestamp, stopped_long_enough):
    valid_lead = lead.status and all(math.isfinite(v) for v in (lead.dRel, lead.vRel, lead.modelProb)) and \
                 0.5 < lead.dRel < 40.0 and lead.modelProb >= 0.7
    if not valid_lead:
      self._clear_lead()
      return False

    identity = (lead.radar, lead.radarTrackId)
    discontinuity = self._lead_identity != identity
    if self._lead_previous_distance is not None:
      # Reject implausible range jumps (including switches between vision-only leads).
      max_change = 0.5 + 10.0 * (timestamp - self._lead_previous_time)
      discontinuity |= abs(lead.dRel - self._lead_previous_distance) > max_change
    if discontinuity:
      self._clear_lead()

    self._lead_identity = identity
    self._lead_previous_distance = lead.dRel
    self._lead_previous_time = timestamp

    if not self._lead_armed:
      if abs(lead.vRel) <= 0.3 and lead.dRel < 15.0:
        if self._lead_stopped_since is None or abs(lead.dRel - self._lead_distance) > 0.5:
          self._lead_stopped_since = timestamp
          self._lead_distance = lead.dRel
        self._lead_armed = stopped_long_enough and timestamp - self._lead_stopped_since >= LEAD_STOPPED_TIME
      else:
        self._lead_stopped_since = None
        self._lead_distance = None
      return False

    if lead.dRel - self._lead_distance > LEAD_DEPART_DISTANCE and lead.vRel > 0.3:
      if self._lead_depart_since is None:
        self._lead_depart_since = timestamp
      return timestamp - self._lead_depart_since >= CONFIRM_TIME

    self._lead_depart_since = None
    return False

  def _lead_box_display(self):
    """How far the lead tracker has got, for the overlay that draws the lead it watches.

    Reads the tracker's own state rather than recomputing anything, so the box cannot drift
    out of step with the chime. DEPARTING is published on the one frame the departure confirms;
    the next frame's _alerted latch clears the tracker, and the overlay holds its own fade.
    """
    if self.lead_depart_alert:
      return LEAD_BOX_DEPARTING
    if self._lead_armed:
      return LEAD_BOX_ARMED
    if self._lead_identity is not None:
      return LEAD_BOX_ACQUIRING
    return LEAD_BOX_NONE

  def _update_green_light(self, model_x, timestamp, stopped_long_enough):
    if len(model_x) != 33 or not all(math.isfinite(x) for x in model_x) or model_x[-1] < 0:
      self._clear_green_light()
      return False

    if not self._green_light_armed:
      # Seeing a short, stopped trajectory first avoids chiming at an ordinary stop
      # where the model has predicted an open road the whole time.
      if model_x[-1] <= GREEN_LIGHT_STOP_DISTANCE:
        if self._model_stopped_since is None:
          self._model_stopped_since = timestamp
        self._green_light_armed = stopped_long_enough and timestamp - self._model_stopped_since >= LEAD_STOPPED_TIME
      else:
        self._model_stopped_since = None
      return False

    if model_x[-1] > GREEN_LIGHT_GO_DISTANCE:
      if self._green_light_go_since is None:
        self._green_light_go_since = timestamp
      return timestamp - self._green_light_go_since >= CONFIRM_TIME

    self._green_light_go_since = None
    return False

  def update(self, *, now, CS, lead, model_x, model_time, radar_time, valid,
             lead_enabled, green_light_enabled, allowed=True):
    self.green_light_alert = False
    self.lead_depart_alert = False
    # Every early return below leaves the overlay with nothing to draw.
    self.lead_box_state = LEAD_BOX_NONE

    if not math.isfinite(now) or (self._last_time is not None and now < self._last_time):
      self._clear_tracking()
      self._last_time = None
      return False
    self._last_time = now

    moving = math.isfinite(CS.vEgo) and abs(CS.vEgo) > 0.1
    if moving:
      self._alerted = False
      self._saw_lead = False

    fresh = all(math.isfinite(t) and 0 <= now - t <= MAX_DATA_AGE for t in (model_time, radar_time))
    if not allowed or not valid or not fresh or not CS.standstill or not math.isfinite(CS.vEgo) or moving or CS.gasPressed:
      # Data loss and a gas tap discard evidence, but do not allow a second chime
      # before the car actually moves.
      self._clear_tracking()
      return False

    self._saw_lead |= lead.status
    if self._alerted or not (lead_enabled or green_light_enabled):
      self._clear_tracking()
      return False

    if self._stopped_since is None:
      self._stopped_since = now
    stopped_long_enough = now - self._stopped_since >= STOPPED_TIME

    if any(previous is not None and current < previous for current, previous in
           ((model_time, self._last_model_time), (radar_time, self._last_radar_time))):
      self._clear_tracking()
      return False

    if not lead_enabled:
      self._clear_lead()
    elif radar_time != self._last_radar_time:
      self.lead_depart_alert = self._update_lead(lead, radar_time, stopped_long_enough)
    # Derived on every selfdrived frame, not only on new radar frames: the tracker's state
    # persists between them, and the overlay is republished at selfdriveStateSP's rate.
    self.lead_box_state = self._lead_box_display()

    if not green_light_enabled or self._saw_lead:
      self._clear_green_light()
    elif model_time != self._last_model_time:
      self.green_light_alert = self._update_green_light(model_x, model_time, stopped_long_enough)

    self._last_model_time = model_time
    self._last_radar_time = radar_time
    self._alerted = self.lead_depart_alert or self.green_light_alert
    return self._alerted
