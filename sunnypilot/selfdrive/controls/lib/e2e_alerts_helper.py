import math


MAX_DATA_AGE = 0.25
STOPPED_TIME = 2.0
CONFIRM_TIME = 0.3
LEAD_STOPPED_TIME = 1.0
# A lead counts as stopped up to this relative speed, and as departing above this one. The
# overlap between the two is deliberate: a slow creep should still be able to trigger, which a
# single shared threshold could not do.
LEAD_STOPPED_SPEED = 0.3
LEAD_DEPART_SPEED = 0.15
# After a reminder, require a quieter stop before rearming. Otherwise continuous creep in the
# overlapping acquisition/departure band would repeatedly count as stopping and pulling away.
LEAD_REARM_STOPPED_SPEED = 0.1
# Range drift tolerated before the anchor and the stationary clock restart. Wider than a lead
# can physically move while it still passes the stopped test, so sensor jitter cannot unarm it.
LEAD_REANCHOR_DISTANCE = 1.0
# Past this a range jump is a different object, not a jittered measurement, even for a tracker
# that has already seen the lead sit still. Two vision-only leads share one identity, so this
# bound and the stationary evidence already gathered are all that tells them apart.
LEAD_JUMP_LIMIT = 2.0
# How long a reminder suppresses the next one when rearming by cooldown.
REARM_COOLDOWN = 6.0
GREEN_LIGHT_STOP_DISTANCE = 10.0
GREEN_LIGHT_GO_DISTANCE = 30.0

# Display-only view of the lead tracker, mirroring cereal's SelfdriveStateSP.LeadDepartState.
# Kept as plain numbers so this module stays free of cereal; DriverNotifications maps them.
LEAD_BOX_NONE, LEAD_BOX_ACQUIRING, LEAD_BOX_ARMED, LEAD_BOX_DEPARTING = 0, 1, 2, 3


class E2EAlertsHelper:
  """Reminders for a departing lead or the model's stop-to-go prediction, while the car is stopped.

  rearm_instant matches the LeadDepartInstantRearm param, which is on: the same stop can remind
  again once the lead goes stationary and departs again. Off, a reminder suppresses the next one
  for REARM_COOLDOWN, and the car moving clears that sooner.
  The model trajectory is a driving prediction, not a traffic-light color detector.
  Timestamps are monotonic seconds; repeated messages never advance confirmation timers.
  """

  def __init__(self):
    self.green_light_alert = False
    self.lead_depart_alert = False
    self.lead_box_state = LEAD_BOX_NONE
    self._last_time = None
    self._alerted_until = None
    self._saw_lead = False
    self._lead_needs_restop = False
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
    range_delta, range_jump = 0.0, False
    reject_motion = False
    if self._lead_previous_distance is not None:
      range_delta = abs(lead.dRel - self._lead_previous_distance)
      # Reject implausible range jumps (including switches between vision-only leads).
      range_jump = range_delta > 0.5 + 10.0 * (timestamp - self._lead_previous_time)
    if self._lead_identity != identity or range_delta > LEAD_JUMP_LIMIT:
      self._clear_lead()
    elif range_jump:
      if not self._lead_armed:
        self._clear_lead()
      else:
        # Armed on a lead that has already proved it is stationary: treat the jump as sensor
        # noise rather than a new object, since dropping the tracker here is what makes a
        # close, partially occluded lead never get its reminder. Re-anchor so the jump cannot
        # read as a departure, and discard partial confirmation for the same reason.
        self._lead_distance = lead.dRel
        self._lead_depart_since = None
        reject_motion = True

    self._lead_identity = identity
    self._lead_previous_distance = lead.dRel
    self._lead_previous_time = timestamp

    if reject_motion:
      # A discontinuous range can also produce a spurious velocity. Start gathering fresh
      # motion evidence on the next sample, rather than counting the jump as a departure.
      return False

    if not self._lead_armed:
      stopped_speed = LEAD_REARM_STOPPED_SPEED if self._lead_needs_restop else LEAD_STOPPED_SPEED
      if abs(lead.vRel) <= stopped_speed and lead.dRel < 15.0:
        if self._lead_stopped_since is None or abs(lead.dRel - self._lead_distance) > LEAD_REANCHOR_DISTANCE:
          self._lead_stopped_since = timestamp
          self._lead_distance = lead.dRel
        self._lead_armed = stopped_long_enough and timestamp - self._lead_stopped_since >= LEAD_STOPPED_TIME
        if self._lead_armed:
          self._lead_needs_restop = False
      else:
        self._lead_stopped_since = None
        self._lead_distance = None
      return False

    # Confirm forward motion without waiting for the lead to travel a fixed distance.
    if lead.vRel > LEAD_DEPART_SPEED:
      if self._lead_depart_since is None:
        self._lead_depart_since = timestamp
      return timestamp - self._lead_depart_since >= CONFIRM_TIME

    self._lead_depart_since = None
    return False

  def _lead_box_display(self):
    """How far the lead tracker has got, for the overlay that draws the lead it watches.

    Reads the tracker's own state rather than recomputing anything, so the box cannot drift
    out of step with the chime. DEPARTING is published on the one frame the departure confirms;
    the suppression that follows drops the tracker in both rearm modes, and the overlay holds
    its own fade. In instant mode the tracker then re-acquires the same lead within a second,
    which the overlay shows as the outline coming back.
    """
    if self.lead_depart_alert:
      return LEAD_BOX_DEPARTING
    if self._lead_armed:
      return LEAD_BOX_ARMED
    # The stopped evidence, not merely an identity: a lead already pulling away is tracked, but
    # the reminder is waiting on nothing, and an outline around a receding car reads as a promise
    # it will not keep. This also keeps the overlay the same in both rearm modes.
    if self._lead_stopped_since is not None:
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
             lead_enabled, green_light_enabled, allowed=True, rearm_instant=True):
    self.green_light_alert = False
    self.lead_depart_alert = False
    # Every early return below leaves the overlay with nothing to draw.
    self.lead_box_state = LEAD_BOX_NONE

    if not math.isfinite(now) or (self._last_time is not None and now < self._last_time):
      self._clear_tracking()
      self._last_time = None
      self._alerted_until = None
      return False
    self._last_time = now

    moving = math.isfinite(CS.vEgo) and abs(CS.vEgo) > 0.1
    if moving:
      self._alerted_until = None
      self._saw_lead = False
      self._lead_needs_restop = False

    fresh = all(math.isfinite(t) and 0 <= now - t <= MAX_DATA_AGE for t in (model_time, radar_time))
    if not allowed or not valid or not fresh or not CS.standstill or not math.isfinite(CS.vEgo) or moving or CS.gasPressed:
      # Data loss and a gas tap discard evidence, but do not allow a second chime
      # before the car actually moves.
      self._clear_tracking()
      return False

    self._saw_lead |= lead.status
    if self._alerted_until is not None and now < self._alerted_until:
      # A reminder is still being suppressed. Evidence is discarded meanwhile so that the next
      # reminder has to be earned from scratch, rather than firing the moment the block ends.
      self._clear_tracking()
      return False
    if not (lead_enabled or green_light_enabled):
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
    fired = self.lead_depart_alert or self.green_light_alert
    if fired:
      if self.lead_depart_alert:
        self._lead_needs_restop = True
      if rearm_instant:
        # Drop both trackers' evidence but keep the ego-stopped clock, which is already satisfied,
        # so the next reminder needs only the lead to sit still for LEAD_STOPPED_TIME again. Both
        # trackers clear: a green light that is still showing would otherwise refire every frame.
        self._clear_lead()
        self._clear_green_light()
        self._alerted_until = now
      else:
        self._clear_tracking()
        self._alerted_until = now + REARM_COOLDOWN
    return fired
