import unittest
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import (
  E2EAlertsHelper, LEAD_BOX_ACQUIRING, LEAD_BOX_ARMED, LEAD_BOX_DEPARTING, LEAD_BOX_NONE
)


class TestE2EAlertsHelper(unittest.TestCase):
  def setUp(self):
    self.helper = E2EAlertsHelper()
    self.now = 0.0
    self.CS = SimpleNamespace(standstill=True, vEgo=0.0, gasPressed=False)
    self.lead = SimpleNamespace(status=True, dRel=6.0, vRel=0.0, modelProb=0.99, radar=True, radarTrackId=1)
    self.model_x = [5.0 * i / 32 for i in range(33)]

  def update(self, **kwargs):
    self.now = round(self.now + 0.05, 3)
    args = dict(now=self.now, CS=self.CS, lead=self.lead, model_x=self.model_x,
                model_time=self.now, radar_time=self.now, valid=True, lead_enabled=True, green_light_enabled=True)
    args.update(kwargs)
    return self.helper.update(**args)

  def wait_at_stop(self, **kwargs):
    for _ in range(60):
      self.assertFalse(self.update(**kwargs))

  def depart(self, **kwargs):
    self.lead.vRel = 1.5
    results = []
    for _ in range(40):
      self.lead.dRel += 0.075
      results.append(self.update(**kwargs))
    return results

  def model_go(self, **kwargs):
    self.model_x = [40.0 * i / 32 for i in range(33)]
    return [self.update(**kwargs) for _ in range(40)]

  def test_cooldown_rearm_gives_one_chime_per_stop(self):
    # The rearm mode the toggle turns off: without a cooldown the same rolling lead would keep
    # chiming, so this pins down what the off side of the toggle does.
    self.wait_at_stop(rearm_instant=False)
    self.assertEqual(sum(self.depart(rearm_instant=False)), 1)
    self.lead.vRel = 0.0
    self.wait_at_stop(rearm_instant=False)
    self.assertFalse(any(self.depart(rearm_instant=False)))

  def test_rearms_after_driving_to_next_stop(self):
    self.wait_at_stop()
    self.assertEqual(sum(self.depart()), 1)
    self.CS.standstill, self.CS.vEgo = False, 2.0
    self.assertFalse(self.update())
    self.CS.standstill, self.CS.vEgo = True, 0.0
    self.lead.dRel, self.lead.vRel = 6.0, 0.0
    self.wait_at_stop()
    self.assertEqual(sum(self.depart()), 1)

  def test_lead_must_first_be_stationary(self):
    self.lead.vRel = 1.5
    self.wait_at_stop()
    self.assertFalse(any(self.depart()))

  def test_lead_track_change_or_loss_does_not_trigger(self):
    for change in ('track', 'loss', 'vision_jump'):
      with self.subTest(change=change):
        self.setUp()
        if change == 'vision_jump':
          self.lead.radar, self.lead.radarTrackId = False, -1
        self.wait_at_stop()
        if change == 'track':
          self.lead.radarTrackId = 2
        elif change == 'loss':
          self.lead.status = False
          self.update()
          self.lead.status = True
        else:
          self.lead.dRel += 5.0
        self.assertFalse(any(self.depart()))

  def test_distance_noise_or_receding_measurement_without_velocity(self):
    self.wait_at_stop()
    for distance in [6.1, 5.9, 6.3, 5.8] * 10:
      self.lead.dRel = distance
      self.assertFalse(self.update())
    for _ in range(40):
      self.lead.dRel += 0.05
      self.assertFalse(self.update())

  def test_invalid_and_stale_data_discard_armed_evidence(self):
    for kwargs in ({'valid': False}, {'model_time': 0.0}, {'radar_time': 0.0}, {'allowed': False}):
      with self.subTest(kwargs=kwargs):
        self.setUp()
        self.wait_at_stop()
        self.assertFalse(self.update(**kwargs))
        self.assertFalse(any(self.depart()))

  def test_duplicate_messages_do_not_confirm_departure(self):
    self.wait_at_stop()
    self.lead.vRel = 1.5
    for _ in range(16):
      self.lead.dRel += 0.075
      self.assertFalse(self.update())
    frozen_time = self.now
    for _ in range(30):
      self.assertFalse(self.update(model_time=frozen_time, radar_time=frozen_time))

  def test_gas_disengagement_and_invalid_speed_suppress(self):
    for attribute, value in (('gasPressed', True), ('standstill', False), ('vEgo', float('nan')), ('vEgo', 0.5)):
      with self.subTest(attribute=attribute, value=value):
        self.setUp()
        self.wait_at_stop()
        setattr(self.CS, attribute, value)
        self.assertFalse(any(self.depart()))

  def test_disabled_lead_and_low_confidence(self):
    for kwargs in ({'lead_enabled': False}, {}):
      with self.subTest(kwargs=kwargs):
        self.setUp()
        if not kwargs:
          self.lead.modelProb = 0.4
        self.wait_at_stop(**kwargs)
        self.assertFalse(any(self.depart(**kwargs)))

  def test_slow_creep_in_traffic_triggers(self):
    # A lead creeping at walking pace is a departure in stop and go traffic, which a speed gate
    # above its own creeping speed would miss entirely.
    self.wait_at_stop()
    self.lead.vRel = 0.2
    results = []
    for _ in range(120):
      self.lead.dRel += 0.01
      results.append(self.update())
    self.assertEqual(sum(results), 1)

  def test_armed_lead_survives_a_measurement_jump(self):
    # A close, partly occluded lead's range jumps by more than it can move. Dropping the tracker
    # there is what leaves the closest lead without a reminder, so it re-anchors instead.
    self.wait_at_stop()
    self.assertEqual(self.helper.lead_box_state, LEAD_BOX_ARMED)
    self.lead.dRel += 1.5
    for _ in range(20):
      self.assertFalse(self.update())
      self.assertEqual(self.helper.lead_box_state, LEAD_BOX_ARMED)
    self.assertEqual(sum(self.depart()), 1)

  def test_instant_rearm_repeats_within_one_stop(self):
    self.wait_at_stop()
    self.assertEqual(sum(self.depart(rearm_instant=True)), 1)
    self.lead.vRel, self.lead.dRel = 0.0, 6.0
    self.wait_at_stop(rearm_instant=True)
    self.assertEqual(sum(self.depart(rearm_instant=True)), 1)

  def test_instant_rearm_still_requires_the_lead_to_stop_again(self):
    self.wait_at_stop()
    self.assertEqual(sum(self.depart(rearm_instant=True)), 1)
    results = []
    for _ in range(120):
      self.lead.dRel += 0.075
      results.append(self.update(rearm_instant=True))
    self.assertEqual(sum(results), 0)

  def test_instant_rearm_does_not_refire_a_model_still_predicting_go(self):
    self.lead.status = False
    self.wait_at_stop()
    self.assertEqual(sum(self.model_go(rearm_instant=True)), 1)
    self.assertEqual(sum(self.update(rearm_instant=True) for _ in range(60)), 0)

  def test_lead_box_none_without_a_lead(self):
    self.lead.status = False
    for _ in range(60):
      self.assertFalse(self.update())
      self.assertEqual(self.helper.lead_box_state, LEAD_BOX_NONE)

  def test_lead_box_acquires_before_it_arms(self):
    states = []
    for _ in range(60):
      self.assertFalse(self.update())
      states.append(self.helper.lead_box_state)
    self.assertIn(LEAD_BOX_ARMED, states)
    armed = states.index(LEAD_BOX_ARMED)
    self.assertGreater(armed, 20)
    self.assertEqual(states[0], LEAD_BOX_ACQUIRING)
    self.assertEqual(set(states[:armed]), {LEAD_BOX_ACQUIRING})
    self.assertEqual(set(states[armed:]), {LEAD_BOX_ARMED})

  def test_lead_box_none_when_the_lead_reminder_is_disabled(self):
    self.wait_at_stop(lead_enabled=False)
    self.assertEqual(self.helper.lead_box_state, LEAD_BOX_NONE)

  def test_lead_box_departing_only_on_the_chime_frame(self):
    self.wait_at_stop()
    self.assertEqual(self.helper.lead_box_state, LEAD_BOX_ARMED)
    self.lead.vRel = 1.5
    alerts, states = [], []
    for _ in range(40):
      self.lead.dRel += 0.075
      alerts.append(self.update())
      states.append(self.helper.lead_box_state)
    self.assertEqual(states.count(LEAD_BOX_DEPARTING), 1)
    self.assertEqual(states[alerts.index(True)], LEAD_BOX_DEPARTING)
    self.assertEqual(set(states[states.index(LEAD_BOX_DEPARTING) + 1:]), {LEAD_BOX_NONE})

  def test_lead_box_stays_none_during_cooldown_until_the_car_moves(self):
    self.wait_at_stop(rearm_instant=False)
    self.assertEqual(sum(self.depart(rearm_instant=False)), 1)
    for _ in range(60):
      self.assertFalse(self.update(rearm_instant=False))
      self.assertEqual(self.helper.lead_box_state, LEAD_BOX_NONE)
    self.CS.standstill, self.CS.vEgo = False, 2.0
    self.assertFalse(self.update(rearm_instant=False))
    self.assertEqual(self.helper.lead_box_state, LEAD_BOX_NONE)
    self.CS.standstill, self.CS.vEgo = True, 0.0
    self.lead.dRel, self.lead.vRel = 6.0, 0.0
    self.wait_at_stop(rearm_instant=False)
    self.assertEqual(self.helper.lead_box_state, LEAD_BOX_ARMED)

  def test_lead_box_holds_between_radar_frames(self):
    self.wait_at_stop()
    self.assertEqual(self.helper.lead_box_state, LEAD_BOX_ARMED)
    frozen = self.now
    for _ in range(4):
      self.assertFalse(self.update(model_time=frozen, radar_time=frozen))
      self.assertEqual(self.helper.lead_box_state, LEAD_BOX_ARMED)

  def test_lead_box_clears_on_gas_or_ego_movement(self):
    for attribute, value in (('gasPressed', True), ('vEgo', 0.5), ('standstill', False)):
      with self.subTest(attribute=attribute, value=value):
        self.setUp()
        self.wait_at_stop()
        self.assertEqual(self.helper.lead_box_state, LEAD_BOX_ARMED)
        setattr(self.CS, attribute, value)
        self.assertFalse(self.update())
        self.assertEqual(self.helper.lead_box_state, LEAD_BOX_NONE)

  def test_lead_box_reacquires_after_an_implausible_range_jump(self):
    self.lead.radar, self.lead.radarTrackId = False, -1
    self.wait_at_stop()
    self.assertEqual(self.helper.lead_box_state, LEAD_BOX_ARMED)
    self.lead.dRel += 5.0
    states = []
    for _ in range(40):
      self.assertFalse(self.update())
      states.append(self.helper.lead_box_state)
    self.assertEqual(states[0], LEAD_BOX_ACQUIRING)
    self.assertNotIn(LEAD_BOX_ARMED, states[:20])
    self.assertEqual(states[-1], LEAD_BOX_ARMED)
    self.assertEqual(set(states[states.index(LEAD_BOX_ARMED):]), {LEAD_BOX_ARMED})

  def test_lead_box_rearms_within_one_stop_when_instant(self):
    # The outline is the tracker's own view, so re-arming shows it again on the same lead rather
    # than leaving the second creep unmarked.
    self.wait_at_stop()
    self.assertEqual(sum(self.depart(rearm_instant=True)), 1)
    self.lead.vRel, self.lead.dRel = 0.0, 6.0
    states = []
    for _ in range(60):
      self.assertFalse(self.update(rearm_instant=True))
      states.append(self.helper.lead_box_state)
    self.assertIn(LEAD_BOX_ACQUIRING, states)
    self.assertEqual(states[-1], LEAD_BOX_ARMED)

  def test_green_light_requires_stopped_to_go_model_transition(self):
    self.lead.status = False
    self.wait_at_stop()
    self.assertEqual(sum(self.model_go()), 1)
    self.assertFalse(any(self.model_go()))

  def test_open_road_prediction_while_stopped_does_not_trigger(self):
    self.lead.status = False
    self.assertFalse(any(self.model_go()))
    self.assertFalse(any(self.model_go()))

  def test_lost_lead_is_not_a_green_light(self):
    self.wait_at_stop()
    self.lead.status = False
    self.wait_at_stop()
    self.assertFalse(any(self.model_go()))

  def test_green_light_disabled(self):
    self.lead.status = False
    self.wait_at_stop(green_light_enabled=False)
    self.assertFalse(any(self.model_go(green_light_enabled=False)))

  def test_green_light_rejects_malformed_model(self):
    for model_x in ([], [5.0], [float('nan')] * 33, [float('inf')] * 33, [-1.0] * 33):
      with self.subTest(model_x=model_x[:1]):
        self.setUp()
        self.lead.status = False
        self.wait_at_stop()
        self.assertFalse(self.update(model_x=model_x))
        self.assertFalse(any(self.model_go()))

  def test_transient_go_prediction_does_not_trigger(self):
    self.lead.status = False
    self.wait_at_stop()
    for _ in range(4):
      self.assertFalse(self.update(model_x=[40.0] * 33))
    self.assertFalse(self.update())
    for _ in range(4):
      self.assertFalse(self.update(model_x=[40.0] * 33))

  def test_invalid_data_gas_or_toggle_does_not_repeat_chime_within_cooldown(self):
    self.lead.status = False
    self.wait_at_stop(rearm_instant=False)
    self.assertEqual(sum(self.model_go(rearm_instant=False)), 1)
    self.assertFalse(self.update(valid=False, rearm_instant=False))
    self.CS.gasPressed = True
    self.assertFalse(self.update(rearm_instant=False))
    self.CS.gasPressed = False
    self.assertFalse(self.update(lead_enabled=False, green_light_enabled=False, rearm_instant=False))
    self.model_x = [5.0] * 33
    self.wait_at_stop(rearm_instant=False)
    self.assertFalse(any(self.model_go(rearm_instant=False)))


if __name__ == '__main__':
  unittest.main()
