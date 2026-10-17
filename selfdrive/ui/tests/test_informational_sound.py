import unittest

import numpy as np

from cereal import car, custom, log
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import ET, Events
from openpilot.selfdrive.ui.soundd import MAX_VOLUME, SAMPLE_BUFFER, Soundd, sound_list
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP


AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName


class TestInformationalSound(unittest.TestCase):
  def setUp(self):
    # Exercise the real playback and Quiet Mode methods without audio hardware
    # or a device Params store. More than one callback is needed to finish.
    self.sound = Soundd.__new__(Soundd)
    self.sound.enabled = False
    self.sound.current_alert = AudibleAlert.none
    self.sound.current_sound_frame = 0
    self.sound.current_volume = MAX_VOLUME
    self.sound.loaded_sounds = {alert: np.ones(5000, dtype=np.float32) for alert in sound_list}

  def finish_sound(self):
    self.sound.get_sound_data(SAMPLE_BUFFER)
    self.sound.get_sound_data(SAMPLE_BUFFER)
    self.assertFalse(np.any(self.sound.get_sound_data(SAMPLE_BUFFER)))

  def test_completed_low_speed_prompt_does_not_suppress_reminder(self):
    self.sound.update_alert(AudibleAlert.prompt)
    self.finish_sound()
    self.sound.update_alert(AudibleAlert.informationalPrompt)
    self.assertEqual(self.sound.current_sound_frame, 0)
    self.assertTrue(np.any(self.sound.get_sound_data(SAMPLE_BUFFER)))

  def test_repeated_reminder_messages_play_only_once(self):
    self.sound.update_alert(AudibleAlert.informationalPrompt)
    self.sound.get_sound_data(SAMPLE_BUFFER)
    frame = self.sound.current_sound_frame
    self.sound.update_alert(AudibleAlert.informationalPrompt)
    self.assertEqual(self.sound.current_sound_frame, frame)
    self.finish_sound()
    frame = self.sound.current_sound_frame
    for _ in range(10):
      self.sound.update_alert(AudibleAlert.informationalPrompt)
      self.assertEqual(self.sound.current_sound_frame, frame)
      self.assertFalse(np.any(self.sound.get_sound_data(SAMPLE_BUFFER)))

  def test_next_reminder_can_play_after_silence(self):
    self.sound.update_alert(AudibleAlert.informationalPrompt)
    self.finish_sound()
    self.sound.update_alert(AudibleAlert.none)
    self.sound.update_alert(AudibleAlert.informationalPrompt)
    self.assertEqual(self.sound.current_sound_frame, 0)
    self.assertTrue(np.any(self.sound.get_sound_data(SAMPLE_BUFFER)))

  def test_stock_sounds_and_enum_values_are_unchanged(self):
    expected = {
      'engage': (1, 'engage.wav', 1),
      'disengage': (2, 'disengage.wav', 1),
      'refuse': (3, 'refuse.wav', 1),
      'warningSoft': (4, 'warning_soft.wav', None),
      'warningImmediate': (5, 'warning_immediate.wav', None),
      'prompt': (6, 'prompt.wav', 1),
      'promptRepeat': (7, 'prompt.wav', None),
      'promptDistracted': (8, 'prompt_distracted.wav', None),
    }
    self.assertEqual(AudibleAlert.none, 0)
    for name, (value, filename, loops) in expected.items():
      with self.subTest(sound=name):
        self.assertEqual(getattr(AudibleAlert, name), value)
        self.assertEqual(sound_list[value], (filename, loops, MAX_VOLUME))
    self.assertEqual(AudibleAlert.informationalPrompt, 9)
    self.assertEqual(sound_list[AudibleAlert.informationalPrompt], ('prompt.wav', 1, MAX_VOLUME))

  def test_reminders_respect_quiet_mode(self):
    self.assertTrue(self.sound.should_play_sound(AudibleAlert.informationalPrompt))
    self.sound.enabled = True
    self.assertFalse(self.sound.should_play_sound(AudibleAlert.informationalPrompt))
    self.assertFalse(self.sound.should_play_sound(AudibleAlert.prompt))
    for alert in (AudibleAlert.warningImmediate, AudibleAlert.warningSoft, AudibleAlert.promptRepeat, AudibleAlert.promptDistracted):
      self.assertTrue(self.sound.should_play_sound(alert))

  def test_reminder_wins_over_older_low_speed_warning(self):
    for reminder in (EventNameSP.steeringSpeedChime, EventNameSP.trafficReadyReminder):
      with self.subTest(reminder=reminder):
        manager = AlertManager()
        events = Events()
        events.add(EventName.belowSteerSpeed)
        args = [car.CarParams.new_message(minSteerSpeed=14.30528), None, None, False, 0, None]
        for frame in range(101):
          alerts = events.create_alerts([ET.WARNING], args)
          if frame == 100:
            notifications = EventsSP()
            notifications.add(reminder)
            alerts += notifications.create_alerts([ET.PERMANENT])
          manager.add_many(frame, alerts)
          manager.process_alerts(frame, set())
          expected = AudibleAlert.informationalPrompt if frame == 100 else AudibleAlert.prompt
          self.assertEqual(manager.current_alert.audible_alert, expected)

  def test_urgent_warning_has_priority_over_reminder(self):
    manager = AlertManager()
    events = Events()
    events.add(EventName.steerUnavailable)
    manager.add_many(0, events.create_alerts([ET.IMMEDIATE_DISABLE]))
    notifications = EventsSP()
    notifications.add(EventNameSP.steeringSpeedChime)
    manager.add_many(1, notifications.create_alerts([ET.PERMANENT]))
    manager.process_alerts(1, set())
    self.assertEqual(manager.current_alert.audible_alert, AudibleAlert.warningImmediate)

  def test_older_equal_priority_warning_is_not_hidden(self):
    for warning, event_type in ((EventName.steerSaturated, ET.WARNING), (EventName.ldw, ET.PERMANENT)):
      for reminder in (EventNameSP.steeringSpeedChime, EventNameSP.trafficReadyReminder):
        with self.subTest(warning=warning, reminder=reminder):
          manager = AlertManager()
          events = Events()
          events.add(warning)
          warning_alerts = events.create_alerts([event_type])
          manager.add_many(0, warning_alerts)
          notifications = EventsSP()
          notifications.add(reminder)
          manager.add_many(1, notifications.create_alerts([ET.PERMANENT]))
          manager.process_alerts(1, set())
          self.assertIs(manager.current_alert, warning_alerts[0])

  def test_expired_or_cleared_warning_does_not_block_reminder(self):
    for expired in (False, True):
      with self.subTest(expired=expired):
        manager = AlertManager()
        events = Events()
        events.add(EventName.steerSaturated)
        warning_alerts = events.create_alerts([ET.WARNING])
        manager.add_many(0, warning_alerts)
        frame = warning_alerts[0].duration + 1 if expired else 1
        notifications = EventsSP()
        notifications.add(EventNameSP.steeringSpeedChime)
        manager.add_many(frame, notifications.create_alerts([ET.PERMANENT]))
        manager.process_alerts(frame, set() if expired else {ET.WARNING})
        self.assertEqual(manager.current_alert.audible_alert, AudibleAlert.informationalPrompt)

  def test_other_alerts_keep_existing_priority_and_recency_order(self):
    for first, second in ((EventName.ldw, EventName.steerSaturated), (EventName.steerSaturated, EventName.ldw)):
      with self.subTest(first=first, second=second):
        manager = AlertManager()
        for frame, event in enumerate((first, second)):
          events = Events()
          events.add(event)
          alerts = events.create_alerts([ET.WARNING, ET.PERMANENT])
          manager.add_many(frame, alerts)
          manager.process_alerts(frame, set())
          self.assertIs(manager.current_alert, alerts[0])


if __name__ == '__main__':
  unittest.main()
