import unittest

from openpilot.sunnypilot.selfdrive.controls.lib.steering_speed_alert import SteeringSpeedAlert


MPH_TO_MS = 0.44704


class TestSteeringSpeedAlert(unittest.TestCase):
  def setUp(self):
    self.alert = SteeringSpeedAlert(32 * MPH_TO_MS, MPH_TO_MS)

  def update(self, mph, enabled=True):
    return self.alert.update(mph * MPH_TO_MS, enabled)

  def test_downward_crossing(self):
    self.assertFalse(self.update(40))
    self.assertFalse(self.update(32.1))
    self.assertTrue(self.update(32))
    self.assertFalse(self.update(30))

  def test_no_upward_or_startup_chime(self):
    for mph in (0, 20, 32, 33, 40):
      self.assertFalse(self.update(mph))

  def test_engaging_just_above_threshold(self):
    self.assertFalse(self.update(32.5))
    self.assertTrue(self.update(32))
    self.assertFalse(self.update(32.5))
    self.assertFalse(self.update(32))

  def test_hysteresis_and_rearm(self):
    self.update(40)
    self.assertTrue(self.update(31))
    for mph in (31.9, 32.1, 32, 32.9, 31):
      self.assertFalse(self.update(mph))
    self.assertFalse(self.update(33))
    self.assertTrue(self.update(32))

  def test_disabled_resets_crossing(self):
    self.update(40)
    self.assertFalse(self.update(32, enabled=False))
    self.assertFalse(self.update(31))
    self.assertFalse(self.update(40, enabled=False))
    self.assertFalse(self.update(32))
    self.update(40)
    self.assertTrue(self.update(30))

  def test_invalid_speed_resets_crossing(self):
    for invalid in (float('nan'), float('inf'), -1):
      self.update(40)
      self.assertFalse(self.update(invalid))
      self.assertFalse(self.update(32))


if __name__ == '__main__':
  unittest.main()
