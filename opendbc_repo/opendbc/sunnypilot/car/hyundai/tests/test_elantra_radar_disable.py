import unittest
from unittest.mock import patch

from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.hyundai.elantra_longitudinal import (
  CHASSIS_MESSAGES, RADAR_OUTPUT_MESSAGES, initialize_elantra_longitudinal, radar_output_stopped,
)


MODULE = "opendbc.sunnypilot.car.hyundai.elantra_longitudinal"


class FakeClock:
  def __init__(self):
    self.now = 0.0

  def sleep(self, duration):
    self.now += duration


class TestElantraRadarDisable(unittest.TestCase):
  def observe(self, frames):
    clock = FakeClock()
    sent = []
    with patch(f"{MODULE}.time.monotonic", side_effect=lambda: clock.now), \
         patch(f"{MODULE}.time.sleep", side_effect=clock.sleep):
      result = radar_output_stopped(lambda: [frames(clock.now)], sent.extend)
    return result, clock.now, sent

  @staticmethod
  def chassis():
    return [CanData(addr, bytes(8), 0) for addr in CHASSIS_MESSAGES]

  def test_requires_live_chassis_for_observation(self):
    success, duration, sent = self.observe(lambda now: self.chassis())
    self.assertTrue(success)
    self.assertGreaterEqual(duration, 1.0)
    self.assertGreaterEqual(len(sent), 2)
    self.assertTrue(all(msg == CanData(0x7D0, b'\x02\x3e\x80\x00\x00\x00\x00\x00', 0) for msg in sent))

  def test_disconnected_stale_incomplete_or_malformed_chassis_fails(self):
    streams = (
      lambda now: [],
      lambda now: self.chassis() if now < 0.2 else [],
      lambda now: self.chassis() if now > 0.5 else [],
      lambda now: self.chassis()[:-1],
      lambda now: [CanData(addr, bytes(7), 0) for addr in CHASSIS_MESSAGES],
      lambda now: [CanData(addr, bytes(8), 1) for addr in CHASSIS_MESSAGES],
    )
    for stream in streams:
      with self.subTest(stream=stream):
        self.assertFalse(self.observe(stream)[0])

  def test_every_stock_radar_output_aborts(self):
    for addr in RADAR_OUTPUT_MESSAGES:
      with self.subTest(address=hex(addr)):
        self.assertFalse(self.observe(lambda now, addr=addr: self.chassis() + [CanData(addr, bytes(8), 0)])[0])

  def test_even_late_radar_output_aborts(self):
    self.assertFalse(self.observe(lambda now: self.chassis() + ([CanData(0x421, bytes(8), 0)] if now > 0.8 else []))[0])

  def test_echoes_and_other_buses_are_not_radar_output(self):
    frames = [CanData(0x421, bytes(8), bus) for bus in (1, 2, 128, 192)]
    frames.append(CanData(0x7D8, bytes(8), 0))
    self.assertTrue(self.observe(lambda now: self.chassis() + frames)[0])

  def test_success_requires_disable_and_observed_silence(self):
    with patch(f"{MODULE}.disable_ecu", return_value=True) as disable, \
         patch(f"{MODULE}.radar_output_stopped", side_effect=(False, True)) as observe:
      recv, send = object(), object()
      initialize_elantra_longitudinal(recv, send)
      disable.assert_called_once_with(recv, send, bus=0, addr=0x7D0)
      self.assertEqual(observe.call_count, 2)
      observe.assert_called_with(recv, send)

  def test_reinitialization_of_silent_radar_needs_no_diagnostic_write(self):
    with patch(f"{MODULE}.disable_ecu") as disable, patch(f"{MODULE}.radar_output_stopped", return_value=True):
      initialize_elantra_longitudinal(object(), object())
      disable.assert_not_called()

  def test_failure_prevents_startup_and_attempts_restore(self):
    for disabled, silent in ((False, True), (True, False)):
      with self.subTest(disabled=disabled, silent=silent), \
           patch(f"{MODULE}.disable_ecu", return_value=disabled), \
           patch(f"{MODULE}.radar_output_stopped", side_effect=(False, silent)) as observe:
        sent = []
        with self.assertRaisesRegex(RuntimeError, "could not be verified"):
          initialize_elantra_longitudinal(object(), sent.extend)
        self.assertEqual(observe.call_count, 1 + int(disabled))
        self.assertEqual(sent, [CanData(0x7D0, b'\x03\x28\x80\x01\x00\x00\x00\x00', 0)])

  def test_transport_exception_prevents_startup(self):
    for failure in ("disable_ecu", "radar_output_stopped"):
      with self.subTest(failure=failure), patch(f"{MODULE}.disable_ecu", return_value=True), \
           patch(f"{MODULE}.radar_output_stopped", return_value=False), \
           patch(f"{MODULE}.{failure}", side_effect=OSError("transport failed")):
        sent = []
        with self.assertRaises(OSError):
          initialize_elantra_longitudinal(object(), sent.extend)
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0].dat[:4], b'\x03\x28\x80\x01')


if __name__ == "__main__":
  unittest.main()
