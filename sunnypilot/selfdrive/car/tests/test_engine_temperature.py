"""Transport/decoder tests runnable without cereal, native CAN, or hardware."""

import importlib.util
import itertools
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location("engine_temperature", Path(__file__).parents[1] / "engine_temperature.py")
assert spec is not None and spec.loader is not None
temperature = importlib.util.module_from_spec(spec)
spec.loader.exec_module(temperature)

OIL_PAYLOAD = bytes.fromhex(
  "62E001D9BDFFFFD2D1AE940A44000000008C00622B80210BAA017100B300006533034A0179B54F0A0100000000000000000000"
)
COOLANT_PAYLOAD = bytes.fromhex("41057E0B2B")
CLUTCH_PAYLOAD = bytes.fromhex(
  "6201A4FFFFFF800A9E002900000005C042120A9E0851C11408070101F0FDC0FF003F002300000000000000000000000000"
)


def frame(data, address=0x7E8, bus=1):
  return address, data, bus


def single_frame(payload, address=0x7E8):
  return frame((bytes([len(payload)]) + payload).ljust(8, b"\x00"), address)


def multiframe(payload, address):
  frames = [frame(bytes([0x10, len(payload)]) + payload[:6], address)]
  for sequence, start in enumerate(range(6, len(payload), 7), 1):
    frames.append(frame((bytes([0x20 | sequence]) + payload[start:start + 7]).ljust(8, b"\x00"), address))
  return frames


def oil_frames(payload=OIL_PAYLOAD):
  return multiframe(payload, 0x7E8)


def clutch_frames(payload=CLUTCH_PAYLOAD):
  return multiframe(payload, 0x7E9)


class TestEngineTemperature(unittest.TestCase):
  def setUp(self):
    self.reader = temperature.EngineTemperatureReader()

  def start_oil(self):
    self.assertEqual(self.reader.update(0.0, []), [temperature.COOLANT_REQUEST])
    self.assertEqual(self.reader.update(1.0, []), [temperature.OIL_REQUEST])

  def assert_unavailable(self, now):
    self.assertFalse(self.reader.readings(now)["oilValid"])
    self.assertFalse(self.reader.readings(now)["coolantValid"])

  def test_recorded_decoding(self):
    self.assertEqual(temperature.decode_oil_temperature(OIL_PAYLOAD), 87.75)
    self.assertEqual(temperature.decode_coolant_temperature(COOLANT_PAYLOAD), 86.0)
    self.assertEqual(temperature.decode_coolant_temperature(COOLANT_PAYLOAD[:3]), 86.0)

  def test_decode_exact_lengths_and_prefixes(self):
    for payload in (b"", OIL_PAYLOAD[:-1], OIL_PAYLOAD + b"\x00", b"\x62\xe0\x02" + OIL_PAYLOAD[3:]):
      with self.subTest(oil=payload.hex()):
        self.assertIsNone(temperature.decode_oil_temperature(payload))
    for payload in (b"", b"\x41\x05", b"\x41\x05\x7e\x00", b"\x41\x05\x7e\x0c\x2b",
                    b"\x41\x04\x7e", COOLANT_PAYLOAD + b"\x00"):
      with self.subTest(coolant=payload.hex()):
        self.assertIsNone(temperature.decode_coolant_temperature(payload))

  def test_unsigned_bytes_and_cold_temperatures(self):
    self.assertEqual(temperature.decode_coolant_temperature(b"\x41\x05\x00"), -40.0)
    self.assertEqual(temperature.decode_coolant_temperature(b"\x41\x05\xff"), 215.0)
    oil = bytearray(OIL_PAYLOAD)
    oil[37] = 0
    self.assertEqual(temperature.decode_oil_temperature(bytes(oil)), -48.0)
    oil[37] = 255
    self.assertEqual(temperature.decode_oil_temperature(bytes(oil)), 143.25)

  def test_successful_readings_and_flow_control(self):
    self.reader.update(0.0, [])
    self.assertEqual(self.reader.update(0.1, [single_frame(COOLANT_PAYLOAD)]), [])
    self.assertTrue(self.reader.readings(0.1)["coolantValid"])
    self.assertEqual(self.reader.readings(0.1)["coolantCelsius"], 86.0)
    self.assertEqual(self.reader.update(1.0, []), [temperature.OIL_REQUEST])
    frames = oil_frames()
    self.assertEqual(self.reader.update(1.1, frames[:1]), [temperature.OIL_FLOW_CONTROL])
    self.assertFalse(self.reader.readings(1.1)["oilValid"])
    self.assertEqual(self.reader.update(1.2, frames[1:]), [])
    self.assertEqual(self.reader.readings(1.2), {"oilCelsius": 87.75, "oilValid": True,
                                              "coolantCelsius": 86.0, "coolantValid": True,
                                              "clutch1Celsius": 0.0, "clutch1Valid": False,
                                              "clutch2Celsius": 0.0, "clutch2Valid": False,
                                              "boostKpa": 0.0, "boostValid": False})

  def test_poll_cadence_with_no_replies(self):
    for index in range(100):
      now = index / 10
      expected = []
      if index % 10 == 0:
        expected = [temperature.COOLANT_REQUEST if index % 20 == 0 else temperature.OIL_REQUEST]
      self.assertEqual(self.reader.update(now, []), expected)

  def test_request_timestamp_tracks_only_pending_transaction(self):
    self.assertIsNone(self.reader.request_started_at)
    self.reader.update(0.0, [])
    self.assertEqual(self.reader.request_started_at, 0.0)
    self.reader.update(0.1, [single_frame(COOLANT_PAYLOAD)])
    self.assertIsNone(self.reader.request_started_at)
    self.reader.update(1.0, [])
    self.assertEqual(self.reader.request_started_at, 1.0)
    self.reader.update(1.5, [])
    self.assertIsNone(self.reader.request_started_at)
    self.reader.update(2.0, [])
    self.reader.reset()
    self.assertIsNone(self.reader.request_started_at)
    self.reader.update(3.0, [])
    self.reader.update(3.1, [], enabled=False)
    self.assertIsNone(self.reader.request_started_at)

  def test_no_catchup_burst_after_long_pause(self):
    self.reader.update(0.0, [])
    self.assertEqual(self.reader.update(100.0, []), [temperature.OIL_REQUEST])
    self.assertEqual(self.reader.update(100.1, []), [])

  def test_unsolicited_or_prior_transaction_frames(self):
    self.reader.update(0.0, [single_frame(COOLANT_PAYLOAD)])
    self.assert_unavailable(0.0)
    self.reader.update(1.0, oil_frames())
    self.assert_unavailable(1.0)
    self.reader.update(1.1, oil_frames()[1:])
    self.assert_unavailable(1.1)

  def test_wrong_address_or_bus(self):
    self.start_oil()
    for wrong_frames in ([frame(data, address=0x7E9) for _, data, _ in oil_frames()],
                         [frame(data, bus=0) for _, data, _ in oil_frames()],
                         [frame(data, bus=2) for _, data, _ in oil_frames()]):
      self.assertEqual(self.reader.update(1.1, wrong_frames), [])
      self.assert_unavailable(1.1)
    self.assertEqual(self.reader.update(1.2, oil_frames()), [temperature.OIL_FLOW_CONTROL])
    self.assertTrue(self.reader.readings(1.2)["oilValid"])

  def test_wrong_classic_can_length(self):
    for data in (b"", b"\x03\x41\x05\x7e", bytes(7), bytes(9), bytes(64)):
      with self.subTest(length=len(data)):
        self.reader.reset()
        self.reader.update(0.0, [])
        self.assertEqual(self.reader.update(0.1, [frame(data)]), [])
        self.assert_unavailable(0.1)

  def test_wrong_oil_first_frame_never_sends_flow_control(self):
    for data in (b"\x10\x32" + OIL_PAYLOAD[:6], b"\x10\x34" + OIL_PAYLOAD[:6],
                 b"\x11\x33" + OIL_PAYLOAD[:6], b"\x10\x33\x62\xe0\x02\x00\x00\x00",
                 b"\x10\x33\x7f\x22\x78\x00\x00\x00"):
      with self.subTest(data=data.hex()):
        self.reader.reset()
        self.start_oil()
        self.assertEqual(self.reader.update(1.1, [frame(data)]), [])
        self.reader.update(1.2, oil_frames()[1:])
        self.assert_unavailable(1.2)

  def test_wrong_consecutive_sequence(self):
    frames = oil_frames()
    for suffix in (frames[2:], [frames[1], frames[1], *frames[2:]], [frames[2], *frames[1:]], frames):
      with self.subTest(suffix=suffix):
        self.reader.reset()
        self.start_oil()
        self.reader.update(1.1, frames[:1])
        self.reader.update(1.2, suffix)
        self.assert_unavailable(1.2)

  def test_timeout_before_first_frame(self):
    self.start_oil()
    self.assertEqual(self.reader.update(1.5, oil_frames()), [])
    self.assert_unavailable(1.5)

  def test_timeout_does_not_extend_on_partial_reply(self):
    self.start_oil()
    self.reader.update(1.4, oil_frames()[:1])
    self.reader.update(1.49, oil_frames()[1:2])
    self.reader.update(1.5, oil_frames()[2:])
    self.assert_unavailable(1.5)

  def test_negative_response_and_response_pending(self):
    for code in (0x11, 0x22, 0x31, 0x78):
      with self.subTest(code=code):
        self.reader.reset()
        self.start_oil()
        self.reader.update(1.1, [single_frame(bytes([0x7F, 0x22, code]))])
        self.assertEqual(self.reader.update(1.2, oil_frames()), [])
        self.assert_unavailable(1.2)

  def test_coolant_rejects_multiframe_and_wrong_payload(self):
    for bad in (oil_frames()[0], single_frame(b"\x41\x04\x7e"), single_frame(b"\x41\x05\x7e\x0c\x2b"),
                frame(b"\x08\x41\x05\x7e\x00\x00\x00\x00"), frame(bytes(8))):
      with self.subTest(frame=bad):
        self.reader.reset()
        self.reader.update(0.0, [])
        self.assertEqual(self.reader.update(0.1, [bad]), [])
        self.assert_unavailable(0.1)

  def test_final_frame_padding_is_not_payload(self):
    self.start_oil()
    frames = oil_frames()
    address, data, bus = frames[-1]
    frames[-1] = address, data[:4] + b"\xff" * 4, bus
    self.reader.update(1.1, frames)
    self.assertEqual(self.reader.readings(1.1)["oilCelsius"], 87.75)
    self.assertTrue(self.reader.readings(1.1)["oilValid"])

  def test_each_reading_expires_independently(self):
    self.reader.update(0.0, [])
    self.reader.update(0.1, [single_frame(COOLANT_PAYLOAD)])
    self.reader.update(1.0, [])
    self.reader.update(1.1, oil_frames())
    self.assertTrue(self.reader.readings(5.09)["coolantValid"])
    self.assertFalse(self.reader.readings(5.1)["coolantValid"])
    self.assertTrue(self.reader.readings(5.1)["oilValid"])
    self.assertFalse(self.reader.readings(6.1)["oilValid"])

  def test_failed_sample_does_not_refresh_last_good_sample(self):
    self.reader.update(0.0, [])
    self.reader.update(0.1, [single_frame(COOLANT_PAYLOAD)])
    self.reader.update(1.0, [])
    self.reader.update(2.0, [])
    self.reader.update(2.1, [single_frame(b"\x7f\x01\x11")])
    self.assertTrue(self.reader.readings(2.1)["coolantValid"])
    self.assertFalse(self.reader.readings(5.1)["coolantValid"])

  def test_disabled_and_reset_clear_readings_and_partial_response(self):
    self.reader.update(0.0, [])
    self.reader.update(0.1, [single_frame(COOLANT_PAYLOAD)])
    self.reader.update(1.0, [])
    self.reader.update(1.1, oil_frames()[:1])
    self.assertEqual(self.reader.update(1.2, oil_frames()[1:], enabled=False), [])
    self.assert_unavailable(1.2)
    self.assertEqual(self.reader.update(1.3, oil_frames()[1:]), [temperature.COOLANT_REQUEST])
    self.reader.update(1.4, [single_frame(COOLANT_PAYLOAD)])
    self.assertTrue(self.reader.readings(1.4)["coolantValid"])
    self.reader.reset()
    self.assert_unavailable(1.5)

  def test_backward_clock_clears_transport_and_readings(self):
    self.reader.update(5.0, [])
    self.reader.update(5.1, [single_frame(COOLANT_PAYLOAD)])
    self.assertFalse(self.reader.readings(5.0)["coolantValid"])
    self.assertEqual(self.reader.update(4.0, []), [temperature.COOLANT_REQUEST])
    self.assert_unavailable(4.0)


class TestBoost(unittest.TestCase):
  def setUp(self):
    self.reader = temperature.EngineTemperatureReader(engine_enabled=False, boost_enabled=True)

  def test_recorded_fallback_then_cached_barometer(self):
    self.assertEqual(self.reader.update(0.0, []), [temperature.PRESSURE_REQUEST])
    self.reader.update(0.1, [single_frame(COOLANT_PAYLOAD)])
    self.assertAlmostEqual(self.reader.readings(0.1)["boostKpa"], -58.325)
    self.assertTrue(self.reader.readings(0.1)["boostValid"])
    self.assertFalse(self.reader.readings(0.1)["coolantValid"])
    self.assertEqual(self.reader.update(0.5, []), [temperature.BAROMETER_REQUEST])
    self.reader.update(0.6, [single_frame(bytes.fromhex("413365"))])
    # Ambient pressure alone must not recalculate or refresh the last MAP sample.
    self.assertAlmostEqual(self.reader.readings(0.6)["boostKpa"], -58.325)
    self.assertEqual(self.reader.update(1.0, []), [temperature.PRESSURE_REQUEST])
    self.reader.update(1.1, [single_frame(COOLANT_PAYLOAD)])
    self.assertEqual(self.reader.readings(1.1)["boostKpa"], -58.0)
    # The same reference produces positive boost when MAP is above ambient.
    self.reader.update(1.5, [])
    self.reader.update(1.6, [single_frame(bytes.fromhex("41057E0B95"))])
    self.assertEqual(self.reader.readings(1.6)["boostKpa"], 48.0)

  def test_combined_reply_updates_coolant_only_when_enabled(self):
    reader = temperature.EngineTemperatureReader(engine_enabled=True, boost_enabled=True)
    reader.update(0.0, [])
    reader.update(0.1, [single_frame(COOLANT_PAYLOAD)])
    self.assertEqual(reader.readings(0.1)["coolantCelsius"], 86.0)
    self.assertTrue(reader.readings(0.1)["coolantValid"])
    self.assertTrue(reader.readings(0.1)["boostValid"])

  def test_exact_pressure_and_barometer_replies(self):
    for payload in (b"", COOLANT_PAYLOAD[:3], COOLANT_PAYLOAD[:-1], COOLANT_PAYLOAD + b"\x00",
                    bytes.fromhex("41057E0C2B"), bytes.fromhex("42057E0B2B"), bytes.fromhex("410B2B")):
      with self.subTest(payload=payload.hex()):
        self.assertIsNone(temperature.decode_manifold_pressure(payload))
        self.reader.reset()
        self.reader.update(0.0, [])
        self.reader.update(0.1, [single_frame(payload)])
        self.assertFalse(self.reader.readings(0.1)["boostValid"])
    for payload in (b"", bytes.fromhex("4133"), bytes.fromhex("413300"), bytes.fromhex("41336500"),
                    bytes.fromhex("413465"), bytes.fromhex("423365")):
      self.assertIsNone(temperature.decode_barometric_pressure(payload))

  def test_wrong_ecu_bus_negative_response_and_multiframe_rejected(self):
    response = single_frame(COOLANT_PAYLOAD)
    for frames in ([frame(response[1], 0x7E9)], [frame(response[1], bus=0)], [frame(response[1], bus=2)],
                   [single_frame(bytes.fromhex("7F0111"))], [single_frame(bytes.fromhex("7F0178"))], oil_frames()):
      with self.subTest(frames=frames):
        self.reader.reset()
        self.reader.update(0.0, [])
        self.assertEqual(self.reader.update(0.1, frames), [])
        self.assertFalse(self.reader.readings(0.1)["boostValid"])

  def test_barometer_retained_on_failed_refresh_and_cleared_on_reset(self):
    self.reader.update(0.0, [])
    self.reader.update(0.5, [])
    self.reader.update(0.6, [single_frame(bytes.fromhex("413365"))])
    self.assertFalse(self.reader.readings(0.6)["boostValid"])
    self.reader.update(1.0, [])
    self.reader.update(1.1, [single_frame(COOLANT_PAYLOAD)])
    self.assertEqual(self.reader.readings(1.1)["boostKpa"], -58.0)
    for tick in range(3, 122):
      self.reader.update(tick / 2, [])
    self.reader.update(60.6, [single_frame(bytes.fromhex("413300"))])
    self.reader.update(61.0, [])
    self.reader.update(61.1, [single_frame(COOLANT_PAYLOAD)])
    self.assertEqual(self.reader.readings(61.1)["boostKpa"], -58.0)
    self.reader.update(61.2, [], enabled=False)
    self.assertFalse(self.reader.readings(61.2)["boostValid"])
    self.reader.update(62.0, [])
    self.reader.update(62.1, [single_frame(COOLANT_PAYLOAD)])
    self.assertAlmostEqual(self.reader.readings(62.1)["boostKpa"], -58.325)

  def test_boost_expires_without_new_map_despite_fresh_barometer(self):
    self.reader.update(0.0, [])
    self.reader.update(0.1, [single_frame(COOLANT_PAYLOAD)])
    self.reader.update(0.5, [])
    self.reader.update(0.6, [single_frame(bytes.fromhex("413365"))])
    self.assertTrue(self.reader.readings(2.09)["boostValid"])
    self.assertFalse(self.reader.readings(2.1)["boostValid"])
    self.assertEqual(self.reader.readings(2.1)["boostKpa"], 0.0)
    self.assertFalse(self.reader.readings(0.09)["boostValid"])

  def test_no_reply_timeout_no_catchup_and_clock_reset(self):
    self.reader.update(0.0, [])
    self.assertEqual(self.reader.update(0.5, [single_frame(COOLANT_PAYLOAD)]), [temperature.BAROMETER_REQUEST])
    self.assertFalse(self.reader.readings(0.5)["boostValid"])
    self.assertEqual(self.reader.update(100.0, []), [temperature.PRESSURE_REQUEST])
    self.assertEqual(self.reader.update(100.1, []), [])
    self.assertEqual(self.reader.update(99.0, []), [temperature.PRESSURE_REQUEST])
    self.reader.update(99.1, [single_frame(COOLANT_PAYLOAD)])
    self.assertAlmostEqual(self.reader.readings(99.1)["boostKpa"], -58.325)

  def test_all_toggle_combinations_keep_enabled_readings_fresh(self):
    replies = {temperature.COOLANT_REQUEST: [single_frame(COOLANT_PAYLOAD[:3])],
               temperature.PRESSURE_REQUEST: [single_frame(COOLANT_PAYLOAD)],
               temperature.BAROMETER_REQUEST: [single_frame(bytes.fromhex("413365"))],
               temperature.OIL_REQUEST: oil_frames(), temperature.CLUTCH_REQUEST: clutch_frames()}
    for engine, clutch, boost in itertools.product((False, True), repeat=3):
      with self.subTest(engine=engine, clutch=clutch, boost=boost):
        reader = temperature.EngineTemperatureReader(engine, clutch, boost)
        pending_frames = []
        primary_times, pressure_times, barometer_times = [], [], []
        for tick in range(6000):
          now = tick / 50
          outgoing = reader.update(now, pending_frames)
          pending_frames = []
          for request in outgoing:
            if request in replies:
              primary_times.append(now)
              pending_frames = replies[request]
            if request == temperature.PRESSURE_REQUEST:
              pressure_times.append(now)
            elif request == temperature.BAROMETER_REQUEST:
              barometer_times.append(now)
          if now > 4.0:
            readings = reader.readings(now)
            for sensor, enabled in (("oil", engine), ("coolant", engine), ("clutch1", clutch), ("clutch2", clutch), ("boost", boost)):
              self.assertEqual(readings[sensor + "Valid"], enabled)
        interval = temperature.BOOST_POLL_INTERVAL if boost else temperature.POLL_INTERVAL
        self.assertTrue(all(b - a >= interval for a, b in itertools.pairwise(primary_times)))
        if boost:
          self.assertTrue(all(b - a <= 1.04 for a, b in itertools.pairwise(pressure_times)))
          self.assertEqual(len(barometer_times), 2)
        else:
          self.assertEqual(pressure_times + barometer_times, [])


class TestClutchTemperatures(unittest.TestCase):
  def setUp(self):
    self.reader = temperature.EngineTemperatureReader(engine_enabled=False, clutch_enabled=True)

  def assert_unavailable(self, now):
    readings = self.reader.readings(now)
    for sensor in ("oil", "coolant", "clutch1", "clutch2"):
      self.assertFalse(readings[sensor + "Valid"])

  def test_recorded_fixture(self):
    self.assertEqual(temperature.decode_clutch_temperatures(CLUTCH_PAYLOAD), (63.0, 35.0))
    self.assertEqual(self.reader.update(0.0, []), [temperature.CLUTCH_REQUEST])
    self.assertEqual(self.reader.update(0.1, clutch_frames()[:1]), [temperature.CLUTCH_FLOW_CONTROL])
    self.assert_unavailable(0.1)
    self.assertEqual(self.reader.update(0.2, clutch_frames()[1:]), [])
    self.assertIsNone(self.reader.request_started_at)
    readings = self.reader.readings(0.2)
    self.assertEqual(readings["clutch1Celsius"], 63.0)
    self.assertEqual(readings["clutch2Celsius"], 35.0)
    self.assertTrue(readings["clutch1Valid"])
    self.assertTrue(readings["clutch2Valid"])
    self.assertFalse(readings["oilValid"])
    self.assertFalse(readings["coolantValid"])

  def test_unsigned_big_endian_words(self):
    payload = bytearray(CLUTCH_PAYLOAD)
    payload[32:36] = bytes.fromhex("012C FEDC")
    self.assertEqual(temperature.decode_clutch_temperatures(bytes(payload)), (300.0, 65244.0))
    payload[32:36] = bytes.fromhex("0000 FFFF")
    self.assertEqual(temperature.decode_clutch_temperatures(bytes(payload)), (0.0, 65535.0))

  def test_clutch_only_cadence(self):
    for index in range(100):
      expected = [temperature.CLUTCH_REQUEST] if index % 10 == 0 else []
      self.assertEqual(self.reader.update(index / 10, []), expected)

  def test_all_groups_share_poll_cadence(self):
    reader = temperature.EngineTemperatureReader(engine_enabled=True, clutch_enabled=True)
    expected_order = [temperature.COOLANT_REQUEST, temperature.OIL_REQUEST, temperature.CLUTCH_REQUEST]
    for index in range(120):
      expected = [expected_order[(index // 10) % 3]] if index % 10 == 0 else []
      self.assertEqual(reader.update(index / 10, []), expected)

  def test_both_groups_disabled(self):
    reader = temperature.EngineTemperatureReader(engine_enabled=False, clutch_enabled=False)
    for index in range(5):
      self.assertEqual(reader.update(float(index), [single_frame(COOLANT_PAYLOAD), *oil_frames(), *clutch_frames()]), [])
      self.assertIsNone(reader.request_started_at)
      self.assertFalse(any(value for key, value in reader.readings(index).items() if key.endswith("Valid")))

  def test_engine_only_cannot_decode_clutches(self):
    reader = temperature.EngineTemperatureReader(engine_enabled=True, clutch_enabled=False)
    self.assertEqual(reader.update(0.0, []), [temperature.COOLANT_REQUEST])
    self.assertEqual(reader.update(0.1, clutch_frames()), [])
    self.assertFalse(reader.readings(0.1)["clutch1Valid"])
    self.assertEqual(reader.update(1.0, []), [temperature.OIL_REQUEST])
    self.assertEqual(reader.update(1.1, clutch_frames()), [])
    self.assertFalse(reader.readings(1.1)["clutch2Valid"])

  def test_exact_clutch_length_and_header(self):
    for payload in (b"", CLUTCH_PAYLOAD[:-1], CLUTCH_PAYLOAD + b"\x00", b"\x62\x01\xa5" + CLUTCH_PAYLOAD[3:],
                    OIL_PAYLOAD, b"\x62\xe0\x01" + CLUTCH_PAYLOAD[3:]):
      with self.subTest(payload=payload.hex()):
        self.assertIsNone(temperature.decode_clutch_temperatures(payload))
        if payload:
          self.reader.reset()
          self.reader.update(0.0, [])
          self.assertEqual(self.reader.update(0.1, clutch_frames(payload)), [])
          self.assert_unavailable(0.1)

  def test_wrong_address_or_bus_does_not_mix_with_tcm(self):
    self.reader.update(0.0, [])
    for wrong_frames in (oil_frames(), [frame(data, 0x7E8) for _, data, _ in clutch_frames()],
                         [frame(data, 0x7EA) for _, data, _ in clutch_frames()],
                         [frame(data, 0x7E9, 0) for _, data, _ in clutch_frames()],
                         [frame(data, 0x7E9, 2) for _, data, _ in clutch_frames()]):
      self.assertEqual(self.reader.update(0.1, wrong_frames), [])
      self.assert_unavailable(0.1)
    self.assertEqual(self.reader.update(0.2, clutch_frames()), [temperature.CLUTCH_FLOW_CONTROL])
    self.assertTrue(self.reader.readings(0.2)["clutch1Valid"])

  def test_cross_ecu_consecutive_frames_cannot_complete_response(self):
    self.reader.update(0.0, [])
    self.reader.update(0.1, clutch_frames()[:1])
    self.reader.update(0.2, oil_frames()[1:])
    self.assert_unavailable(0.2)
    self.reader.update(0.3, clutch_frames()[1:])
    self.assertTrue(self.reader.readings(0.3)["clutch1Valid"])
    reader = temperature.EngineTemperatureReader(engine_enabled=True, clutch_enabled=True)
    reader.update(0.0, [])
    reader.update(1.0, [])
    reader.update(1.1, oil_frames()[:1])
    reader.update(1.2, clutch_frames()[1:])
    self.assertFalse(reader.readings(1.2)["oilValid"])
    reader.update(1.3, oil_frames()[1:])
    self.assertTrue(reader.readings(1.3)["oilValid"])

  def test_tcm_payload_cannot_satisfy_oil_transaction(self):
    reader = temperature.EngineTemperatureReader(engine_enabled=True, clutch_enabled=True)
    reader.update(0.0, [])
    reader.update(1.0, [])
    self.assertEqual(reader.update(1.1, oil_frames(CLUTCH_PAYLOAD)), [])
    self.assertFalse(reader.readings(1.1)["oilValid"])
    self.assertFalse(reader.readings(1.1)["clutch1Valid"])

  def test_wrong_sequence_and_duplicate_first_frame(self):
    frames = clutch_frames()
    for suffix in (frames[2:], [frames[1], frames[1], *frames[2:]], [frames[2], *frames[1:]], frames):
      with self.subTest(suffix=suffix):
        self.reader.reset()
        self.reader.update(0.0, [])
        self.reader.update(0.1, frames[:1])
        self.assertEqual(self.reader.update(0.2, suffix), [])
        self.assert_unavailable(0.2)

  def test_dlc_negative_response_and_single_frame_rejected(self):
    for bad in (frame(b"", 0x7E9), frame(bytes(7), 0x7E9), frame(bytes(9), 0x7E9), frame(bytes(64), 0x7E9),
                single_frame(bytes.fromhex("7F 22 11"), 0x7E9), single_frame(bytes.fromhex("7F 22 78"), 0x7E9)):
      with self.subTest(frame=bad):
        self.reader.reset()
        self.reader.update(0.0, [])
        self.assertEqual(self.reader.update(0.1, [bad]), [])
        self.assertEqual(self.reader.update(0.2, clutch_frames()), [])
        self.assert_unavailable(0.2)
    self.reader.reset()
    self.reader.update(0.0, [])
    self.assertEqual(self.reader.update(0.1, [single_frame(CLUTCH_PAYLOAD[:6], 0x7E9)]), [])
    self.assert_unavailable(0.1)

  def test_fixed_timeout_and_staleness(self):
    self.reader.update(0.0, [])
    self.reader.update(0.4, clutch_frames()[:1])
    self.reader.update(0.49, clutch_frames()[1:2])
    self.reader.update(0.5, clutch_frames()[2:])
    self.assert_unavailable(0.5)
    self.reader.update(1.0, [])
    self.reader.update(1.1, clutch_frames())
    self.assertTrue(self.reader.readings(6.09)["clutch1Valid"])
    self.assertFalse(self.reader.readings(6.1)["clutch1Valid"])
    self.assertFalse(self.reader.readings(6.1)["clutch2Valid"])

  def test_reset_retains_configuration_but_clears_values_and_partial_response(self):
    self.reader.update(0.0, [])
    self.reader.update(0.1, clutch_frames())
    self.assertTrue(self.reader.readings(0.1)["clutch1Valid"])
    self.reader.reset()
    self.assert_unavailable(0.2)
    self.assertEqual(self.reader.update(1.0, []), [temperature.CLUTCH_REQUEST])
    self.reader.update(1.1, clutch_frames()[:1])
    self.assertEqual(self.reader.update(1.2, clutch_frames()[1:], enabled=False), [])
    self.assert_unavailable(1.2)
    self.assertEqual(self.reader.update(1.3, clutch_frames()[1:]), [temperature.CLUTCH_REQUEST])
    self.assert_unavailable(1.3)

  def test_both_groups_have_independent_freshness(self):
    reader = temperature.EngineTemperatureReader(engine_enabled=True, clutch_enabled=True)
    reader.update(0.0, [])
    reader.update(0.1, [single_frame(COOLANT_PAYLOAD)])
    reader.update(1.0, [])
    reader.update(1.1, oil_frames())
    reader.update(2.0, [])
    reader.update(2.1, clutch_frames())
    readings = reader.readings(6.1)
    self.assertFalse(readings["oilValid"])
    self.assertFalse(readings["coolantValid"])
    self.assertTrue(readings["clutch1Valid"])
    self.assertTrue(readings["clutch2Valid"])

  def test_last_frame_padding_is_not_a_temperature(self):
    frames = clutch_frames()
    address, data, bus = frames[-1]
    frames[-1] = address, data[:2] + b"\xff" * 6, bus
    self.reader.update(0.0, [])
    self.reader.update(0.1, frames)
    self.assertEqual(self.reader.readings(0.1)["clutch1Celsius"], 63.0)
    self.assertEqual(self.reader.readings(0.1)["clutch2Celsius"], 35.0)


if __name__ == "__main__":
  unittest.main()
