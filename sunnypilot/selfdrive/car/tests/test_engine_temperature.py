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
                                              "gear": 0.0, "nextGear": 0.0, "engineRpm": 0.0,
                                              "inputShaft1Rpm": 0.0, "inputShaft2Rpm": 0.0,
                                              "clutch1SlipRpm": 0.0, "clutch2SlipRpm": 0.0,
                                              "clutch1State": 0, "clutch2State": 0,
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
        interval = temperature.REQUEST_SPACING if clutch else (
            temperature.BOOST_POLL_INTERVAL if boost else temperature.POLL_INTERVAL)
        self.assertTrue(all(b - a >= interval for a, b in itertools.pairwise(primary_times)))
        # Explicit contract with pandad's global 230 ms floor in engine_temperature.h:
        # asking more often than that only burns request timeouts.
        self.assertTrue(all(b - a >= 0.23 for a, b in itertools.pairwise(primary_times)))
        if boost:
          self.assertTrue(all(b - a <= 1.04 for a, b in itertools.pairwise(pressure_times)))
          self.assertEqual(len(barometer_times), 2)
        else:
          self.assertEqual(pressure_times + barometer_times, [])

  def test_temperatures_yield_their_slots_to_the_manifold(self):
    # Coolant and oil move on a tens-of-seconds timescale, so when the manifold is live they give up
    # most of their slots. They still have to survive lost replies: the temp stamp is taken at
    # request, so a lost read re-arms the whole interval, and a reading that ages past STALE_AFTER
    # shows a dash instead of a number. Every fourth oil reply is lost here to keep that honest.
    replies = {temperature.OIL_REQUEST: oil_frames(), temperature.CLUTCH_REQUEST: clutch_frames(),
               temperature.PRESSURE_REQUEST: [single_frame(COOLANT_PAYLOAD)],
               temperature.BAROMETER_REQUEST: [single_frame(bytes.fromhex("413365"))]}
    reader = temperature.EngineTemperatureReader(engine_enabled=True, clutch_enabled=True, boost_enabled=True)
    wanted = (temperature.CLUTCH_REQUEST, temperature.PRESSURE_REQUEST, temperature.OIL_REQUEST)
    pending, times, served = [], {request: [] for request in wanted}, {r: 0 for r in replies}
    for tick in range(6000):
      now = tick / 50
      outgoing = reader.update(now, pending)
      pending = []
      for request in outgoing:
        if request in times:
          times[request].append(now)
        if request in replies:
          served[request] += 1
          if request != temperature.OIL_REQUEST or served[request] % 4:
            pending = replies[request]
      if now > 6.0:
        self.assertTrue(reader.readings(now)["oilValid"])
    clutch, pressure, oil = (times[request] for request in wanted)
    self.assertAlmostEqual(len(clutch) / 120.0, 2.0, delta=0.1)
    # Two consecutive losses at TEMP_POLL_INTERVAL must stay inside STALE_AFTER: at 2.0 s they did
    # not, and the cell blanked.
    self.assertLess(max(b - a for a, b in itertools.pairwise(oil)), 3.5)
    # 1.04 s is the no-loss contract the toggle test holds; a lost reply burns a whole timeout,
    # so under injection the manifold waits longer without the gauge ever going blank.
    self.assertLess(max(b - a for a, b in itertools.pairwise(pressure)), 1.4)
    self.assertGreater(len(pressure), 2 * len(oil))  # the freed slots went to MAP, not to oil


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

  def measure_rates(self, reader, seconds=60, answers=None):
    """Rates over a window, since 0.24 s spacing does not align with a 0.1 s grid."""
    replies = {temperature.COOLANT_REQUEST: [single_frame(COOLANT_PAYLOAD[:3])],
               temperature.PRESSURE_REQUEST: [single_frame(COOLANT_PAYLOAD)],
               temperature.BAROMETER_REQUEST: [single_frame(bytes.fromhex("413365"))],
               temperature.OIL_REQUEST: oil_frames(), temperature.CLUTCH_REQUEST: clutch_frames()}
    counts = {name: 0 for name in ("coolant", "oil", "clutch", "pressure", "barometer")}
    pending = []
    for tick in range(seconds * 50):
      outgoing = reader.update(10.0 + tick / 50, pending)
      pending = []
      for request in outgoing:
        if request not in replies or (answers is not None and request not in answers):
          continue  # Flow control follows a primary request; not a slot of its own.
        counts[temperature.REQUEST_KINDS[request]] += 1
        pending = replies[request]
    return {name: count / seconds for name, count in counts.items()}

  def test_clutch_only_cadence(self):
    rates = self.measure_rates(self.reader)
    self.assertAlmostEqual(rates["clutch"], 2.0, delta=0.1)
    self.assertEqual(rates["coolant"] + rates["oil"] + rates["pressure"], 0)

  def test_all_groups_share_poll_cadence(self):
    reader = temperature.EngineTemperatureReader(engine_enabled=True, clutch_enabled=True)
    rates = self.measure_rates(reader)
    # The clutch drives a live indicator; the slow-moving temperatures split the rest.
    self.assertAlmostEqual(rates["clutch"], 2.0, delta=0.1)
    self.assertAlmostEqual(rates["coolant"], 1.0, delta=0.1)
    self.assertAlmostEqual(rates["oil"], 1.0, delta=0.1)
    self.assertLessEqual(sum(rates.values()), 4.3)  # pandad's 230 ms floor

  def test_unanswered_clutch_backs_off_and_lets_temperatures_through(self):
    # Priority must not starve the ECM reads if only the TCM stops answering.
    reader = temperature.EngineTemperatureReader(engine_enabled=True, clutch_enabled=True)
    rates = self.measure_rates(reader, answers={temperature.COOLANT_REQUEST, temperature.OIL_REQUEST})
    self.assertLess(rates["clutch"], 1.0)
    self.assertGreater(rates["oil"], 0.5)
    self.assertGreater(rates["coolant"], 0.5)

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
    # Step until an oil transaction is the one in flight, so this does not depend
    # on which slot the scheduler happens to give the clutch.
    reader = temperature.EngineTemperatureReader(engine_enabled=True, clutch_enabled=True)
    pending, request, started = [], [], 0.0
    for tick in range(200):
      started = tick / 50
      request = reader.update(started, pending)
      pending = []
      for req in request:
        if req == temperature.CLUTCH_REQUEST:
          pending = clutch_frames()
        elif req == temperature.COOLANT_REQUEST:
          pending = [single_frame(COOLANT_PAYLOAD[:3])]
      if request == [temperature.OIL_REQUEST]:
        break
    self.assertEqual(request, [temperature.OIL_REQUEST])
    reader.update(started + 0.1, oil_frames()[:1])
    reader.update(started + 0.2, clutch_frames()[1:])
    self.assertFalse(reader.readings(started + 0.2)["oilValid"])
    reader.update(started + 0.3, oil_frames()[1:])
    self.assertTrue(reader.readings(started + 0.3)["oilValid"])

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
    replies = {temperature.COOLANT_REQUEST: [single_frame(COOLANT_PAYLOAD[:3])],
               temperature.OIL_REQUEST: oil_frames(), temperature.CLUTCH_REQUEST: clutch_frames()}
    pending = []
    for tick in range(1200):
      now = tick / 50
      outgoing = reader.update(now, pending)
      pending = []
      for request in outgoing:
        # The ECM goes silent after 10 s; only the TCM keeps replying.
        if request in replies and (request == temperature.CLUTCH_REQUEST or now < 10.0):
          pending = replies[request]
    readings = reader.readings(23.9)
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


def clutch_payload(gear=1, next_gear=1, engine=800.0, shaft1=0.0, shaft2=0.0, slip1=0.0, slip2=0.0):
  """The recorded fixture with its mapped fields replaced, offsets written once."""
  payload = bytearray(CLUTCH_PAYLOAD)
  for index, scale, value in ((7, 4, engine), (12, 10, shaft1), (14, 10, shaft2), (18, 4, slip1), (20, 4, slip2)):
    payload[index:index + 2] = (int(round(value * scale)) & 0xFFFF).to_bytes(2, "big")
  payload[26], payload[27] = gear, next_gear
  return bytes(payload)


def clutch_sample(gear=1, shaft1=0.0, shaft2=0.0, slip1=0.0, slip2=0.0):
  return {"gear": float(gear), "nextGear": float(gear), "engineRpm": 800.0,
          "shaft1": shaft1, "shaft2": shaft2, "clutch1Slip": slip1, "clutch2Slip": slip2,
          "clutch1": 0.0, "clutch2": 0.0}


class TestClutchState(unittest.TestCase):
  """Odd gears run on clutch 1 and even gears on clutch 2, so parity decides which is which."""

  def setUp(self):
    self.reader = temperature.EngineTemperatureReader(engine_enabled=False, clutch_enabled=True)

  def poll_states(self, payloads, spacing=0.5):
    readings = []
    for index, payload in enumerate(payloads):
      frames = clutch_frames(payload)
      start = index * spacing
      self.assertEqual(self.reader.update(start, []), [temperature.CLUTCH_REQUEST])
      self.assertEqual(self.reader.update(start + 0.01, frames[:1]), [temperature.CLUTCH_FLOW_CONTROL])
      self.assertEqual(self.reader.update(start + 0.02, frames[1:]), [])
      readings.append(self.reader.readings(start + 0.02))
    return readings

  def test_recorded_fixture_decodes_every_field(self):
    self.assertEqual(temperature.decode_clutch_payload(CLUTCH_PAYLOAD),
                     {"engineRpm": 679.5, "shaft1": 0.0, "shaft2": 147.2, "clutch1Slip": 679.5,
                      "clutch2Slip": 532.25, "gear": 1.0, "nextGear": 1.0, "clutch1": 63.0, "clutch2": 35.0})
    self.assertIsNone(temperature.decode_clutch_payload(CLUTCH_PAYLOAD[:-1]))
    self.assertIsNone(temperature.decode_clutch_payload(b"\x62\x01\xa5" + CLUTCH_PAYLOAD[3:]))

  def test_slip_wraps_at_its_high_bit_and_shaft_speed_does_not(self):
    # Slip is signed, so it wraps. Shaft speed is not: the recorded profile reaches
    # 6550 rpm there, and its own slip = engine - shaft identity only closes unsigned.
    values = temperature.decode_clutch_payload(clutch_payload(slip1=-400.0, shaft2=6549.8))
    self.assertEqual(values["clutch1Slip"], -400.0)
    self.assertEqual(values["shaft2"], 6549.8)

  def test_fast_input_shaft_keeps_a_driving_clutch_engaged(self):
    # 3400 rpm sets the shaft word's high bit. Read signed it becomes -3153 rpm and
    # falls through the standstill gate, so a driving clutch would render disengaged.
    sample = temperature.decode_clutch_payload(clutch_payload(gear=3, engine=3400.0, shaft1=3400.0))
    self.assertEqual(sample["shaft1"], 3400.0)
    self.assertEqual(temperature.classify_clutch(1, sample, temperature.ENGAGED), temperature.ENGAGED)

  def test_unmapped_payload_still_yields_temperatures(self):
    self.assertEqual(temperature.decode_clutch_temperatures(clutch_payload()), (63.0, 35.0))

  def test_no_gear_or_the_invalid_sentinel_is_unknown(self):
    for gear in (0, 8, 14, 255):
      sample = clutch_sample(gear=gear, shaft1=700.0, slip1=400.0)
      self.assertEqual(temperature.classify_clutch(1, sample, temperature.ENGAGED), temperature.UNKNOWN)
      self.assertEqual(temperature.classify_clutch(2, sample, temperature.SLIPPING), temperature.UNKNOWN)

  def test_standstill_slip_is_an_open_clutch_not_slipping(self):
    # Recorded at a brake-on stop in 1st: shaft 1 reads zero while slip reads full engine speed.
    sample = clutch_sample(gear=1, shaft1=0.0, slip1=679.5)
    self.assertEqual(temperature.classify_clutch(1, sample, temperature.ENGAGED), temperature.DISENGAGED)

  def test_preselected_clutch_speed_difference_is_not_slipping(self):
    sample = clutch_sample(gear=1, shaft1=700.0, shaft2=147.2, slip1=400.0, slip2=532.25)
    self.assertEqual(temperature.classify_clutch(2, sample, temperature.ENGAGED), temperature.DISENGAGED)
    self.assertEqual(temperature.classify_clutch(1, sample, temperature.ENGAGED), temperature.SLIPPING)

  def test_slip_band_holds_the_previous_state(self):
    self.assertEqual(temperature.classify_clutch(1, clutch_sample(gear=1, shaft1=700.0, slip1=40.0),
                                                 temperature.DISENGAGED), temperature.ENGAGED)
    self.assertEqual(temperature.classify_clutch(1, clutch_sample(gear=1, shaft1=700.0, slip1=400.0),
                                                 temperature.DISENGAGED), temperature.SLIPPING)
    for previous in (temperature.ENGAGED, temperature.SLIPPING):
      inside_band = clutch_sample(gear=1, shaft1=700.0, slip1=150.0)  # Between the two thresholds.
      self.assertEqual(temperature.classify_clutch(1, inside_band, previous), previous)

  def latch_time(self, spacing):
    tracker = temperature.ClutchStateTracker()
    slipping = clutch_sample(gear=1, shaft1=700.0, slip1=400.0)
    for index in range(20):
      if tracker.update(index * spacing, slipping)[0] == temperature.SLIPPING:
        return index * spacing
    raise AssertionError("slipping never latched")

  def test_dwell_measures_time_not_sample_count(self):
    # The scheduler's rate varies, so a sample-count dwell would latch late at 2 Hz: two samples
    # is 0.5 s at 4 Hz but a full second at 2 Hz, while the time dwell latches at 0.5 s either way.
    latched_at = {spacing: self.latch_time(spacing) for spacing in (0.25, 0.5)}
    self.assertEqual(latched_at[0.25], latched_at[0.5])
    self.assertGreaterEqual(latched_at[0.25], temperature.SLIP_DWELL_ON)
    self.assertLess(latched_at[0.25], temperature.SLIP_DWELL_ON + 0.25)

  def test_a_lone_slip_sample_does_not_flicker_the_glyph(self):
    tracker = temperature.ClutchStateTracker()
    engaged = clutch_sample(gear=1, shaft1=700.0, slip1=40.0)
    slipping = clutch_sample(gear=1, shaft1=700.0, slip1=400.0)
    states = [tracker.update(index * 0.5, sample)[0]
              for index, sample in enumerate((engaged, slipping, engaged, slipping, engaged))]
    self.assertEqual(states, [temperature.ENGAGED] * 5)

  def test_sustained_slip_latches_and_holds_after_the_reader(self):
    engaged = clutch_payload(gear=1, shaft1=700.0, slip1=40.0)
    slipping = clutch_payload(gear=1, shaft1=700.0, slip1=400.0)
    states = [readings["clutch1State"] for readings in
              self.poll_states((engaged, slipping, slipping, slipping, slipping))]
    # The dwell starts at the first slipping sample, so one more half-second poll passes before it
    # latches: the shortest dwell that still needs two samples of evidence.
    self.assertEqual(states, [temperature.ENGAGED, temperature.ENGAGED, temperature.SLIPPING,
                              temperature.SLIPPING, temperature.SLIPPING])

  def test_latch_survives_the_shortest_sample_gap_the_scheduler_produces(self):
    # Requests are CLUTCH_POLL_INTERVAL apart, but the tracker is stamped when a reply's last frame
    # lands, so its own samples are 0.44-0.54 s apart depending on ECU latency. A dwell set to the
    # nominal spacing quietly costs an extra sample on the short gaps: these three polls land 0.52 s
    # then 0.44 s apart, and two samples of evidence still have to latch across the 0.44 s one.
    engaged = clutch_payload(gear=1, shaft1=700.0, slip1=40.0)
    slipping = clutch_payload(gear=1, shaft1=700.0, slip1=400.0)
    states = []
    for index, (payload, delay) in enumerate(((engaged, 0.02), (slipping, 0.06), (slipping, 0.02))):
      at = index * temperature.CLUTCH_POLL_INTERVAL
      frames = clutch_frames(payload)
      self.assertEqual(self.reader.update(at, []), [temperature.CLUTCH_REQUEST])
      self.assertEqual(self.reader.update(at + 0.01, frames[:1]), [temperature.CLUTCH_FLOW_CONTROL])
      self.assertEqual(self.reader.update(at + delay, frames[1:]), [])
      states.append(self.reader.readings(at + delay)["clutch1State"])
    self.assertEqual(states, [temperature.ENGAGED, temperature.ENGAGED, temperature.SLIPPING])

  def test_slipping_clears_only_once_slip_has_actually_stopped(self):
    engaged = clutch_payload(gear=1, shaft1=700.0, slip1=40.0)
    slipping = clutch_payload(gear=1, shaft1=700.0, slip1=400.0)
    states = [readings["clutch1State"] for readings in
              self.poll_states((engaged, slipping, slipping, slipping, slipping,
                                engaged, engaged, engaged, engaged))]
    # Latches on the second slipping sample, then holds until slip has been absent for the whole
    # release dwell: 2.5 s of slip, cleared only at 3.5 s, one poll after the 0.6 s dwell is met.
    self.assertEqual(states, [temperature.ENGAGED, temperature.ENGAGED,
                              temperature.SLIPPING, temperature.SLIPPING, temperature.SLIPPING,
                              temperature.SLIPPING, temperature.SLIPPING, temperature.ENGAGED,
                              temperature.ENGAGED])

  def test_newest_reading_at_moves_on_a_reply_not_on_a_request(self):
    # What the daemon publishes on. It has to move when a value lands, not when one is asked for,
    # or publishing on it would push the pre-reply snapshot out faster instead of the fresh one
    # sooner; update()'s return value is the request feed and must not be used for this.
    slipping = clutch_payload(gear=1, shaft1=700.0, slip1=400.0)
    before = self.reader.newest_reading_at
    self.assertEqual(self.reader.update(0.0, []), [temperature.CLUTCH_REQUEST])
    self.assertEqual(self.reader.newest_reading_at, before)  # asked for one, nothing back yet
    frames = clutch_frames(slipping)
    self.assertEqual(self.reader.update(0.01, frames[:1]), [temperature.CLUTCH_FLOW_CONTROL])
    self.assertEqual(self.reader.newest_reading_at, before)  # the first frame of several is not a value
    self.assertEqual(self.reader.update(0.02, frames[1:]), [])
    self.assertGreater(self.reader.newest_reading_at, before)

  def test_release_dwell_clears_on_the_third_sample_at_the_real_spacing(self):
    # At poll_states' default 0.5 s spacing this cannot tell 0.6 from 1.0, both clearing on the
    # third non-evidence sample. At the 0.48 s the scheduler produces, 0.45 clears on the second,
    # 0.6 on the third and 1.0 on the fourth: that spread is what holds the release at 0.6.
    engaged = clutch_payload(gear=1, shaft1=700.0, slip1=40.0)
    slipping = clutch_payload(gear=1, shaft1=700.0, slip1=400.0)
    states = [readings["clutch1State"] for readings in
              self.poll_states((engaged, slipping, slipping, slipping,
                                engaged, engaged, engaged, engaged, engaged), spacing=0.48)]
    self.assertEqual(states, [temperature.ENGAGED, temperature.ENGAGED, temperature.SLIPPING,
                              temperature.SLIPPING, temperature.SLIPPING, temperature.SLIPPING,
                              temperature.ENGAGED, temperature.ENGAGED, temperature.ENGAGED])

  def test_recorded_clamped_and_creep_bands_classify_apart(self):
    # 5 and 136 rpm are the recorded median and p90 of slip on the driving clutch while it is
    # clamped at speed, and 155 is the recorded median while creeping: none of them is slipping.
    # The recorded creep p90 of 599 is, which is what the glyph is for.
    for slip, expected in ((5.0, temperature.ENGAGED), (136.0, temperature.ENGAGED),
                           (155.0, temperature.ENGAGED), (250.0, temperature.ENGAGED),
                           (599.0, temperature.SLIPPING)):
      with self.subTest(slip=slip):
        sample = clutch_sample(gear=2, shaft2=2000.0, slip2=slip)
        self.assertEqual(temperature.classify_clutch(2, sample, temperature.ENGAGED), expected)
    # The recorded clamped p90 is inside the release threshold, so it settles engaged rather than
    # holding whatever state it was handed.
    self.assertEqual(temperature.classify_clutch(2, clutch_sample(gear=2, shaft2=2000.0, slip2=136.0),
                                                 temperature.DISENGAGED), temperature.ENGAGED)

  def test_creep_between_the_thresholds_does_not_hold_the_glyph_on(self):
    # Creeping in a driving gear runs 150-600 rpm of reported slip, which sits above the release
    # threshold and below the enter one for as long as the queue keeps moving. A held slipping
    # state has to expire regardless, or the glyph stays lit for the whole crawl.
    latched = clutch_payload(gear=2, shaft2=700.0, slip2=400.0)
    creeping = clutch_payload(gear=2, shaft2=700.0, slip2=155.0)
    states = [readings["clutch2State"] for readings in
              self.poll_states((latched, latched, latched, creeping, creeping, creeping,
                                creeping, creeping))]
    self.assertIn(temperature.SLIPPING, states)
    self.assertEqual(states[-1], temperature.ENGAGED)

  def test_the_other_clutch_tracks_the_opposite_parity(self):
    # 2nd gear: clutch 2 drives, clutch 1 sits preselected with a large speed difference.
    readings = self.poll_states([clutch_payload(gear=2, next_gear=2, shaft1=600.0, shaft2=700.0,
                                                slip1=400.0, slip2=40.0)])[0]
    self.assertEqual((readings["clutch1State"], readings["clutch2State"]),
                     (temperature.DISENGAGED, temperature.ENGAGED))

  def test_raw_values_and_states_expire_with_their_payload(self):
    readings = self.poll_states([clutch_payload(gear=1, shaft1=700.0, slip1=40.0)])[0]
    self.assertEqual(readings["gear"], 1.0)
    self.assertEqual(readings["engineRpm"], 800.0)
    self.assertEqual(readings["inputShaft1Rpm"], 700.0)
    self.assertEqual(readings["clutch1SlipRpm"], 40.0)
    self.assertEqual(readings["clutch1State"], temperature.ENGAGED)
    readings = self.reader.readings(0.02 + temperature.STALE_AFTER)
    self.assertEqual((readings["clutch1State"], readings["clutch2State"]),
                     (temperature.UNKNOWN, temperature.UNKNOWN))
    self.assertEqual(readings["gear"] + readings["engineRpm"] + readings["inputShaft1Rpm"], 0.0)

  def test_a_garbled_reply_leaves_the_last_state_in_place(self):
    self.poll_states([clutch_payload(gear=1, shaft1=700.0, slip1=40.0)])
    self.assertEqual(self.reader.update(0.5, []), [temperature.CLUTCH_REQUEST])
    self.assertEqual(self.reader.update(0.52, clutch_frames(CLUTCH_PAYLOAD[:-1])), [])
    self.assertEqual(self.reader.readings(0.52)["clutch1State"], temperature.ENGAGED)


if __name__ == "__main__":
  unittest.main()
