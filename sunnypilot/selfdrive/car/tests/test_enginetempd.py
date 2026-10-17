import itertools
import unittest
from types import SimpleNamespace as NS

from openpilot.sunnypilot.selfdrive.car.enginetempd import (
  SUPPORTED_CAR, EngineTemperatureSession, configure_engine_temperatures, diagnostics_ready, state_messages_fresh, supports_engine_temperatures,
)


class TestEngineTemperatureGates(unittest.TestCase):
  def setUp(self):
    self.config = NS(safetyModel="hyundaiLegacy", safetyParam=2048)
    self.CP = NS(carFingerprint=SUPPORTED_CAR, brand="hyundai", notCar=False, passive=False, dashcamOnly=False,
                 safetyConfigs=[self.config])
    self.panda = NS(pandaType="dos", safetyModel="hyundaiLegacy", safetyParam=2048, ignitionLine=True, ignitionCan=False)

  def test_vehicle_and_flag(self):
    self.assertTrue(supports_engine_temperatures(self.CP))
    for field, value in (("carFingerprint", "other Hyundai"), ("brand", "toyota"), ("notCar", True), ("passive", True),
                         ("dashcamOnly", True), ("safetyConfigs", []), ("safetyConfigs", [self.config, self.config])):
      with self.subTest(field=field):
        previous = getattr(self.CP, field)
        setattr(self.CP, field, value)
        self.assertFalse(supports_engine_temperatures(self.CP))
        setattr(self.CP, field, previous)
    self.config.safetyParam = 0
    self.assertFalse(supports_engine_temperatures(self.CP))
    self.config.safetyParam = 2048
    for mode in ("hyundai", "hyundaiCanfd", "elm327", "noOutput"):
      self.config.safetyModel = mode
      self.assertFalse(supports_engine_temperatures(self.CP))

  def test_each_display_selects_restricted_safety_without_changing_other_flags(self):
    for engine, clutch, boost in itertools.product((False, True), repeat=3):
      with self.subTest(engine=engine, clutch=clutch, boost=boost):
        self.config.safetyParam = 64
        configure_engine_temperatures(self.CP, engine, clutch, boost)
        self.assertEqual(self.config.safetyParam, 64 | (2048 if engine or clutch or boost else 0))
        self.assertEqual(supports_engine_temperatures(self.CP), engine or clutch or boost)
    configure_engine_temperatures(self.CP, False, False, False)
    self.assertEqual(self.config.safetyParam, 64)

  def test_never_enables_diagnostics_for_unsupported_configuration(self):
    for field, value in (("carFingerprint", "other Hyundai"), ("brand", "toyota"), ("notCar", True), ("passive", True),
                         ("dashcamOnly", True), ("safetyConfigs", []), ("safetyConfigs", [self.config, self.config])):
      with self.subTest(field=field):
        self.config.safetyParam = 64
        previous = getattr(self.CP, field)
        setattr(self.CP, field, value)
        configure_engine_temperatures(self.CP, True, True, True)
        self.assertEqual(self.config.safetyParam, 64)
        setattr(self.CP, field, previous)
    for mode in ("hyundai", "hyundaiCanfd", "elm327", "noOutput"):
      self.config.safetyModel = mode
      self.config.safetyParam = 64
      configure_engine_temperatures(self.CP, True, True, True)
      self.assertEqual(self.config.safetyParam, 64)

  def test_waits_for_actual_panda_mode_and_drive(self):
    args = [True, True, True, [self.panda], True]
    self.assertTrue(diagnostics_ready(*args))
    for idx in (0, 1, 2, 4):
      altered = args.copy()
      altered[idx] = False
      self.assertFalse(diagnostics_ready(*altered))
    for field, value in (("pandaType", "whitePanda"), ("safetyModel", "elm327"), ("safetyParam", 0), ("ignitionLine", False)):
      previous = getattr(self.panda, field)
      setattr(self.panda, field, value)
      self.assertFalse(diagnostics_ready(*args))
      setattr(self.panda, field, previous)
    self.assertFalse(diagnostics_ready(True, True, True, [], True))

  def test_state_freshness_uses_receive_clock(self):
    self.assertTrue(state_messages_fresh(100.0, {"pandaStates": 99.9, "deviceState": 99.5}))
    for timestamp in (0.0, 98.0, 100.1):
      self.assertFalse(state_messages_fresh(100.0, {"pandaStates": timestamp, "deviceState": 99.5}))
      self.assertFalse(state_messages_fresh(100.0, {"pandaStates": 99.9, "deviceState": timestamp}))


class TestEngineTemperatureSession(unittest.TestCase):
  @staticmethod
  def event(timestamp, valid=True, address=0x7E8, bus=1):
    return NS(logMonoTime=int(timestamp * 1e9), valid=valid,
              can=[NS(address=address, dat=bytes.fromhex("03 41 05 7E 00 00 00 00"), src=bus)])

  def test_fresh_ecm_reading_and_disable(self):
    session = EngineTemperatureSession()
    self.assertEqual(session.update(1.0, [], True), ["coolant"])
    self.assertEqual(session.update(1.05, [self.event(1.04)], True), [])
    self.assertEqual(session.reader.readings(1.05)["coolantCelsius"], 86)
    self.assertTrue(session.reader.readings(1.05)["coolantValid"])
    self.assertEqual(session.update(1.1, [], False), [])
    self.assertFalse(session.reader.readings(1.1)["coolantValid"])

  def test_never_accepts_stale_or_unrequested_frames(self):
    for event in (self.event(0.99), self.event(1.01), self.event(1.3), self.event(1.2, valid=False),
                  self.event(1.2, bus=0), self.event(1.2, address=0x7E9)):
      with self.subTest(event=event):
        session = EngineTemperatureSession()
        session.update(1.0, [], True)
        session.update(1.21, [event], True)
        self.assertFalse(session.reader.readings(1.21)["coolantValid"])
    session = EngineTemperatureSession()
    session.update(1.0, [self.event(1.0)], True)
    self.assertFalse(session.reader.readings(1.0)["coolantValid"])

  def test_clutch_only_message_delivery(self):
    session = EngineTemperatureSession(engine_enabled=False, clutch_enabled=True)
    payload = bytes.fromhex("6201A4FFFFFF800A9E002900000005C042120A9E0851C11408070101F0FDC0FF003F002300000000000000000000000000")
    self.assertEqual(session.update(1.0, [], True), ["clutch"])
    first = NS(logMonoTime=1_020_000_000, valid=True,
               can=[NS(address=0x7E9, src=1, dat=bytes([0x10, len(payload)]) + payload[:6])])
    self.assertEqual(session.update(1.03, [first], True), ["clutchFlowControl"])
    for sequence, start in enumerate(range(6, len(payload), 7), 1):
      now = 1.03 + sequence * 0.02
      data = (bytes([0x20 | sequence]) + payload[start:start + 7]).ljust(8, b"\x00")
      event = NS(logMonoTime=int(now * 1e9), valid=True, can=[NS(address=0x7E9, src=1, dat=data)])
      self.assertEqual(session.update(now, [event], True), [])
    readings = session.reader.readings(1.2)
    self.assertEqual(readings["clutch1Celsius"], 63)
    self.assertEqual(readings["clutch2Celsius"], 35)
    self.assertTrue(readings["clutch1Valid"])
    self.assertTrue(readings["clutch2Valid"])
    self.assertFalse(readings["oilValid"])
    self.assertFalse(readings["coolantValid"])

  def test_changing_toggles_discards_pending_responses(self):
    session = EngineTemperatureSession()
    self.assertEqual(session.update(1.0, [], True), ["coolant"])
    session.configure(engine_enabled=False, clutch_enabled=True)
    self.assertEqual(session.update(1.05, [self.event(1.04)], True), ["clutch"])
    self.assertFalse(session.reader.readings(1.05)["coolantValid"])
    # Reapplying an unchanged setting must not restart an in-flight request.
    reader = session.reader
    session.configure(engine_enabled=False, clutch_enabled=True)
    self.assertIs(session.reader, reader)
    self.assertEqual(session.reader.request_started_at, 1.05)
    session.configure(engine_enabled=False, clutch_enabled=False)
    self.assertEqual(session.update(2.0, [], True), [])
    self.assertFalse(any(value for key, value in session.reader.readings(2.0).items() if key.endswith("Valid")))

  def test_boost_only_messages_and_reconfiguration(self):
    session = EngineTemperatureSession(engine_enabled=False, boost_enabled=True)
    self.assertEqual(session.update(1.0, [], True), ["pressure"])
    pressure = NS(logMonoTime=1_040_000_000, valid=True,
                  can=[NS(address=0x7E8, src=1, dat=bytes.fromhex("05 41 05 7E 0B 2B 00 00"))])
    self.assertEqual(session.update(1.05, [pressure], True), [])
    self.assertAlmostEqual(session.reader.readings(1.05)["boostKpa"], -58.325)
    self.assertTrue(session.reader.readings(1.05)["boostValid"])
    self.assertFalse(session.reader.readings(1.05)["coolantValid"])
    self.assertEqual(session.update(1.5, [], True), ["barometer"])
    session.configure(engine_enabled=False, clutch_enabled=False, boost_enabled=False)
    barometer = NS(logMonoTime=1_540_000_000, valid=True,
                   can=[NS(address=0x7E8, src=1, dat=bytes.fromhex("03 41 33 65 00 00 00 00"))])
    self.assertEqual(session.update(1.55, [barometer], True), [])
    self.assertFalse(session.reader.readings(1.55)["boostValid"])
    session.configure(engine_enabled=False, clutch_enabled=False, boost_enabled=True)
    self.assertEqual(session.update(2.0, [pressure, barometer], True), ["pressure"])
    self.assertFalse(session.reader.readings(2.0)["boostValid"])
    pressure.logMonoTime = 2_040_000_000
    session.update(2.05, [pressure], True)
    self.assertAlmostEqual(session.reader.readings(2.05)["boostKpa"], -58.325)


if __name__ == "__main__":
  unittest.main()
