"""Exercise the daemon's real message construction with isolated I/O."""

import itertools
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock, patch

import cereal
from cereal import car, log
from openpilot.sunnypilot.selfdrive.car.enginetempd import SUPPORTED_CAR, configure_engine_temperatures, main


class StopDaemon(Exception):
  pass


class TestEngineTemperatureRuntime(unittest.TestCase):
  @staticmethod
  def car_params(longitudinal=False):
    return car.CarParams.new_message(brand="hyundai", carFingerprint=SUPPORTED_CAR,
                                     openpilotLongitudinalControl=longitudinal,
                                     safetyConfigs=[{"safetyModel": "hyundai" if longitudinal else "hyundaiLegacy",
                                                     "safetyParam": 2048 | (4 if longitudinal else 0)}])

  def test_safety_flag_serializes_before_daemon_starts(self):
    for longitudinal in (False, True):
      cp = self.car_params(longitudinal)
      base_flags = 64 | (4 if longitudinal else 0)
      cp.safetyConfigs[0].safetyParam = base_flags
      configure_engine_temperatures(cp, False, True)
      with car.CarParams.from_bytes(cp.to_bytes()) as restored:
        self.assertEqual(restored.safetyConfigs[0].safetyParam, base_flags | 2048)

  def test_daemon_publishes_valid_messages_and_boottime_timestamps(self):
    for engine, clutch, boost, longitudinal in itertools.product((False, True), repeat=4):
      with self.subTest(engine=engine, clutch=clutch, boost=boost, longitudinal=longitudinal):
        request_kind = "pressure" if boost else "coolant" if engine else "clutch" if clutch else None
        flags = {"EngineTemperatureDisplay": engine, "DctTemperatureDisplay": clutch, "BoostDisplay": boost, "ControlsReady": True}
        cp = self.car_params(longitudinal)
        params = NS(get=Mock(return_value=cp.to_bytes()), get_bool=lambda key, flags=flags: flags.get(key, False))
        messages = []

        class SubMaster(dict):
          def __init__(self, services, cp=cp):
            super().__init__(deviceState=NS(started=True),
                             pandaStates=[log.PandaState.new_message(pandaType="dos", safetyModel=cp.safetyConfigs[0].safetyModel,
                                                                      safetyParam=cp.safetyConfigs[0].safetyParam, ignitionLine=True)])
            self.recv_time = dict.fromkeys(services, 9.9)

          def update(self, timeout):
            pass

          def all_checks(self):
            return True

        def new_message(service, **kwargs):
          # Match the new messaging API's default. The daemon must opt into valid.
          msg = log.Event.new_message(**{"valid": False, "logMonoTime": 0, **kwargs})
          msg.init(service)
          return msg

        messaging = ModuleType("cereal.messaging")
        messaging.SubMaster = SubMaster
        messaging.PubMaster = lambda services, messages=messages: NS(send=lambda topic, message: messages.append((topic, message.to_bytes())))
        messaging.sub_sock = lambda service: object()
        messaging.drain_sock = lambda socket: []
        messaging.new_message = new_message
        keeper = NS(keep_time=Mock(side_effect=StopDaemon))
        with patch.dict('sys.modules', {"cereal.messaging": messaging}), \
             patch.object(cereal, "messaging", messaging, create=True), \
             patch('openpilot.common.params.Params', return_value=params), \
             patch('openpilot.common.realtime.Ratekeeper', return_value=keeper, create=True), \
             patch('time.monotonic', return_value=10.0), \
             patch('time.CLOCK_BOOTTIME', 7, create=True), \
             patch('time.clock_gettime', return_value=100.0, create=True):
          with self.assertRaises(StopDaemon):
            main()

        self.assertEqual([service for service, _ in messages],
                         (["engineTemperatureRequest"] if request_kind else []) + ["engineTemperature"])
        for service, data in messages:
          with log.Event.from_bytes(data) as event:
            self.assertTrue(event.valid)
            self.assertEqual(event.logMonoTime, 100_000_000_000)
            if service == "engineTemperatureRequest":
              self.assertEqual(event.engineTemperatureRequest.kind, request_kind)
            else:
              readings = event.engineTemperature
              self.assertTrue(readings.supported)
              self.assertFalse(readings.oilValid or readings.coolantValid or readings.clutch1Valid or readings.clutch2Valid or readings.boostValid)

  def test_boost_reading_and_request_schema_round_trip(self):
    for kind in ("pressure", "barometer"):
      msg = log.Event.new_message(valid=True)
      msg.init("engineTemperatureRequest").kind = kind
      with log.Event.from_bytes(msg.to_bytes()) as restored:
        self.assertEqual(restored.engineTemperatureRequest.kind, kind)
    msg = log.Event.new_message(valid=True)
    msg.init("engineTemperature")
    msg.engineTemperature = {"boostKpa": -58.325, "boostValid": True, "supported": True}
    with log.Event.from_bytes(msg.to_bytes()) as restored:
      self.assertAlmostEqual(restored.engineTemperature.boostKpa, -58.325, places=4)
      self.assertTrue(restored.engineTemperature.boostValid)


if __name__ == '__main__':
  unittest.main()
