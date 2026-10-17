"""Exercise the daemon's real message construction with isolated I/O."""

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
  def car_params():
    return car.CarParams.new_message(brand="hyundai", carFingerprint=SUPPORTED_CAR,
                                     safetyConfigs=[{"safetyModel": "hyundaiLegacy", "safetyParam": 2048}])

  def test_safety_flag_serializes_before_daemon_starts(self):
    cp = self.car_params()
    cp.safetyConfigs[0].safetyParam = 64
    configure_engine_temperatures(cp, False, True)
    with car.CarParams.from_bytes(cp.to_bytes()) as restored:
      self.assertEqual(restored.safetyConfigs[0].safetyParam, 64 | 2048)

  def test_daemon_publishes_valid_messages_and_boottime_timestamps(self):
    for engine, clutch, request_kind in ((True, False, "coolant"), (False, True, "clutch"),
                                        (True, True, "coolant"), (False, False, None)):
      with self.subTest(engine=engine, clutch=clutch):
        flags = {"EngineTemperatureDisplay": engine, "DctTemperatureDisplay": clutch, "ControlsReady": True}
        cp = self.car_params()
        params = NS(get=Mock(return_value=cp.to_bytes()), get_bool=lambda key: flags.get(key, False))
        messages = []

        class SubMaster(dict):
          def __init__(self, services):
            super().__init__(deviceState=NS(started=True),
                             pandaStates=[log.PandaState.new_message(pandaType="dos", safetyModel="hyundaiLegacy",
                                                                      safetyParam=2048, ignitionLine=True)])
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
        messaging.PubMaster = lambda services: NS(send=lambda service, message: messages.append((service, message.to_bytes())))
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
              self.assertFalse(readings.oilValid or readings.coolantValid or readings.clutch1Valid or readings.clutch2Valid)


if __name__ == '__main__':
  unittest.main()
