"""Optional display telemetry, isolated from vehicle control and CAN validity."""

from openpilot.sunnypilot.selfdrive.car.engine_temperature import (
  COOLANT_REQUEST, OIL_REQUEST, OIL_FLOW_CONTROL, CLUTCH_REQUEST, CLUTCH_FLOW_CONTROL, ECM_RESPONSE, TCM_RESPONSE, EngineTemperatureReader,
  PRESSURE_REQUEST, BAROMETER_REQUEST,
)


# Match the vehicle and flag selected by card/Panda, without importing
# the vehicle parser or native control modules into this display-only process.
SUPPORTED_CAR = "HYUNDAI_ELANTRA_GT_I30"
TEMPERATURE_SAFETY_FLAG = 2048
LONGITUDINAL_SAFETY_FLAG = 4
LONGITUDINAL_TEMPERATURE_FLAGS = LONGITUDINAL_SAFETY_FLAG | 64 | 512 | TEMPERATURE_SAFETY_FLAG
REQUEST_KINDS = {
  COOLANT_REQUEST: "coolant", OIL_REQUEST: "oil", OIL_FLOW_CONTROL: "flowControl",
  CLUTCH_REQUEST: "clutch", CLUTCH_FLOW_CONTROL: "clutchFlowControl",
  PRESSURE_REQUEST: "pressure", BAROMETER_REQUEST: "barometer",
}
OBD_PANDA_TYPES = {"blackPanda", "uno", "dos", "redPanda", "redPandaV2", "tres"}


def supported_safety_configuration(safety_model, safety_param: int) -> bool:
  return (safety_model == "hyundaiLegacy" or
          (safety_model == "hyundai" and bool(safety_param & LONGITUDINAL_SAFETY_FLAG) and
           not safety_param & ~LONGITUDINAL_TEMPERATURE_FLAGS))


def supported_vehicle_configuration(CP) -> bool:
  if not (CP.carFingerprint == SUPPORTED_CAR and CP.brand == "hyundai" and not CP.notCar and
          not CP.passive and not CP.dashcamOnly and len(CP.safetyConfigs) == 1):
    return False
  config = CP.safetyConfigs[0]
  return (supported_safety_configuration(config.safetyModel, config.safetyParam) and
          (config.safetyModel == "hyundaiLegacy" or CP.openpilotLongitudinalControl))


def configure_engine_temperatures(CP, engine_enabled: bool, clutch_enabled: bool, boost_enabled: bool = False) -> None:
  """Select restricted OBD reads before card saves the final CarParams."""
  if supported_vehicle_configuration(CP):
    CP.safetyConfigs[0].safetyParam &= ~TEMPERATURE_SAFETY_FLAG
    if engine_enabled or clutch_enabled or boost_enabled:
      CP.safetyConfigs[0].safetyParam |= TEMPERATURE_SAFETY_FLAG


def supports_engine_temperatures(CP) -> bool:
  return supported_vehicle_configuration(CP) and bool(CP.safetyConfigs[0].safetyParam & TEMPERATURE_SAFETY_FLAG)


def diagnostics_ready(supported: bool, controls_ready: bool, started: bool, panda_states, fresh: bool) -> bool:
  if not (supported and controls_ready and started and fresh and len(panda_states)):
    return False
  panda = panda_states[0]  # Only the internal panda connects to OBD.
  return (str(panda.pandaType) in OBD_PANDA_TYPES and supported_safety_configuration(panda.safetyModel, panda.safetyParam) and
          bool(panda.safetyParam & TEMPERATURE_SAFETY_FLAG) and (panda.ignitionLine or panda.ignitionCan))


def state_messages_fresh(now: float, receive_times) -> bool:
  # SubMaster receive times and this clock are MONOTONIC; publishers may use
  # either MONOTONIC or BOOTTIME for their log timestamps.
  return all(0 < receive_times[s] <= now and now - receive_times[s] < 2.0 for s in ("pandaStates", "deviceState"))


class EngineTemperatureSession:
  def __init__(self, engine_enabled: bool = True, clutch_enabled: bool = False, boost_enabled: bool = False):
    self._configuration: tuple[bool, bool, bool] | None = None
    self.reader: EngineTemperatureReader
    self.configure(engine_enabled, clutch_enabled, boost_enabled)

  def configure(self, engine_enabled: bool, clutch_enabled: bool, boost_enabled: bool = False) -> None:
    configuration = (engine_enabled, clutch_enabled, boost_enabled)
    if configuration != self._configuration:
      # A changed selection starts a new transaction; queued old replies cannot
      # repopulate readings that the user just disabled.
      self.reader = EngineTemperatureReader(engine_enabled=engine_enabled, clutch_enabled=clutch_enabled, boost_enabled=boost_enabled)
      self._configuration = configuration

  def update(self, now: float, can_events, ready: bool) -> list[str]:
    frames = []
    request_started = self.reader.request_started_at
    if ready and request_started is not None:
      for event in can_events:
        received = event.logMonoTime / 1e9
        if event.valid and request_started <= received <= now and now - received < 0.1:
          frames.extend((frame.address, bytes(frame.dat), frame.src) for frame in event.can
                        if frame.src == 1 and frame.address in (ECM_RESPONSE, TCM_RESPONSE))
    return [REQUEST_KINDS[frame] for frame in self.reader.update(now, frames, enabled=ready)]


def main():
  import time

  import cereal.messaging as messaging
  from cereal import car
  from openpilot.common.params import Params
  from openpilot.common.realtime import Ratekeeper

  params = Params()
  with car.CarParams.from_bytes(params.get("CarParams", block=True)) as CP:
    supported = supports_engine_temperatures(CP)
  if not supported:
    return

  session = EngineTemperatureSession()
  can_sock = messaging.sub_sock("can")
  sm = messaging.SubMaster(["pandaStates", "deviceState"])
  pm = messaging.PubMaster(["engineTemperature", "engineTemperatureRequest"])
  rk = Ratekeeper(50, print_delay_threshold=None)
  next_publication = 0.0

  while True:
    sm.update(0)
    events = messaging.drain_sock(can_sock)
    # CAN events from pandad use CLOCK_BOOTTIME, including time spent suspended.
    now = time.clock_gettime(time.CLOCK_BOOTTIME)
    fresh = sm.all_checks() and state_messages_fresh(time.monotonic(), sm.recv_time)
    engine_enabled = params.get_bool("EngineTemperatureDisplay")
    clutch_enabled = params.get_bool("DctTemperatureDisplay")
    boost_enabled = params.get_bool("BoostDisplay")
    session.configure(engine_enabled, clutch_enabled, boost_enabled)
    ready = diagnostics_ready(supported, params.get_bool("ControlsReady") and (engine_enabled or clutch_enabled or boost_enabled),
                              sm["deviceState"].started, sm["pandaStates"], fresh)
    for kind in session.update(now, events, ready):
      request = messaging.new_message("engineTemperatureRequest", valid=True)
      request.logMonoTime = int(now * 1e9)
      request.engineTemperatureRequest.kind = kind
      pm.send("engineTemperatureRequest", request)

    if now >= next_publication:
      msg = messaging.new_message("engineTemperature", valid=True)
      msg.logMonoTime = int(now * 1e9)
      msg.engineTemperature = dict(session.reader.readings(now), supported=supported)
      pm.send("engineTemperature", msg)
      next_publication = now + 0.5
    rk.keep_time()


if __name__ == "__main__":
  main()
