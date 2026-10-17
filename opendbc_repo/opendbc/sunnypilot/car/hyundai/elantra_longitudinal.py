"""Experimental radar-disable port for the recorded Elantra GT hardware only."""

import time

from opendbc.car import make_tester_present_msg, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.hyundai.values import CAR


Ecu = structs.CarParams.Ecu

# The supplied stock-ACC capture passes normal Hyundai checksum/counter checks.
# Other cars sharing this platform identifier still require legacy safety.
ELANTRA_LONG_FW = {
  (Ecu.fwdRadar, 0x7D0): b'\xf1\x00PD__ SCC F-CUP      1.01 1.00 96400-G3100         ',
  (Ecu.abs, 0x7D1): bytes.fromhex('f100504420455343200b203130331711302035383932302d4733333530'),
}
ELANTRA_LONG_MESSAGES = frozenset((0x420, 0x421, 0x389, 0x50A, 0x38D))
RADAR_OUTPUT_MESSAGES = ELANTRA_LONG_MESSAGES | {0x483}
CHASSIS_MESSAGES = frozenset((0x260, 0x386, 0x394))


def supports_elantra_longitudinal(candidate, fingerprint, car_fw) -> bool:
  if candidate != CAR.HYUNDAI_ELANTRA_GT_I30:
    return False
  bus_zero = fingerprint.get(0, {})
  # Both recorded segments lack FCA12 and FRT_RADAR11. The port deliberately
  # omits them; a different observed message layout needs its own validation.
  if any(addr in bus_zero for addr in (0x2AB, 0x483, 0x4A2)) or any(bus_zero.get(addr) != 8 for addr in ELANTRA_LONG_MESSAGES):
    return False

  for (ecu, address), expected in ELANTRA_LONG_FW.items():
    versions = {bytes(fw.fwVersion) for fw in car_fw if fw.ecu == ecu and fw.address == address and not fw.subAddress}
    if versions != {expected}:
      return False
  return True


def radar_output_stopped(can_recv: CanRecvCallable, can_send: CanSendCallable) -> bool:
  """Observe a full second of live chassis traffic without radar SCC/FCA output.

  The ordinary disable helper does not require a positive communication-control
  response. Require actual silence before ControlsReady allows the LONG safety
  mode to transmit. A disconnected/stale bus must not count as successful disable.
  Panda's stock-SCC conflict checks remain active after this startup observation.
  """
  can_recv()  # discard messages queued before the observation
  start = time.monotonic()
  last_seen = {}
  next_keepalive = start
  while time.monotonic() - start < 1.0:
    now = time.monotonic()
    if now >= next_keepalive:
      can_send([make_tester_present_msg(0x7D0, 0, suppress_response=True)])
      next_keepalive = now + 0.5

    # Nonblocking receive makes the observation deadline independent of transport
    # timeout. The short sleep also avoids busy spinning on a disconnected bus.
    for packet in can_recv():
      for msg in packet:
        if msg.src == 0:
          if msg.address in RADAR_OUTPUT_MESSAGES:
            return False
          if msg.address in CHASSIS_MESSAGES and len(msg.dat) == 8:
            last_seen[msg.address] = now
    if now - start >= 0.1 and any(now - last_seen.get(addr, start) >= 0.1 for addr in CHASSIS_MESSAGES):
      return False
    time.sleep(0.01)

  now = time.monotonic()
  return all(now - last_seen.get(addr, start - 1.0) < 0.1 for addr in CHASSIS_MESSAGES)


def initialize_elantra_longitudinal(can_recv: CanRecvCallable, can_send: CanSendCallable) -> None:
  try:
    # selfdrived can restart while Panda remains in LONG safety. That mode only
    # permits tester-present, so recognize an already-silent radar without
    # repeating the diagnostic session/communication-control requests.
    if not radar_output_stopped(can_recv, can_send):
      if not disable_ecu(can_recv, can_send, bus=0, addr=0x7D0) or not radar_output_stopped(can_recv, can_send):
        raise RuntimeError("Elantra experimental longitudinal: radar disable could not be verified")
  except Exception:
    # Best-effort restore of normal communication on startup failure. Do not
    # proceed to ControlsReady or start replacement SCC transmission on failure.
    can_send([CanData(0x7D0, b'\x03\x28\x80\x01\x00\x00\x00\x00', 0)])
    raise
  carlog.warning("Elantra experimental longitudinal: radar output stopped; factory AEB disabled")
