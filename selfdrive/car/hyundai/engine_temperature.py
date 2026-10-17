"""Display-only temperatures from the user's recorded Elantra AD ECM/TCM profile.

Frames are classic CAN (address, data, bus) tuples. The caller supplies monotonic
seconds and only frames received during the current drive, after the last update.
This reader owns no sockets, sleeps, control state, or vehicle actuation.
"""

from collections.abc import Iterable


CanFrame = tuple[int, bytes, int]

DIAGNOSTIC_BUS = 1
ECM_RESPONSE = 0x7E8
TCM_RESPONSE = 0x7E9
POLL_INTERVAL = 1.0  # One request per second total across the enabled groups.
RESPONSE_TIMEOUT = 0.5
STALE_AFTER = 5.0

COOLANT_REQUEST = (0x7DF, bytes.fromhex("02 01 05 00 00 00 00 00"), DIAGNOSTIC_BUS)
OIL_REQUEST = (0x7E0, bytes.fromhex("03 22 E0 01 00 00 00 00"), DIAGNOSTIC_BUS)
OIL_FLOW_CONTROL = (0x7E0, bytes.fromhex("30 00 0A 00 00 00 00 00"), DIAGNOSTIC_BUS)
CLUTCH_REQUEST = (0x7E1, bytes.fromhex("03 22 01 A4 00 00 00 00"), DIAGNOSTIC_BUS)
CLUTCH_FLOW_CONTROL = (0x7E1, bytes.fromhex("30 00 0A 00 00 00 00 00"), DIAGNOSTIC_BUS)
OIL_RESPONSE_LENGTH = 51
CLUTCH_RESPONSE_LENGTH = 49
REQUESTS = {"coolant": COOLANT_REQUEST, "oil": OIL_REQUEST, "clutch": CLUTCH_REQUEST}


def decode_coolant_temperature(payload: bytes) -> float | None:
  """Decode a complete application response, retaining its 41 05 prefix."""
  if len(payload) not in (3, 5) or payload[:2] != b"\x41\x05":
    return None
  if len(payload) == 5 and payload[3] != 0x0B:
    return None
  return float(payload[2] - 40)


def decode_oil_temperature(payload: bytes) -> float | None:
  """The recorded 62 E0 01 response is exactly 51 bytes, including its prefix."""
  if len(payload) != OIL_RESPONSE_LENGTH or payload[:3] != b"\x62\xe0\x01":
    return None
  return payload[37] * 0.75 - 48.0


def decode_clutch_temperatures(payload: bytes) -> tuple[float, float] | None:
  """Recorded TCM-reported temperatures: unsigned big-endian words in Celsius."""
  if len(payload) != CLUTCH_RESPONSE_LENGTH or payload[:3] != b"\x62\x01\xa4":
    return None
  return float(int.from_bytes(payload[32:34], "big")), float(int.from_bytes(payload[34:36], "big"))


class EngineTemperatureReader:
  def __init__(self, engine_enabled: bool = True, clutch_enabled: bool = False):
    self._poll_order = (("coolant", "oil") if engine_enabled else ()) + (("clutch",) if clutch_enabled else ())
    self.reset()

  @property
  def request_started_at(self) -> float | None:
    """Timestamp boundary for the caller to discard pre-request CAN events."""
    return self._last_request if self._pending is not None else None

  def reset(self) -> None:
    """Drop all transport and cached readings when disabled or leaving a drive."""
    self._values: dict[str, tuple[float, float]] = {}
    self._last_update: float | None = None
    self._last_request: float | None = None
    self._poll_index = 0
    self._finish_request()

  def _finish_request(self) -> None:
    self._pending: str | None = None
    self._deadline = 0.0
    self._payload = bytearray()
    self._next_sequence = 1

  def _accept(self, sensor: str, value: float | None, now: float) -> None:
    if value is not None:
      self._values[sensor] = (value, now)
    self._finish_request()

  def _receive(self, data: bytes, now: float) -> bool:
    """Consume one matching ECU frame; return whether flow control is needed."""
    frame_type = data[0] >> 4
    expected_length = CLUTCH_RESPONSE_LENGTH if self._pending == "clutch" else OIL_RESPONSE_LENGTH
    expected_prefix = b"\x62\x01\xa4" if self._pending == "clutch" else b"\x62\xe0\x01"

    if frame_type == 0:
      length = data[0] & 0x0F
      if not 1 <= length <= 7:
        self._finish_request()
        return False
      payload = data[1:1 + length]
      expected_service = 0x01 if self._pending == "coolant" else 0x22
      if len(payload) == 3 and payload[:2] == bytes((0x7F, expected_service)):
        # Negative responses, including "response pending", never extend timeout.
        self._finish_request()
      elif self._pending == "coolant":
        self._accept("coolant", decode_coolant_temperature(payload), now)
      elif self._payload:
        self._finish_request()
      return False

    if frame_type == 1:
      length = ((data[0] & 0x0F) << 8) | data[1]
      if (self._pending not in ("oil", "clutch") or self._payload or length != expected_length or
          data[2:5] != expected_prefix):
        self._finish_request()
        return False
      self._payload.extend(data[2:])
      return True

    if frame_type == 2 and self._pending in ("oil", "clutch") and self._payload:
      if (data[0] & 0x0F) != self._next_sequence:
        self._finish_request()
        return False
      remaining = expected_length - len(self._payload)
      self._payload.extend(data[1:1 + min(7, remaining)])
      self._next_sequence = (self._next_sequence + 1) & 0x0F
      if len(self._payload) == expected_length:
        if self._pending == "oil":
          self._accept("oil", decode_oil_temperature(bytes(self._payload)), now)
        else:
          values = decode_clutch_temperatures(bytes(self._payload))
          if values is not None:
            self._values["clutch1"] = (values[0], now)
            self._values["clutch2"] = (values[1], now)
          self._finish_request()
      return False

    self._finish_request()
    return False

  def update(self, now: float, frames: Iterable[CanFrame], enabled: bool = True) -> list[CanFrame]:
    """Advance one nonblocking step; return only fixed requests/flow control.

    Responses arriving without a current request, or at/after its deadline, are
    discarded. Receive handling precedes issuing a request so queued old frames
    cannot satisfy a request issued in this same update.
    """
    if not enabled or not self._poll_order:
      self.reset()
      return []
    if self._last_update is not None and now < self._last_update:
      self.reset()
    self._last_update = now

    if self._pending is not None and now >= self._deadline:
      self._finish_request()

    outgoing = []
    for address, data, bus in frames:
      if self._pending is None:
        break
      expected_address = TCM_RESPONSE if self._pending == "clutch" else ECM_RESPONSE
      if bus != DIAGNOSTIC_BUS or address != expected_address:
        continue
      if len(data) != 8:
        self._finish_request()
        continue
      if self._receive(data, now):
        outgoing.append(CLUTCH_FLOW_CONTROL if self._pending == "clutch" else OIL_FLOW_CONTROL)

    if self._pending is None and (self._last_request is None or now - self._last_request >= POLL_INTERVAL):
      self._pending = self._poll_order[self._poll_index]
      self._poll_index = (self._poll_index + 1) % len(self._poll_order)
      self._last_request = now
      self._deadline = now + RESPONSE_TIMEOUT
      outgoing.append(REQUESTS[self._pending])

    return outgoing

  def readings(self, now: float) -> dict[str, float | bool]:
    readings: dict[str, float | bool] = {}
    for sensor in ("oil", "coolant", "clutch1", "clutch2"):
      value, timestamp = self._values.get(sensor, (0.0, float("-inf")))
      valid = 0 <= now - timestamp < STALE_AFTER
      readings[f"{sensor}Celsius"] = value if valid else 0.0
      readings[f"{sensor}Valid"] = valid
    return readings
