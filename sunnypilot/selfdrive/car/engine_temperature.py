"""Display-only telemetry from the user's recorded Elantra AD ECM/TCM profile.

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
BOOST_POLL_INTERVAL = 0.5  # Alternate MAP with slower temperature/barometer reads.
# Panda enforces one global floor across every request kind (230 ms in
# engine_temperature.h), so ~4.3 requests per second is the whole budget.
REQUEST_SPACING = 0.24  # Above panda's 230 ms floor and a whole number of 50 Hz loop ticks,
                        # so slots do not drift and stretch the slower groups' intervals.
CLUTCH_POLL_INTERVAL = 0.48  # Exactly two slots. 0.5 was not a multiple of the slot spacing, so the
                             # TCM drifted to gaps of 0.74 s between reads; this holds 0.50 s mean
                             # and 0.52 s max at a fixed reply latency, 0.54 s once it jitters.
TEMP_POLL_INTERVAL = 2.0  # Coolant and oil move on a tens-of-seconds timescale, so the slots they no
                          # longer need go to the manifold. What makes this safe is the retry in
                          # update(): the stamp is taken at request, so without it k consecutive
                          # lost replies age the reading (k+1)*T and two of them blank the cell.
MAP_MAX_AGE = 1.0  # A slow read may take a slot only if the next MAP read still lands inside this.
                   # Half of BOOST_STALE_AFTER, so the scheduler itself never blanks the gauge; lost
                   # MAP replies still can, and no scheduling rule prevents that.
SLOW_VETO_MAX = 2  # Consecutive slots a slow read may be vetoed for. Uncapped, the guard above
                   # starves a temperature outright when a slow TCM reply leaves the clutch due
                   # again on the very next slot.
CLUTCH_PENALTY_MAX = 4.0  # Back off its priority slot if the TCM stops answering.
BAROMETER_INTERVAL = 60.0
BAROMETER_FALLBACK_KPA = 101.325
RESPONSE_TIMEOUT = 0.5
STALE_AFTER = 5.0
BOOST_STALE_AFTER = 2.0

COOLANT_REQUEST = (0x7DF, bytes.fromhex("02 01 05 00 00 00 00 00"), DIAGNOSTIC_BUS)
PRESSURE_REQUEST = (0x7DF, bytes.fromhex("03 01 05 0B 00 00 00 00"), DIAGNOSTIC_BUS)
BAROMETER_REQUEST = (0x7DF, bytes.fromhex("02 01 33 00 00 00 00 00"), DIAGNOSTIC_BUS)
OIL_REQUEST = (0x7E0, bytes.fromhex("03 22 E0 01 00 00 00 00"), DIAGNOSTIC_BUS)
OIL_FLOW_CONTROL = (0x7E0, bytes.fromhex("30 00 0A 00 00 00 00 00"), DIAGNOSTIC_BUS)
CLUTCH_REQUEST = (0x7E1, bytes.fromhex("03 22 01 A4 00 00 00 00"), DIAGNOSTIC_BUS)
CLUTCH_FLOW_CONTROL = (0x7E1, bytes.fromhex("30 00 0A 00 00 00 00 00"), DIAGNOSTIC_BUS)
OIL_RESPONSE_LENGTH = 51
CLUTCH_RESPONSE_LENGTH = 49
REQUESTS = {"coolant": COOLANT_REQUEST, "oil": OIL_REQUEST, "clutch": CLUTCH_REQUEST,
            "pressure": PRESSURE_REQUEST, "barometer": BAROMETER_REQUEST}
REQUEST_KINDS = {frame: kind for kind, frame in REQUESTS.items()}

UNKNOWN, DISENGAGED, ENGAGED, SLIPPING = 0, 1, 2, 3
# Corroborated across 18903 recorded 62 01 A4 replies: the driving clutch reads
# near-zero slip while the preselected one always reads 300-700 rpm of shaft
# speed difference, so parity against current gear decides which is which.
STANDSTILL_SHAFT_RPM = 60.0  # A stopped shaft means the clutch is open, whatever slip reads.
# Split on what the box reports while its clutch is doing each thing, from 3287 scan-tool
# samples taken while the gear parity said that clutch was the driving one: with the input shaft
# above 1200 rpm slip reads a median of 5 rpm and a p90 of 136, whereas creeping at low shaft
# speed it modulates at a median of 155 rpm with a p90 of 599. Both thresholds sit either side
# of the clamped band, so ordinary creep no longer reads as slipping.
SLIP_RPM = 300.0
SLIP_RELEASE_RPM = 140.0  # Hysteresis band between the two holds the previous state.
# The dwell is compared against the tracker's own sample spacing, not the nominal poll interval:
# requests are CLUTCH_POLL_INTERVAL apart, but a sample is stamped when the reply's last frame
# lands, so the spacing the dwell sees is 0.44-0.54 s depending on ECU latency. A dwell of 0.5 is
# longer than most of those gaps and quietly costs a third sample, and 0.45 still did on the short
# ones. Any dwell above zero still needs two consecutive readings at or above SLIP_RPM, since a
# reading between the thresholds resets the count, so going under the shortest gap costs nothing in
# noise immunity. The shortest-gap latch test pins it.
SLIP_DWELL_ON = 0.4
# The release counts consecutive samples that are not slip evidence and clears on the
# ceil(dwell/spacing)+1-th: three at 0.6, two at 0.45. A lone low reading resets that count, so this
# filters interruptions of about a second rather than noise, and those are real: a gear change flips
# the parity gate and a creeping shaft crosses standstill, either of which would end the event at
# 0.45 and force a full re-latch. 0.45 would clear a genuine stop one sample (about 0.5 s) sooner,
# and the recorded route has no two-sample interruption to trip over, so it stays here until a route
# that has one says otherwise.
SLIP_DWELL_OFF = 0.6
GEAR_MIN, GEAR_MAX = 1, 7  # Seven-speed box. Recorded raw values are 0 (no gear) and 14 (invalid).
# Raw values ride to the UI so thresholds can be retuned from a logged route.
CLUTCH_READING_KEYS = (("gear", "gear"), ("nextGear", "nextGear"), ("engineRpm", "engineRpm"),
                       ("shaft1", "inputShaft1Rpm"), ("shaft2", "inputShaft2Rpm"),
                       ("clutch1Slip", "clutch1SlipRpm"), ("clutch2Slip", "clutch2SlipRpm"))


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


def decode_manifold_pressure(payload: bytes) -> float | None:
  """MAP in kPa from the recorded combined coolant/MAP response 41 05 A 0B B."""
  if len(payload) != 5 or payload[:2] != b"\x41\x05" or payload[3] != 0x0B:
    return None
  return float(payload[4])


def decode_barometric_pressure(payload: bytes) -> float | None:
  if len(payload) != 3 or payload[:2] != b"\x41\x33" or payload[2] == 0:
    return None
  return float(payload[2])


def _signed_word(payload: bytes, index: int) -> float:
  value = int.from_bytes(payload[index:index + 2], "big")
  return value - 65536 if value & 0x8000 else float(value)


def _unsigned_word(payload: bytes, index: int) -> float:
  return float(int.from_bytes(payload[index:index + 2], "big"))


def decode_clutch_payload(payload: bytes) -> dict[str, float] | None:
  """Every mapped field of the recorded TCM reply, in one 49-byte response.

  Byte 7 is engine speed and bytes 18/20 are the two clutch slip speeds; slip is
  signed, and equals engine speed minus that clutch's input shaft speed. The
  shaft speeds at bytes 12/14 are unsigned: they top out near 6550 rpm, well
  past the redline, and the recorded profile's own slip = engine - shaft
  identity only closes when they are read unsigned.
  """
  if len(payload) != CLUTCH_RESPONSE_LENGTH or payload[:3] != b"\x62\x01\xa4":
    return None
  gear, next_gear = payload[26], payload[27]
  return {
    "engineRpm": _signed_word(payload, 7) / 4,
    "shaft1": _unsigned_word(payload, 12) / 10,
    "shaft2": _unsigned_word(payload, 14) / 10,
    "clutch1Slip": _signed_word(payload, 18) / 4,
    "clutch2Slip": _signed_word(payload, 20) / 4,
    "gear": float(gear),
    "nextGear": float(next_gear),
    "clutch1": float(int.from_bytes(payload[32:34], "big")),
    "clutch2": float(int.from_bytes(payload[34:36], "big")),
  }


def decode_clutch_temperatures(payload: bytes) -> tuple[float, float] | None:
  """Recorded TCM-reported temperatures: unsigned big-endian words in Celsius."""
  values = decode_clutch_payload(payload)
  return None if values is None else (values["clutch1"], values["clutch2"])


def classify_clutch(clutch: int, sample: dict[str, float], previous: int) -> int:
  """Instantaneous state for one clutch. Clutch 1 drives odd gears, clutch 2 even."""
  gear = int(sample["gear"])
  if not GEAR_MIN <= gear <= GEAR_MAX:
    return UNKNOWN  # Neutral, parked, or the invalid sentinel: no clutch can be driving.
  if sample[f"shaft{clutch}"] <= STANDSTILL_SHAFT_RPM:
    return DISENGAGED
  if gear % 2 != clutch % 2:
    return DISENGAGED  # Preselected: its speed difference is not slip.
  slip = abs(sample[f"clutch{clutch}Slip"])
  if slip >= SLIP_RPM:
    return SLIPPING
  if slip <= SLIP_RELEASE_RPM:
    return ENGAGED
  return previous


class ClutchStateTracker:
  """Time-based dwell, so a single sample between engaged readings cannot flicker."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._states = [UNKNOWN, UNKNOWN]
    self._slip_since: list[float | None] = [None, None]
    self._clear_since: list[float | None] = [None, None]

  def update(self, now: float, sample: dict[str, float]) -> tuple[int, int]:
    for index, clutch in enumerate((1, 2)):
      candidate = classify_clutch(clutch, sample, self._states[index])
      # A returned SLIPPING is not evidence on its own: inside the dead band classify holds the
      # previous state, and a creeping clutch sits there indefinitely. Only a reading at or above
      # the enter threshold renews the slip dwell.
      slipping_now = candidate == SLIPPING and abs(sample[f"clutch{clutch}Slip"]) >= SLIP_RPM
      if candidate == UNKNOWN:
        self._states[index] = UNKNOWN
        self._slip_since[index] = self._clear_since[index] = None
      elif slipping_now:
        self._clear_since[index] = None
        if self._slip_since[index] is None:
          self._slip_since[index] = now
        if now - self._slip_since[index] >= SLIP_DWELL_ON:
          self._states[index] = SLIPPING
          self._slip_since[index] = None
      elif self._states[index] == SLIPPING:
        # Below the enter threshold for the whole release dwell is the evidence that slipping
        # ended, so the dwell lands on engaged even when the reading never dropped as far as the
        # release threshold. Landing back on the held state instead is what left the glyph on
        # for as long as the car crept in a gear.
        if self._clear_since[index] is None:
          self._clear_since[index] = now
        if now - self._clear_since[index] >= SLIP_DWELL_OFF:
          self._states[index] = ENGAGED if candidate == SLIPPING else candidate
          self._slip_since[index] = self._clear_since[index] = None
      else:
        self._states[index] = candidate
        self._slip_since[index] = self._clear_since[index] = None
    return self._states[0], self._states[1]


class EngineTemperatureReader:
  def __init__(self, engine_enabled: bool = True, clutch_enabled: bool = False, boost_enabled: bool = False):
    self._engine_enabled = engine_enabled
    self._boost_enabled = boost_enabled
    self._clutch_enabled = clutch_enabled
    if clutch_enabled:
      # The clutch takes two slots per second; the rest split what panda allows.
      self._poll_interval = REQUEST_SPACING if (engine_enabled or boost_enabled) else CLUTCH_POLL_INTERVAL
    else:
      self._poll_interval = BOOST_POLL_INTERVAL if boost_enabled else POLL_INTERVAL
    self._poll_order = (("oil",) if boost_enabled else ("coolant", "oil")) if engine_enabled else ()
    self._clutch_tracker = ClutchStateTracker()
    self.reset()

  @property
  def request_started_at(self) -> float | None:
    """Timestamp boundary for the caller to discard pre-request CAN events."""
    return self._last_request if self._pending is not None else None

  def reset(self) -> None:
    """Drop all transport and cached readings when disabled or leaving a drive."""
    self._values: dict[str, tuple[float, float]] = {}
    self._clutch_states: tuple[int, int] = (UNKNOWN, UNKNOWN)
    self._clutch_tracker.reset()
    self._last_update: float | None = None
    self._last_request: float | None = None
    self._last_temp: dict[str, float] = {}
    self._barometer_kpa: float | None = None
    self._last_barometer_request: float | None = None
    self._last_pressure: float | None = None
    self._last_clutch: float | None = None
    self._clutch_penalty = 0.0
    self._slow_vetoes = 0
    self._finish_request()

  def _clutch_due(self, now: float) -> bool:
    if not self._clutch_enabled:
      return False
    if self._last_clutch is None:
      return True
    return now - self._last_clutch >= CLUTCH_POLL_INTERVAL + self._clutch_penalty

  def _temp_due(self, now: float) -> str | None:
    """The temperature read that has aged past TEMP_POLL_INTERVAL, or None if none has."""
    if not self._poll_order:
      return None
    oldest, sensor = min((self._last_temp.get(s, float("-inf")), s) for s in self._poll_order)
    return sensor if now - oldest >= TEMP_POLL_INTERVAL else None

  def _slow_due(self, now: float) -> str | None:
    """The barometric or temperature read that has aged out, if either has. Reference first."""
    if self._barometer_due(now):
      return "barometer"
    return self._temp_due(now)

  def _barometer_due(self, now: float) -> bool:
    # The reference only matters once there is a manifold pressure to compare it against.
    return (self._boost_enabled and self._last_pressure is not None and
            (self._last_barometer_request is None or now - self._last_barometer_request >= BAROMETER_INTERVAL))

  def _next_request(self, now: float) -> str:
    # Priority rather than rotation. The clutch drives a live indicator and the manifold is the only
    # fast-moving reading, so they take the slots; the slow reads only have to stay inside their own
    # staleness window, and the manifold absorbs every slot they do not need.
    if self._clutch_due(now):
      self._last_clutch = now
      request = "clutch"
    elif self._boost_enabled and self._last_pressure is None:
      request = "pressure"  # MAP first, so the gauge reads against the fallback reference at once
    else:
      slow = self._slow_due(now)
      # Age, not slot count: the clutch claims every other slot, so a slow read can leave the
      # manifold waiting three of them, and the wait one would cost is two slots with it running.
      # The veto is capped, because if the TCM's reply lands near RESPONSE_TIMEOUT the clutch is
      # due again on the very next slot and an uncapped veto starves the temperatures outright.
      wait = CLUTCH_POLL_INTERVAL if self._clutch_enabled else self._poll_interval
      if slow is not None and self._boost_enabled and self._last_pressure is not None and \
             self._slow_vetoes < SLOW_VETO_MAX and now - self._last_pressure + wait > MAP_MAX_AGE:
        self._slow_vetoes += 1
        slow = None
      else:
        self._slow_vetoes = 0
      if slow is not None:
        request = slow
      elif self._boost_enabled:
        request = "pressure"
      elif self._poll_order:
        # Nothing faster is waiting, so keep the temperatures as fresh as the bus allows.
        request = min(self._poll_order, key=lambda sensor: self._last_temp.get(sensor, float("-inf")))
      else:
        request = "clutch"
    if request in ("coolant", "oil"):
      self._last_temp[request] = now
    elif request == "barometer":
      self._last_barometer_request = now
    elif request == "pressure":
      self._last_pressure = now
    return request

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
      expected_service = 0x01 if self._pending in ("coolant", "pressure", "barometer") else 0x22
      if len(payload) == 3 and payload[:2] == bytes((0x7F, expected_service)):
        # Negative responses, including "response pending", never extend timeout.
        self._finish_request()
      elif self._pending == "coolant":
        self._accept("coolant", decode_coolant_temperature(payload), now)
      elif self._pending == "pressure":
        manifold_kpa = decode_manifold_pressure(payload)
        if manifold_kpa is not None:
          if self._engine_enabled:
            self._values["coolant"] = (float(payload[2] - 40), now)
          # calculated.boost_gauge in the supplied profile: preserve vacuum.
          reference_kpa = self._barometer_kpa if self._barometer_kpa is not None else BAROMETER_FALLBACK_KPA
          self._values["boost"] = (manifold_kpa - reference_kpa, now)
        self._finish_request()
      elif self._pending == "barometer":
        barometer_kpa = decode_barometric_pressure(payload)
        if barometer_kpa is not None:
          self._barometer_kpa = barometer_kpa
        self._finish_request()
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
          values = decode_clutch_payload(bytes(self._payload))
          if values is not None:
            for sensor, value in values.items():
              self._values[sensor] = (value, now)
            self._clutch_states = self._clutch_tracker.update(now, values)
            self._clutch_penalty = 0.0
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
    if not enabled or not (self._poll_order or self._boost_enabled or self._clutch_enabled):
      self.reset()
      return []
    if self._last_update is not None and now < self._last_update:
      self.reset()
    self._last_update = now

    if self._pending is not None and now >= self._deadline:
      if self._pending == "clutch":
        # Unanswered, hand the priority slot back to the temperatures instead of
        # retrying the TCM forever while the ECM is still answering fine.
        self._clutch_penalty = min(self._clutch_penalty + CLUTCH_POLL_INTERVAL, CLUTCH_PENALTY_MAX)
      elif self._pending in ("coolant", "oil"):
        # The temp stamp is taken at request, so a lost reply would otherwise cost a whole
        # TEMP_POLL_INTERVAL before the read is tried again: age the stamp so it is due on the next
        # free slot. The ladder still caps how many slots a dead ECM can attract.
        self._last_temp[self._pending] = now - TEMP_POLL_INTERVAL + REQUEST_SPACING
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

    if self._pending is None and (self._last_request is None or now - self._last_request >= self._poll_interval):
      self._pending = self._next_request(now)
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
    for sensor, key in CLUTCH_READING_KEYS:
      value, timestamp = self._values.get(sensor, (0.0, float("-inf")))
      readings[key] = value if 0 <= now - timestamp < STALE_AFTER else 0.0
    _, timestamp = self._values.get("clutch1", (0.0, float("-inf")))
    # Every clutch field shares one payload timestamp, so state expires with the temperatures.
    readings["clutch1State"], readings["clutch2State"] = (
        self._clutch_states if 0 <= now - timestamp < STALE_AFTER else (UNKNOWN, UNKNOWN))
    value, timestamp = self._values.get("boost", (0.0, float("-inf")))
    valid = 0 <= now - timestamp < BOOST_STALE_AFTER
    readings["boostKpa"] = value if valid else 0.0
    readings["boostValid"] = valid
    return readings
