import pytest

from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.sunnypilot.car.hyundai.carstate_ext import CarStateExt, TPMS_STALE_NANOS

TPMS11 = 0x593
CGW1 = 0x541
BUS = 0

# Recorded on the 2018 Elantra GT Sport: 32 / 32 / 32 / 31 psi, no telltale lit,
# unit bits reporting psi. Byte 1 holds the status and unit bits, bytes 2 to 5
# hold the four corner pressures.
PRESSURE_FRAME = bytes([0x00, 0x01, 32, 32, 32, 31])


def build_parser() -> CANParser:
  return CANParser(DBC[CAR.HYUNDAI_ELANTRA_GT_I30][Bus.pt], [("TPMS11", float("nan")), ("CGW1", 20)], BUS)


def feed(parser: CANParser, nanos: int, tpms_frame: bytes | None = PRESSURE_FRAME) -> None:
  frames = [(CGW1, bytes(8), BUS)]
  if tpms_frame is not None:
    frames.append((TPMS11, tpms_frame, BUS))
  parser.update([(nanos, frames)])


@pytest.fixture
def ext():
  return CarStateExt(structs.CarParams(), structs.CarParamsSP())


def read(ext: CarStateExt, parser: CANParser) -> structs.CarStateSP:
  ret_sp = structs.CarStateSP()
  ext.update_sp_ext(ret_sp, {Bus.pt: parser})
  return ret_sp


class TestTpms:
  def test_elantra_parser_subscribes_tpms(self):
    car_params = structs.CarParams()
    car_params.carFingerprint = CAR.HYUNDAI_ELANTRA_GT_I30
    parsers = CarState(car_params, structs.CarParamsSP()).get_can_parsers(car_params, structs.CarParamsSP())
    assert TPMS11 in parsers[Bus.pt].addresses

  def test_absent_signal_does_not_fault_the_bus(self):
    # interfaces.py folds every parser's can_valid into carState.canValid, which
    # selfdrived treats as a CAN error. A vehicle that never sends TPMS11 has to
    # keep reporting a healthy bus.
    parser = build_parser()
    for i in range(6):
      feed(parser, 1_000_000_000 + i * 100_000_000, tpms_frame=None)
    assert parser.can_valid

  def test_published_pressures(self, ext):
    parser = build_parser()
    feed(parser, 1_000_000_000)
    ret_sp = read(ext, parser)
    assert ret_sp.tirePressureValid
    assert ret_sp.tirePressuresPsi == [32.0, 32.0, 32.0, 31.0]
    assert not ret_sp.tpmsWarning

  def test_stale_frames_are_dropped(self, ext):
    parser = build_parser()
    feed(parser, 1_000_000_000)
    assert read(ext, parser).tirePressureValid

    # The body controller stops reporting while the bus keeps talking, as it
    # does when the vehicle sleeps or a gateway stops forwarding the message.
    feed(parser, 1_000_000_000 + TPMS_STALE_NANOS + 1_000_000_000, tpms_frame=None)
    assert not read(ext, parser).tirePressureValid

  def test_unreported_pressures_are_dropped(self, ext):
    parser = build_parser()
    feed(parser, 1_000_000_000, tpms_frame=None)
    ret_sp = read(ext, parser)
    assert not ret_sp.tirePressureValid
    assert ret_sp.tirePressuresPsi == []

  def test_other_cluster_units_are_not_guessed(self, ext):
    parser = build_parser()
    unit_bits_set = bytes([0x00, 0x01 | (1 << 3), 230, 230, 230, 228])
    feed(parser, 1_000_000_000, tpms_frame=unit_bits_set)
    assert not read(ext, parser).tirePressureValid

  def test_unlearned_sensor_is_dropped(self, ext):
    parser = build_parser()
    feed(parser, 1_000_000_000, tpms_frame=bytes([0x00, 0x01, 32, 32, 0, 31]))
    assert not read(ext, parser).tirePressureValid

  def test_vehicle_telltale_is_forwarded(self, ext):
    parser = build_parser()
    telltale_lit = bytes([0x01, 0x01, 32, 32, 32, 20])
    feed(parser, 1_000_000_000, tpms_frame=telltale_lit)
    ret_sp = read(ext, parser)
    assert ret_sp.tirePressureValid
    assert ret_sp.tpmsWarning
