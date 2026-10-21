from dataclasses import asdict
import pytest

from cereal import custom
from opendbc.car import Bus, structs
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.carstate_ext import CarStateExt, CURRENT_GEAR_STALE_NANOS


TCU12 = 0x112
TPMS11 = 0x593
CGW1 = 0x541
START_NANOS = 1_000_000_000
# Actual bus-0 frames from the recorded 2018 Elantra GT. All 20 corresponding
# 62 01 A4 replies agreed with CUR_GR; nextGear differed in two replies.
RECORDED_GEARS = {
  4: bytes.fromhex("ff94000067000000"),
  5: bytes.fromhex("ff85000067000000"),
  6: bytes.fromhex("ff16000067000000"),
  7: bytes.fromhex("ff57000067000000"),
}


def make_params(fingerprint=CAR.HYUNDAI_ELANTRA_GT_I30, flags=0):
  return structs.CarParams(carFingerprint=fingerprint, flags=int(flags))


def make_parser(cp=None):
  cp = cp or make_params()
  return CarState(cp, structs.CarParamsSP()).get_can_parsers(cp, structs.CarParamsSP())[Bus.pt]


def read(parser, cp=None):
  ret = structs.CarStateSP()
  CarStateExt(cp or make_params(), structs.CarParamsSP()).update_sp_ext(ret, {Bus.pt: parser})
  return ret


class TestCurrentGear:
  @pytest.mark.parametrize("gear,frame", RECORDED_GEARS.items())
  def test_recorded_forward_gears_and_serialization(self, gear, frame):
    parser = make_parser()
    parser.update([(START_NANOS + gear * 10_000_000, [(TCU12, frame, 0)])])
    ret = read(parser)
    assert ret.currentGearValid
    assert ret.currentGear == gear
    message = custom.CarStateSP.new_message(**asdict(ret))
    with custom.CarStateSP.from_bytes(message.to_bytes()) as decoded:
      assert decoded.currentGearValid
      assert decoded.currentGear == gear

  def test_gear_does_not_depend_on_tpms_or_cgw1(self):
    parser = make_parser()
    parser.update([(START_NANOS, [(TCU12, RECORDED_GEARS[4], 0)])])
    ret = read(parser)
    assert ret.currentGearValid
    assert not ret.tirePressureValid

  def test_missing_optional_gear_keeps_bus_valid(self):
    parser = make_parser()
    # Pre-register required traffic without making TPMS or TCU12 mandatory.
    parser.vl["CGW1"]
    for index in range(60):
      parser.update([(START_NANOS + index * 100_000_000, [(CGW1, bytes(8), 0)])])
      assert not read(parser).currentGearValid
      assert parser.can_valid

  def test_stale_gear_expires_even_when_whole_bus_stops(self):
    parser = make_parser()
    parser.update([(START_NANOS, [(TCU12, RECORDED_GEARS[4], 0)])])
    parser.update([(START_NANOS + CURRENT_GEAR_STALE_NANOS, [])])
    assert read(parser).currentGearValid
    parser.update([(START_NANOS + CURRENT_GEAR_STALE_NANOS + 1, [])])
    ret = read(parser)
    assert not ret.currentGearValid
    assert ret.currentGear == 0

  def test_future_timestamp_is_rejected(self):
    parser = make_parser()
    parser.update([(START_NANOS, [(TCU12, RECORDED_GEARS[4], 0)])])
    parser.update([(START_NANOS - 1, [])])
    assert not read(parser).currentGearValid

  def test_other_bus_does_not_publish_gear(self):
    parser = make_parser()
    parser.update([(START_NANOS, [(TCU12, RECORDED_GEARS[4], 2)])])
    assert not read(parser).currentGearValid

  def test_other_vehicle_does_not_publish_gear_or_subscribe(self):
    cp = make_params(CAR.HYUNDAI_ELANTRA)
    parser = make_parser(cp)
    assert TCU12 not in parser.addresses
    parser.update([(START_NANOS, [(TCU12, RECORDED_GEARS[4], 0)])])
    assert not read(parser, cp).currentGearValid
    assert TCU12 not in parser.addresses

  @pytest.mark.parametrize("fingerprint", (CAR.HYUNDAI_ELANTRA_GT_I30, CAR.HYUNDAI_ELANTRA))
  def test_control_gear_keeps_its_alive_check(self, fingerprint):
    cp = make_params(fingerprint, HyundaiFlags.TCU_GEARS)
    parser = make_parser(cp)
    parser.vl["TCU12"]  # The existing control path dynamically registers it.
    assert not parser.message_states[TCU12].ignore_alive
    parser.update([(START_NANOS, [(CGW1, bytes(8), 0)])])
    assert not parser.can_valid

  @pytest.mark.parametrize("gear", (0, 14))
  def test_non_forward_codes_are_fresh_without_inventing_a_gear(self, gear):
    parser = make_parser()
    frame = bytearray(RECORDED_GEARS[4])
    frame[1] = (frame[1] & 0xf0) | gear
    parser.update([(START_NANOS, [(TCU12, bytes(frame), 0)])])
    ret = read(parser)
    assert ret.currentGearValid
    assert ret.currentGear == gear
    assert not 1 <= ret.currentGear <= 7

  @pytest.mark.parametrize("gear", (8, 9, 10, 11, 12, 13, 15))
  def test_unknown_codes_are_invalid(self, gear):
    parser = make_parser()
    frame = bytearray(RECORDED_GEARS[4])
    frame[1] = (frame[1] & 0xf0) | gear
    parser.update([(START_NANOS, [(TCU12, bytes(frame), 0)])])
    assert not read(parser).currentGearValid

  def test_gear_and_tire_pressures_publish_independently(self):
    parser = make_parser()
    parser.vl["CGW1"]
    pressure_frame = bytes([0x00, 0x01, 32, 32, 32, 31])
    parser.update([(START_NANOS, [(TCU12, RECORDED_GEARS[4], 0), (TPMS11, pressure_frame, 0), (CGW1, bytes(8), 0)])])
    ret = read(parser)
    assert ret.currentGearValid
    assert ret.tirePressureValid
    assert ret.tirePressuresPsi == [32, 32, 32, 31]
    parser.update([(START_NANOS + CURRENT_GEAR_STALE_NANOS + 1, [(CGW1, bytes(8), 0)])])
    ret = read(parser)
    assert not ret.currentGearValid
    assert ret.tirePressureValid
