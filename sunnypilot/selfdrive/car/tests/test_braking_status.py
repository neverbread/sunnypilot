from cereal import custom
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags
from openpilot.selfdrive.car.helpers import convert_to_capnp


def test_elantra_brake_lights_reach_ui_message():
  cp = structs.CarParams(carFingerprint=CAR.HYUNDAI_ELANTRA_GT_I30,
                        flags=int(HyundaiFlags.TCU_GEARS), wheelSpeedFactor=1.0)
  cp_sp = structs.CarParamsSP()
  state = CarState(cp, cp_sp)
  parsers = state.get_can_parsers(cp, cp_sp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  state.update(parsers)  # Register the messages read by the dynamic parser.

  # Exercise the real CAN decoder, Hyundai update, and published Cap'n Proto
  # conversion. Releases between cases catch a lamp state that gets latched.
  cases = (
    ("coasting", 0, 0, False),
    ("stock SCC braking without pedal", 1, 0, True),
    ("SCC brake release", 0, 0, False),
    ("pedal before lamp signal", 0, 2, True),
    ("pedal release", 0, 0, False),
  )
  for frame, (description, lamp, override, expected) in enumerate(cases, start=1):
    messages = [
      packer.make_can_msg("TCS13", 0, {"BrakeLight": lamp, "DriverOverride": override, "aBasis": -1.0}),
      packer.make_can_msg("SCC11", 0, {"MainMode_ACC": 1}),
      packer.make_can_msg("SCC12", 0, {"ACCMode": 1}),
    ]
    parsers[Bus.pt].update([(frame * 10_000_000, messages)])
    car_state, car_state_sp = state.update(parsers)
    assert car_state.cruiseState.enabled, description
    assert car_state.brakePressed == (override == 2), description
    assert car_state_sp.brakeLights == expected, description
    published = convert_to_capnp(car_state_sp)
    with custom.CarStateSP.from_bytes(published.to_bytes()) as received:
      assert received.brakeLights == expected, description
