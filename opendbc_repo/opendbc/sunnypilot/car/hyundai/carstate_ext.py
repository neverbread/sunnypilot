"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

# TPMS11 arrives about every 200 ms on the vehicles that report it. Five seconds
# rides out a dropped publish without letting a parked reading hold a stale
# number on the driving screen.
TPMS_STALE_NANOS = int(5e9)
TPMS_UNIT_PSI = 0
TPMS_SIGNALS = ("PRESSURE_FL", "PRESSURE_FR", "PRESSURE_RL", "PRESSURE_RR")


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser], speed_conv: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS13"]["aBasis"]

    if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      cruise_msg = "LABEL11" if self.CP.flags & HyundaiFlags.EV else \
                   "E_CRUISE_CONTROL" if self.CP.flags & HyundaiFlags.HYBRID else \
                   "EMS16"
      cruise_available_sig = "CC_React" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_M"
      cruise_enabled_sig = "CC_ACT" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_S"
      cruise_speed_msg = "E_EMS11" if self.CP.flags & HyundaiFlags.EV else \
                         "ELECT_GEAR" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "LVR12"
      cruise_speed_sig = "Cruise_Limit_Target" if self.CP.flags & HyundaiFlags.EV else \
                         "SLC_SET_SPEED" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "CF_Lvr_CruiseSet"
      ret.cruiseState.available = cp.vl[cruise_msg][cruise_available_sig] != 0
      ret.cruiseState.enabled = cp.vl[cruise_msg][cruise_enabled_sig] != 0
      ret.cruiseState.speed = cp.vl[cruise_speed_msg][cruise_speed_sig] * speed_conv
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False

      if not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_NO_FCA:
        cp_cruise = cp if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_RADAR_FCA else cp_cam

        aeb_src = "FCA11"
        aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
        aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src]["FCA_CmdAct"] != 0
        ret.stockFcw = aeb_warning and not aeb_braking
        ret.stockAeb = aeb_warning and aeb_braking

  def update_canfd_ext(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    self.aBasis = cp.vl["TCS"]["aBasis"]

  def update_sp_ext(self, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    tpms = cp.vl["TPMS11"]

    # One message carries all four corners, so they share a freshness check.
    # CGW1 has no checksum or counter to fail and is always present while
    # driving, so its timestamp stands in for a clock.
    tpms_nanos = cp.ts_nanos["TPMS11"]["PRESSURE_FL"]
    bus_nanos = cp.ts_nanos["CGW1"]["CF_Gway_IGNSw"]
    if not tpms_nanos or bus_nanos - tpms_nanos > TPMS_STALE_NANOS:
      return

    # The pressure bytes follow the unit the cluster is set to, which the UNIT
    # bits report. Only the psi setting is decoded, so any other setting leaves
    # the reading unpublished instead of guessing at a scale.
    if tpms["UNIT"] != TPMS_UNIT_PSI:
      return

    pressures = [float(tpms[sig]) for sig in TPMS_SIGNALS]
    # A zero corner is how the body controller reports a sensor it has not
    # learned. A genuinely flat tire still reports a pressure and raises the
    # vehicle's own telltale, which is published separately below.
    if not all(pressures):
      return

    ret_sp.tirePressuresPsi = pressures
    ret_sp.tirePressureValid = True
    ret_sp.tpmsWarning = tpms["TPMS_W_LAMP"] != 0
