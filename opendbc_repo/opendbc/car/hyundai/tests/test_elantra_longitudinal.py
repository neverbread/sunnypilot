import unittest
from unittest.mock import Mock, patch

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.hyundaican import create_acc_commands, create_acc_opt, hyundai_checksum
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags, LEGACY_SAFETY_MODE_CAR
from opendbc.sunnypilot.car.hyundai.elantra_longitudinal import supports_elantra_longitudinal
from opendbc.sunnypilot.car.hyundai.escc import ESCC_MSG
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanLeadData
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalState
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP


Ecu = structs.CarParams.Ecu
TARGET = CAR.HYUNDAI_ELANTRA_GT_I30
# Recorded identification and bus layout, independent of the implementation's allowlist.
RECORDED_FW = (
  (Ecu.fwdRadar, 0x7d0, b'\xf1\x00PD__ SCC F-CUP      1.01 1.00 96400-G3100         '),
  (Ecu.abs, 0x7d1, b'\xf1\x00PD ESC \x0b 103\x17\x110 58920-G3350'),
)
SCC_MESSAGES = (0x420, 0x421, 0x389, 0x50a, 0x38d)


def recorded_firmware():
  return [structs.CarParams.CarFw(ecu=ecu, address=address, fwVersion=version, bus=0)
          for ecu, address, version in RECORDED_FW]


def recorded_fingerprint():
  fingerprint = gen_empty_fingerprint()
  fingerprint[0] = {address: 8 for address in SCC_MESSAGES}
  return fingerprint


class TestElantraLongitudinalParams(unittest.TestCase):
  def setUp(self):
    self.fingerprint = recorded_fingerprint()
    self.firmware = recorded_firmware()

  def get_params(self, alpha=False, candidate=TARGET, is_release=False, docs=False):
    return CarInterface.get_params(candidate, self.fingerprint, self.firmware, alpha, is_release, docs)

  def assert_unavailable(self, **kwargs):
    cp = self.get_params(alpha=True, **kwargs)
    self.assertFalse(cp.alphaLongitudinalAvailable)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertTrue(cp.pcmCruise)
    self.assertEqual(cp.safetyConfigs[-1].safetyModel, structs.CarParams.SafetyModel.hyundaiLegacy)
    self.assertFalse(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG)
    self.assertFalse(cp.flags & HyundaiFlags.ELANTRA_GT_LONGITUDINAL)

  def test_stock_mode_remains_legacy(self):
    cp = self.get_params()
    self.assertTrue(cp.alphaLongitudinalAvailable)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertTrue(cp.pcmCruise)
    self.assertEqual(cp.safetyConfigs[-1].safetyModel, structs.CarParams.SafetyModel.hyundaiLegacy)
    self.assertFalse(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG)
    self.assertFalse(cp.flags & HyundaiFlags.ELANTRA_GT_LONGITUDINAL)

  def test_alpha_selects_normal_longitudinal_safety(self):
    cp = self.get_params(alpha=True)
    cp_sp = CarInterface.get_params_sp(cp, TARGET, self.fingerprint, self.firmware, True, False)
    self.assertTrue(cp.alphaLongitudinalAvailable)
    self.assertTrue(cp.openpilotLongitudinalControl)
    self.assertFalse(cp.pcmCruise)
    self.assertEqual(cp.safetyConfigs[-1].safetyModel, structs.CarParams.SafetyModel.hyundai)
    self.assertEqual(cp.safetyConfigs[-1].safetyParam, HyundaiSafetyFlags.LONG)
    self.assertTrue(cp.flags & HyundaiFlags.LEGACY)
    self.assertTrue(cp.flags & HyundaiFlags.USE_FCA)
    self.assertTrue(cp.flags & HyundaiFlags.MIN_STEER_32_MPH)
    self.assertTrue(cp.flags & HyundaiFlags.ELANTRA_GT_LONGITUDINAL)
    self.assertAlmostEqual(cp.minSteerSpeed, 32 * CV.MPH_TO_MS, places=5)
    self.assertTrue(cp.radarUnavailable)
    self.assertEqual(cp_sp.safetyParam, 0)

  def test_release_and_documentation_do_not_enable_port(self):
    self.assert_unavailable(is_release=True)
    self.assert_unavailable(docs=True)

  def test_missing_radar_or_abs_firmware_is_rejected(self):
    for missing_index in range(len(RECORDED_FW)):
      with self.subTest(missing_index=missing_index):
        self.firmware = recorded_firmware()
        del self.firmware[missing_index]
        self.assert_unavailable()

  def test_different_firmware_or_address_is_rejected(self):
    for fw_index in range(len(RECORDED_FW)):
      for field in ('fwVersion', 'address'):
        with self.subTest(fw_index=fw_index, field=field):
          self.firmware = recorded_firmware()
          fw = self.firmware[fw_index]
          setattr(fw, field, fw.fwVersion + b'other' if field == 'fwVersion' else fw.address + 1)
          self.assert_unavailable()

  def test_identical_firmware_on_two_query_buses_is_accepted(self):
    duplicate = recorded_firmware()
    for fw in duplicate:
      fw.bus = 1
    self.firmware.extend(duplicate)
    self.assertTrue(self.get_params(alpha=True).openpilotLongitudinalControl)

  def test_conflicting_duplicate_firmware_is_rejected(self):
    duplicate = recorded_firmware()[0]
    duplicate.fwVersion += b'other'
    self.firmware.append(duplicate)
    self.assert_unavailable()

  def test_subaddressed_firmware_does_not_match(self):
    self.firmware[0].subAddress = 1
    self.assert_unavailable()

  def test_missing_or_wrong_length_scc_message_is_rejected(self):
    for address in SCC_MESSAGES:
      for length in (None, 7):
        with self.subTest(address=hex(address), length=length):
          self.fingerprint = recorded_fingerprint()
          if length is None:
            del self.fingerprint[0][address]
          else:
            self.fingerprint[0][address] = length
          self.assert_unavailable()

  def test_scc_on_camera_bus_does_not_match(self):
    self.fingerprint[2] = self.fingerprint[0]
    self.fingerprint[0] = {}
    self.assert_unavailable()

  def test_escc_does_not_select_software_only_port(self):
    self.fingerprint[0][ESCC_MSG] = 8
    self.assert_unavailable()

  def test_other_supported_hyundai_does_not_select_variant(self):
    cp = self.get_params(alpha=True, candidate=CAR.HYUNDAI_SONATA)
    self.assertTrue(cp.openpilotLongitudinalControl)
    self.assertFalse(cp.flags & HyundaiFlags.ELANTRA_GT_LONGITUDINAL)

  def test_unrecorded_radar_option_layout_is_rejected(self):
    for address, length in ((0x483, 8), (0x4a2, 2)):
      with self.subTest(address=hex(address)):
        self.fingerprint = recorded_fingerprint()
        self.fingerprint[0][address] = length
        self.assert_unavailable()

  def test_other_legacy_platforms_remain_unavailable(self):
    for candidate in LEGACY_SAFETY_MODE_CAR - {TARGET}:
      with self.subTest(candidate=candidate):
        self.assert_unavailable(candidate=candidate)

  def test_non_scc_platform_is_rejected(self):
    candidate = CAR.HYUNDAI_KONA_NON_SCC
    self.assertFalse(supports_elantra_longitudinal(candidate, self.fingerprint, self.firmware))
    cp = self.get_params(alpha=True, candidate=candidate)
    CarInterface.get_params_sp(cp, candidate, self.fingerprint, self.firmware, True, False)
    self.assertFalse(cp.alphaLongitudinalAvailable)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertTrue(cp.pcmCruise)


class TestElantraLongitudinalInitialization(unittest.TestCase):
  def setUp(self):
    self.can_recv = Mock(return_value=[])
    self.can_send = Mock()

  @staticmethod
  def get_params(alpha=True, candidate=TARGET):
    fingerprint = recorded_fingerprint()
    firmware = recorded_firmware()
    cp = CarInterface.get_params(candidate, fingerprint, firmware, alpha, False, False)
    cp_sp = CarInterface.get_params_sp(cp, candidate, fingerprint, firmware, alpha, False)
    return cp, cp_sp

  def test_target_uses_verified_radar_initialization(self):
    cp, cp_sp = self.get_params()
    with patch('opendbc.car.hyundai.interface.initialize_elantra_longitudinal') as initialize, \
         patch('opendbc.car.hyundai.interface.disable_ecu') as disable:
      CarInterface.init(cp, cp_sp, self.can_recv, self.can_send)
      initialize.assert_called_once_with(self.can_recv, self.can_send)
      disable.assert_not_called()

  def test_target_initialization_failure_propagates(self):
    cp, cp_sp = self.get_params()
    failure = RuntimeError('radar output remains active')
    with patch('opendbc.car.hyundai.interface.initialize_elantra_longitudinal', side_effect=failure) as initialize, \
         patch('opendbc.car.hyundai.interface.disable_ecu') as disable:
      with self.assertRaises(RuntimeError) as caught:
        CarInterface.init(cp, cp_sp, self.can_recv, self.can_send)
      self.assertIs(caught.exception, failure)
      initialize.assert_called_once_with(self.can_recv, self.can_send)
      disable.assert_not_called()

  def test_stock_target_does_not_disable_radar(self):
    cp, cp_sp = self.get_params(alpha=False)
    with patch('opendbc.car.hyundai.interface.initialize_elantra_longitudinal') as initialize, \
         patch('opendbc.car.hyundai.interface.disable_ecu') as disable:
      CarInterface.init(cp, cp_sp, self.can_recv, self.can_send)
      initialize.assert_not_called()
      disable.assert_not_called()
      self.can_send.assert_not_called()

  def test_existing_supported_hyundai_keeps_standard_initialization(self):
    cp, cp_sp = self.get_params(candidate=CAR.HYUNDAI_SONATA)
    self.assertTrue(cp.openpilotLongitudinalControl)
    with patch('opendbc.car.hyundai.interface.initialize_elantra_longitudinal') as initialize, \
         patch('opendbc.car.hyundai.interface.disable_ecu') as disable:
      CarInterface.init(cp, cp_sp, self.can_recv, self.can_send)
      initialize.assert_not_called()
      disable.assert_called_once_with(self.can_recv, self.can_send, bus=0, addr=0x7d0, com_cont_req=b'\x28\x83\x01')


class TestElantraLongitudinalCommands(unittest.TestCase):
  def setUp(self):
    self.cp = CarInterface.get_params(TARGET, recorded_fingerprint(), recorded_firmware(), True, False, False)
    self.packer = CANPacker('hyundai_kia_generic')
    self.parser = CANParser('hyundai_kia_generic', [('SCC11', 50), ('SCC12', 50), ('SCC14', 50), ('FCA11', 50),
                                                 ('SCC13', 5), ('FCA12', 5)], 0)
    self.hud = structs.CarControl.HUDControl(leadDistanceBars=2)
    self.lead = CanLeadData(0, 0, 0, False)

  def commands(self, index, enabled=True, override=False, accel=0.0, stopping=False):
    tuning = LongitudinalState(desired_accel=accel, actual_accel=accel, stopping=stopping,
                               jerk_upper=3.0, jerk_lower=5.0)
    return create_acc_commands(self.packer, enabled, accel, 3.0, index, self.lead, self.hud, 60,
                               stopping, override, True, self.cp, True, tuning)

  def decode(self, messages, index=0):
    self.parser.update([(1_000_000_000 + index * 20_000_000, messages)])
    return self.parser.vl

  def test_acceleration_and_override_encoding(self):
    for index, (accel, override) in enumerate(((-3.5, False), (-1.0, False), (0.0, True), (2.0, False))):
      with self.subTest(accel=accel, override=override):
        values = self.decode(self.commands(index, override=override, accel=accel), index)
        self.assertAlmostEqual(values['SCC12']['aReqRaw'], accel)
        self.assertAlmostEqual(values['SCC12']['aReqValue'], accel)
        self.assertEqual(values['SCC12']['ACCMode'], 2 if override else 1)
        self.assertEqual(values['SCC14']['ACCMode'], 2 if override else 1)

  def test_counters_checksums_and_nonactuating_fca(self):
    for index in range(32):
      messages = self.commands(index, accel=-1.0)
      values = self.decode(messages, index)
      self.assertEqual({address for address, _, _ in messages}, {0x420, 0x421, 0x389, 0x38d})
      self.assertTrue(all(bus == 0 and len(data) == 8 for _, data, bus in messages))
      self.assertEqual(values['SCC11']['AliveCounterACC'], index % 16)
      self.assertEqual(values['SCC12']['CR_VSM_Alive'], index % 15)
      # The generic DBC counter position is unused on the recorded legacy FCA.
      self.assertEqual(values['FCA11']['CR_FCA_Alive'], 0)
      fca11 = next(data for address, data, _ in messages if address == 0x38d)
      self.assertEqual(fca11[7] & 15, index % 15)
      self.assertEqual(sum((byte >> 4) + (byte & 15) for byte in fca11) % 16, 0)
      scc12 = next(data for address, data, _ in messages if address == 0x421)
      self.assertEqual(sum((byte >> 4) + (byte & 15) for byte in scc12) % 16, 0)
      for message, signals in (
        ('SCC12', ('AEB_CmdAct', 'CF_VSM_DecCmdAct', 'CR_VSM_DecCmd')),
        ('FCA11', ('FCA_CmdAct', 'CF_VSM_DecCmdAct', 'CR_VSM_DecCmd', 'FCA_StopReq')),
      ):
        for signal in signals:
          self.assertEqual(values[message][signal], 0, (message, signal))
      self.assertEqual(values['FCA11']['FCA_Status'], 1)

  def test_disabled_control_and_stopping_encoding(self):
    values = self.decode(self.commands(0, enabled=False))
    self.assertEqual(values['SCC12']['ACCMode'], 0)
    self.assertEqual(values['SCC12']['aReqRaw'], 0)
    self.assertEqual(values['SCC12']['aReqValue'], 0)
    self.assertEqual(values['SCC11']['VSetDis'], 0)
    self.assertEqual(values['SCC12']['StopReq'], 0)
    values = self.decode(self.commands(1, stopping=True, accel=-1.0), 1)
    self.assertEqual(values['SCC12']['StopReq'], 1)

  def test_options_report_aeb_disabled(self):
    messages = create_acc_opt(self.packer, self.cp)
    values = self.decode(messages)
    self.assertEqual({address for address, _, _ in messages}, {0x50a})
    self.assertEqual(values['SCC13']['SCC_Equip'], 1)
    self.assertEqual(values['SCC13']['SCCDrvModeRValue'], 2)
    self.assertEqual(values['SCC13']['Lead_Veh_Dep_Alert_USM'], 0)
    # This variant reports disabled status in FCA11, not the absent FCA12.
    self.assertEqual(self.decode(self.commands(0))['FCA11']['FCA_Status'], 1)

  def test_legacy_fca_golden_packets_and_counter_wrap(self):
    # Derived independently from recorded 00 00 48 00 00 00 00 xx frames:
    # change only FCA_Status 2 -> 1, then update counter/checksum byte.
    expected = {
      0: '0000440000000080', 1: '0000440000000071', 2: '0000440000000062',
      14: '00004400000000ae', 15: '0000440000000080', 16: '0000440000000071',
    }
    for index, payload in expected.items():
      with self.subTest(index=index):
        fca = next(data for address, data, _ in self.commands(index) if address == 0x38d)
        self.assertEqual(fca.hex(), payload)

  def test_other_hyundai_retains_crc_fca_and_options(self):
    self.cp = CarInterface.get_params(CAR.HYUNDAI_SONATA, recorded_fingerprint(), recorded_firmware(), True, False, False)
    for index in range(32):
      messages = self.commands(index)
      values = self.decode(messages, index)
      fca = next(data for address, data, _ in messages if address == 0x38d)
      self.assertEqual(fca[7], hyundai_checksum(fca[:7]))
      self.assertEqual(values['FCA11']['CR_FCA_Alive'], index % 15)
      self.assertEqual(values['FCA11']['PAINT1_Status'], 1)
    messages = create_acc_opt(self.packer, self.cp)
    values = self.decode(messages, 32)
    self.assertEqual({address for address, _, _ in messages}, {0x50a, 0x483})
    self.assertEqual(values['FCA12']['FCA_USM'], 1)
    self.assertEqual(values['SCC13']['Lead_Veh_Dep_Alert_USM'], 2)


class TestElantraLongitudinalRuntime(unittest.TestCase):
  def setUp(self):
    fingerprint, firmware = recorded_fingerprint(), recorded_firmware()
    cp = CarInterface.get_params(TARGET, fingerprint, firmware, True, False, False)
    cp_sp = CarInterface.get_params_sp(cp, TARGET, fingerprint, firmware, True, False)
    # card sets these after get_params; test the actual runtime configuration.
    cp_sp.flags |= HyundaiFlagsSP.LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE
    cp_sp.safetyParam |= HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE
    self.ci = CarInterface(cp, cp_sp)
    self.packer = CANPacker('hyundai_kia_generic')
    self.frame = 0

  def update(self, main=0, button=0, brake=False, gas=False, fault=0):
    overrides = {
      'CLU11': {'CF_Clu_CruiseSwMain': main, 'CF_Clu_CruiseSwState': button},
      'TCS13': {'ACCEnable': fault, 'DriverOverride': 2 if brake else 0},
      'EMS16': {'CF_Ems_AclAct': int(gas)},
      'WHL_SPD11': {key: 50 for key in ('WHL_SPD_FL', 'WHL_SPD_FR', 'WHL_SPD_RL', 'WHL_SPD_RR')},
    }
    # Supply every configured chassis/camera message; never supply radar output.
    messages = [self.packer.make_can_msg(state.name, 2 if bus == Bus.cam else 0, overrides.get(state.name, {}))
                for bus, parser in self.ci.can_parsers.items() for state in parser.message_states.values()]
    self.assertFalse({address for address, _, _ in messages} & {0x420, 0x421, 0x389, 0x38d, 0x50a, 0x483, 0x4a2})
    self.frame += 1
    return self.ci.update([(self.frame * 10_000_000, messages)])[0]

  def test_chassis_states_remain_available_without_radar(self):
    for _ in range(100):
      state = self.update()
    self.assertTrue(state.canValid)
    self.assertFalse(state.canTimeout)
    self.assertFalse(state.accFaulted)
    self.assertAlmostEqual(state.vEgo, 50 / 3.6, places=2)
    self.assertFalse(state.cruiseState.available)
    self.assertTrue(self.update(main=1).cruiseState.available)
    self.assertTrue(self.update(main=1).cruiseState.available)
    self.update()
    self.assertTrue(self.update(brake=True).brakePressed)
    self.assertFalse(self.update().brakePressed)
    self.assertTrue(self.update(gas=True).gasPressed)
    self.assertFalse(self.update().gasPressed)
    state = self.update(fault=1)
    self.assertTrue(state.accFaulted)
    self.assertFalse(state.cruiseState.available)
    self.assertTrue(self.update().cruiseState.available)
    self.assertFalse(self.update(main=1).cruiseState.available)

  def test_controller_schedule_and_legacy_fca(self):
    from collections import Counter

    counts = Counter()
    cc, cc_sp = structs.CarControl(), structs.CarControlSP()
    for frame in range(100):
      self.update(main=int(frame == 0))
      _, messages = self.ci.apply(cc.as_reader(), cc_sp, self.frame * 10_000_000)
      for address, data, bus in messages:
        self.assertEqual(bus, 0)
        counts[address] += 1
        if address == 0x38d:
          self.assertEqual(data[:7], bytes.fromhex('00004400000000'))
          self.assertEqual(data[7] & 15, (frame // 2) % 15)
          self.assertEqual(sum((byte >> 4) + (byte & 15) for byte in data) % 16, 0)
        elif address == 0x7d0:
          self.assertEqual(data, bytes.fromhex('023e800000000000'))
    self.assertEqual(counts, {0x340: 100, 0x420: 50, 0x421: 50, 0x389: 50, 0x38d: 50, 0x50a: 5, 0x7d0: 1})


if __name__ == '__main__':
  unittest.main()
