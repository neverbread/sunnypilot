// Host regressions for the restricted Elantra longitudinal port, using real Panda hooks.
// gcc -std=gnu11 -fno-builtin -DCANFD -I opendbc_repo opendbc_repo/opendbc/safety/tests/test_elantra_longitudinal.c -o build/test_elantra_longitudinal
// This validates software enforcement only, not the vehicle's acceptance of commands.

#include "libsafety/safety.c"

static unsigned int checks = 0U;
#define CHECK(expression) do { \
  checks++; \
  if (!(expression)) { \
    printf("FAIL line %d: %s\n", __LINE__, #expression); \
    exit(1); \
  } \
} while (0)

static const uint16_t LONG_PARAM = 4U;

static CANPacket_t packet(uint32_t address, uint8_t bus, uint8_t length) {
  CANPacket_t msg = {0};
  msg.addr = address;
  msg.bus = bus;
  msg.data_len_code = length;
  return msg;
}

static void configure_with_sp(uint16_t extra_flags, bool mads, uint16_t param_sp) {
  set_current_safety_param_sp(param_sp);
  CHECK(set_safety_hooks(SAFETY_HYUNDAI, LONG_PARAM | extra_flags) == 0);
  init_tests();
  // init_tests resets this metadata; retain the configuration the hooks read.
  set_current_safety_param_sp(param_sp);
  set_mads_params(mads, false, false);
  set_controls_allowed_lat(mads);
  set_controls_requested_lat(mads);
  CHECK(hyundai_longitudinal);
  CHECK(!hyundai_legacy);
}

static void configure(uint16_t extra_flags, bool mads) {
  configure_with_sp(extra_flags, mads, 0U);
}

static CANPacket_t acceleration(int raw, int value) {
  CANPacket_t msg = packet(0x421U, 0U, 8U);
  const unsigned int raw_encoded = (unsigned int)(raw + 1023) & 0x7FFU;
  const unsigned int value_encoded = (unsigned int)(value + 1023) & 0x7FFU;
  msg.data[3] = raw_encoded & 0xFFU;
  msg.data[4] = ((raw_encoded >> 8U) & 7U) | ((value_encoded & 7U) << 5U);
  msg.data[5] = (value_encoded >> 3U) & 0xFFU;
  return msg;
}

static void receive_button(uint8_t button, uint8_t counter) {
  CANPacket_t msg = packet(0x4F1U, 0U, 4U);
  msg.data[0] = button;
  msg.data[3] = (counter & 0xFU) << 4U;
  CHECK(safety_rx_hook(&msg));
}

static void receive_main_button(bool pressed, uint8_t counter) {
  receive_button(pressed ? 8U : 0U, counter);
}

static void send_main_state(bool enabled) {
  CANPacket_t msg = packet(0x420U, 0U, 8U);
  msg.data[0] = enabled ? 1U : 0U;
  CHECK(safety_tx_hook(&msg));
}

static void receive_brake(bool pressed, uint8_t counter) {
  CANPacket_t msg = packet(0x394U, 0U, 8U);
  msg.data[1] = (counter & 7U) << 5U;
  msg.data[5] = pressed ? 0x40U : 0U;
  msg.data[6] = hyundai_compute_checksum(&msg);
  CHECK(safety_rx_hook(&msg));
}

static void receive_moving_wheels(void) {
  CANPacket_t msg = packet(0x386U, 0U, 8U);
  msg.data[0] = HYUNDAI_STANDSTILL_THRSLD + 1U;
  msg.data[6] = HYUNDAI_STANDSTILL_THRSLD + 1U;
  const uint32_t checksum = hyundai_compute_checksum(&msg);
  msg.data[5] |= (checksum & 3U) << 6U;
  msg.data[7] |= (checksum >> 2U) << 6U;
  CHECK(safety_rx_hook(&msg));
  CHECK(get_vehicle_moving());
}

static void test_acceleration_enforcement(uint16_t extra_flags) {
  for (unsigned int mads = 0U; mads < 2U; mads++) {
    configure(extra_flags, mads != 0U);
    for (unsigned int enabled = 0U; enabled < 2U; enabled++) {
      for (unsigned int gas = 0U; gas < 2U; gas++) {
        set_controls_allowed(enabled != 0U);
        set_gas_pressed_prev(gas != 0U);
        for (int accel = -500; accel <= 350; accel++) {
          const bool expected = (accel == 0) || ((enabled != 0U) && (gas == 0U) && (accel >= -350) && (accel <= 200));
          // Both fields are independently bounded; a valid companion field must
          // not allow an unsafe raw request or an unsafe jerk-limited request.
          CANPacket_t raw = acceleration(accel, 0);
          CANPacket_t value = acceleration(0, accel);
          CHECK(safety_tx_hook(&raw) == expected);
          CHECK(safety_tx_hook(&value) == expected);
        }
      }
    }
  }
}

static void test_buttons_and_brake(uint16_t extra_flags) {
  for (unsigned int mads = 0U; mads < 2U; mads++) {
    for (uint8_t enable = HYUNDAI_BTN_RESUME; enable <= HYUNDAI_BTN_SET; enable++) {
      configure(extra_flags, mads != 0U);
      CANPacket_t positive = acceleration(100, 100);
      CANPacket_t negative = acceleration(-100, -100);
      CANPacket_t zero = acceleration(0, 0);
      receive_button(enable, 0U);
      CHECK(!get_controls_allowed());
      receive_button(HYUNDAI_BTN_NONE, 1U);
      CHECK(get_controls_allowed());
      CHECK(safety_tx_hook(&positive));
      receive_button(HYUNDAI_BTN_CANCEL, 2U);
      CHECK(!get_controls_allowed());
      CHECK(!safety_tx_hook(&positive));
      CHECK(!safety_tx_hook(&negative));
      CHECK(safety_tx_hook(&zero));
      receive_button(HYUNDAI_BTN_NONE, 3U);
      CHECK(!get_controls_allowed());
      receive_button(enable, 4U);
      receive_button(HYUNDAI_BTN_NONE, 5U);
      CHECK(get_controls_allowed());

      // Brake press always removes longitudinal authority, including with MADS.
      receive_brake(true, 0U);
      CHECK(!get_controls_allowed());
      CHECK(!safety_tx_hook(&positive));
      CHECK(!safety_tx_hook(&negative));
      CHECK(safety_tx_hook(&zero));
      // Brake release alone cannot engage; a new Set/Resume release is required.
      receive_brake(false, 1U);
      CHECK(!get_controls_allowed());
      receive_button(enable, 6U);
      receive_button(HYUNDAI_BTN_NONE, 7U);
      CHECK(get_controls_allowed());

      // Set/Resume cannot restore acceleration while braking in motion.
      receive_moving_wheels();
      receive_brake(true, 2U);
      receive_button(enable, 8U);
      receive_button(HYUNDAI_BTN_NONE, 9U);
      CHECK(!get_controls_allowed());
      CHECK(!safety_tx_hook(&positive));
      CHECK(!safety_tx_hook(&negative));
    }
  }
}

static void test_radar_conflict(uint16_t extra_flags) {
  const uint32_t stock_messages[] = {0x420U, 0x421U, 0x50AU, 0x389U};
  for (unsigned int i = 0U; i < sizeof(stock_messages) / sizeof(stock_messages[0]); i++) {
    for (uint8_t bus = 0U; bus < 3U; bus++) {
      configure(extra_flags, false);
      set_controls_allowed(true);
      CANPacket_t incoming = packet(stock_messages[i], bus, 8U);
      CANPacket_t outgoing = acceleration(100, 100);
      CHECK(safety_tx_hook(&outgoing));
      CHECK(safety_rx_hook(&incoming));
      CHECK(get_relay_malfunction() == (bus == 0U));
      CHECK(safety_tx_hook(&outgoing) == (bus != 0U));
      if (bus == 0U) {
        outgoing = acceleration(0, 0);
        CHECK(!safety_tx_hook(&outgoing));
        incoming = packet(0x123U, 0U, 8U);
        CHECK(safety_rx_hook(&incoming));
        // Recovery of unrelated traffic must not clear the latched fault.
        CHECK(get_relay_malfunction());
        CHECK(safety_fwd_hook(0, 0x123) == -1);
      }
    }
  }
}

static void test_aeb_rejected(uint16_t extra_flags) {
  configure(extra_flags, false);
  set_controls_allowed(true);
  CANPacket_t scc = acceleration(0, 0);
  CANPacket_t fca = packet(0x38DU, 0U, 8U);
  CHECK(safety_tx_hook(&scc));
  CHECK(safety_tx_hook(&fca));
  for (unsigned int decel = 1U; decel < 256U; decel++) {
    scc.data[2] = decel;
    fca.data[1] = decel;
    CHECK(!safety_tx_hook(&scc));
    CHECK(!safety_tx_hook(&fca));
  }
  scc = acceleration(0, 0);
  scc.data[6] = 0x40U;  // SCC12.AEB_CmdAct
  CHECK(!safety_tx_hook(&scc));
  fca = packet(0x38DU, 0U, 8U);
  fca.data[2] = 0x10U;  // FCA11.FCA_CmdAct
  CHECK(!safety_tx_hook(&fca));
  fca.data[2] = 0U;
  fca.data[3] = 0x80U;  // FCA11.CF_VSM_DecCmdAct
  CHECK(!safety_tx_hook(&fca));
}

static void test_only_tester_present(uint16_t extra_flags) {
  configure(extra_flags, false);
  CANPacket_t allowed = packet(0x7D0U, 0U, 8U);
  allowed.data[0] = 2U;
  allowed.data[1] = 0x3EU;
  allowed.data[2] = 0x80U;
  CHECK(safety_tx_hook(&allowed));
  for (unsigned int byte = 0U; byte < 8U; byte++) {
    for (unsigned int value = 0U; value < 256U; value++) {
      CANPacket_t changed = allowed;
      changed.data[byte] = value;
      CHECK(safety_tx_hook(&changed) == (value == allowed.data[byte]));
    }
  }
  for (uint8_t bus = 0U; bus < 8U; bus++) {
    for (uint8_t dlc = 0U; dlc < 16U; dlc++) {
      CANPacket_t changed = allowed;
      changed.bus = bus;
      changed.data_len_code = dlc;
      CHECK(safety_tx_hook(&changed) == ((bus == 0U) && (dlc == 8U)));
    }
  }
  // Diagnostic sessions and communication-control writes remain forbidden
  // after startup switches Panda out of the diagnostic safety configuration.
  CANPacket_t diagnostic = packet(0x7D0U, 0U, 8U);
  diagnostic.data[0] = 2U;
  diagnostic.data[1] = 0x10U;
  diagnostic.data[2] = 3U;
  CHECK(!safety_tx_hook(&diagnostic));
  diagnostic.data[0] = 3U;
  diagnostic.data[1] = 0x28U;
  diagnostic.data[2] = 0x83U;
  diagnostic.data[3] = 1U;
  CHECK(!safety_tx_hook(&diagnostic));
}

static void test_checked_receivers(uint16_t extra_flags) {
  configure(extra_flags, false);
  const uint32_t checked_messages[] = {0x260U, 0x386U, 0x394U};
  for (unsigned int i = 0U; i < sizeof(checked_messages) / sizeof(checked_messages[0]); i++) {
    CANPacket_t incoming = packet(checked_messages[i], 0U, 8U);
    const int index = get_addr_check_index(&incoming, current_safety_config.rx_checks, current_safety_config.rx_checks_len);
    CHECK(index >= 0);
    const CanMsgCheck *check = &current_safety_config.rx_checks[index].msg[0];
    CHECK(!check->ignore_checksum);
    CHECK(!check->ignore_counter);
    CHECK(check->max_counter > 0U);
  }
  receive_brake(false, 0U);
  set_controls_allowed(true);
  CANPacket_t corrupt = packet(0x394U, 0U, 8U);
  corrupt.data[1] = 1U << 5U;
  corrupt.data[6] = hyundai_compute_checksum(&corrupt) ^ 1U;
  CHECK(!safety_rx_hook(&corrupt));
  CHECK(!get_controls_allowed());
}

static void test_runtime_main_cruise_configuration(uint16_t extra_flags) {
  // card adds this SP flag after constructing CarParams on the actual car.
  // Exercise it together with LONG and the optional OBD telemetry selection.
  for (unsigned int mads = 0U; mads < 2U; mads++) {
    configure_with_sp(extra_flags, mads != 0U, HYUNDAI_PARAM_SP_LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE);
    set_mads_params(mads != 0U, false, false);
    CHECK(hyundai_longitudinal_main_cruise_toggleable);
    CHECK(hyundai_engine_temperatures == ((extra_flags & HYUNDAI_PARAM_ENGINE_TEMPERATURES) != 0U));
    CHECK(safety_uses_obd_can2() == hyundai_engine_temperatures);

    CANPacket_t positive = acceleration(100, 100);
    CANPacket_t negative = acceleration(-100, -100);
    CANPacket_t zero = acceleration(0, 0);
    receive_main_button(false, 0U);
    CHECK(!get_acc_main_on());
    CHECK(!get_controls_allowed());
    CHECK(!get_controls_allowed_lat());

    receive_main_button(true, 1U);
    CHECK(get_acc_main_on());
    CHECK(!get_controls_allowed());  // Main alone never authorizes acceleration.
    CHECK(!safety_tx_hook(&positive));
    CHECK(get_controls_allowed_lat() == (mads != 0U));
    receive_main_button(true, 2U);  // Holding Main must not toggle repeatedly.
    CHECK(get_acc_main_on());
    receive_main_button(false, 3U);
    send_main_state(true);
    CHECK(get_acc_main_on_mismatches() == 0U);

    receive_button(HYUNDAI_BTN_SET, 4U);
    CHECK(!get_controls_allowed());
    receive_button(HYUNDAI_BTN_NONE, 5U);
    CHECK(get_controls_allowed());
    CHECK(safety_tx_hook(&positive));
    CHECK(safety_tx_hook(&negative));
    receive_button(HYUNDAI_BTN_CANCEL, 6U);
    CHECK(!get_controls_allowed());
    CHECK(!safety_tx_hook(&positive));
    CHECK(!safety_tx_hook(&negative));
    CHECK(safety_tx_hook(&zero));
    CHECK(get_acc_main_on());  // Cancel leaves Main enabled for a later Resume.
    send_main_state(true);
    CHECK(get_acc_main_on_mismatches() == 0U);
    receive_button(HYUNDAI_BTN_RESUME, 7U);
    CHECK(!get_controls_allowed());
    receive_button(HYUNDAI_BTN_NONE, 8U);
    CHECK(get_controls_allowed());
    CHECK(safety_tx_hook(&positive));

    receive_main_button(true, 9U);
    CHECK(!get_acc_main_on());
    CHECK(!get_controls_allowed_lat());
    send_main_state(false);
    CHECK(get_acc_main_on_mismatches() == 0U);
    receive_button(HYUNDAI_BTN_CANCEL, 10U);
    CHECK(!get_controls_allowed());
    receive_main_button(true, 11U);
    CHECK(get_acc_main_on());
    receive_main_button(false, 12U);

    // A transient old SCC11 main state is tolerated and a matching update resets it.
    send_main_state(false);
    send_main_state(false);
    CHECK(get_acc_main_on());
    CHECK(get_acc_main_on_mismatches() == 2U);
    send_main_state(true);
    CHECK(get_acc_main_on_mismatches() == 0U);
    CHECK(get_controls_allowed_lat() == (mads != 0U));

    // Three contradictory controller frames remove lateral permission. A later
    // CAN/button update must not silently re-enable it without a new action.
    send_main_state(false);
    send_main_state(false);
    send_main_state(false);
    CHECK(!get_acc_main_on());
    CHECK(!get_controls_allowed_lat());
    receive_main_button(false, 13U);
    CHECK(!get_acc_main_on());
    CHECK(!get_controls_allowed_lat());
    send_main_state(false);
    CHECK(get_acc_main_on_mismatches() == 0U);
    receive_main_button(true, 14U);
    receive_main_button(false, 15U);
    CHECK(get_acc_main_on());
    CHECK(get_controls_allowed_lat() == (mads != 0U));
    send_main_state(true);
    CHECK(get_acc_main_on_mismatches() == 0U);
    CHECK(!get_controls_allowed());
    CHECK(!safety_tx_hook(&positive));
  }
}

int main(void) {
  for (uint16_t telemetry = 0U; telemetry <= HYUNDAI_PARAM_ENGINE_TEMPERATURES; telemetry += HYUNDAI_PARAM_ENGINE_TEMPERATURES) {
    test_acceleration_enforcement(telemetry);
    test_buttons_and_brake(telemetry);
    test_radar_conflict(telemetry);
    test_aeb_rejected(telemetry);
    test_only_tester_present(telemetry);
    test_checked_receivers(telemetry);
    test_runtime_main_cruise_configuration(telemetry);
  }
  printf("PASS: %u Elantra longitudinal safety regression checks\n", checks);
  return 0;
}
