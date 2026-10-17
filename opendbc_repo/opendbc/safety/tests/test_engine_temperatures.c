// Standalone host regression test using the actual Panda safety implementation.
// From the repository root (also works with MinGW gcc on Windows):
// gcc -std=gnu11 -fno-builtin -DCANFD -I opendbc_repo opendbc_repo/opendbc/safety/tests/test_engine_temperatures.c -o build/test_hyundai_engine_temperatures

#include "libsafety/safety.c"

static unsigned int checks = 0U;
#define CHECK(expression) do { \
  checks++; \
  if (!(expression)) { \
    printf("FAIL line %d: %s\n", __LINE__, #expression); \
    exit(1); \
  } \
} while (0)

static const uint32_t request_addrs[] = {0x7E0U, 0x7DFU, 0x7E0U, 0x7E1U, 0x7E1U};
static const uint8_t requests[][8] = {
  {0x03U, 0x22U, 0xE0U, 0x01U, 0U, 0U, 0U, 0U},
  {0x02U, 0x01U, 0x05U, 0U, 0U, 0U, 0U, 0U},
  {0x30U, 0U, 0x0AU, 0U, 0U, 0U, 0U, 0U},
  {0x03U, 0x22U, 0x01U, 0xA4U, 0U, 0U, 0U, 0U},
  {0x30U, 0U, 0x0AU, 0U, 0U, 0U, 0U, 0U},
};
static const unsigned int request_count = sizeof(requests) / sizeof(requests[0]);
_Static_assert(sizeof(requests) / sizeof(requests[0]) == sizeof(request_addrs) / sizeof(request_addrs[0]), "request count mismatch");

static bool expected_request(uint32_t addr, const uint8_t *data) {
  for (unsigned int index = 0U; index < request_count; index++) {
    bool matches = addr == request_addrs[index];
    for (unsigned int byte = 0U; byte < 8U; byte++) {
      matches = matches && (data[byte] == requests[index][byte]);
    }
    if (matches) return true;
  }
  return false;
}

static CANPacket_t request_packet(unsigned int index) {
  CANPacket_t packet = {0};
  packet.addr = request_addrs[index];
  packet.bus = 1U;
  packet.data_len_code = 8U;
  for (unsigned int i = 0U; i < 8U; i++) {
    packet.data[i] = requests[index][i];
  }
  return packet;
}

static void configure(uint16_t mode, uint16_t param) {
  CHECK(set_safety_hooks(mode, param) == 0);
  init_tests();
  set_mads_params(false, false, false);
  set_controls_allowed_lat(false);
  set_controls_requested_lat(false);
}

static void test_exact_requests(void) {
  configure(SAFETY_HYUNDAI_LEGACY, HYUNDAI_PARAM_ENGINE_TEMPERATURES);
  for (unsigned int index = 0U; index < request_count; index++) {
    CANPacket_t packet = request_packet(index);
    for (unsigned int controls = 0U; controls < 2U; controls++) {
      set_controls_allowed(controls != 0U);
      CHECK(safety_tx_hook(&packet) == 1);
      CHECK(get_controls_allowed() == (controls != 0U));
    }
    for (unsigned int byte = 0U; byte < 8U; byte++) {
      for (unsigned int value = 0U; value < 256U; value++) {
        packet = request_packet(index);
        packet.data[byte] = value;
        CHECK(safety_tx_hook(&packet) == (value == requests[index][byte]));
      }
    }
    for (unsigned int bus = 0U; bus < 8U; bus++) {
      for (unsigned int dlc = 0U; dlc < 16U; dlc++) {
        packet = request_packet(index);
        packet.bus = bus;
        packet.data_len_code = dlc;
        CHECK(safety_tx_hook(&packet) == ((bus == 1U) && (dlc == 8U)));
      }
    }
    for (unsigned int addr = 0x700U; addr < 0x800U; addr++) {
      packet = request_packet(index);
      packet.addr = addr;
      CHECK(safety_tx_hook(&packet) == expected_request(addr, packet.data));
    }
    packet = request_packet(index);
    packet.extended = 1U;
    CHECK(safety_tx_hook(&packet) == 0);
    packet.extended = 0U;
    packet.fd = 1U;
    CHECK(!safety_tx_hook(&packet));
    packet.fd = 0U;
    packet.extended = 1U;
    packet.addr = 0x18DA10F1U;
    CHECK(safety_tx_hook(&packet) == 0);
    packet.addr = 0x18DB33F1U;
    CHECK(safety_tx_hook(&packet) == 0);
    set_relay_malfunction(true);
    packet = request_packet(index);
    CHECK(!safety_tx_hook(&packet));
    set_relay_malfunction(false);
  }
}

static void test_flags_and_routing(void) {
  const uint16_t modes[] = {SAFETY_HYUNDAI_LEGACY, SAFETY_HYUNDAI, SAFETY_HYUNDAI_CANFD, SAFETY_SILENT, SAFETY_NOOUTPUT};
  const uint16_t params[] = {0U, 1U, 1024U, 2048U, 0xFFFFU};
  for (unsigned int mode = 0U; mode < sizeof(modes) / sizeof(modes[0]); mode++) {
    for (unsigned int param = 0U; param < sizeof(params) / sizeof(params[0]); param++) {
      // Start enabled each time to catch stale Hyundai flag state after transitions.
      configure(SAFETY_HYUNDAI_LEGACY, HYUNDAI_PARAM_ENGINE_TEMPERATURES);
      configure(modes[mode], params[param]);
      bool enabled = (modes[mode] == SAFETY_HYUNDAI_LEGACY) && ((params[param] & 2048U) != 0U);
      CHECK(safety_uses_obd_can2() == enabled);
      for (unsigned int index = 0U; index < request_count; index++) {
        CANPacket_t packet = request_packet(index);
        CHECK(safety_tx_hook(&packet) == enabled);
      }
    }
  }
  for (unsigned int mode = 0U; mode <= SAFETY_VOLKSWAGEN_MEB; mode++) {
    for (unsigned int param = 0U; param < sizeof(params) / sizeof(params[0]); param++) {
      if (set_safety_hooks(mode, params[param]) != 0) continue;
      const bool expected = ((current_safety_mode == SAFETY_ELM327) && (params[param] == 0U)) ||
                            ((current_safety_mode == SAFETY_HYUNDAI_LEGACY) && ((params[param] & 2048U) != 0U));
      CHECK(safety_uses_obd_can2() == expected);
    }
  }
  // Model set_safety_mode's failure fallback, which must restore normal routing.
  configure(SAFETY_HYUNDAI_LEGACY, HYUNDAI_PARAM_ENGINE_TEMPERATURES);
  CHECK(set_safety_hooks(0xFFFFU, HYUNDAI_PARAM_ENGINE_TEMPERATURES) == -1);
  configure(SAFETY_SILENT, 0U);
  CHECK(!safety_uses_obd_can2());
}

static CANPacket_t steer_packet(int torque) {
  CANPacket_t packet = {0};
  packet.addr = 0x340U;
  packet.bus = 0U;
  packet.data_len_code = 8U;
  unsigned int encoded = ((unsigned int)(torque + 1024) & 0x7FFU) | 0x800U;
  packet.data[2] = encoded & 0xFFU;
  packet.data[3] = (encoded >> 8U) & 0xFFU;
  return packet;
}

static void test_unchanged_controls_and_forwarding(void) {
  for (unsigned int flag = 0U; flag <= 2048U; flag += 2048U) {
    configure(SAFETY_HYUNDAI_LEGACY, flag);
    for (int bus = 0; bus < 3; bus++) {
      for (int addr = 0; addr < 0x800; addr++) {
        int expected = bus == 0 ? 2 : -1;
        if ((bus == 2) && (addr != 0x340) && (addr != 0x485)) {
          expected = 0;
        }
        CHECK(safety_fwd_hook(bus, addr) == expected);
      }
    }
    for (int torque = -400; torque <= 400; torque++) {
      configure(SAFETY_HYUNDAI_LEGACY, flag);
      CANPacket_t packet = steer_packet(torque);
      set_controls_allowed(false);
      CHECK(safety_tx_hook(&packet) == (torque == 0));
      set_controls_allowed(true);
      set_desired_torque_last(torque);
      set_rt_torque_last(torque);
      set_torque_driver(0, 0);
      CHECK(safety_tx_hook(&packet) == ((torque >= -384) && (torque <= 384)));
    }
    for (int torque = -8; torque <= 8; torque++) {
      configure(SAFETY_HYUNDAI_LEGACY, flag);
      CANPacket_t packet = steer_packet(torque);
      set_controls_allowed(true);
      CHECK(safety_tx_hook(&packet) == ((torque >= -3) && (torque <= 3)));
    }
    configure(SAFETY_HYUNDAI_LEGACY, flag);
    CANPacket_t response = {0};
    response.addr = 0x7E8U;
    response.bus = 1U;
    response.data_len_code = 8U;
    response.data[0] = 0x10U;
    response.data[1] = 0x33U;
    response.data[2] = 0x62U;
    for (unsigned int addr = 0x7E8U; addr <= 0x7E9U; addr++) {
      response.addr = addr;
      CHECK(safety_rx_hook(&response) == 1);
      CHECK(!get_controls_allowed());
    }
    CANPacket_t acceleration = {0};
    acceleration.addr = 0x421U;
    acceleration.data_len_code = 8U;
    CHECK(safety_tx_hook(&acceleration) == 0);
  }
}

static void test_mads_and_rx_configuration_preserved(void) {
  configure(SAFETY_HYUNDAI_LEGACY, 0U);
  const RxCheck *original_rx_checks = current_safety_config.rx_checks;
  const int original_rx_count = current_safety_config.rx_checks_len;
  configure(SAFETY_HYUNDAI_LEGACY, HYUNDAI_PARAM_ENGINE_TEMPERATURES);
  CHECK(current_safety_config.rx_checks == original_rx_checks);
  CHECK(current_safety_config.rx_checks_len == original_rx_count);

  for (unsigned int enabled = 0U; enabled < 2U; enabled++) {
    for (unsigned int lateral = 0U; lateral < 2U; lateral++) {
      for (unsigned int controls = 0U; controls < 2U; controls++) {
        configure(SAFETY_HYUNDAI_LEGACY, HYUNDAI_PARAM_ENGINE_TEMPERATURES);
        set_mads_params(enabled, false, false);
        set_controls_allowed_lat(lateral);
        set_controls_requested_lat(lateral);
        set_controls_allowed(controls);
        const bool mads_before = get_enable_mads();
        const bool lateral_before = get_controls_allowed_lat();
        const bool requested_before = get_controls_requested_lat();
        for (unsigned int index = 0U; index < request_count; index++) {
          CANPacket_t packet = request_packet(index);
          CHECK(safety_tx_hook(&packet));
          CHECK(get_enable_mads() == mads_before);
          CHECK(get_controls_allowed_lat() == lateral_before);
          CHECK(get_controls_requested_lat() == requested_before);
          CHECK(get_controls_allowed() == (controls != 0U));
        }
      }
    }
  }
}

int main(void) {
  test_exact_requests();
  test_flags_and_routing();
  test_unchanged_controls_and_forwarding();
  test_mads_and_rx_configuration_preserved();
  printf("PASS: %u engine/DCT temperature safety and regression checks\n", checks);
  return 0;
}
