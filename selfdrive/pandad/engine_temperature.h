#pragma once

#include <array>
#include <cstdint>
#include <optional>

// Only these fixed read-only requests can cross the telemetry IPC boundary.
// Panda independently checks the address, bus, length and every payload byte.
struct EngineTemperatureFrame {
  uint32_t address;
  std::array<uint8_t, 8> data;
};

class EngineTemperatureRequests {
public:
  enum Kind { COOLANT = 0, OIL = 1, FLOW_CONTROL = 2, CLUTCH = 3, CLUTCH_FLOW_CONTROL = 4, PRESSURE = 5, BAROMETER = 6 };

  std::optional<EngineTemperatureFrame> get(unsigned int kind, uint64_t timestamp, uint64_t now, bool enabled) {
    if (!enabled) {
      flow_control_address = 0;
      return std::nullopt;
    }
    if (timestamp > now || now - timestamp >= 100000000ULL) return std::nullopt;

    if (kind == FLOW_CONTROL || kind == CLUTCH_FLOW_CONTROL) {
      const uint32_t address = kind == CLUTCH_FLOW_CONTROL ? 0x7E1 : 0x7E0;
      if (flow_control_address != address || now < last_request || now - last_request >= 500000000ULL) return std::nullopt;
      flow_control_address = 0;
      return EngineTemperatureFrame{address, {0x30, 0x00, 0x0A, 0, 0, 0, 0, 0}};
    }
    if (kind != COOLANT && kind != OIL && kind != CLUTCH && kind != PRESSURE && kind != BAROMETER) return std::nullopt;
    // Host-side shared diagnostic floor; the reader spaces requests by 240 ms.
    if (request_sent && (now < last_request || now - last_request < 230000000ULL)) return std::nullopt;
    last_request = now;
    request_sent = true;
    flow_control_address = kind == OIL ? 0x7E0 : (kind == CLUTCH ? 0x7E1 : 0);
    if (kind == CLUTCH) return EngineTemperatureFrame{0x7E1, {0x03, 0x22, 0x01, 0xA4, 0, 0, 0, 0}};
    if (kind == OIL) return EngineTemperatureFrame{0x7E0, {0x03, 0x22, 0xE0, 0x01, 0, 0, 0, 0}};
    if (kind == PRESSURE) return EngineTemperatureFrame{0x7DF, {0x03, 0x01, 0x05, 0x0B, 0, 0, 0, 0}};
    if (kind == BAROMETER) return EngineTemperatureFrame{0x7DF, {0x02, 0x01, 0x33, 0, 0, 0, 0, 0}};
    return EngineTemperatureFrame{0x7DF, {0x02, 0x01, 0x05, 0, 0, 0, 0, 0}};
  }

private:
  uint64_t last_request = 0;
  bool request_sent = false;
  uint32_t flow_control_address = 0;
};
