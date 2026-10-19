#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "selfdrive/pandad/engine_temperature.h"

TEST_CASE("Temperature requests contain only the fixed diagnostic frames") {
  EngineTemperatureRequests requests;
  auto coolant = requests.get(0, 1000000000ULL, 1000000000ULL, true);
  REQUIRE(coolant);
  CHECK(coolant->address == 0x7DF);
  CHECK(coolant->data == std::array<uint8_t, 8>{2, 1, 5, 0, 0, 0, 0, 0});
  CHECK_FALSE(requests.get(2, 1100000000ULL, 1100000000ULL, true));
  auto oil = requests.get(1, 2000000000ULL, 2000000000ULL, true);
  REQUIRE(oil);
  CHECK(oil->address == 0x7E0);
  CHECK(oil->data == std::array<uint8_t, 8>{3, 0x22, 0xE0, 1, 0, 0, 0, 0});
  auto flow = requests.get(2, 2100000000ULL, 2100000000ULL, true);
  REQUIRE(flow);
  CHECK(flow->address == 0x7E0);
  CHECK(flow->data == std::array<uint8_t, 8>{0x30, 0, 0x0A, 0, 0, 0, 0, 0});
  CHECK_FALSE(requests.get(2, 2200000000ULL, 2200000000ULL, true));
}

TEST_CASE("Temperature requests expire and cannot flood the bus") {
  EngineTemperatureRequests requests;
  CHECK_FALSE(requests.get(0, 0, 100000000ULL, true));
  CHECK_FALSE(requests.get(0, 2, 1, true));
  CHECK_FALSE(requests.get(0, 1, 1, false));
  CHECK_FALSE(requests.get(7, 1, 1, true));
  CHECK_FALSE(requests.get(65535, 1, 1, true));
  REQUIRE(requests.get(1, 1000000000ULL, 1000000000ULL, true));
  CHECK_FALSE(requests.get(0, 1229999999ULL, 1229999999ULL, true));
  CHECK_FALSE(requests.get(2, 1230000000ULL, 1230000000ULL, true));
  REQUIRE(requests.get(0, 1230000000ULL, 1230000000ULL, true));
  REQUIRE(requests.get(1, 2000000000ULL, 2000000000ULL, true));
  CHECK_FALSE(requests.get(2, 2100000000ULL, 2100000000ULL, false));
  CHECK_FALSE(requests.get(2, 2200000000ULL, 2200000000ULL, true));
  REQUIRE(requests.get(0, 3000000000ULL, 3000000000ULL, true));
  CHECK_FALSE(requests.get(2, 3100000000ULL, 3100000000ULL, true));
}

TEST_CASE("Boost uses only the fixed pressure reads without flow control") {
  EngineTemperatureRequests requests;
  auto pressure = requests.get(EngineTemperatureRequests::PRESSURE, 1000000000ULL, 1000000000ULL, true);
  REQUIRE(pressure);
  CHECK(pressure->address == 0x7DF);
  CHECK(pressure->data == std::array<uint8_t, 8>{3, 1, 5, 0x0B, 0, 0, 0, 0});
  CHECK_FALSE(requests.get(EngineTemperatureRequests::FLOW_CONTROL, 1100000000ULL, 1100000000ULL, true));
  CHECK_FALSE(requests.get(EngineTemperatureRequests::CLUTCH_FLOW_CONTROL, 1100000000ULL, 1100000000ULL, true));
  CHECK_FALSE(requests.get(EngineTemperatureRequests::BAROMETER, 1229999999ULL, 1229999999ULL, true));
  auto barometer = requests.get(EngineTemperatureRequests::BAROMETER, 1230000000ULL, 1230000000ULL, true);
  REQUIRE(barometer);
  CHECK(barometer->address == 0x7DF);
  CHECK(barometer->data == std::array<uint8_t, 8>{2, 1, 0x33, 0, 0, 0, 0, 0});
  CHECK_FALSE(requests.get(EngineTemperatureRequests::FLOW_CONTROL, 1500000000ULL, 1500000000ULL, true));
  CHECK_FALSE(requests.get(EngineTemperatureRequests::CLUTCH_FLOW_CONTROL, 1500000000ULL, 1500000000ULL, true));
  CHECK_FALSE(requests.get(EngineTemperatureRequests::PRESSURE, 1440000000ULL, 1440000000ULL, true));
  CHECK_FALSE(requests.get(EngineTemperatureRequests::PRESSURE, 2000000000ULL, 2100000000ULL, true));
  CHECK_FALSE(requests.get(EngineTemperatureRequests::BAROMETER, 2000000000ULL, 2000000000ULL, false));
}

TEST_CASE("Pressure requests clear any pending flow control") {
  for (const auto primary : {EngineTemperatureRequests::OIL, EngineTemperatureRequests::CLUTCH}) {
    for (const auto pressure : {EngineTemperatureRequests::PRESSURE, EngineTemperatureRequests::BAROMETER}) {
      EngineTemperatureRequests requests;
      REQUIRE(requests.get(primary, 1000000000ULL, 1000000000ULL, true));
      REQUIRE(requests.get(pressure, 1450000000ULL, 1450000000ULL, true));
      CHECK_FALSE(requests.get(EngineTemperatureRequests::FLOW_CONTROL, 1450000001ULL, 1450000001ULL, true));
      CHECK_FALSE(requests.get(EngineTemperatureRequests::CLUTCH_FLOW_CONTROL, 1450000001ULL, 1450000001ULL, true));
    }
  }
}

TEST_CASE("DCT flow control is limited to its own pending request") {
  EngineTemperatureRequests requests;
  auto clutch = requests.get(3, 1000000000ULL, 1000000000ULL, true);
  REQUIRE(clutch);
  CHECK(clutch->address == 0x7E1);
  CHECK(clutch->data == std::array<uint8_t, 8>{3, 0x22, 0x01, 0xA4, 0, 0, 0, 0});
  CHECK_FALSE(requests.get(2, 1100000000ULL, 1100000000ULL, true));
  auto flow = requests.get(4, 1200000000ULL, 1200000000ULL, true);
  REQUIRE(flow);
  CHECK(flow->address == 0x7E1);
  CHECK(flow->data == std::array<uint8_t, 8>{0x30, 0, 0x0A, 0, 0, 0, 0, 0});
  CHECK_FALSE(requests.get(4, 1300000000ULL, 1300000000ULL, true));
  REQUIRE(requests.get(1, 2000000000ULL, 2000000000ULL, true));
  CHECK_FALSE(requests.get(4, 2100000000ULL, 2100000000ULL, true));
  REQUIRE(requests.get(2, 2200000000ULL, 2200000000ULL, true));
  REQUIRE(requests.get(3, 3000000000ULL, 3000000000ULL, true));
  CHECK_FALSE(requests.get(4, 3500000000ULL, 3500000000ULL, true));
  REQUIRE(requests.get(3, 4000000000ULL, 4000000000ULL, true));
  CHECK_FALSE(requests.get(4, 4100000000ULL, 4100000000ULL, false));
  CHECK_FALSE(requests.get(4, 4200000000ULL, 4200000000ULL, true));
}
