#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "selfdrive/boardd/engine_temperature.h"

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
  CHECK_FALSE(requests.get(5, 1, 1, true));
  CHECK_FALSE(requests.get(65535, 1, 1, true));
  REQUIRE(requests.get(1, 1000000000ULL, 1000000000ULL, true));
  CHECK_FALSE(requests.get(0, 1500000000ULL, 1500000000ULL, true));
  CHECK_FALSE(requests.get(2, 1500000000ULL, 1500000000ULL, true));
  REQUIRE(requests.get(1, 2000000000ULL, 2000000000ULL, true));
  CHECK_FALSE(requests.get(2, 2100000000ULL, 2100000000ULL, false));
  CHECK_FALSE(requests.get(2, 2200000000ULL, 2200000000ULL, true));
  REQUIRE(requests.get(0, 3000000000ULL, 3000000000ULL, true));
  CHECK_FALSE(requests.get(2, 3100000000ULL, 3100000000ULL, true));
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
