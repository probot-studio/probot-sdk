// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/light/OPT3001.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"

using namespace prosdk;

namespace {
test::FakeI2CBus::Device& AddOpt3001(test::FakeI2CBus& bus, uint16_t deviceId = 0x3001) {
  auto& dev = bus.Add(OPT3001::kDefaultAddress, 8, 16);  // 16-bit registers, MSB first
  dev.regs[0x7E] = 0x5449;
  dev.regs[0x7F] = deviceId;
  dev.regs[0x01] = 0xC810;  // reset value: shutdown
  return dev;
}
}  // namespace

TEST_CASE("OPT3001 decodes the result register (datasheet Table 9)", "[opt3001]") {
  using Catch::Approx;
  CHECK(OPT3001::ResultToLux(0x0001) == Approx(0.01));
  CHECK(OPT3001::ResultToLux(0x0FFF) == Approx(40.95));
  CHECK(OPT3001::ResultToLux(0x3456) == Approx(88.80));
  CHECK(OPT3001::ResultToLux(0x789A) == Approx(2818.56));
  CHECK(OPT3001::ResultToLux(0x8800) == Approx(5242.88));
  CHECK(OPT3001::ResultToLux(0xBFFF) == Approx(83865.60));
}

TEST_CASE("OPT3001 checks the IDs, starts conversions and reads lux", "[opt3001]") {
  test::FakeI2CBus bus;
  auto& dev = AddOpt3001(bus);

  OPT3001 sensor{I2C::Port::PORT_0};
  CHECK(sensor.IsConnected());
  CHECK(dev.regs[0x01] == 0xCC10);  // automatic range, 800 ms, continuous
  CHECK(sensor.GetLux().value() == 0.0);

  dev.regs[0x00] = 0x3456;
  sensor.Update();
  CHECK(sensor.IsConnected());
  CHECK(sensor.GetLux().value() == Catch::Approx(88.80));

  dev.regs[0x01] = 0xC810;  // power loss: the chip is back in shutdown
  dev.regs[0x00] = 0x789A;
  sensor.Update();
  CHECK(dev.regs[0x01] == 0xCC10);
  CHECK(sensor.GetLux().value() == Catch::Approx(2818.56));

  dev.nack = true;
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());
  CHECK(sensor.GetLux().value() == Catch::Approx(2818.56));  // last good value kept
}

TEST_CASE("OPT3001 rejects another chip and retries in Update", "[opt3001]") {
  test::FakeI2CBus bus;
  auto& dev = AddOpt3001(bus, 0x3002);

  OPT3001 sensor{I2C::Port::PORT_0};
  CHECK_FALSE(sensor.IsConnected());
  CHECK(dev.regs[0x01] == 0xC810);  // not configured

  dev.regs[0x7F] = 0x3001;
  dev.regs[0x00] = 0x0FFF;
  sensor.Update();
  CHECK(sensor.IsConnected());
  CHECK(dev.regs[0x01] == 0xCC10);
  CHECK(sensor.GetLux().value() == Catch::Approx(40.95));
}

TEST_CASE("OPT3001 reports a missing chip", "[opt3001]") {
  test::FakeI2CBus bus;
  OPT3001 sensor{I2C::Port::PORT_0, 0x45};
  CHECK_FALSE(sensor.IsConnected());
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());
}
