// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/environment/LM75A.hpp"

#include <cstdint>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"

using namespace prosdk;

namespace {
void SetTemp(test::FakeI2CBus::Device& dev, uint8_t msb, uint8_t lsb) {
  dev.regs[0x00] = msb;
  dev.regs[0x01] = lsb;
}
}  // namespace

TEST_CASE("LM75A reads the temperature register", "[lm75a]") {
  test::FakeI2CBus bus;
  auto& dev = bus.Add(0x48);
  dev.regs[0x01] = 0x01;  // left in shutdown by an earlier program
  LM75A sensor{I2C::Port::PORT_0};
  CHECK(sensor.IsConnected());
  REQUIRE(dev.writes.size() == 1);
  CHECK(dev.writes[0] == std::vector<uint8_t>{0x01, 0x00});  // Conf: normal mode
  CHECK(dev.regs[0x01] == 0x00);

  // data sheet Table 10 values (11-bit two's complement in the top bits, 0.125 C per LSB)
  SetTemp(dev, 0x7F, 0x00);  // 0x3F8
  CHECK(sensor.GetTemperature().value() == Catch::Approx(127.0));
  SetTemp(dev, 0x19, 0x00);  // 0x0C8
  CHECK(sensor.GetTemperature().value() == Catch::Approx(25.0));
  SetTemp(dev, 0x00, 0x20);  // 0x001
  CHECK(sensor.GetTemperature().value() == Catch::Approx(0.125));
  SetTemp(dev, 0xFF, 0xE0);  // 0x7FF
  CHECK(sensor.GetTemperature().value() == Catch::Approx(-0.125));
  SetTemp(dev, 0xE7, 0x00);  // 0x738
  CHECK(sensor.GetTemperature().value() == Catch::Approx(-25.0));
  SetTemp(dev, 0xC9, 0x20);  // 0x649
  CHECK(sensor.GetTemperature().value() == Catch::Approx(-54.875));
  SetTemp(dev, 0x19, 0x1F);  // the 5 unused low bits are ignored
  CHECK(sensor.GetTemperature().value() == Catch::Approx(25.0));

  sensor.SetShutdown(true);
  CHECK(dev.writes.back() == std::vector<uint8_t>{0x01, 0x01});
  sensor.SetShutdown(false);
  CHECK(dev.writes.back() == std::vector<uint8_t>{0x01, 0x00});
}

TEST_CASE("LM75A keeps the last value when the chip stops answering", "[lm75a]") {
  test::FakeI2CBus bus;
  auto& dev = bus.Add(0x49);
  LM75A sensor{I2C::Port::PORT_0, 0x49};
  SetTemp(dev, 0x15, 0x80);  // 21.5 C
  CHECK(sensor.GetTemperature().value() == Catch::Approx(21.5));
  dev.nack = true;
  CHECK(sensor.GetTemperature().value() == Catch::Approx(21.5));
  CHECK_FALSE(sensor.IsConnected());
  dev.nack = false;
  SetTemp(dev, 0x16, 0x00);
  CHECK(sensor.GetTemperature().value() == Catch::Approx(22.0));
  CHECK(sensor.IsConnected());
}

TEST_CASE("LM75A without a chip is not connected", "[lm75a]") {
  test::FakeI2CBus bus;
  LM75A sensor{I2C::Port::PORT_0};
  CHECK_FALSE(sensor.IsConnected());
  CHECK(sensor.GetTemperature().value() == Catch::Approx(0.0).margin(1e-12));
  CHECK_FALSE(sensor.IsConnected());
}
