// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/environment/DPS310.hpp"

#include <cstdint>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {
// Coefficients c0 209, c1 -261, c00 80469, c10 -54769, c01 -2165, c11 1484, c20 -10715, c21 231,
// c30 -1554 packed as in data sheet Table 18.
constexpr uint8_t kCoef[18] = {0x0D, 0x1E, 0xFB, 0x13, 0xA5, 0x5F, 0x2A, 0x0F, 0xF7,
                               0x8B, 0x05, 0xCC, 0xD6, 0x25, 0x00, 0xE7, 0xF9, 0xEE};

test::FakeI2CBus::Device& AddChip(test::FakeI2CBus& bus) {
  auto& dev = bus.Add(0x77);
  dev.regs[0x0D] = 0x10;  // REV_ID 1, PROD_ID 0
  dev.regs[0x08] = 0xC0;  // COEF_RDY, SENSOR_RDY
  dev.regs[0x28] = 0x80;  // coefficients made with the external (MEMS) temperature sensor
  for (int i = 0; i < 18; ++i) {
    dev.regs[0x10 + i] = kCoef[i];
  }
  // Praw = -2900000, Traw = 2395000 (24-bit two's complement, MSB first)
  const uint8_t results[] = {0xD3, 0xBF, 0xE0, 0x24, 0x8B, 0x78};
  for (int i = 0; i < 6; ++i) {
    dev.regs[i] = results[i];
  }
  return dev;
}

// Traw / kT and Praw / kP with kT = kP = 7864320 (8 times oversampling), then data sheet
// 4.9.1 and 4.9.2
constexpr double kTemperature = 25.015060424804688;
constexpr double kPressure = 98469.77598745225;
}  // namespace

TEST_CASE("DPS310 configures continuous mode and compensates", "[dps310]") {
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  DPS310 baro{I2C::Port::PORT_0};
  baro.Update();  // product ID
  CHECK(baro.IsConnected());
  baro.Update();  // ready: coefficients, configuration
  CHECK(dev.regs[0x06] == 0x33);  // 8 per second, 8 times
  CHECK(dev.regs[0x07] == 0xB3);  // TMP_EXT as in COEF_SRCE, 8 per second, 8 times
  CHECK(dev.regs[0x09] == 0x00);  // no result shift
  CHECK(dev.regs[0x08] == 0x07);  // continuous pressure and temperature
  CHECK(dev.writes.back() == std::vector<uint8_t>{0x08, 0x07});
  CHECK_FALSE(baro.HasData());
  CHECK(baro.GetAltitude().value() == Catch::Approx(0.0).margin(1e-12));  // no pressure yet

  dev.regs[0x08] = 0xF7;  // TMP_RDY and PRS_RDY
  baro.Update();
  CHECK(baro.HasData());
  CHECK(baro.GetTemperature().value() == Catch::Approx(kTemperature));
  CHECK(baro.GetPressure().value() == Catch::Approx(kPressure));
  // 44330 m * (1 - (98469.776 / 101325)^(1 / 5.255))
  CHECK(baro.GetAltitude().value() == Catch::Approx(240.4695).margin(1e-3));

  dev.regs[0x08] = 0xC7;  // nothing new: only the status read
  int reads = dev.reads;
  baro.Update();
  CHECK(dev.reads == reads + 1);
  CHECK(baro.GetPressure().value() == Catch::Approx(kPressure));
}

TEST_CASE("DPS310 waits for a temperature before compensating pressure", "[dps310]") {
  sim::PauseTiming();  // the ready wait gives up after 0.1 s
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  dev.regs[0x28] = 0x00;  // internal (ASIC) temperature sensor
  DPS310 baro{I2C::Port::PORT_0};
  baro.Update();
  dev.regs[0x08] = 0x40;  // SENSOR_RDY only: the coefficients are not ready yet
  baro.Update();
  CHECK(dev.regs[0x06] == 0x00);
  dev.regs[0x08] = 0xC0;
  baro.Update();
  CHECK(dev.regs[0x07] == 0x33);

  dev.regs[0x08] = 0xD7;  // PRS_RDY only
  baro.Update();
  CHECK_FALSE(baro.HasData());
  dev.regs[0x08] = 0xE7;  // TMP_RDY only
  baro.Update();
  CHECK_FALSE(baro.HasData());
  CHECK(baro.GetTemperature().value() == Catch::Approx(kTemperature));
  dev.regs[0x08] = 0xD7;
  baro.Update();
  CHECK(baro.HasData());
  CHECK(baro.GetPressure().value() == Catch::Approx(kPressure));
  sim::ResumeTiming();
}

TEST_CASE("DPS310 rejects another chip and a missing one", "[dps310]") {
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  dev.regs[0x0D] = 0x58;  // PROD_ID not 0
  DPS310 baro{I2C::Port::PORT_0};
  baro.Update();
  CHECK_FALSE(baro.IsConnected());
  for (const auto& write : dev.writes) {
    CHECK(write.size() == 1);  // register pointers only, nothing configured
  }

  bus.Remove(0x77);
  DPS310 missing{I2C::Port::PORT_0};
  missing.Update();
  CHECK_FALSE(missing.IsConnected());
  CHECK_FALSE(missing.HasData());
}
