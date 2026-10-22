// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/environment/BME688.hpp"

#include <cstdint>
#include <utility>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {
using Bytes = std::vector<uint8_t>;

// A made-up calibration set (not from a real chip), packed at the addresses of data sheet
// Tables 13 to 16: t1 25829, t2 26431, t3 3, p1 36379, p2 -10417, p3 88, p4 6936, p5 -48, p6 30,
// p7 33, p8 -1350, p9 -2768, p10 30, h1 751, h2 1022, h3 0, h4 45, h5 20, h6 120, h7 -100,
// g1 -53, g2 -11536, g3 18, res_heat_range 1, res_heat_val 40.
const std::pair<int, uint8_t> kCalibration[] = {
    {0x00, 0x28}, {0x02, 0x10}, {0x8A, 0x3F}, {0x8B, 0x67}, {0x8C, 0x03}, {0x8E, 0x1B},
    {0x8F, 0x8E}, {0x90, 0x4F}, {0x91, 0xD7}, {0x92, 0x58}, {0x94, 0x18}, {0x95, 0x1B},
    {0x96, 0xD0}, {0x97, 0xFF}, {0x98, 0x21}, {0x99, 0x1E}, {0x9C, 0xBA}, {0x9D, 0xFA},
    {0x9E, 0x30}, {0x9F, 0xF5}, {0xA0, 0x1E}, {0xE1, 0x3F}, {0xE2, 0xEF}, {0xE3, 0x2E},
    {0xE4, 0x00}, {0xE5, 0x2D}, {0xE6, 0x14}, {0xE7, 0x78}, {0xE8, 0x9C}, {0xE9, 0xE5},
    {0xEA, 0x64}, {0xEB, 0xF0}, {0xEC, 0xD2}, {0xED, 0xCB}, {0xEE, 0x12}};

constexpr BME688::Calibration kCal = {
    .t1 = 25829, .t2 = 26431, .t3 = 3,     .p1 = 36379, .p2 = -10417, .p3 = 88,
    .p4 = 6936,  .p5 = -48,   .p6 = 30,    .p7 = 33,    .p8 = -1350,  .p9 = -2768,
    .p10 = 30,   .h1 = 751,   .h2 = 1022,  .h3 = 0,     .h4 = 45,     .h5 = 20,
    .h6 = 120,   .h7 = -100,  .g1 = -53,   .g2 = -11536, .g3 = 18,    .resHeatRange = 1,
    .resHeatVal = 40};

// Field 0 (0x1D..0x2D): new data; press_adc 400000, temp_adc 500000, hum_adc 25000, gas_adc 600,
// gas_valid, heat_stab, gas_range 5.
constexpr uint8_t kField0[17] = {0x80, 0x00, 0x61, 0xA8, 0x00, 0x7A, 0x12, 0x00, 0x61,
                                 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x35};

// Expected values from the data sheet 3.6 / 3.7 floating point formulas
constexpr double kTemperature = 27.333064052229748;
constexpr double kPressure = 92713.85188615424;
constexpr double kHumidity = 73.44791141354764;
constexpr double kGasResistance = 1878899.0825688073;

test::FakeI2CBus::Device& AddChip(test::FakeI2CBus& bus) {
  auto& dev = bus.Add(0x77);
  dev.regs[0xD0] = 0x61;  // chip_id
  dev.regs[0xF0] = 0x01;  // variant_id: BME688
  for (const auto& [reg, value] : kCalibration) {
    dev.regs[reg] = value;
  }
  for (int i = 0; i < 17; ++i) {
    dev.regs[0x1D + i] = kField0[i];
  }
  return dev;
}
}  // namespace

TEST_CASE("BME688 compensation formulas", "[bme688]") {
  double tFine = 0.0;
  double temperature = BME688::CompensateTemperature(kCal, 500000, tFine);
  CHECK(temperature == Catch::Approx(kTemperature));
  // the data sheet integer formulas give 27.33 C, 92715 Pa and 73.433 %
  CHECK(BME688::CompensatePressure(kCal, 400000, tFine) == Catch::Approx(kPressure));
  CHECK(BME688::CompensatePressure(kCal, 400000, tFine) == Catch::Approx(92715).margin(2.0));
  CHECK(BME688::CompensateHumidity(kCal, 25000, temperature) == Catch::Approx(kHumidity));
  CHECK(BME688::CompensateHumidity(kCal, 0xFFFF, temperature) ==
        Catch::Approx(538.3231988557321));  // not limited here
  CHECK(BME688::GasResistance(600, 5) == Catch::Approx(kGasResistance));
  CHECK(BME688::GasResistance(512, 0) == Catch::Approx(64000000.0));
  CHECK(BME688::HeaterCode(kCal, 320.0, 25.0) == 111);
  CHECK(BME688::HeaterCode(kCal, 300.0, kTemperature) == 106);
}

TEST_CASE("BME688 configures, triggers and reads forced measurements", "[bme688]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  BME688 sensor{I2C::Port::PORT_0};
  sensor.Update();
  CHECK(sensor.IsConnected());
  CHECK_FALSE(sensor.HasData());
  CHECK(dev.regs[0x72] == 0x01);  // humidity 1x
  CHECK(dev.regs[0x75] == 0x00);  // IIR filter off
  CHECK(dev.regs[0x70] == 0x00);  // heater on
  CHECK(dev.regs[0x71] == 0x20);  // run_gas, heater step 0
  CHECK(dev.regs[0x64] == 0x66);  // 38 * 4 = 152 ms
  CHECK(dev.regs[0x5A] == 111);   // 320 C at 25 C ambient
  CHECK(dev.writes.back() == Bytes{0x74, 0x55});  // T 2x, P 16x, forced mode

  sensor.Update();  // new data: read, then the next trigger
  CHECK(sensor.HasData());
  CHECK(sensor.GetTemperature().value() == Catch::Approx(kTemperature));
  CHECK(sensor.GetPressure().value() == Catch::Approx(kPressure));
  CHECK(sensor.GetHumidity() == Catch::Approx(kHumidity));
  CHECK(sensor.GetGasResistance().value() == Catch::Approx(kGasResistance));
  CHECK(sensor.IsGasValid());
  REQUIRE(dev.writes.size() >= 2);
  CHECK(dev.writes[dev.writes.size() - 2] == Bytes{0x1D});  // heater code unchanged: no write
  CHECK(dev.writes.back() == Bytes{0x74, 0x55});

  sensor.SetHeater(300_degC, 100_ms);
  sensor.Update();
  CHECK(dev.regs[0x64] == 0x59);  // 100 ms (the data sheet 3.5 example value)
  CHECK(dev.regs[0x5A] == 106);   // 300 C at the measured ambient
  CHECK(dev.writes.back() == Bytes{0x74, 0x55});

  sensor.SetHeater(500_degC, 5000_ms);  // limited to 400 C and 4032 ms
  sensor.Update();
  CHECK(dev.regs[0x64] == 0xFF);

  // no new data: no trigger until the heating time (4032 ms) plus 1 s has passed
  dev.regs[0x1D] = 0x00;
  sensor.Update();
  CHECK(dev.writes.back() == Bytes{0x1D});
  sim::StepTiming(4500_ms);
  sensor.Update();
  CHECK(dev.writes.back() == Bytes{0x1D});
  sim::StepTiming(600_ms);
  sensor.Update();
  CHECK(dev.writes.back() == Bytes{0x74, 0x55});

  // a gas measurement without gas_valid / heat_stab; humidity is limited to 100 %
  dev.regs[0x1D] = 0x80;
  dev.regs[0x2D] = 0x05;
  dev.regs[0x25] = 0xFF;
  dev.regs[0x26] = 0xFF;
  sensor.Update();
  CHECK_FALSE(sensor.IsGasValid());
  CHECK(sensor.GetHumidity() == Catch::Approx(100.0));
  sim::ResumeTiming();
}

TEST_CASE("BME688 rejects another chip and retries a missing one", "[bme688]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  dev.regs[0xF0] = 0x00;  // BME680 variant
  BME688 sensor{I2C::Port::PORT_0};
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());
  for (const auto& write : dev.writes) {
    CHECK(write.size() == 1);  // register pointers only, nothing configured
  }

  dev.regs[0xF0] = 0x01;
  sim::StepTiming(500_ms);
  sensor.Update();  // within the 1 s retry period
  CHECK_FALSE(sensor.IsConnected());
  sim::StepTiming(600_ms);
  sensor.Update();
  CHECK(sensor.IsConnected());

  dev.nack = true;
  sensor.Update();
  CHECK_FALSE(sensor.IsConnected());
  CHECK_FALSE(sensor.HasData());
  sim::ResumeTiming();
}
