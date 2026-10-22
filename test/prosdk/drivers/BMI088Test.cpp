// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/imu/BMI088.hpp"

#include <cstdint>
#include <numbers>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {
constexpr double kDeg = std::numbers::pi / 180.0;

// BMI088 data registers are little-endian (LSB at the lower address)
void SetWord(test::FakeI2CBus::Device& dev, int reg, int16_t value) {
  dev.regs[reg] = static_cast<uint8_t>(value & 0xFF);
  dev.regs[reg + 1] = static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8);
}

// Register writes (pointer + one data byte) in order.
std::vector<std::vector<uint8_t>> RegisterWrites(const test::FakeI2CBus::Device& dev) {
  std::vector<std::vector<uint8_t>> out;
  for (const auto& w : dev.writes) {
    if (w.size() == 2) {
      out.push_back(w);
    }
  }
  return out;
}

using Writes = std::vector<std::vector<uint8_t>>;
}  // namespace

TEST_CASE("BMI088 powers up the accelerometer and configures both parts", "[bmi088]") {
  test::FakeI2CBus bus;
  auto& accel = bus.Add(0x19);
  auto& gyro = bus.Add(0x69);
  accel.regs[0x00] = 0x1E;
  gyro.regs[0x00] = 0x0F;

  BMI088 imu{I2C::Port::PORT_0, BMI088::kDefaultAccelAddress, BMI088::kDefaultGyroAddress, 0_ms};
  // ACC_PWR_CTRL = 4, then ACC_PWR_CONF = 0 (active), ACC_CONF 100 Hz normal, ACC_RANGE +/-6 g
  CHECK(RegisterWrites(accel) == Writes{{0x7D, 0x04}, {0x7C, 0x00}, {0x40, 0xA8}, {0x41, 0x01}});
  // GYRO_RANGE +/-1000 deg/s, GYRO_BANDWIDTH 400 Hz / 47 Hz
  CHECK(RegisterWrites(gyro) == Writes{{0x0F, 0x01}, {0x10, 0x03}});
  imu.Update();
  CHECK(imu.IsConnected());

  // the gyro stops answering
  gyro.nack = true;
  imu.Update();
  CHECK_FALSE(imu.IsConnected());
  gyro.nack = false;
  imu.Update();
  CHECK(imu.IsConnected());

  // wrong chip ID: never connected
  accel.regs[0x00] = 0x00;
  BMI088 wrong{I2C::Port::PORT_0, BMI088::kDefaultAccelAddress, BMI088::kDefaultGyroAddress, 0_ms};
  wrong.Update();
  CHECK_FALSE(wrong.IsConnected());
}

TEST_CASE("BMI088 uses the JP2 addresses", "[bmi088]") {
  test::FakeI2CBus bus;
  auto& accel = bus.Add(0x18);
  auto& gyro = bus.Add(0x68);
  accel.regs[0x00] = 0x1E;
  gyro.regs[0x00] = 0x0F;
  BMI088 imu{I2C::Port::PORT_0, 0x18, 0x68, 0_ms};
  imu.Update();
  CHECK(imu.IsConnected());
  CHECK(accel.regs[0x7D] == 0x04);
  CHECK(gyro.regs[0x0F] == 0x01);

  // default addresses with no chip there: no answer
  BMI088 none{I2C::Port::PORT_0, BMI088::kDefaultAccelAddress, BMI088::kDefaultGyroAddress, 0_ms};
  none.Update();
  CHECK_FALSE(none.IsConnected());
}

TEST_CASE("BMI088 scales, calibrates and integrates yaw on simulated time", "[bmi088]") {
  test::FakeI2CBus bus;
  auto& accel = bus.Add(0x19);
  auto& gyro = bus.Add(0x69);
  accel.regs[0x00] = 0x1E;
  gyro.regs[0x00] = 0x0F;
  SetWord(accel, 0x12 + 4, 5461);  // accel Z = 1 g (+/-6 g range: 32768 / 6 LSB per g)
  SetWord(gyro, 0x02 + 4, 1024);   // gyro Z = 31.25 deg/s (32.768 LSB per deg/s)
  accel.regs[0x22] = 0x00;         // TEMP_MSB
  accel.regs[0x23] = 0x60;         // TEMP_LSB: 23.375 deg C (datasheet 5.3.7 example)

  BMI088 imu{I2C::Port::PORT_0, BMI088::kDefaultAccelAddress, BMI088::kDefaultGyroAddress, 0_ms};
  REQUIRE(imu.Calibrate(5));  // bias: 31.25 deg/s on Z
  SetWord(gyro, 0x02 + 4, 4096);  // 125 deg/s raw = 93.75 deg/s after the bias

  sim::PauseTiming();
  imu.Update();
  CHECK(imu.IsConnected());
  CHECK(imu.GetYaw().value() == Catch::Approx(0.0).margin(1e-9));
  sim::StepTiming(100_ms);
  imu.Update();
  CHECK(imu.GetGyroRateZ().value() == Catch::Approx(93.75 * kDeg));
  CHECK(imu.GetYaw().value() == Catch::Approx(9.375 * kDeg));
  CHECK(imu.GetRotation2d().Degrees().value() == Catch::Approx(9.375));
  CHECK(imu.GetAngleZ().value() == Catch::Approx(9.375 * kDeg));
  CHECK(imu.GetAccelZ().value() == Catch::Approx(9.80665).epsilon(1e-3));
  CHECK(imu.GetAccelX().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetAngleX().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetTemperature().value() == Catch::Approx(23.375));

  imu.ResetYaw();
  CHECK(imu.GetYaw().value() == Catch::Approx(0.0).margin(1e-9));
  sim::StepTiming(50_ms);
  imu.Update();
  CHECK(imu.GetYaw().value() == Catch::Approx(4.6875 * kDeg));

  // negative temperature: TEMP_MSB 0xC1, TEMP_LSB 0x00 is -40 deg C (5.3.7)
  accel.regs[0x22] = 0xC1;
  accel.regs[0x23] = 0x00;
  // tilted: gravity on +Y gives a roll of 90 degrees, tracked by the complementary filter
  SetWord(gyro, 0x02 + 4, 1024);  // no rotation
  SetWord(accel, 0x12 + 4, 0);
  SetWord(accel, 0x12 + 2, 5461);
  for (int i = 0; i < 400; ++i) {
    sim::StepTiming(5_ms);
    imu.Update();
  }
  CHECK(imu.GetAngleX().value() == Catch::Approx(90 * kDeg).margin(0.01));
  CHECK(imu.GetTemperature().value() == Catch::Approx(-40.0));
  sim::ResumeTiming();
  CHECK(imu.GetTelemetryType() == "Gyro");
}
