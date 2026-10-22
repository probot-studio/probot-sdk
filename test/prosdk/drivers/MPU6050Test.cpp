// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/imu/MPU6050.hpp"

#include <numbers>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CDevice.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {
void SetWord(test::FakeI2CDevice& dev, int reg, int16_t value) {
  dev.regs[reg] = static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8);
  dev.regs[reg + 1] = static_cast<uint8_t>(value & 0xFF);
}
constexpr double kDeg = std::numbers::pi / 180.0;
}  // namespace

TEST_CASE("MPU6050 configures, calibrates and integrates yaw", "[mpu6050]") {
  test::FakeI2CDevice dev;
  dev.regs[0x75] = 0x68;
  SetWord(dev, 0x3F, 8192);  // accel Z = 1 g (+/-4 g range)
  SetWord(dev, 0x47, 328);   // gyro Z = 10 deg/s (+/-1000 deg/s range: 32.8 LSB per deg/s)

  MPU6050 imu{I2C::Port::PORT_0, MPU6050::kDefaultAddress, 0_ms};
  CHECK(dev.regs[0x6B] == 0x01);
  CHECK(dev.regs[0x1A] == 0x03);
  CHECK(dev.regs[0x1B] == 0x10);
  CHECK(dev.regs[0x1C] == 0x08);

  REQUIRE(imu.Calibrate(5));  // bias: 10 deg/s on Z
  SetWord(dev, 0x47, 3280);   // 100 deg/s raw = 90 deg/s after the bias

  sim::PauseTiming();
  imu.Update();
  CHECK(imu.IsConnected());
  CHECK(imu.GetYaw().value() == Catch::Approx(0.0).margin(1e-9));
  sim::StepTiming(100_ms);
  imu.Update();
  CHECK(imu.GetGyroRateZ().value() == Catch::Approx(90 * kDeg));
  CHECK(imu.GetYaw().value() == Catch::Approx(9 * kDeg));
  CHECK(imu.GetRotation2d().Degrees().value() == Catch::Approx(9.0));
  CHECK(imu.GetAngleZ().value() == Catch::Approx(9 * kDeg));
  CHECK(imu.GetAngleX().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetAngleY().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetAccelZ().value() == Catch::Approx(9.80665));
  CHECK(imu.GetAccelX().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetTemperature().value() == Catch::Approx(36.53));
  CHECK(imu.GetRotation3d().Z().value() == Catch::Approx(9 * kDeg));

  imu.ResetYaw();
  CHECK(imu.GetYaw().value() == Catch::Approx(0.0).margin(1e-9));
  sim::StepTiming(50_ms);
  imu.Update();
  CHECK(imu.GetYaw().value() == Catch::Approx(4.5 * kDeg));

  // tilted: gravity on +Y gives a roll of 90 degrees, tracked by the complementary filter
  SetWord(dev, 0x47, 328);  // no rotation
  SetWord(dev, 0x3F, 0);
  SetWord(dev, 0x3D, 8192);
  for (int i = 0; i < 400; ++i) {
    sim::StepTiming(5_ms);
    imu.Update();
  }
  CHECK(imu.GetAngleX().value() == Catch::Approx(90 * kDeg).margin(0.01));
  sim::ResumeTiming();
  CHECK(imu.GetTelemetryType() == "Gyro");
}
