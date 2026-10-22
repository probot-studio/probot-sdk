// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/imu/LSM303AGR.hpp"

#include <cstdint>
#include <numbers>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"

using namespace prosdk;

namespace {
constexpr double kDeg = std::numbers::pi / 180.0;

void SetWord(test::FakeI2CBus::Device& dev, int reg, int16_t value) {
  dev.regs[reg] = static_cast<uint8_t>(value & 0xFF);
  dev.regs[reg + 1] = static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8);
}

// The accelerometer is read at sub-address 0x28 | 0x80 (auto-increment); the fake has no
// auto-increment bit, so its register file is indexed by 0xA8. Values are 12-bit counts
// (high-resolution mode, left-justified on the bus).
void SetAccel(test::FakeI2CBus::Device& dev, int x, int y, int z) {
  SetWord(dev, 0xA8, static_cast<int16_t>(x * 16));
  SetWord(dev, 0xAA, static_cast<int16_t>(y * 16));
  SetWord(dev, 0xAC, static_cast<int16_t>(z * 16));
}

// Magnetometer counts, 1.5 mgauss = 0.15 uT per count.
void SetMag(test::FakeI2CBus::Device& dev, int x, int y, int z) {
  SetWord(dev, 0x68, static_cast<int16_t>(x));
  SetWord(dev, 0x6A, static_cast<int16_t>(y));
  SetWord(dev, 0x6C, static_cast<int16_t>(z));
}

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

TEST_CASE("LSM303AGR switches both sensors on", "[lsm303agr]") {
  test::FakeI2CBus bus;
  auto& accel = bus.Add(0x19);
  auto& mag = bus.Add(0x1E);
  accel.regs[0x0F] = 0x33;
  mag.regs[0x4F] = 0x40;
  LSM303AGR imu{I2C::Port::PORT_0, 0_ms};
  // CFG_REG_A_M: COMP_TEMP_EN, HR, 100 Hz, continuous; CFG_REG_B_M: OFF_CANC + LPF; C: BDU
  CHECK(RegisterWrites(mag) == Writes{{0x60, 0x8C}, {0x61, 0x03}, {0x62, 0x10}});
  // CTRL_REG1_A: 100 Hz, XYZ; CTRL_REG4_A: BDU, +/-4 g, HR
  CHECK(RegisterWrites(accel) == Writes{{0x20, 0x57}, {0x23, 0x98}});
  imu.Update();
  CHECK(imu.IsConnected());
  mag.nack = true;
  imu.Update();
  CHECK_FALSE(imu.IsConnected());
  mag.nack = false;

  mag.regs[0x4F] = 0x3D;  // another chip
  LSM303AGR wrong{I2C::Port::PORT_0, 0_ms};
  wrong.Update();
  CHECK_FALSE(wrong.IsConnected());
  CHECK(wrong.GetTelemetryType() == "Gyro");
}

TEST_CASE("LSM303AGR scales and gives the tilt-compensated heading", "[lsm303agr]") {
  test::FakeI2CBus bus;
  auto& accel = bus.Add(0x19);
  auto& mag = bus.Add(0x1E);
  accel.regs[0x0F] = 0x33;
  mag.regs[0x4F] = 0x40;
  LSM303AGR imu{I2C::Port::PORT_0, 0_ms};

  // Field: 30 uT horizontal (200 counts) toward north, 45 uT (300 counts) down. Flat, X north.
  SetAccel(accel, 0, 0, 513);  // 513 * 1.95 mg = 1.0 g
  SetMag(mag, 200, 0, -300);
  imu.Update();
  CHECK(imu.GetAccelZ().value() == Catch::Approx(9.80665).epsilon(1e-3));
  CHECK(imu.GetAccelX().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetMagX().value() == Catch::Approx(30e-6));
  CHECK(imu.GetMagZ().value() == Catch::Approx(-45e-6));
  CHECK(imu.GetHeading().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetAngleX().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetAngleY().value() == Catch::Approx(0.0).margin(1e-9));

  // X west (north on -Y): the robot turned 90 degrees counter-clockwise
  SetMag(mag, 0, -200, -300);
  imu.Update();
  CHECK(imu.GetHeading().value() == Catch::Approx(90 * kDeg));
  CHECK(imu.GetYaw().value() == Catch::Approx(90 * kDeg));
  // back to north, then X east: -90 degrees
  SetMag(mag, 200, 0, -300);
  imu.Update();
  CHECK(imu.GetYaw().value() == Catch::Approx(0.0).margin(1e-9));
  SetMag(mag, 0, 200, -300);
  imu.Update();
  CHECK(imu.GetHeading().value() == Catch::Approx(-90 * kDeg));
  CHECK(imu.GetYaw().value() == Catch::Approx(-90 * kDeg));
  // 30 degrees counter-clockwise from north
  SetMag(mag, 173, -100, -300);
  imu.Update();
  CHECK(imu.GetHeading().value() == Catch::Approx(30 * kDeg).margin(0.1 * kDeg));

  // Rolled 30 degrees about X, still facing north: the heading does not move
  SetAccel(accel, 0, 256, 444);
  SetMag(mag, 200, -150, -260);
  imu.Update();
  CHECK(imu.GetAngleX().value() == Catch::Approx(30 * kDeg).margin(0.1 * kDeg));
  CHECK(imu.GetHeading().value() == Catch::Approx(0.0).margin(0.1 * kDeg));
  // Rolled 30 degrees and facing west
  SetMag(mag, 0, -323, -160);
  imu.Update();
  CHECK(imu.GetHeading().value() == Catch::Approx(90 * kDeg).margin(0.1 * kDeg));
}

TEST_CASE("LSM303AGR yaw is continuous and relative to ResetYaw", "[lsm303agr]") {
  test::FakeI2CBus bus;
  auto& accel = bus.Add(0x19);
  auto& mag = bus.Add(0x1E);
  accel.regs[0x0F] = 0x33;
  mag.regs[0x4F] = 0x40;
  LSM303AGR imu{I2C::Port::PORT_0, 0_ms};
  SetAccel(accel, 0, 0, 513);

  SetMag(mag, -197, -35, -300);  // 170 degrees
  imu.Update();
  CHECK(imu.GetYaw().value() == Catch::Approx(169.926 * kDeg).margin(0.01 * kDeg));
  imu.ResetYaw();
  SetMag(mag, -197, 35, -300);  // -170 degrees: 20 degrees further counter-clockwise
  imu.Update();
  CHECK(imu.GetHeading().value() == Catch::Approx(-169.926 * kDeg).margin(0.01 * kDeg));
  CHECK(imu.GetYaw().value() == Catch::Approx(20.148 * kDeg).margin(0.01 * kDeg));
  CHECK(imu.GetRotation2d().Degrees().value() == Catch::Approx(20.148).margin(0.01));

  // hard-iron offset: a constant 15 uT (100 counts) on X is removed
  imu.SetHardIronOffset(units::tesla_t{15e-6}, units::tesla_t{0.0}, units::tesla_t{0.0});
  SetMag(mag, 300, 0, -300);
  imu.Update();
  CHECK(imu.GetMagX().value() == Catch::Approx(30e-6));
  CHECK(imu.GetHeading().value() == Catch::Approx(0.0).margin(1e-9));
}
