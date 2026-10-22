// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/imu/BNO055.hpp"

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

// BNO055 data registers are little-endian (LSB at the lower address)
void SetWord(test::FakeI2CBus::Device& dev, int reg, int16_t value) {
  dev.regs[reg] = static_cast<uint8_t>(value & 0xFF);
  dev.regs[reg + 1] = static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8);
}

// A freshly powered chip: CHIP_ID, clock free, fusion running, flat, heading 0.
test::FakeI2CBus::Device& AddChip(test::FakeI2CBus& bus, int address = 0x28) {
  auto& dev = bus.Add(address);
  dev.regs[0x00] = 0xA0;      // CHIP_ID
  dev.regs[0x38] = 0x00;      // SYS_CLK_STATUS: ST_MAIN_CLK clear
  dev.regs[0x39] = 5;         // SYS_STATUS: fusion running
  SetWord(dev, 0x2E + 4, 981);  // gravity Z = 9.81 m/s^2
  return dev;
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

TEST_CASE("BNO055 configures the fusion mode and the crystal", "[bno055]") {
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  BNO055 imu{I2C::Port::PORT_0, BNO055::kDefaultAddress, BNO055::Mode::kNDOF, 0_ms};
  // PAGE_ID 0, CONFIG mode, normal power, units, external crystal, then NDOF (0x0C)
  CHECK(RegisterWrites(dev) ==
        Writes{{0x07, 0x00}, {0x3D, 0x00}, {0x3E, 0x00}, {0x3B, 0x00}, {0x3F, 0x80}, {0x3D, 0x0C}});
  imu.Update();
  CHECK(imu.IsConnected());

  // CLK_SEL already set (a restart without power cycle): the crystal is not selected again
  test::FakeI2CBus::Device& dev2 = AddChip(bus, 0x29);
  dev2.regs[0x3F] = 0x80;
  BNO055 imu2{I2C::Port::PORT_0, 0x29, BNO055::Mode::kIMU, 0_ms, true};
  CHECK(RegisterWrites(dev2) ==
        Writes{{0x07, 0x00}, {0x3D, 0x00}, {0x3E, 0x00}, {0x3B, 0x00}, {0x3D, 0x08}});

  // no crystal: SYS_TRIGGER is not touched
  bus.Remove(0x29);
  test::FakeI2CBus::Device& dev3 = AddChip(bus, 0x29);
  BNO055 imu3{I2C::Port::PORT_0, 0x29, BNO055::Mode::kIMU, 0_ms, false};
  CHECK(RegisterWrites(dev3) ==
        Writes{{0x07, 0x00}, {0x3D, 0x00}, {0x3E, 0x00}, {0x3B, 0x00}, {0x3D, 0x08}});
  CHECK(dev3.regs[0x3F] == 0x00);
}

TEST_CASE("BNO055 scales the data and negates the heading", "[bno055]") {
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  SetWord(dev, 0x08 + 4, 981);       // accel Z = 9.81 m/s^2 (100 LSB per m/s^2)
  SetWord(dev, 0x14 + 4, 160);       // gyro Z = 10 deg/s (16 LSB per deg/s)
  SetWord(dev, 0x1A, 90 * 16);       // heading 90 degrees (clockwise from north)
  dev.regs[0x34] = 0xFB;             // TEMP -5 deg C
  dev.regs[0x35] = 0b11100100;       // CALIB_STAT: sys 3, gyro 2, accel 1, mag 0

  BNO055 imu{I2C::Port::PORT_0, BNO055::kDefaultAddress, BNO055::Mode::kNDOF, 0_ms};
  imu.Update();
  CHECK(imu.IsConnected());
  CHECK(imu.GetAccelZ().value() == Catch::Approx(9.81));
  CHECK(imu.GetAccelX().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetGyroRateZ().value() == Catch::Approx(10 * kDeg));
  CHECK(imu.GetTemperature().value() == Catch::Approx(-5.0));
  auto cal = imu.GetCalibrationStatus();
  CHECK(cal.system == 3);
  CHECK(cal.gyro == 2);
  CHECK(cal.accel == 1);
  CHECK(cal.mag == 0);

  // 90 degrees clockwise is -90 degrees in the CCW-positive convention
  CHECK(imu.GetYaw().value() == Catch::Approx(-90 * kDeg));
  CHECK(imu.GetAngleX().value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(imu.GetAngleY().value() == Catch::Approx(0.0).margin(1e-9));

  imu.ResetYaw();
  CHECK(imu.GetYaw().value() == Catch::Approx(0.0).margin(1e-9));
  SetWord(dev, 0x1A, 80 * 16);  // the robot turned 10 degrees counter-clockwise
  imu.Update();
  CHECK(imu.GetYaw().value() == Catch::Approx(10 * kDeg));
  CHECK(imu.GetRotation2d().Degrees().value() == Catch::Approx(10.0));
  CHECK(imu.GetAngleZ().value() == Catch::Approx(10 * kDeg));

  // continuous across the chip's 0/360 wrap: 80 -> 10 -> 350 degrees is 90 degrees CCW in all
  SetWord(dev, 0x1A, 10 * 16);
  imu.Update();
  SetWord(dev, 0x1A, 350 * 16);
  imu.Update();
  CHECK(imu.GetYaw().value() == Catch::Approx(100 * kDeg));

  // fusion not running: orientation is kept, the raw data still updates
  dev.regs[0x39] = 0;
  SetWord(dev, 0x1A, 0);
  SetWord(dev, 0x14 + 4, 320);
  imu.Update();
  CHECK(imu.GetYaw().value() == Catch::Approx(100 * kDeg));
  CHECK(imu.GetGyroRateZ().value() == Catch::Approx(20 * kDeg));
  dev.regs[0x39] = 5;

  // gravity on +Y: roll 90 degrees
  SetWord(dev, 0x1A, 350 * 16);
  SetWord(dev, 0x2E + 2, 981);
  SetWord(dev, 0x2E + 4, 0);
  imu.Update();
  CHECK(imu.GetAngleX().value() == Catch::Approx(90 * kDeg));
  CHECK(imu.GetRotation3d().X().value() == Catch::Approx(90 * kDeg));
  CHECK(imu.GetTelemetryType() == "Gyro");
}

TEST_CASE("BNO055 reads and writes the calibration profile in CONFIG mode", "[bno055]") {
  test::FakeI2CBus bus;
  auto& dev = AddChip(bus);
  for (int i = 0; i < 22; ++i) {
    dev.regs[0x55 + i] = static_cast<uint8_t>(i + 1);
  }
  BNO055 imu{I2C::Port::PORT_0, BNO055::kDefaultAddress, BNO055::Mode::kIMU, 0_ms};
  dev.writes.clear();

  auto offsets = imu.GetOffsets();
  REQUIRE(offsets.has_value());
  for (int i = 0; i < 22; ++i) {
    CHECK((*offsets)[i] == i + 1);
  }
  CHECK(RegisterWrites(dev) == Writes{{0x3D, 0x00}, {0x3D, 0x08}});

  dev.writes.clear();
  BNO055::Offsets profile{};
  for (int i = 0; i < 22; ++i) {
    profile[i] = static_cast<uint8_t>(0xA0 + i);
  }
  REQUIRE(imu.SetOffsets(profile));
  auto writes = RegisterWrites(dev);
  REQUIRE(writes.size() == 24);
  CHECK(writes.front() == std::vector<uint8_t>{0x3D, 0x00});
  for (int i = 0; i < 22; ++i) {  // address order: each block's MSB is written last
    CHECK(writes[1 + i] == std::vector<uint8_t>{static_cast<uint8_t>(0x55 + i),
                                                static_cast<uint8_t>(0xA0 + i)});
  }
  CHECK(writes.back() == std::vector<uint8_t>{0x3D, 0x08});
  CHECK(dev.regs[0x3D] == 0x08);
}

TEST_CASE("BNO055 retries a chip that is missing or still booting", "[bno055]") {
  test::FakeI2CBus bus;
  BNO055 imu{I2C::Port::PORT_0, BNO055::kDefaultAddress, BNO055::Mode::kNDOF, 0_ms};
  imu.Update();
  CHECK_FALSE(imu.IsConnected());
  CHECK_FALSE(imu.GetOffsets().has_value());

  sim::PauseTiming();
  auto& dev = AddChip(bus);
  dev.regs[0x00] = 0x00;  // still in power-on reset: wrong CHIP_ID
  sim::StepTiming(200_ms);
  imu.Update();
  CHECK_FALSE(imu.IsConnected());
  CHECK(RegisterWrites(dev).empty());

  dev.regs[0x00] = 0xA0;
  sim::StepTiming(50_ms);  // within the 100 ms retry period: no new attempt
  imu.Update();
  CHECK_FALSE(imu.IsConnected());
  sim::StepTiming(100_ms);
  imu.Update();
  CHECK(imu.IsConnected());
  CHECK(dev.regs[0x3D] == 0x0C);

  // the chip stops answering, then comes back reset: configured again, yaw does not jump
  SetWord(dev, 0x1A, 30 * 16);
  imu.Update();
  CHECK(imu.GetYaw().value() == Catch::Approx(-30 * kDeg));
  dev.nack = true;
  imu.Update();
  CHECK_FALSE(imu.IsConnected());
  dev.nack = false;
  dev.regs[0x3D] = 0x00;
  SetWord(dev, 0x1A, 0);  // the reset chip's heading starts over
  sim::StepTiming(200_ms);
  imu.Update();
  CHECK(imu.IsConnected());
  CHECK(dev.regs[0x3D] == 0x0C);
  CHECK(imu.GetYaw().value() == Catch::Approx(-30 * kDeg));
  SetWord(dev, 0x1A, 350 * 16);  // 10 degrees CCW from the new reference
  imu.Update();
  CHECK(imu.GetYaw().value() == Catch::Approx(-20 * kDeg));
  sim::ResumeTiming();
}
