// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/pwm/PCA9685.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CDevice.hpp"
#include "prosdk/drivers/ProHub.hpp"
#include "prosdk/drivers/motor/ProHubMotor.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {
void SetRobotEnabled(bool enabled) {
  sim::DriverStationSim::SetDsAttached(true);
  sim::DriverStationSim::SetEnabled(enabled);
  sim::DriverStationSim::NotifyNewData();
}

// LEDn_ON_L .. LEDn_OFF_H of a channel in the fake register file
std::array<uint8_t, 4> Led(const test::FakeI2CDevice& dev, int channel) {
  int r = 0x06 + 4 * channel;
  return {dev.regs[r], dev.regs[r + 1], dev.regs[r + 2], dev.regs[r + 3]};
}
}  // namespace

TEST_CASE("PCA9685 configures the chip", "[pca9685]") {
  test::FakeI2CDevice dev;
  PCA9685 pca{I2C::Port::PORT_0, 0x40, 50_Hz, 0_ms};
  CHECK(dev.regs[0xFE] == 121);   // prescale: 25 MHz / (4096 * 50 Hz) - 1
  CHECK(dev.regs[0x00] == 0x21);  // awake, auto-increment, all-call
  CHECK(dev.regs[0x01] == 0x04);  // totem-pole outputs
  CHECK(dev.regs[0xFD] == 0x10);  // ALL_LED full off
  CHECK(pca.IsConnected());
  CHECK(pca.GetAddress() == 0x40);
  CHECK(pca.GetFrequency().value() == Catch::Approx(25e6 / (4096.0 * 122)));
  CHECK(pca.GetPeriod().value() == Catch::Approx(4096.0 * 122 / 25.0));
  CHECK_THROWS_AS((PCA9685{I2C::Port::PORT_0, 0x40, 2000_Hz, 0_ms}), RuntimeError);
  CHECK_THROWS_AS(pca.SetDutyCycle(16, 0.5), RuntimeError);
}

TEST_CASE("PCA9685 holds the outputs off while disabled", "[pca9685]") {
  test::FakeI2CDevice dev;
  PCA9685 pca{I2C::Port::PORT_0, 0x40, 50_Hz, 0_ms};
  SetRobotEnabled(false);
  pca.SetDutyCycle(0, 0.5);
  dev.writes.clear();
  pca.Update();
  CHECK(dev.writes.empty());
  CHECK_FALSE(pca.OutputsEnabled());

  SetRobotEnabled(true);
  pca.SetDutyCycle(2, 1.0);
  pca.SetPulseWidth(3, 1500_us);
  pca.SetDutyCycle(4, -1.0);
  pca.Update();
  CHECK(pca.OutputsEnabled());
  CHECK(dev.writes.size() == 16);  // every channel after ALL_LED_OFF
  CHECK(Led(dev, 0) == std::array<uint8_t, 4>{0, 0, 0x00, 0x08});  // 2048 counts
  CHECK(Led(dev, 1) == std::array<uint8_t, 4>{0, 0, 0, 0x10});     // full off
  CHECK(Led(dev, 2) == std::array<uint8_t, 4>{0, 0x10, 0, 0});     // full on
  CHECK(Led(dev, 3) == std::array<uint8_t, 4>{0, 0, 307 & 0xFF, 307 >> 8});
  CHECK(Led(dev, 4) == std::array<uint8_t, 4>{0, 0, 0, 0x10});

  dev.writes.clear();
  pca.Update();
  CHECK(dev.writes.empty());  // nothing changed
  pca.SetDutyCycle(0, 0.25);
  pca.Update();
  REQUIRE(dev.writes.size() == 1);
  CHECK(Led(dev, 0) == std::array<uint8_t, 4>{0, 0, 0x00, 0x04});

  SetRobotEnabled(false);
  dev.regs[0xFD] = 0;
  dev.writes.clear();
  pca.Update();
  REQUIRE(dev.writes.size() == 1);
  CHECK(dev.regs[0xFD] == 0x10);  // ALL_LED full off
  dev.writes.clear();
  pca.Update();
  CHECK(dev.writes.empty());
  CHECK(pca.GetDutyCycle(0) == 0.25);  // kept for the next enable
}

TEST_CASE("ProHubMotor drives a channel pair", "[pca9685][prohub]") {
  test::FakeI2CDevice dev;
  PCA9685 pca{I2C::Port::PORT_0, prohub::kPcaAddress0, prohub::kMotorPwmFrequency, 0_ms};
  CHECK(dev.regs[0xFE] == 6);  // Probot Lib 0.2.6 prescale
  ProHubMotor m1{pca, 1};
  ProHubMotor m2{pca, 2};
  CHECK(prohub::GetMotorChannels(4).forward == 9);
  CHECK(prohub::GetMotorChannels(5).forward == 0);
  CHECK(prohub::GetMotorPcaAddress(5) == 0x41);

  m1.SetThrottle(0.5);
  CHECK(pca.GetDutyCycle(0) == 0.5);
  CHECK(pca.GetDutyCycle(1) == 0.0);
  m1.SetThrottle(-0.25);
  CHECK(pca.GetDutyCycle(0) == 0.0);
  CHECK(pca.GetDutyCycle(1) == 0.25);
  CHECK(m1.GetThrottle() == -0.25);

  m2.SetInverted(true);
  m2.SetThrottle(0.75);
  CHECK(m2.GetThrottle() == 0.75);
  CHECK(pca.GetDutyCycle(3) == 0.0);
  CHECK(pca.GetDutyCycle(4) == 0.75);
  m2.SetThrottle(4.0);
  CHECK(m2.GetThrottle() == 1.0);
  CHECK(pca.GetDutyCycle(4) == 1.0);

  m2.StopMotor();
  CHECK(m2.GetThrottle() == 0.0);
  CHECK(pca.GetDutyCycle(3) == 0.0);
  CHECK(pca.GetDutyCycle(4) == 0.0);
  CHECK(m1.GetDescription() == "ProHub motor 1");
  CHECK(m1.GetTelemetryType() == "Motor Controller");
  CHECK_THROWS_AS((ProHubMotor{pca, 9}), RuntimeError);
}
