// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/servo/Servo.hpp"

#include <limits>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CDevice.hpp"
#include "prosdk/drivers/ProHub.hpp"
#include "prosdk/drivers/pwm/PCA9685.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/simulation/PWMSim.hpp"
#include "prosdk/simulation/SimDeviceSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

TEST_CASE("Servo maps position and angle onto the pulse", "[servo]") {
  Servo servo{1};
  sim::PWMSim pwm{1};
  CHECK(servo.GetChannel() == 1);
  CHECK(servo.GetMinAngle() == 0.0);
  CHECK(servo.GetMaxAngle() == 180.0);
  CHECK(servo.Get() == 0.0);
  CHECK(pwm.GetPulseMicrosecond() == 0);  // offline until the first Set

  servo.Set(0.5);
  CHECK(pwm.GetPulseMicrosecond() == 1500);
  CHECK(servo.Get() == 0.5);
  CHECK(servo.GetAngle() == Catch::Approx(90.0));

  servo.SetAngle(45.0);
  CHECK(pwm.GetPulseMicrosecond() == 1050);
  CHECK(servo.Get() == Catch::Approx(0.25));

  servo.SetAngle(200.0);
  CHECK(pwm.GetPulseMicrosecond() == 2400);
  CHECK(servo.GetAngle() == Catch::Approx(180.0));
  servo.SetAngle(-5.0);
  CHECK(pwm.GetPulseMicrosecond() == 600);
  servo.Set(2.0);
  CHECK(servo.Get() == 1.0);
  servo.Set(-1.0);
  CHECK(servo.Get() == 0.0);

  sim::SimDeviceSim dev{"Servo", 1};
  servo.Set(0.75);
  CHECK(dev.GetDouble("Position").Get() == Catch::Approx(0.75));

  servo.SetOffline();
  CHECK(pwm.GetPulseMicrosecond() == 0);
  CHECK(servo.Get() == 0.0);
  servo.Set(0.5);
  servo.Set(std::numeric_limits<double>::quiet_NaN());
  CHECK(pwm.GetPulseMicrosecond() == 0);

  servo.SetBounds(2000_us, 0_us, 0_us, 0_us, 1000_us);
  CHECK(servo.GetMaxPulse() == 2000_us);
  CHECK(servo.GetMinPulse() == 1000_us);
  servo.Set(0.5);
  CHECK(pwm.GetPulseMicrosecond() == 1500);
  servo.Set(1.0);
  CHECK(pwm.GetPulseMicrosecond() == 2000);
  CHECK_THROWS_AS(servo.SetBounds(1000_us, 0_us, 0_us, 0_us, 2000_us), RuntimeError);
  CHECK(servo.GetTelemetryType() == "Servo");
}

TEST_CASE("Servo on a PCA9685 channel", "[servo][pca9685]") {
  test::FakeI2CDevice dev;
  PCA9685 pca{I2C::Port::PORT_0, prohub::kPcaAddress0, 50_Hz, 0_ms};
  sim::DriverStationSim::SetDsAttached(true);
  sim::DriverStationSim::SetEnabled(true);
  sim::DriverStationSim::NotifyNewData();

  {
    Servo servo{pca, prohub::kFirstServoChannel};
    CHECK(servo.GetChannel() == 12);
    servo.Set(0.5);  // 1500 us of a 19988 us period
    CHECK(pca.GetDutyCycle(12) == Catch::Approx(307.0 / 4096));
    pca.Update();
    int reg = 0x06 + 4 * 12;
    CHECK(dev.regs[reg + 2] == (307 & 0xFF));
    CHECK(dev.regs[reg + 3] == (307 >> 8));
    servo.SetOffline();
    CHECK(pca.GetDutyCycle(12) == 0.0);
    servo.Set(1.0);
  }
  CHECK(pca.GetDutyCycle(12) == 0.0);  // the destructor turns the output off

  PCA9685 fast{I2C::Port::PORT_0, prohub::kPcaAddress1, prohub::kMotorPwmFrequency, 0_ms};
  CHECK_THROWS_AS((Servo{fast, 12}), RuntimeError);
  CHECK_THROWS_AS((Servo{pca, 16}), RuntimeError);

  sim::DriverStationSim::SetEnabled(false);
  sim::DriverStationSim::NotifyNewData();
}
