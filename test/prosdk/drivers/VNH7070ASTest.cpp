// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/motor/VNH7070AS.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/simulation/AnalogInputSim.hpp"
#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/simulation/DigitalPWMSim.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/simulation/SimDeviceSim.hpp"

using namespace prosdk;

namespace {
void SetRobotEnabled(bool enabled) {
  sim::DriverStationSim::SetDsAttached(true);
  sim::DriverStationSim::SetEnabled(enabled);
  sim::DriverStationSim::NotifyNewData();
}
}  // namespace

TEST_CASE("VNH7070AS direction, SEL0, brake and coast", "[vnh7070as]") {
  VNH7070AS m{4, 5, 6, 7};
  sim::DIOSim ina{4};
  sim::DIOSim inb{5};
  auto pwm = sim::DigitalPWMSim::CreateForChannel(6);
  sim::DIOSim sel0{7};
  CHECK_FALSE(ina.GetValue());
  CHECK_FALSE(inb.GetValue());
  CHECK_FALSE(sel0.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.0);

  m.SetThrottle(0.5);  // forward: SEL0 = INA = high (CS reports HSDA)
  CHECK(ina.GetValue());
  CHECK_FALSE(inb.GetValue());
  CHECK(sel0.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.5);
  m.SetThrottle(-0.75);  // reverse: SEL0 = INA = low (CS reports HSDB)
  CHECK_FALSE(ina.GetValue());
  CHECK(inb.GetValue());
  CHECK_FALSE(sel0.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.75);

  CHECK(m.GetBrakeMode());
  m.SetThrottle(0.0);  // brake to ground: INA = INB = low, PWM = strength
  CHECK_FALSE(ina.GetValue());
  CHECK_FALSE(inb.GetValue());
  CHECK_FALSE(sel0.GetValue());
  CHECK(pwm.GetDutyCycle() == 1.0);
  m.SetBrakeStrength(0.3);
  CHECK(m.GetBrakeStrength() == 0.3);
  m.SetThrottle(0.0);
  CHECK(pwm.GetDutyCycle() == 0.3);
  m.SetBrakeMode(false);
  m.SetThrottle(0.0);  // coast: INA = INB = SEL0 = PWM = 0 (standby)
  CHECK(pwm.GetDutyCycle() == 0.0);

  m.SetInverted(true);
  m.SetThrottle(0.2);
  CHECK(m.GetThrottle() == 0.2);
  CHECK(inb.GetValue());
  CHECK_FALSE(sel0.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.2);
  sim::SimDeviceSim dev{"VNH7070AS", 4};
  CHECK(dev.GetDouble("Throttle").Get() == -0.2);

  m.SetThrottle(-0.6);
  m.Disable();
  CHECK(m.GetThrottle() == 0.0);
  CHECK(pwm.GetDutyCycle() == 0.0);
  CHECK_FALSE(ina.GetValue());
  CHECK_FALSE(inb.GetValue());
  CHECK_FALSE(sel0.GetValue());
  CHECK(m.GetDescription() == "VNH7070AS 4/5/6");
  CHECK(m.GetTelemetryType() == "Motor Controller");
}

TEST_CASE("VNH7070AS current sense and fault", "[vnh7070as]") {
  VNH7070AS m{8, 9, 10, 11, 1};
  sim::AnalogInputSim cs{1};

  // board defaults: 1.5 kOhm to ground, no divider, K = 1540
  cs.SetVoltage(1.5);
  m.SetThrottle(0.5);
  CHECK(m.GetCurrent().value() == Catch::Approx(1.5 / 1500.0 * 1540.0));
  cs.SetVoltage(3.1);  // ADC full scale: below the 5 V fault level without a divider
  CHECK_FALSE(m.IsFault());

  m.SetCurrentSense(units::ohm_t{1000.0}, 0.5);
  cs.SetVoltage(1.5);  // 3 V at CS
  CHECK(m.GetCurrent().value() == Catch::Approx(3.0 / 1000.0 * 1540.0));
  CHECK_FALSE(m.IsFault());
  cs.SetVoltage(2.5);  // 5 V at CS: VSENSEH while driving
  CHECK(m.IsFault());
  m.SetThrottle(-0.5);
  CHECK(m.IsFault());

  m.SetCurrentSense(units::ohm_t{-1.0}, 2.0);  // invalid values are ignored
  CHECK(m.GetCurrent().value() == Catch::Approx(5.0 / 1000.0 * 1540.0));

  // braking: CS is valid only while the output gate lets the PWM through
  SetRobotEnabled(true);
  m.SetThrottle(0.0);
  CHECK(m.IsFault());
  SetRobotEnabled(false);
  CHECK_FALSE(m.IsFault());

  // coast and Disable: off-state diagnostic, never a fault
  m.SetBrakeMode(false);
  m.SetThrottle(0.0);
  CHECK_FALSE(m.IsFault());
  m.SetThrottle(0.4);
  m.Disable();
  CHECK_FALSE(m.IsFault());
}

TEST_CASE("VNH7070AS without SEL0 and CS", "[vnh7070as]") {
  VNH7070AS m{12, 13, 14};
  auto pwm = sim::DigitalPWMSim::CreateForChannel(14);
  m.SetThrottle(-1.0);
  CHECK(pwm.GetDutyCycle() == 1.0);
  CHECK(m.GetCurrent().value() == 0.0);
  CHECK_FALSE(m.IsFault());
  m.StopMotor();
  CHECK(pwm.GetDutyCycle() == 0.0);
}
