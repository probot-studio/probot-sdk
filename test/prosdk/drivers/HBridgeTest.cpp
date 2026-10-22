// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/drivers/motor/BTS7960.hpp"
#include "prosdk/drivers/motor/VNH5019.hpp"
#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/simulation/DigitalPWMSim.hpp"
#include "prosdk/simulation/SimDeviceSim.hpp"

using namespace prosdk;

TEST_CASE("BTS7960 with the PWM on RPWM/LPWM", "[bts7960]") {
  BTS7960 m{10, 11, 12};
  auto r = sim::DigitalPWMSim::CreateForChannel(10);
  auto l = sim::DigitalPWMSim::CreateForChannel(11);
  sim::DIOSim en{12};
  CHECK(m.GetWiring() == BTS7960::Wiring::PWM_ON_INPUTS);
  CHECK(r.GetDutyCycle() == 0.0);
  CHECK_FALSE(en.GetValue());

  m.SetThrottle(0.5);
  CHECK(r.GetDutyCycle() == 0.5);
  CHECK(l.GetDutyCycle() == 0.0);
  CHECK(en.GetValue());
  m.SetThrottle(-0.25);
  CHECK(r.GetDutyCycle() == 0.0);
  CHECK(l.GetDutyCycle() == 0.25);
  m.SetThrottle(0.0);  // brake: both low, enabled
  CHECK(r.GetDutyCycle() == 0.0);
  CHECK(l.GetDutyCycle() == 0.0);
  CHECK(en.GetValue());

  m.SetInverted(true);
  m.SetThrottle(0.5);
  CHECK(m.GetThrottle() == 0.5);
  CHECK(r.GetDutyCycle() == 0.0);
  CHECK(l.GetDutyCycle() == 0.5);
  sim::SimDeviceSim dev{"BTS7960", 10};
  CHECK(dev.GetDouble("Throttle").Get() == -0.5);

  m.SetThrottle(3.0);
  CHECK(m.GetThrottle() == 1.0);
  m.SetThrottle(std::numeric_limits<double>::quiet_NaN());
  CHECK(m.GetThrottle() == 0.0);

  m.SetThrottle(-1.0);
  m.Disable();  // coast: enable low
  CHECK(m.GetThrottle() == 0.0);
  CHECK_FALSE(en.GetValue());
  CHECK(r.GetDutyCycle() == 0.0);
  CHECK(l.GetDutyCycle() == 0.0);
  CHECK(m.GetDescription() == "BTS7960 10/11");
  CHECK(m.GetTelemetryType() == "Motor Controller");
}

TEST_CASE("BTS7960 with the PWM on the enable pins", "[bts7960]") {
  BTS7960 m{13, 14, 15, BTS7960::Wiring::PWM_ON_ENABLE};
  auto en = sim::DigitalPWMSim::CreateForChannel(15);
  sim::DIOSim r{13};
  sim::DIOSim l{14};

  m.SetThrottle(0.4);
  CHECK(r.GetValue());
  CHECK_FALSE(l.GetValue());
  CHECK(en.GetDutyCycle() == 0.4);
  m.SetThrottle(-0.6);
  CHECK_FALSE(r.GetValue());
  CHECK(l.GetValue());
  CHECK(en.GetDutyCycle() == 0.6);
  m.SetThrottle(0.0);
  CHECK_FALSE(r.GetValue());
  CHECK_FALSE(l.GetValue());
  CHECK(en.GetDutyCycle() == 0.0);
  m.SetThrottle(0.3);
  m.StopMotor();
  CHECK(en.GetDutyCycle() == 0.0);
  CHECK_FALSE(r.GetValue());
}

TEST_CASE("VNH5019 direction, brake and coast", "[vnh5019]") {
  VNH5019 m{16, 17, 18, 19, 20};
  sim::DIOSim ina{16};
  sim::DIOSim inb{17};
  auto pwm = sim::DigitalPWMSim::CreateForChannel(18);
  sim::DIOSim ena{19};
  sim::DIOSim enb{20};

  m.SetThrottle(0.5);
  CHECK(ina.GetValue());
  CHECK_FALSE(inb.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.5);
  CHECK(ena.GetValue());
  CHECK(enb.GetValue());
  m.SetThrottle(-0.75);
  CHECK_FALSE(ina.GetValue());
  CHECK(inb.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.75);

  CHECK(m.GetBrakeMode());
  m.SetThrottle(0.0);  // brake to ground: INA = INB = low, PWM = strength
  CHECK_FALSE(ina.GetValue());
  CHECK_FALSE(inb.GetValue());
  CHECK(pwm.GetDutyCycle() == 1.0);
  m.SetBrakeStrength(0.3);
  m.SetThrottle(0.0);
  CHECK(pwm.GetDutyCycle() == 0.3);
  m.SetBrakeMode(false);
  m.SetThrottle(0.0);
  CHECK(pwm.GetDutyCycle() == 0.0);

  m.SetInverted(true);
  m.SetThrottle(0.2);
  CHECK(m.GetThrottle() == 0.2);
  CHECK(inb.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.2);

  m.Disable();
  CHECK(m.GetThrottle() == 0.0);
  CHECK(pwm.GetDutyCycle() == 0.0);
  CHECK_FALSE(ena.GetValue());
  CHECK_FALSE(enb.GetValue());
  CHECK(m.GetDescription() == "VNH5019 16/17/18");
}

TEST_CASE("VNH5019 without enable pins", "[vnh5019]") {
  VNH5019 m{21, 22, 23};
  auto pwm = sim::DigitalPWMSim::CreateForChannel(23);
  m.SetThrottle(-1.0);
  CHECK(pwm.GetDutyCycle() == 1.0);
  m.Disable();
  CHECK(pwm.GetDutyCycle() == 0.0);
}
