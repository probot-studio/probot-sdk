// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/motor/TwoPinHBridge.hpp"

#include <limits>
#include <type_traits>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/simulation/DigitalPWMSim.hpp"
#include "prosdk/simulation/SimDeviceSim.hpp"

using namespace prosdk;

static_assert(std::is_same_v<L9110S, TwoPinHBridge>);
static_assert(std::is_same_v<BA6208, TwoPinHBridge>);

TEST_CASE("TwoPinHBridge direction, brake and coast", "[twopinhbridge]") {
  TwoPinHBridge m{0, 1};
  auto a = sim::DigitalPWMSim::CreateForChannel(0);
  auto b = sim::DigitalPWMSim::CreateForChannel(1);
  CHECK(m.GetZeroBehavior() == TwoPinHBridge::ZeroBehavior::kBrake);
  CHECK(a.GetDutyCycle() == 0.0);
  CHECK(b.GetDutyCycle() == 0.0);

  m.SetThrottle(0.5);  // forward: PWM on A, B low
  CHECK(a.GetDutyCycle() == 0.5);
  CHECK(b.GetDutyCycle() == 0.0);
  m.SetThrottle(-0.25);  // reverse: PWM on B, A low
  CHECK(a.GetDutyCycle() == 0.0);
  CHECK(b.GetDutyCycle() == 0.25);
  m.SetThrottle(0.0);  // brake: both high
  CHECK(a.GetDutyCycle() == 1.0);
  CHECK(b.GetDutyCycle() == 1.0);
  m.SetThrottle(0.75);  // from brake: B drops first, then A carries the PWM
  CHECK(a.GetDutyCycle() == 0.75);
  CHECK(b.GetDutyCycle() == 0.0);

  m.SetZeroBehavior(TwoPinHBridge::ZeroBehavior::kCoast);
  CHECK(m.GetZeroBehavior() == TwoPinHBridge::ZeroBehavior::kCoast);
  m.SetThrottle(0.0);  // coast: both low
  CHECK(a.GetDutyCycle() == 0.0);
  CHECK(b.GetDutyCycle() == 0.0);

  m.SetInverted(true);
  CHECK(m.GetInverted());
  m.SetThrottle(0.5);
  CHECK(m.GetThrottle() == 0.5);
  CHECK(a.GetDutyCycle() == 0.0);
  CHECK(b.GetDutyCycle() == 0.5);
  sim::SimDeviceSim dev{"TwoPinHBridge", 0};
  CHECK(dev.GetDouble("Throttle").Get() == -0.5);

  m.SetThrottle(3.0);
  CHECK(m.GetThrottle() == 1.0);
  m.SetThrottle(std::numeric_limits<double>::quiet_NaN());
  CHECK(m.GetThrottle() == 0.0);

  m.SetThrottle(-1.0);
  m.Disable();  // both low
  CHECK(m.GetThrottle() == 0.0);
  CHECK(a.GetDutyCycle() == 0.0);
  CHECK(b.GetDutyCycle() == 0.0);
  CHECK(dev.GetDouble("Throttle").Get() == 0.0);
  CHECK(m.GetDescription() == "TwoPinHBridge 0/1");
  CHECK(m.GetTelemetryType() == "Motor Controller");
}

TEST_CASE("TwoPinHBridge aliases and coast at zero", "[twopinhbridge]") {
  L9110S m{2, 3, TwoPinHBridge::ZeroBehavior::kCoast};
  auto a = sim::DigitalPWMSim::CreateForChannel(2);
  auto b = sim::DigitalPWMSim::CreateForChannel(3);
  m.SetThrottle(-0.4);
  CHECK(a.GetDutyCycle() == 0.0);
  CHECK(b.GetDutyCycle() == 0.4);
  m.SetThrottle(0.0);
  CHECK(a.GetDutyCycle() == 0.0);
  CHECK(b.GetDutyCycle() == 0.0);
  m.SetThrottle(1.0);
  m.StopMotor();
  CHECK(a.GetDutyCycle() == 0.0);
  CHECK(b.GetDutyCycle() == 0.0);
  CHECK(m.GetThrottle() == 0.0);
}
