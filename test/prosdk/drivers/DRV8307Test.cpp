// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/motor/DRV8307.hpp"

#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/simulation/DigitalPWMSim.hpp"
#include "prosdk/simulation/SimDeviceSim.hpp"

using namespace prosdk;

TEST_CASE("DRV8307 enable, direction, brake and coast", "[drv8307]") {
  DRV8307 m{15, 16, 17, 18, 19, 20, 21};
  auto pwm = sim::DigitalPWMSim::CreateForChannel(15);
  sim::DIOSim enableN{16};
  sim::DIOSim dir{17};
  sim::DIOSim brake{18};
  sim::DIOSim lockN{19};
  sim::DIOSim faultN{21};
  CHECK(enableN.GetValue());  // disabled until the first non-zero throttle
  CHECK_FALSE(dir.GetValue());
  CHECK_FALSE(brake.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.0);

  m.SetThrottle(0.6);
  CHECK_FALSE(enableN.GetValue());
  CHECK(dir.GetValue());
  CHECK_FALSE(brake.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.6);
  m.SetThrottle(-0.3);
  CHECK_FALSE(enableN.GetValue());
  CHECK_FALSE(dir.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.3);

  CHECK(m.GetBrakeMode());
  m.SetThrottle(0.0);  // stop with ENABLEn high and BRAKE high, not with 0 % duty alone
  CHECK(enableN.GetValue());
  CHECK(brake.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.0);
  m.SetThrottle(0.5);
  CHECK_FALSE(brake.GetValue());
  CHECK_FALSE(enableN.GetValue());
  m.SetBrakeMode(false);
  m.SetThrottle(0.0);  // coast
  CHECK(enableN.GetValue());
  CHECK_FALSE(brake.GetValue());

  m.SetInverted(true);
  m.SetThrottle(0.2);
  CHECK(m.GetThrottle() == 0.2);
  CHECK_FALSE(dir.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.2);
  sim::SimDeviceSim dev{"DRV8307", 15};
  CHECK(dev.GetDouble("Throttle").Get() == -0.2);

  m.SetThrottle(std::numeric_limits<double>::quiet_NaN());
  CHECK(m.GetThrottle() == 0.0);
  m.SetBrakeMode(true);
  m.SetThrottle(-2.0);
  CHECK(m.GetThrottle() == -1.0);
  m.Disable();  // coast: ENABLEn high, BRAKE low
  CHECK(m.GetThrottle() == 0.0);
  CHECK(enableN.GetValue());
  CHECK_FALSE(brake.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.0);

  lockN.SetValue(false);
  CHECK(m.IsLocked());
  lockN.SetValue(true);
  CHECK_FALSE(m.IsLocked());
  faultN.SetValue(false);
  CHECK(m.IsFault());
  faultN.SetValue(true);
  CHECK_FALSE(m.IsFault());

  CHECK(m.GetPolePairs() == 1);
  m.SetPolePairs(4);
  CHECK(m.GetPolePairs() == 4);
  m.SetPolePairs(0);  // ignored
  CHECK(m.GetPolePairs() == 4);
  CHECK(m.GetRpm().value() == 0.0);  // the sim counter reports no edges

  CHECK(m.GetDescription() == "DRV8307 15/16");
  CHECK(m.GetTelemetryType() == "Motor Controller");
}

TEST_CASE("DRV8307 with DIR and BRAKE set on the board", "[drv8307]") {
  DRV8307 m{22, 23, -1, -1};
  auto pwm = sim::DigitalPWMSim::CreateForChannel(22);
  sim::DIOSim enableN{23};
  m.SetThrottle(0.5);
  CHECK_FALSE(enableN.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.5);
  m.SetThrottle(-0.5);  // no DIR pin: negative throttles stop
  CHECK(enableN.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.0);
  CHECK_FALSE(m.IsLocked());
  CHECK_FALSE(m.IsFault());
  CHECK(m.GetRpm().value() == 0.0);
  m.SetThrottle(0.3);
  m.StopMotor();
  CHECK(enableN.GetValue());
  CHECK(pwm.GetDutyCycle() == 0.0);
}
