// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetLimitSwitch: limit switch snippets (upstream
// wpilibcExamples/src/main/cpp/snippets/LimitSwitch), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   GPIO 17  top limit switch     GPIO 18  bottom limit switch (to GND, internal pull-up)
//   GPIO 4   PWMVictorSPX (or any PWM ESC)
// Joystick 0, axis 2 drives the motor.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drivers/motor/PWMVictorSPX.hpp>
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/discrete/DigitalInput.hpp>

/**
 * Limit Switch snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/limit-switch.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  void TeleopPeriodic() override { SetMotorVelocity(joystick.GetRawAxis(2)); }
  void SetMotorVelocity(double velocity) {
    if (velocity > 0) {
      if (toplimitSwitch.Get()) {
        // We are going up and top limit is tripped so stop
        motor.SetThrottle(0);
      } else {
        // We are going up but top limit is not tripped so go at commanded
        // velocity
        motor.SetThrottle(velocity);
      }
    } else {
      if (bottomlimitSwitch.Get()) {
        // We are going down and bottom limit is tripped so stop
        motor.SetThrottle(0);
      } else {
        // We are going down but bottom limit is not tripped so go at commanded
        // velocity
        motor.SetThrottle(velocity);
      }
    }
  }

 private:
  prosdk::DigitalInput toplimitSwitch{17};
  prosdk::DigitalInput bottomlimitSwitch{18};
  prosdk::PWMVictorSPX motor{4};
  prosdk::Joystick joystick{0};
};

PROSDK_ROBOT_MAIN(Robot)
