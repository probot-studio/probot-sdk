// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetMotorControl: a motor from a joystick, encoder distance on the Driver Station
// (upstream wpilibcExamples/src/main/cpp/snippets/MotorControl), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 4 PWMSparkMax (or any PWM ESC), GPIO 6/7
// encoder A/B. Joystick 0, Y axis.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <numbers>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/telemetry/Telemetry.hpp>

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and motor controller inputs as
 * range from -1 to 1 making it easy to work together.
 *
 * In addition, the encoder value of an encoder connected to GPIO 6 and 7 is
 * consistently sent to the Dashboard.
 *
 * Finally, short code snippets show how to invert the motor direction and how
 * to use the motor safety for frc-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  void TeleopPeriodic() override { motor.SetThrottle(stick.GetY()); }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  void RobotPeriodic() override {
    prosdk::telemetry::Log("Encoder", encoder.GetDistance());
  }

  Robot() {
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    encoder.SetDistancePerPulse((std::numbers::pi * 6) / 360.0);

    // show motor inversion
    motor.SetInverted(true);

    // show motor safety features
    motor.SetSafetyEnabled(true);
    motor.SetSafetyEnabled(false);
    motor.SetExpiration(0.1_s);
    motor.Feed();
  }

 private:
  prosdk::Joystick stick{0};
  prosdk::PWMSparkMax motor{4};
  prosdk::Encoder encoder{6, 7};
};

PROSDK_ROBOT_MAIN(Robot)
