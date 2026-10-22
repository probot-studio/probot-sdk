// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ArcadeDriveGamepad: split-stick arcade drive with a gamepad (upstream
// wpilibcExamples/src/main/cpp/examples/ArcadeDriveGamepad),
// ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 4 left, GPIO 5 right PWMSparkMax (or any PWM
// ESC). Gamepad 0: left stick Y drives, right stick X turns.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drive/DifferentialDrive.hpp>
#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Gamepad.hpp>
#include <prosdk/framework/TimedRobot.hpp>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with split arcade steering and a Gamepad.
 */
class Robot : public prosdk::TimedRobot {
  prosdk::PWMSparkMax leftMotor{4};
  prosdk::PWMSparkMax rightMotor{5};
  prosdk::DifferentialDrive robotDrive{
      [&](double output) { leftMotor.SetThrottle(output); },
      [&](double output) { rightMotor.SetThrottle(output); }};
  prosdk::Gamepad driverController{0};

 public:
  Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.SetInverted(true);
  }

  void TeleopPeriodic() override {
    // Drive with split arcade style
    // That means that the Y axis of the left stick moves forward
    // and backward, and the X of the right stick turns left and right.
    robotDrive.ArcadeDrive(-driverController.GetLeftY(),
                           -driverController.GetRightX());
  }
};

PROSDK_ROBOT_MAIN(Robot)
