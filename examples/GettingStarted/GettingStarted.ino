// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// GettingStarted: a differential-drive robot with split-stick arcade drive and a 2 s timed
// autonomous (upstream wpilibcExamples/src/main/cpp/examples/GettingStarted), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 4 left, GPIO 5 right PWMSparkMax (or any PWM
// ESC). Gamepad 0: left stick Y drives, right stick X steers.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drive/DifferentialDrive.hpp>
#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Gamepad.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/system/Timer.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    right.SetInverted(true);
  }

  void AutonomousInit() override { timer.Restart(); }

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (timer.Get() < 2_s) {
      // Drive forwards half velocity, make sure to turn input squaring off
      robotDrive.ArcadeDrive(0.5, 0.0, false);
    } else {
      // Stop robot
      robotDrive.ArcadeDrive(0.0, 0.0, false);
    }
  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
    // Drive with arcade style (use right stick to steer). prosdk: X is negated
    // (upstream passes +GetRightX()): zRotation is counterclockwise-positive and
    // the stick's X is right-positive, so stick right must turn right.
    robotDrive.ArcadeDrive(-controller.GetLeftY(), -controller.GetRightX());
  }

  void UtilityInit() override {}

  void UtilityPeriodic() override {}

 private:
  // Robot drive system
  prosdk::PWMSparkMax left{4};
  prosdk::PWMSparkMax right{5};
  prosdk::DifferentialDrive robotDrive{
      [&](double output) { left.SetThrottle(output); },
      [&](double output) { right.SetThrottle(output); }};

  prosdk::Gamepad controller{0};
  prosdk::Timer timer;
};

PROSDK_ROBOT_MAIN(Robot)
