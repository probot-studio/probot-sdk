// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DifferentialDriveBot: differential-drive kinematics, wheel velocity PID + feedforward and
// odometry (upstream wpilibcExamples/src/main/cpp/examples/DifferentialDriveBot), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   GPIO 38 left leader     GPIO 37 left follower    PWMSparkMax (or any PWM ESC)
//   GPIO 36 right leader    GPIO 35 right follower
//   GPIO 39/40 left encoder A/B           GPIO 41/42 right encoder A/B
// Gamepad 0: left stick Y drives, right stick X turns.
// The Probot board has no onboard IMU: upstream's OnboardIMU is replaced by the kit IMU,
// prosdk::MPU6050 (same getters), on I2C port 0 (GPIO 1 SDA, GPIO 2 SCL: J3, the HAL default).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Drivetrain.hpp"
#include <prosdk/driverstation/Gamepad.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/math/filter/SlewRateLimiter.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  void AutonomousPeriodic() override {
    TeleopPeriodic();
    drive.UpdateOdometry();
  }

  void TeleopPeriodic() override {
    // Get the x velocity. We are inverting this because gamepads return
    // negative values when we push forward.
    const auto xVelocity = -velocityLimiter.Calculate(controller.GetLeftY()) *
                           Drivetrain::MAX_VELOCITY;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Gamepads return positive values when you pull to
    // the right by default.
    const auto rot = -rotLimiter.Calculate(controller.GetRightX()) *
                     Drivetrain::MAX_ANGULAR_VELOCITY;

    drive.Drive(xVelocity, rot);
  }

 private:
  prosdk::Gamepad controller{0};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> velocityLimiter{3 / 1_s};
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> rotLimiter{3 / 1_s};

  Drivetrain drive;
};

PROSDK_ROBOT_MAIN(Robot)
