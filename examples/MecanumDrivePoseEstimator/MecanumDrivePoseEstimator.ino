// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// MecanumDrivePoseEstimator: mecanum odometry fused with a (simulated, noisy) global
// measurement in a MecanumDrivePoseEstimator (upstream
// wpilibcExamples/src/main/cpp/examples/MecanumDrivePoseEstimator), ported to
// prosdk. ExampleGlobalMeasurementSensor stands in for a vision system.
//
// Wiring (GPIO numbers; channel = GPIO):
//   PWMSparkMax (or any PWM ESC): GPIO 4 front left, GPIO 5 front right, GPIO 12 back left,
//   GPIO 13 back right
//   Encoders A/B: GPIO 6/7 front left, GPIO 15/16 front right, GPIO 38/39 back left,
//   GPIO 40/41 back right (all 4 PCNT units of the ESP32-S3)
// Gamepad 0: left stick drives (field relative in teleop), right stick X turns.
// The Probot board has no onboard IMU: upstream's OnboardIMU is replaced by the kit IMU,
// prosdk::MPU6050 (same getters), on I2C port 0 (GPIO 8 SDA, GPIO 9 SCL, the HAL default).

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
    DriveWithJoystick(false);
    mecanum.UpdateOdometry();
  }

  void TeleopPeriodic() override { DriveWithJoystick(true); }

 private:
  prosdk::Gamepad controller{0};
  Drivetrain mecanum;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> xVelocityLimiter{3 / 1_s};
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> yVelocityLimiter{3 / 1_s};
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> rotLimiter{3 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {
    // Get the x velocity. We are inverting this because gamepads return
    // negative values when we push forward.
    const auto xVelocity = -xVelocityLimiter.Calculate(controller.GetLeftY()) *
                           Drivetrain::MAX_VELOCITY;

    // Get the y velocity or sideways/strafe velocity. We are inverting this
    // because we want a positive value when we pull to the left. Gamepads
    // return positive values when you pull to the right by default.
    const auto yVelocity = -yVelocityLimiter.Calculate(controller.GetLeftX()) *
                           Drivetrain::MAX_VELOCITY;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Gamepads return positive values when you pull to
    // the right by default.
    const auto rot = -rotLimiter.Calculate(controller.GetRightX()) *
                     Drivetrain::MAX_ANGULAR_VELOCITY;

    mecanum.Drive(xVelocity, yVelocity, rot, fieldRelative, GetPeriod());
  }
};

PROSDK_ROBOT_MAIN(Robot)
