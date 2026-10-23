// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SwerveBot: swerve kinematics, module PID + feedforward and odometry (upstream
// wpilibcExamples/src/main/cpp/examples/SwerveBot),
// ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO), module = drive PWM, turning PWM, drive encoder A/B,
// turning encoder A/B:
//   front left  4, 14,  6/7,   40/41      front right 5, 21, 15/16, 42/1
//   back left  12, 47, 17/18,  2/10       back right 13, 48, 38/39, 11/46
// GPIO 46 is a strapping pin, used for lack of free pins. It counts at reset only while BOOT
// (GPIO 0) is held, where a high level blocks download mode; a normal boot ignores it. GPIO 45
// (flash voltage) is avoided: an encoder line high at reset can stop a 3.3 V-flash module from
// booting.
// Does not fit the ESP32-S3 as written: upstream uses 8 quadrature encoders and the chip has 4
// PCNT units (encoders and counters together), so on the robot the fifth Encoder fails and the
// program halts with the error on the Driver Station console. It runs in simulation; on real
// hardware use absolute turning encoders (DutyCycleEncoder, 6 inputs; AnalogEncoder, GPIO 1-10).
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
#include <prosdk/math/util/MathUtil.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  void AutonomousPeriodic() override {
    DriveWithJoystick(false);
    swerve.UpdateOdometry();
  }

  void TeleopPeriodic() override { DriveWithJoystick(true); }

 private:
  prosdk::Gamepad controller{0};
  Drivetrain swerve;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> xVelocityLimiter{3 / 1_s};
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> yVelocityLimiter{3 / 1_s};
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> rotLimiter{3 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {
    // Get the x velocity. We are inverting this because gamepads return
    // negative values when we push forward.
    const auto xVelocity = -xVelocityLimiter.Calculate(prosdk::math::ApplyDeadband(
                               controller.GetLeftY(), 0.02)) *
                           Drivetrain::MAX_VELOCITY;

    // Get the y velocity or sideways/strafe velocity. We are inverting this
    // because we want a positive value when we pull to the left. Gamepads
    // return positive values when you pull to the right by default.
    const auto yVelocity = -yVelocityLimiter.Calculate(prosdk::math::ApplyDeadband(
                               controller.GetLeftX(), 0.02)) *
                           Drivetrain::MAX_VELOCITY;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Gamepads return positive values when you pull to
    // the right by default.
    const auto rot = -rotLimiter.Calculate(prosdk::math::ApplyDeadband(
                         controller.GetRightX(), 0.02)) *
                     Drivetrain::MAX_ANGULAR_VELOCITY;

    swerve.Drive(xVelocity, yVelocity, rot, fieldRelative, GetPeriod());
  }
};

PROSDK_ROBOT_MAIN(Robot)
