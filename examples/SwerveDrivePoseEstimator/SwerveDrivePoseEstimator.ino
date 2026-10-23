// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SwerveDrivePoseEstimator: swerve odometry fused with a (simulated, noisy) global
// measurement in a SwerveDrivePoseEstimator (upstream
// wpilibcExamples/src/main/cpp/examples/SwerveDrivePoseEstimator), ported to
// prosdk. ExampleGlobalMeasurementSensor stands in for a vision system.
//
// Wiring (GPIO numbers; channel = GPIO), module = drive PWM, turning PWM, drive encoder A/B,
// turning encoder A/B:
//   front left  4, 14,  6/7,   40/41      front right 5, 21, 15/16, 42/8
//   back left  12, 47, 17/18,  9/10       back right 13, 48, 38/39, 11/46
// GPIO 46 is a strapping pin, used for lack of free pins. It counts at reset only while BOOT
// (GPIO 0) is held, where a high level blocks download mode; a normal boot ignores it. GPIO 45
// (flash voltage) is avoided: an encoder line high at reset can stop a 3.3 V-flash module from
// booting. The 24 signals do not fit the 20 header pins of the Probot main board (Boardoza
// ESP32-S3 Pulse) either; GPIO 1/2 stay free for the IMU's I2C (J3).
// Does not fit the ESP32-S3 as written: upstream uses 8 quadrature encoders and the chip has 4
// PCNT units (encoders and counters together), so on the robot the fifth Encoder fails and the
// program halts with the error on the Driver Station console (there is no desktop simulation of
// a sketch, design/examples.md). On real hardware use absolute turning encoders
// (DutyCycleEncoder, 6 inputs; AnalogEncoder, GPIO 1-10).
// Gamepad 0: left stick drives (field relative in teleop), right stick X turns.
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

    swerve.Drive(xVelocity, yVelocity, rot, fieldRelative, GetPeriod());
  }
};

PROSDK_ROBOT_MAIN(Robot)
