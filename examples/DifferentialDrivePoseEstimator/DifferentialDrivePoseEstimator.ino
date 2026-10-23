// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DifferentialDrivePoseEstimator: differential drive odometry fused with vision-style
// measurements computed from a simulated camera looking at AprilTag 1 of the 2024 field
// (upstream wpilibcExamples/src/main/cpp/examples/DifferentialDrivePoseEstimator), ported to
// prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   PWMSparkMax (or any PWM ESC): GPIO 4 left leader, GPIO 5 left follower,
//   GPIO 12 right leader, GPIO 13 right follower
//   Encoders A/B: GPIO 6/7 left, GPIO 15/16 right
// Gamepad 0: left Y drives, right X turns.
// The Probot board has no onboard IMU: upstream's OnboardIMU is replaced by the kit IMU,
// prosdk::MPU6050 (same getters), on I2C port 0 (GPIO 8 SDA, GPIO 9 SCL, the HAL default).
// Changes from upstream: prosdk has no NetworkTables, so the simulated camera-to-object
// transform goes through a std::array member instead of an NT topic; the MPU6050 has no sim
// class, so the simulated heading is not written back (upstream's imuSim.SetYaw is commented
// out, as upstream SimpleDifferentialDriveSimulation already does with its gyro).
// The Driver Station's "Veri" page draws the Field2d as a field widget (probot-core #40).

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

  void RobotPeriodic() override { drive.Periodic(); }

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

  void SimulationPeriodic() override { drive.SimulationPeriodic(); }

 private:
  prosdk::Gamepad controller{0};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> velocityLimiter{3 / 1_s};
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> rotLimiter{3 / 1_s};

  Drivetrain drive;
};

PROSDK_ROBOT_MAIN(Robot)
