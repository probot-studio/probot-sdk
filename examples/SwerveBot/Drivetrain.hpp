// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include "SwerveModule.hpp"
#include <prosdk/drivers/imu/MPU6050.hpp>
#include <prosdk/math/geometry/Translation2d.hpp>
#include <prosdk/math/kinematics/SwerveDriveKinematics.hpp>
#include <prosdk/math/kinematics/SwerveDriveOdometry.hpp>

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { imu.ResetYaw(); }

  void Drive(prosdk::units::meters_per_second_t xVelocity,
             prosdk::units::meters_per_second_t yVelocity,
             prosdk::units::radians_per_second_t rot, bool fieldRelative,
             prosdk::units::second_t period);
  void UpdateOdometry();

  static constexpr prosdk::units::meters_per_second_t MAX_VELOCITY =
      3.0_mps;  // 3 meters per second
  static constexpr prosdk::units::radians_per_second_t MAX_ANGULAR_VELOCITY{
      std::numbers::pi};  // 1/2 rotation per second

 private:
  prosdk::math::Translation2d frontLeftLocation{+0.381_m, +0.381_m};
  prosdk::math::Translation2d frontRightLocation{+0.381_m, -0.381_m};
  prosdk::math::Translation2d backLeftLocation{-0.381_m, +0.381_m};
  prosdk::math::Translation2d backRightLocation{-0.381_m, -0.381_m};

  SwerveModule frontLeft{4, 14, 6, 7, 40, 41};
  SwerveModule frontRight{5, 21, 15, 16, 42, 1};
  SwerveModule backLeft{12, 47, 17, 18, 2, 10};
  SwerveModule backRight{13, 48, 38, 39, 11, 45};

  prosdk::MPU6050 imu{prosdk::I2C::Port::PORT_0};  // kit IMU (upstream: OnboardIMU)

  prosdk::math::SwerveDriveKinematics<4> kinematics{
      frontLeftLocation, frontRightLocation, backLeftLocation,
      backRightLocation};

  prosdk::math::SwerveDriveOdometry<4> odometry{
      kinematics,
      imu.GetRotation2d(),
      {frontLeft.GetPosition(), frontRight.GetPosition(),
       backLeft.GetPosition(), backRight.GetPosition()}};
};
