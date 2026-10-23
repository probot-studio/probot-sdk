// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/drivers/imu/MPU6050.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/math/geometry/Translation2d.hpp>
#include <prosdk/math/kinematics/MecanumDriveKinematics.hpp>
#include <prosdk/math/kinematics/MecanumDriveOdometry.hpp>
#include <prosdk/math/kinematics/MecanumDriveWheelVelocities.hpp>

/**
 * Represents a mecanum drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() {
    imu.ResetYaw();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    frontRightMotor.SetInverted(true);
    backRightMotor.SetInverted(true);
  }

  prosdk::math::MecanumDriveWheelPositions GetCurrentWheelDistances() const;
  prosdk::math::MecanumDriveWheelVelocities GetCurrentWheelVelocities() const;
  void SetVelocities(
      const prosdk::math::MecanumDriveWheelVelocities& wheelVelocities);
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
  prosdk::PWMSparkMax frontLeftMotor{38};
  prosdk::PWMSparkMax frontRightMotor{37};
  prosdk::PWMSparkMax backLeftMotor{36};
  prosdk::PWMSparkMax backRightMotor{35};

  prosdk::Encoder frontLeftEncoder{39, 40};
  prosdk::Encoder frontRightEncoder{41, 42};
  prosdk::Encoder backLeftEncoder{47, 48};
  prosdk::Encoder backRightEncoder{15, 16};

  prosdk::math::Translation2d frontLeftLocation{0.381_m, 0.381_m};
  prosdk::math::Translation2d frontRightLocation{0.381_m, -0.381_m};
  prosdk::math::Translation2d backLeftLocation{-0.381_m, 0.381_m};
  prosdk::math::Translation2d backRightLocation{-0.381_m, -0.381_m};

  prosdk::math::PIDController frontLeftPIDController{1.0, 0.0, 0.0};
  prosdk::math::PIDController frontRightPIDController{1.0, 0.0, 0.0};
  prosdk::math::PIDController backLeftPIDController{1.0, 0.0, 0.0};
  prosdk::math::PIDController backRightPIDController{1.0, 0.0, 0.0};

  prosdk::MPU6050 imu{prosdk::I2C::Port::PORT_0};  // kit IMU (upstream: OnboardIMU)

  prosdk::math::MecanumDriveKinematics kinematics{
      frontLeftLocation, frontRightLocation, backLeftLocation,
      backRightLocation};

  prosdk::math::MecanumDriveOdometry odometry{kinematics, imu.GetRotation2d(),
                                           GetCurrentWheelDistances()};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  prosdk::math::SimpleMotorFeedforward<prosdk::units::meters> feedforward{
      1_V, 3_V / 1_mps};
};
