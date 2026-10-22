// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/controller/ProfiledPIDController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/math/kinematics/SwerveModulePosition.hpp>
#include <prosdk/math/kinematics/SwerveModuleVelocity.hpp>
#include <prosdk/units/angular_velocity.hpp>
#include <prosdk/units/time.hpp>
#include <prosdk/units/velocity.hpp>
#include <prosdk/units/voltage.hpp>

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int driveEncoderChannelA, int driveEncoderChannelB,
               int turningEncoderChannelA, int turningEncoderChannelB);
  prosdk::math::SwerveModulePosition GetPosition() const;
  prosdk::math::SwerveModuleVelocity GetVelocity() const;
  void SetDesiredVelocity(prosdk::math::SwerveModuleVelocity& desiredVelocity);

 private:
  static constexpr double WHEEL_RADIUS = 0.0508;
  static constexpr int ENCODER_RESOLUTION = 4096;

  static constexpr auto MODULE_MAX_ANGULAR_VELOCITY =
      std::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr auto MODULE_MAX_ANGULAR_ACCELERATION =
      std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

  prosdk::PWMSparkMax driveMotor;
  prosdk::PWMSparkMax turningMotor;

  prosdk::Encoder driveEncoder;
  prosdk::Encoder turningEncoder;

  prosdk::math::PIDController drivePIDController{1.0, 0, 0};
  prosdk::math::ProfiledPIDController<prosdk::units::radians> turningPIDController{
      1.0,
      0.0,
      0.0,
      {MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION}};

  prosdk::math::SimpleMotorFeedforward<prosdk::units::meters> driveFeedforward{
      1_V, 3_V / 1_mps};
  prosdk::math::SimpleMotorFeedforward<prosdk::units::radians> turnFeedforward{
      1_V, 0.5_V / 1_rad_per_s};
};
