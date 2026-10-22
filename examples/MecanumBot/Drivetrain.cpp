// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.hpp"

#include <prosdk/math/kinematics/ChassisVelocities.hpp>

prosdk::math::MecanumDriveWheelPositions Drivetrain::GetCurrentWheelDistances()
    const {
  return {prosdk::units::meter_t{frontLeftEncoder.GetDistance()},
          prosdk::units::meter_t{frontRightEncoder.GetDistance()},
          prosdk::units::meter_t{backLeftEncoder.GetDistance()},
          prosdk::units::meter_t{backRightEncoder.GetDistance()}};
}

prosdk::math::MecanumDriveWheelVelocities Drivetrain::GetCurrentWheelVelocities()
    const {
  return {prosdk::units::meters_per_second_t{frontLeftEncoder.GetRate()},
          prosdk::units::meters_per_second_t{frontRightEncoder.GetRate()},
          prosdk::units::meters_per_second_t{backLeftEncoder.GetRate()},
          prosdk::units::meters_per_second_t{backRightEncoder.GetRate()}};
}

void Drivetrain::SetVelocities(
    const prosdk::math::MecanumDriveWheelVelocities& wheelVelocities) {
  const auto frontLeftFeedforward =
      feedforward.Calculate(wheelVelocities.frontLeft);
  const auto frontRightFeedforward =
      feedforward.Calculate(wheelVelocities.frontRight);
  const auto backLeftFeedforward =
      feedforward.Calculate(wheelVelocities.rearLeft);
  const auto backRightFeedforward =
      feedforward.Calculate(wheelVelocities.rearRight);

  const double frontLeftOutput = frontLeftPIDController.Calculate(
      frontLeftEncoder.GetRate(), wheelVelocities.frontLeft.value());
  const double frontRightOutput = frontRightPIDController.Calculate(
      frontRightEncoder.GetRate(), wheelVelocities.frontRight.value());
  const double backLeftOutput = backLeftPIDController.Calculate(
      backLeftEncoder.GetRate(), wheelVelocities.rearLeft.value());
  const double backRightOutput = backRightPIDController.Calculate(
      backRightEncoder.GetRate(), wheelVelocities.rearRight.value());

  frontLeftMotor.SetVoltage(prosdk::units::volt_t{frontLeftOutput} +
                            frontLeftFeedforward);
  frontRightMotor.SetVoltage(prosdk::units::volt_t{frontRightOutput} +
                             frontRightFeedforward);
  backLeftMotor.SetVoltage(prosdk::units::volt_t{backLeftOutput} +
                           backLeftFeedforward);
  backRightMotor.SetVoltage(prosdk::units::volt_t{backRightOutput} +
                            backRightFeedforward);
}

void Drivetrain::Drive(prosdk::units::meters_per_second_t xVelocity,
                       prosdk::units::meters_per_second_t yVelocity,
                       prosdk::units::radians_per_second_t rot, bool fieldRelative,
                       prosdk::units::second_t period) {
  prosdk::math::ChassisVelocities chassisVelocities{xVelocity, yVelocity, rot};
  if (fieldRelative) {
    chassisVelocities = chassisVelocities.ToRobotRelative(imu.GetRotation2d());
  }
  SetVelocities(
      kinematics.ToWheelVelocities(chassisVelocities.Discretize(period))
          .Desaturate(MAX_VELOCITY));
}

void Drivetrain::UpdateOdometry() {
  odometry.Update(imu.GetRotation2d(), GetCurrentWheelDistances());
}
