// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.hpp"

#include "ExampleGlobalMeasurementSensor.hpp"
#include <prosdk/system/Timer.hpp>

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
  std::function<void(prosdk::units::meters_per_second_t, const prosdk::Encoder&,
                     prosdk::math::PIDController&, prosdk::PWMSparkMax&)>
      calcAndSetVelocities =
          [&feedforward = feedforward](prosdk::units::meters_per_second_t velocity,
                                       const auto& encoder, auto& controller,
                                       auto& motor) {
            auto ff = feedforward.Calculate(velocity);
            double output =
                controller.Calculate(encoder.GetRate(), velocity.value());
            motor.SetVoltage(prosdk::units::volt_t{output} + ff);
          };

  calcAndSetVelocities(wheelVelocities.frontLeft, frontLeftEncoder,
                       frontLeftPIDController, frontLeftMotor);
  calcAndSetVelocities(wheelVelocities.frontRight, frontRightEncoder,
                       frontRightPIDController, frontRightMotor);
  calcAndSetVelocities(wheelVelocities.rearLeft, backLeftEncoder,
                       backLeftPIDController, backLeftMotor);
  calcAndSetVelocities(wheelVelocities.rearRight, backRightEncoder,
                       backRightPIDController, backRightMotor);
}

void Drivetrain::Drive(prosdk::units::meters_per_second_t xVelocity,
                       prosdk::units::meters_per_second_t yVelocity,
                       prosdk::units::radians_per_second_t rot, bool fieldRelative,
                       prosdk::units::second_t period) {
  prosdk::math::ChassisVelocities chassisVelocities{xVelocity, yVelocity, rot};
  if (fieldRelative) {
    chassisVelocities = chassisVelocities.ToRobotRelative(
        poseEstimator.GetEstimatedPosition().Rotation());
  }
  SetVelocities(
      kinematics.ToWheelVelocities(chassisVelocities.Discretize(period))
          .Desaturate(MAX_VELOCITY));
}

void Drivetrain::UpdateOdometry() {
  poseEstimator.Update(
      imu.GetRotation2d(),
      GetCurrentWheelDistances());  // TODO(Ryan): fixup when sim implemented

  // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example -- on a real robot, this must be calculated based either on latency
  // or timestamps.
  poseEstimator.AddVisionMeasurement(
      ExampleGlobalMeasurementSensor::GetEstimatedGlobalPose(
          poseEstimator.GetEstimatedPosition()),
      prosdk::Timer::GetTimestamp() - 0.3_s);
}
