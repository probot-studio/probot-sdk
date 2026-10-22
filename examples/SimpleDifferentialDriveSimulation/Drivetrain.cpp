// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.hpp"

#include <prosdk/system/RobotController.hpp>
#include <prosdk/telemetry/Telemetry.hpp>

void Drivetrain::SetVelocities(
    const prosdk::math::DifferentialDriveWheelVelocities& velocities) {
  auto leftFeedforward = feedforward.Calculate(velocities.left);
  auto rightFeedforward = feedforward.Calculate(velocities.right);
  double leftOutput = leftPIDController.Calculate(leftEncoder.GetRate(),
                                                  velocities.left.value());
  double rightOutput = rightPIDController.Calculate(rightEncoder.GetRate(),
                                                    velocities.right.value());

  leftLeader.SetVoltage(prosdk::units::volt_t{leftOutput} + leftFeedforward);
  rightLeader.SetVoltage(prosdk::units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(prosdk::units::meters_per_second_t xVelocity,
                       prosdk::units::radians_per_second_t rot) {
  SetVelocities(kinematics.ToWheelVelocities({xVelocity, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry() {
  odometry.Update(imu.GetRotation2d(),
                  prosdk::units::meter_t{leftEncoder.GetDistance()},
                  prosdk::units::meter_t{rightEncoder.GetDistance()});
}

void Drivetrain::ResetOdometry(const prosdk::math::Pose2d& pose) {
  drivetrainSimulator.SetPose(pose);
  odometry.ResetPosition(imu.GetRotation2d(),
                         prosdk::units::meter_t{leftEncoder.GetDistance()},
                         prosdk::units::meter_t{rightEncoder.GetDistance()}, pose);
}

void Drivetrain::SimulationPeriodic() {
  // To update our simulation, we set motor voltage inputs, update the
  // simulation, and write the simulated positions and velocities to our
  // simulated encoder and gyro. We negate the right side so that positive
  // voltages make the right side move forward.
  drivetrainSimulator.SetInputs(prosdk::units::volt_t{leftLeader.GetThrottle()} *
                                    prosdk::RobotController::GetInputVoltage(),
                                prosdk::units::volt_t{rightLeader.GetThrottle()} *
                                    prosdk::RobotController::GetInputVoltage());
  drivetrainSimulator.Update(20_ms);

  leftEncoderSim.SetDistance(drivetrainSimulator.GetLeftPosition().value());
  leftEncoderSim.SetRate(drivetrainSimulator.GetLeftVelocity().value());
  rightEncoderSim.SetDistance(drivetrainSimulator.GetRightPosition().value());
  rightEncoderSim.SetRate(drivetrainSimulator.GetRightVelocity().value());
  // gyroSim.SetAngle(-drivetrainSimulator.GetHeading().Degrees().value());
}

void Drivetrain::Periodic() {
  UpdateOdometry();
  fieldSim.SetRobotPose(odometry.GetPose());
  prosdk::telemetry::Log("Field", fieldSim);
}
