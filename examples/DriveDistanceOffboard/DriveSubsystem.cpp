// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveSubsystem.hpp"

#include <prosdk/system/RobotController.hpp>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : leftLeader{LEFT_MOTOR1_PORT},
      leftFollower{LEFT_MOTOR2_PORT},
      rightLeader{RIGHT_MOTOR1_PORT},
      rightFollower{RIGHT_MOTOR2_PORT},
      feedforward{ks, kv, ka} {
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  rightLeader.SetInverted(true);

  leftFollower.Follow(leftLeader);
  rightFollower.Follow(rightLeader);

  leftLeader.SetPID(kp, 0, 0);
  rightLeader.SetPID(kp, 0, 0);
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveSubsystem::SetDriveStates(
    prosdk::math::TrapezoidProfile<prosdk::units::meters>::State currentLeft,
    prosdk::math::TrapezoidProfile<prosdk::units::meters>::State currentRight,
    prosdk::math::TrapezoidProfile<prosdk::units::meters>::State nextLeft,
    prosdk::math::TrapezoidProfile<prosdk::units::meters>::State nextRight) {
  // Feedforward is divided by battery voltage to normalize it to [-1, 1]
  leftLeader.SetSetpoint(
      ExampleSmartMotorController::PIDMode::POSITION,
      currentLeft.position.value(),
      feedforward.Calculate(currentLeft.velocity, nextLeft.velocity) /
          prosdk::RobotController::GetBatteryVoltage());
  rightLeader.SetSetpoint(
      ExampleSmartMotorController::PIDMode::POSITION,
      currentRight.position.value(),
      feedforward.Calculate(currentRight.velocity, nextRight.velocity) /
          prosdk::RobotController::GetBatteryVoltage());
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  drive.ArcadeDrive(fwd, rot);
}

void DriveSubsystem::ResetEncoders() {
  leftLeader.ResetEncoder();
  rightLeader.ResetEncoder();
}

prosdk::units::meter_t DriveSubsystem::GetLeftEncoderDistance() {
  return prosdk::units::meter_t{leftLeader.GetEncoderDistance()};
}

prosdk::units::meter_t DriveSubsystem::GetRightEncoderDistance() {
  return prosdk::units::meter_t{rightLeader.GetEncoderDistance()};
}

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  drive.SetMaxOutput(maxOutput);
}

prosdk::cmd::CommandPtr DriveSubsystem::ProfiledDriveDistance(
    prosdk::units::meter_t distance) {
  return StartRun(
             [this] {
               // Restart timer so profile setpoints start at the beginning
               timer.Restart();
               ResetEncoders();
             },
             [this, distance] {
               // Current state never changes, so we need to use a timer to get
               // the setpoints we need to be at
               auto currentTime = timer.Get();
               auto currentSetpoint =
                   profile.Calculate(currentTime, {}, {distance, 0_mps});
               auto nextSetpoint =
                   profile.Calculate(currentTime + DT, {}, {distance, 0_mps});
               SetDriveStates(currentSetpoint, currentSetpoint, nextSetpoint,
                              nextSetpoint);
             })
      .Until([this] { return profile.IsFinished(0_s); });
}

prosdk::cmd::CommandPtr DriveSubsystem::DynamicProfiledDriveDistance(
    prosdk::units::meter_t distance) {
  return StartRun(
             [this] {
               // Restart timer so profile setpoints start at the beginning
               timer.Restart();
               // Store distance so we know the target distance for each encoder
               initialLeftDistance = GetLeftEncoderDistance();
               initialRightDistance = GetRightEncoderDistance();
             },
             [this, distance] {
               // Current state never changes for the duration of the command,
               // so we need to use a timer to get the setpoints we need to be
               // at
               auto currentTime = timer.Get();

               auto currentLeftSetpoint =
                   profile.Calculate(currentTime, {initialLeftDistance, 0_mps},
                                     {initialLeftDistance + distance, 0_mps});
               auto currentRightSetpoint =
                   profile.Calculate(currentTime, {initialRightDistance, 0_mps},
                                     {initialRightDistance + distance, 0_mps});

               auto nextLeftSetpoint = profile.Calculate(
                   currentTime + DT, {initialLeftDistance, 0_mps},
                   {initialLeftDistance + distance, 0_mps});
               auto nextRightSetpoint = profile.Calculate(
                   currentTime + DT, {initialRightDistance, 0_mps},
                   {initialRightDistance + distance, 0_mps});
               SetDriveStates(currentLeftSetpoint, currentRightSetpoint,
                              nextLeftSetpoint, nextRightSetpoint);
             })
      .Until([this] { return profile.IsFinished(0_s); });
}
