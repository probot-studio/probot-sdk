// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.hpp"
#include "ExampleSmartMotorController.hpp"
#include <prosdk/commands2/CommandPtr.hpp>
#include <prosdk/commands2/SubsystemBase.hpp>
#include <prosdk/drive/DifferentialDrive.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/math/trajectory/TrapezoidProfile.hpp>
#include <prosdk/system/Timer.hpp>
#include <prosdk/units/length.hpp>

class DriveSubsystem : public prosdk::cmd::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Attempts to follow the given drive states using offboard PID.
   *
   * @param currentLeft The current left wheel state.
   * @param currentRight The current right wheel state.
   * @param nextLeft The next left wheel state.
   * @param nextRight The next right wheel state.
   */
  void SetDriveStates(
      prosdk::math::TrapezoidProfile<prosdk::units::meters>::State currentLeft,
      prosdk::math::TrapezoidProfile<prosdk::units::meters>::State currentRight,
      prosdk::math::TrapezoidProfile<prosdk::units::meters>::State nextLeft,
      prosdk::math::TrapezoidProfile<prosdk::units::meters>::State nextRight);

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  void ArcadeDrive(double fwd, double rot);

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Gets the distance of the left encoder.
   *
   * @return the average of the TWO encoder readings
   */
  prosdk::units::meter_t GetLeftEncoderDistance();

  /**
   * Gets the distance of the right encoder.
   *
   * @return the average of the TWO encoder readings
   */
  prosdk::units::meter_t GetRightEncoderDistance();

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive
   * more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  void SetMaxOutput(double maxOutput);

  /**
   * Creates a command to drive forward a specified distance using a motion
   * profile.
   *
   * @param distance The distance to drive forward.
   * @return A command.
   */
  prosdk::cmd::CommandPtr ProfiledDriveDistance(prosdk::units::meter_t distance);

  /**
   * Creates a command to drive forward a specified distance using a motion
   * profile without resetting the encoders.
   *
   * @param distance The distance to drive forward.
   * @return A command.
   */
  prosdk::cmd::CommandPtr DynamicProfiledDriveDistance(
      prosdk::units::meter_t distance);

 private:
  prosdk::math::TrapezoidProfile<prosdk::units::meters> profile{
      {DriveConstants::MAX_VELOCITY, DriveConstants::MAX_ACCELERATION}};
  prosdk::Timer timer;
  prosdk::units::meter_t initialLeftDistance;
  prosdk::units::meter_t initialRightDistance;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  ExampleSmartMotorController leftLeader;
  ExampleSmartMotorController leftFollower;
  ExampleSmartMotorController rightLeader;
  ExampleSmartMotorController rightFollower;

  // A feedforward component for the drive
  prosdk::math::SimpleMotorFeedforward<prosdk::units::meters> feedforward;

  // The robot's drive
  prosdk::DifferentialDrive drive{[&](double output) { leftLeader.Set(output); },
                               [&](double output) { rightLeader.Set(output); }};
};
