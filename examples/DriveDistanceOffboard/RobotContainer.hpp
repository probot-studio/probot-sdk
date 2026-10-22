// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.hpp"
#include "DriveSubsystem.hpp"
#include <prosdk/commands2/Command.hpp>
#include <prosdk/commands2/CommandPtr.hpp>
#include <prosdk/commands2/Commands.hpp>
#include <prosdk/commands2/button/CommandGamepad.hpp>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  prosdk::cmd::CommandPtr GetAutonomousCommand();

 private:
  // The driver's controller
  prosdk::cmd::CommandGamepad driverController{
      OIConstants::DRIVER_CONTROLLER_PORT};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem drive;

  // RobotContainer-owned commands
  prosdk::cmd::CommandPtr driveHalfVelocity =
      prosdk::cmd::RunOnce([this] { drive.SetMaxOutput(0.5); }, {});
  prosdk::cmd::CommandPtr driveFullVelocity =
      prosdk::cmd::RunOnce([this] { drive.SetMaxOutput(1); }, {});

  void ConfigureButtonBindings();
};
