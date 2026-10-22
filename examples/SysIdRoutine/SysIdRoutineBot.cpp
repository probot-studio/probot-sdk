// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SysIdRoutineBot.hpp"

#include <prosdk/commands2/Commands.hpp>

SysIdRoutineBot::SysIdRoutineBot() {
  ConfigureBindings();
}

void SysIdRoutineBot::ConfigureBindings() {
  drive.SetDefaultCommand(drive.ArcadeDriveCommand(
      [this] { return -driverController.GetLeftY(); },
      [this] { return -driverController.GetRightX(); }));

  // Using bumpers as a modifier and combining it with the buttons so that we
  // can have both sets of bindings at once
  (driverController.FaceDown() && driverController.RightBumper())
      .WhileTrue(drive.SysIdQuasistatic(prosdk::cmd::sysid::Direction::FORWARD));
  (driverController.FaceRight() && driverController.RightBumper())
      .WhileTrue(drive.SysIdQuasistatic(prosdk::cmd::sysid::Direction::REVERSE));
  (driverController.FaceLeft() && driverController.RightBumper())
      .WhileTrue(drive.SysIdDynamic(prosdk::cmd::sysid::Direction::FORWARD));
  (driverController.FaceUp() && driverController.RightBumper())
      .WhileTrue(drive.SysIdDynamic(prosdk::cmd::sysid::Direction::REVERSE));

  shooter.SetDefaultCommand(shooter.RunShooterCommand(
      [this] { return driverController.GetLeftTrigger(); }));

  (driverController.FaceDown() && driverController.LeftBumper())
      .WhileTrue(shooter.SysIdQuasistatic(prosdk::cmd::sysid::Direction::FORWARD));
  (driverController.FaceRight() && driverController.LeftBumper())
      .WhileTrue(shooter.SysIdQuasistatic(prosdk::cmd::sysid::Direction::REVERSE));
  (driverController.FaceLeft() && driverController.LeftBumper())
      .WhileTrue(shooter.SysIdDynamic(prosdk::cmd::sysid::Direction::FORWARD));
  (driverController.FaceUp() && driverController.LeftBumper())
      .WhileTrue(shooter.SysIdDynamic(prosdk::cmd::sysid::Direction::REVERSE));
}

prosdk::cmd::CommandPtr SysIdRoutineBot::GetAutonomousCommand() {
  return drive.Run([] {});
}
