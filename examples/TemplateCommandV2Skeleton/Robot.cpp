// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

#include <prosdk/commands2/CommandScheduler.hpp>

Robot::Robot() {}

void Robot::RobotPeriodic() {
  prosdk::cmd::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  autonomousCommand = container.GetAutonomousCommand();

  if (autonomousCommand) {
    prosdk::cmd::CommandScheduler::GetInstance().Schedule(
        autonomousCommand.value());
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {
  if (autonomousCommand) {
    autonomousCommand->Cancel();
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::UtilityInit() {
  prosdk::cmd::CommandScheduler::GetInstance().CancelAll();
}

void Robot::UtilityPeriodic() {}

void Robot::UtilityExit() {}
