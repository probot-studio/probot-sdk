// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

#include "DefaultTeleop.hpp"

Robot::Robot() {
  AddOpMode<DefaultTeleop>(prosdk::RobotMode::TELEOPERATED, "Default Teleop");
  PublishOpModes();
}
