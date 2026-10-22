// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <prosdk/commands2/CommandPtr.hpp>

class RobotContainer {
 public:
  RobotContainer();

  prosdk::cmd::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
};
