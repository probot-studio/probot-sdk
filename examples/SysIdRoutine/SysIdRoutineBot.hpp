// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.hpp"
#include "Drive.hpp"
#include "Shooter.hpp"
#include <prosdk/commands2/CommandPtr.hpp>
#include <prosdk/commands2/button/CommandGamepad.hpp>

class SysIdRoutineBot {
 public:
  SysIdRoutineBot();

  prosdk::cmd::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  prosdk::cmd::CommandGamepad driverController{
      constants::oi::DRIVER_CONTROLLER_PORT};
  Drive drive;
  Shooter shooter;
};
