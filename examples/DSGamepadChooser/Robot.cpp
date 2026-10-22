// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

#include "BalanceAuto.hpp"
#include "DoNothingTeleop.hpp"
#include "ScoreAuto.hpp"

Robot::Robot() {
  AddOpMode<DoNothingTeleop>(prosdk::RobotMode::TELEOPERATED, "Do Nothing Teleop");
  AddOpMode<ScoreAuto>(prosdk::RobotMode::AUTONOMOUS, "Score Auto");
  AddOpMode<BalanceAuto>(prosdk::RobotMode::AUTONOMOUS, "Balance Auto");
  PublishOpModes();
}
