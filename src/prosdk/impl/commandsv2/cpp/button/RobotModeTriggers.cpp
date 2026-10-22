// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/button/RobotModeTriggers.hpp"

#include "prosdk/driverstation/RobotState.hpp"

using namespace prosdk::cmd;

Trigger RobotModeTriggers::Autonomous() {
  return Trigger{&prosdk::RobotState::IsAutonomousEnabled};
}

Trigger RobotModeTriggers::Teleop() {
  return Trigger{&prosdk::RobotState::IsTeleopEnabled};
}

Trigger RobotModeTriggers::Disabled() {
  return Trigger{&prosdk::RobotState::IsDisabled};
}

Trigger RobotModeTriggers::Utility() {
  return Trigger{&prosdk::RobotState::IsUtilityEnabled};
}
