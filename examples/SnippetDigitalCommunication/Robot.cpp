// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

#include <prosdk/driverstation/MatchState.hpp>
#include <prosdk/driverstation/RobotState.hpp>

void Robot::RobotPeriodic() {
  // pull alliance port high if on red alliance, pull low if on blue alliance
  allianceOutput.Set(prosdk::MatchState::GetAlliance() == prosdk::Alliance::RED);

  // pull enabled port high if enabled, low if disabled
  enabledOutput.Set(prosdk::RobotState::IsEnabled());

  // pull auto port high if in autonomous, low if in teleop (or disabled)
  autonomousOutput.Set(prosdk::RobotState::IsAutonomous());

  // pull alert port high if match time remaining is between 30 and 25 seconds
  auto matchTime = prosdk::MatchState::GetMatchTime();
  alertOutput.Set(matchTime <= 30_s && matchTime >= 25_s);
}
