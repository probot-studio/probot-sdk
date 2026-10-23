// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <LittleFS.h>

#include "SysIdRoutineBot.hpp"
#include <prosdk/commands2/CommandPtr.hpp>
#include <prosdk/framework/TimedRobot.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void UtilityInit() override;
  void UtilityPeriodic() override;
  void UtilityExit() override;

 private:
  // first member: mount the log storage (/littlefs/logs) before anything logs
  bool m_fs = LittleFS.begin(true);

  std::optional<prosdk::cmd::CommandPtr> autonomousCommand;

  SysIdRoutineBot container;
};
