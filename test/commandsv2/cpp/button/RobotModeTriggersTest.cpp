// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/button/RobotModeTriggers.hpp"

#include "../CommandTestBase.hpp"
#include "prosdk/commands2/button/Trigger.hpp"
#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/simulation/DriverStationSim.hpp"

using namespace prosdk::cmd;
using namespace prosdk::sim;
class RobotModeTriggersTest : public CommandTestBase {};

TEST_CASE("RobotModeTriggersTest Autonomous", "[commandsv2][command]") {
  DriverStationSim::ResetData();
  DriverStationSim::SetRobotMode(prosdk::hal::RobotMode::AUTONOMOUS);
  DriverStationSim::SetEnabled(true);
  DriverStationSim::NotifyNewData();
  prosdk::internal::DriverStationBackend::ObserveUserProgramStarting();
  Trigger autonomous = RobotModeTriggers::Autonomous();
  CHECK(autonomous.Get());
}

TEST_CASE("RobotModeTriggersTest Teleop", "[commandsv2][command]") {
  DriverStationSim::ResetData();
  DriverStationSim::SetRobotMode(prosdk::hal::RobotMode::TELEOPERATED);
  DriverStationSim::SetEnabled(true);
  DriverStationSim::NotifyNewData();
  prosdk::internal::DriverStationBackend::ObserveUserProgramStarting();
  Trigger teleop = RobotModeTriggers::Teleop();
  CHECK(teleop.Get());
}

TEST_CASE("RobotModeTriggersTest Disabled", "[commandsv2][command]") {
  DriverStationSim::ResetData();
  DriverStationSim::SetEnabled(false);
  DriverStationSim::NotifyNewData();
  Trigger disabled = RobotModeTriggers::Disabled();
  prosdk::internal::DriverStationBackend::ObserveUserProgramStarting();
  CHECK(disabled.Get());
}

TEST_CASE("RobotModeTriggersTest UtilityMode", "[commandsv2][command]") {
  DriverStationSim::ResetData();
  DriverStationSim::SetRobotMode(prosdk::hal::RobotMode::UTILITY);
  DriverStationSim::SetEnabled(true);
  DriverStationSim::NotifyNewData();
  Trigger test = RobotModeTriggers::Utility();
  prosdk::internal::DriverStationBackend::ObserveUserProgramStarting();
  CHECK(test.Get());
}
