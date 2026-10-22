// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CommandTestBase.hpp"

using namespace prosdk::cmd;

CommandTestBase::CommandTestBase() {
  auto& scheduler = CommandScheduler::GetInstance();
  scheduler.CancelAll();
  scheduler.Enable();
  scheduler.GetActiveButtonLoop()->Clear();
  scheduler.UnregisterAllSubsystems();

  SetDSEnabled(true);
}

CommandTestBase::~CommandTestBase() {
  CommandScheduler::GetInstance().GetActiveButtonLoop()->Clear();
  CommandScheduler::GetInstance().UnregisterAllSubsystems();
}

CommandScheduler CommandTestBase::GetScheduler() {
  return CommandScheduler();
}

void CommandTestBase::SetDSEnabled(bool enabled) {
  prosdk::sim::DriverStationSim::SetDsAttached(true);
  prosdk::sim::DriverStationSim::SetEnabled(enabled);
  prosdk::sim::DriverStationSim::NotifyNewData();
}
