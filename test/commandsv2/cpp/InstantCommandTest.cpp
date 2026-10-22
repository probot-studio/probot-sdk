// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/InstantCommand.hpp"

#include "CommandTestBase.hpp"
#include "prosdk/commands2/Commands.hpp"

using namespace prosdk::cmd;
class InstantCommandTest : public CommandTestBase {};

TEST_CASE_METHOD(InstantCommandTest,
                 "InstantCommandTest InstantCommandSchedule",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();

  int counter = 0;

  auto command = RunOnce([&counter] { counter++; });

  scheduler.Schedule(command);
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(command));
  CHECK(counter == 1);
}
