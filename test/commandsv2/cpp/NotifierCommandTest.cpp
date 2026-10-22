// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/NotifierCommand.hpp"

#include "CommandTestBase.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk::cmd;
using namespace std::chrono_literals;

class NotifierCommandTest : public CommandTestBase {};

TEST_CASE_METHOD(NotifierCommandTest,
                 "NotifierCommandTest NotifierCommandSchedule",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();

  prosdk::sim::PauseTiming();

  int counter = 0;
  NotifierCommand command([&] { counter++; }, 10_ms, {});

  scheduler.Schedule(&command);
  for (int i = 0; i < 5; ++i) {
    prosdk::sim::StepTiming(5_ms);
  }
  scheduler.Cancel(&command);

  prosdk::sim::ResumeTiming();

  CHECK(counter == 2);
}
