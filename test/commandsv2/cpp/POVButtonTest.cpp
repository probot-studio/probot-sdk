// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/button/POVButton.hpp"

#include <catch2/catch_test_macros.hpp>

#include "CommandTestBase.hpp"
#include "prosdk/commands2/CommandScheduler.hpp"
#include "prosdk/commands2/WaitUntilCommand.hpp"
#include "prosdk/driverstation/Joystick.hpp"
#include "prosdk/simulation/JoystickSim.hpp"

using namespace prosdk::cmd;
class POVButtonTest : public CommandTestBase {};

TEST_CASE_METHOD(POVButtonTest, "POVButtonTest SetPOV",
                 "[commandsv2][command]") {
  prosdk::sim::JoystickSim joysim(1);
  joysim.SetPOV(prosdk::POVDirection::UP);
  joysim.NotifyNewData();

  auto& scheduler = CommandScheduler::GetInstance();
  bool finished = false;

  WaitUntilCommand command([&finished] { return finished; });

  prosdk::Joystick joy(1);
  POVButton(&joy, prosdk::POVDirection::RIGHT).OnTrue(&command);
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(&command));

  joysim.SetPOV(prosdk::POVDirection::RIGHT);
  joysim.NotifyNewData();

  scheduler.Run();
  CHECK(scheduler.IsScheduled(&command));
  finished = true;
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(&command));
}
