// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "CommandTestBase.hpp"
#include "prosdk/commands2/FunctionalCommand.hpp"
#include "prosdk/commands2/InstantCommand.hpp"
#include "prosdk/commands2/RunCommand.hpp"
#include "prosdk/tunables/MockTunableBackend.hpp"
#include "prosdk/tunables/TunableConfig.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/tunables/detail/TunableMember.hpp"

using namespace prosdk::cmd;
class CommandScheduleTest : public CommandTestBase {};

TEST_CASE_METHOD(CommandScheduleTest, "CommandScheduleTest InstantSchedule",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();
  MockCommand command;

  command.ExpectInitialize(1);
  command.ExpectExecute(1);
  command.ExpectEnd(false, 1);

  command.SetFinished(true);
  scheduler.Schedule(&command);
  CHECK(scheduler.IsScheduled(&command));
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(&command));
}

TEST_CASE_METHOD(CommandScheduleTest,
                 "CommandScheduleTest SingleIterationSchedule",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();
  MockCommand command;

  command.ExpectInitialize(1);
  command.ExpectExecute(2);
  command.ExpectEnd(false, 1);

  scheduler.Schedule(&command);
  CHECK(scheduler.IsScheduled(&command));
  scheduler.Run();
  command.SetFinished(true);
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(&command));
}

TEST_CASE_METHOD(CommandScheduleTest, "CommandScheduleTest MultiSchedule",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();
  MockCommand command1;
  MockCommand command2;
  MockCommand command3;

  command1.ExpectInitialize(1);
  command1.ExpectExecute(2);
  command1.ExpectEnd(false, 1);

  command2.ExpectInitialize(1);
  command2.ExpectExecute(3);
  command2.ExpectEnd(false, 1);

  command3.ExpectInitialize(1);
  command3.ExpectExecute(4);
  command3.ExpectEnd(false, 1);

  scheduler.Schedule(&command1);
  scheduler.Schedule(&command2);
  scheduler.Schedule(&command3);
  CHECK(scheduler.IsScheduled({&command1, &command2, &command3}));
  scheduler.Run();
  CHECK(scheduler.IsScheduled({&command1, &command2, &command3}));
  command1.SetFinished(true);
  scheduler.Run();
  CHECK(scheduler.IsScheduled({&command2, &command3}));
  CHECK_FALSE(scheduler.IsScheduled(&command1));
  command2.SetFinished(true);
  scheduler.Run();
  CHECK(scheduler.IsScheduled(&command3));
  CHECK_FALSE(scheduler.IsScheduled({&command1, &command2}));
  command3.SetFinished(true);
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled({&command1, &command2, &command3}));
}

TEST_CASE_METHOD(CommandScheduleTest, "CommandScheduleTest SchedulerCancel",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();
  MockCommand command;

  command.ExpectInitialize(1);
  command.ExpectExecute(1);
  command.ExpectEnd(false, 0);
  command.ExpectEnd(true, 1);

  scheduler.Schedule(&command);
  scheduler.Run();
  CHECK(scheduler.IsScheduled(&command));
  scheduler.Cancel(&command);
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(&command));
}

TEST_CASE_METHOD(CommandScheduleTest,
                 "CommandScheduleTest CommandKnowsWhenItEnded",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();

  prosdk::cmd::FunctionalCommand* commandPtr = nullptr;
  auto command = prosdk::cmd::FunctionalCommand(
      [] {}, [] {},
      [&](auto isForced) { CHECK_FALSE(scheduler.IsScheduled(commandPtr)); },
      [] { return true; });
  commandPtr = &command;

  scheduler.Schedule(commandPtr);
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(commandPtr));
}

TEST_CASE_METHOD(CommandScheduleTest,
                 "CommandScheduleTest ScheduleCommandInCommand",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();
  int counter = 0;
  prosdk::cmd::InstantCommand commandToGetScheduled{[&counter] { counter++; }};

  auto command =
      prosdk::cmd::RunCommand([&counter, &scheduler, &commandToGetScheduled] {
        scheduler.Schedule(&commandToGetScheduled);
        CHECK(counter == 1);
      });

  scheduler.Schedule(&command);
  scheduler.Run();
  CHECK(counter == 1);
  CHECK(scheduler.IsScheduled(&commandToGetScheduled));

  scheduler.Run();
  CHECK(counter == 1);
  CHECK_FALSE(scheduler.IsScheduled(&commandToGetScheduled));
}

TEST_CASE_METHOD(CommandScheduleTest, "CommandScheduleTest NotScheduledCancel",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();
  MockCommand command;

  CHECK_NOTHROW(scheduler.Cancel(&command));
}

TEST_CASE_METHOD(CommandScheduleTest, "CommandScheduleTest TunableCancel",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();
  auto backend = std::make_shared<prosdk::tunables::MockTunableBackend>();
  prosdk::tunables::TunableRegistry::RegisterBackend("", backend);
  prosdk::tunables::Publish("Scheduler", scheduler);

  auto namesUid = backend->GetUid("/Scheduler/Names");
  CHECK(namesUid);
  auto namesInfo = prosdk::tunables::TunableRegistry::GetTunable(*namesUid);
  REQUIRE(namesInfo);
  REQUIRE(namesInfo.config);
  CHECK_FALSE(namesInfo.config->isMutable);
  CHECK(namesInfo.config->polling ==
        prosdk::tunables::TunableConfig::Polling::kAlwaysGet);

  auto idsUid = backend->GetUid("/Scheduler/Ids");
  REQUIRE(idsUid);
  auto idsInfo = prosdk::tunables::TunableRegistry::GetTunable(*idsUid);
  REQUIRE(idsInfo);
  REQUIRE(idsInfo.config);
  CHECK_FALSE(idsInfo.config->isMutable);
  CHECK(idsInfo.config->polling ==
        prosdk::tunables::TunableConfig::Polling::kAlwaysGet);

  MockCommand command;
  scheduler.Schedule(&command);
  scheduler.Run();
  CHECK(scheduler.IsScheduled(&command));
  prosdk::tunables::TunableRegistry::Update();

  const auto& names = static_cast<prosdk::tunables::detail::TunableMemberValueBase<
      std::vector<std::string>>*>(namesInfo.tunable)
                          ->Get(namesInfo.config->parent);
  REQUIRE(1U == names.size());
  CHECK(command.GetName() == names[0]);

  const auto& ids =
      static_cast<
          prosdk::tunables::detail::TunableMemberValueBase<std::vector<int64_t>>*>(
          idsInfo.tunable)
          ->Get(idsInfo.config->parent);
  REQUIRE(1U == ids.size());

  uintptr_t ptrTmp = reinterpret_cast<uintptr_t>(&command);
  CHECK(static_cast<int64_t>(ptrTmp) == ids[0]);
  backend->SetInt64Vector(
      "/Scheduler/Cancel",
      std::span<const int64_t>{{static_cast<int64_t>(ptrTmp)}});
  prosdk::tunables::TunableRegistry::Update();
  scheduler.Run();
  CHECK_FALSE(scheduler.IsScheduled(&command));
  prosdk::tunables::TunableRegistry::Reset();
}

// prosdk: "CommandScheduleTest NetworkTablesTunableCancelClearsValue" needs the
// NetworkTables tunable backend (ntcore is not ported). The same behavior on
// the Driver Station backend: test/prosdk/commands2/DashboardCommandsTest.cpp.
