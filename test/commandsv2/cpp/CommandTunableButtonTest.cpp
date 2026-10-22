// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <memory>

#include "CommandTestBase.hpp"
#include "prosdk/commands2/Commands.hpp"
#include "prosdk/commands2/FunctionalCommand.hpp"
#include "prosdk/tunables/MockTunableBackend.hpp"
#include "prosdk/tunables/TunableConfig.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/tunables/Tunables.hpp"

using namespace prosdk::cmd;

class CommandTunableButtonTest : public CommandTestBase {
 protected:
  CommandTunableButtonTest() {
    m_schedule = 0;
    m_cancel = 0;
    m_backend = std::make_shared<prosdk::tunables::MockTunableBackend>();
    prosdk::tunables::TunableRegistry::RegisterBackend("", m_backend);
    m_command = StartEnd([this] { m_schedule++; }, [this] { m_cancel++; });
    prosdk::tunables::Publish("command", *m_command->get());
  }

  ~CommandTunableButtonTest() override {
    prosdk::tunables::TunableRegistry::Reset();
  }

  int m_schedule;
  int m_cancel;
  std::shared_ptr<prosdk::tunables::MockTunableBackend> m_backend;
  std::optional<CommandPtr> m_command;
};

TEST_CASE_METHOD(CommandTunableButtonTest,
                 "CommandTunableButtonTest trueAndNotScheduledSchedules",
                 "[commandsv2][command]") {
  // Not scheduled and true -> scheduled
  GetScheduler().Run();
  CHECK_FALSE(m_command->IsScheduled());
  CHECK(0 == m_schedule);
  CHECK(0 == m_cancel);

  m_backend->SetBool("/command/running", true);
  prosdk::tunables::TunableRegistry::Update();
  GetScheduler().Run();
  CHECK(m_command->IsScheduled());
  CHECK(1 == m_schedule);
  CHECK(0 == m_cancel);
}

TEST_CASE_METHOD(
    CommandTunableButtonTest,
    "CommandTunableButtonTest runningTunableAlwaysGetsSchedulerState",
    "[commandsv2][command]") {
  auto uid = m_backend->GetUid("/command/running");
  REQUIRE(uid);

  auto info = prosdk::tunables::TunableRegistry::GetTunable(*uid);
  REQUIRE(info);
  REQUIRE(info.config != nullptr);
  CHECK(info.config->polling ==
        prosdk::tunables::TunableConfig::Polling::kAlwaysGet);
}

TEST_CASE_METHOD(CommandTunableButtonTest,
                 "CommandTunableButtonTest nameTunablePublishesCommandName",
                 "[commandsv2][command]") {
  m_command->get()->SetName("Renamed Command");
  prosdk::tunables::TunableRegistry::Update();

  auto uid = m_backend->GetUid("/command/name");
  REQUIRE(uid);

  auto info = prosdk::tunables::TunableRegistry::GetTunable(*uid);
  REQUIRE(info);
  REQUIRE(info.config != nullptr);
  CHECK_FALSE(info.config->isMutable);
  CHECK(info.config->polling ==
        prosdk::tunables::TunableConfig::Polling::kGetOnChange);
  CHECK("Renamed Command" == m_backend->GetString("/command/name"));
}

TEST_CASE_METHOD(CommandTunableButtonTest,
                 "CommandTunableButtonTest trueAndScheduledNoOp",
                 "[commandsv2][command]") {
  // Scheduled and true -> no-op
  prosdk::cmd::CommandScheduler::GetInstance().Schedule(m_command.value());
  GetScheduler().Run();
  CHECK(m_command->IsScheduled());
  CHECK(1 == m_schedule);
  CHECK(0 == m_cancel);

  m_backend->SetBool("/command/running", true);
  prosdk::tunables::TunableRegistry::Update();
  GetScheduler().Run();
  CHECK(m_command->IsScheduled());
  CHECK(1 == m_schedule);
  CHECK(0 == m_cancel);
}

TEST_CASE_METHOD(CommandTunableButtonTest,
                 "CommandTunableButtonTest falseAndNotScheduledNoOp",
                 "[commandsv2][command]") {
  // Not scheduled and false -> no-op
  GetScheduler().Run();
  CHECK_FALSE(m_command->IsScheduled());
  CHECK(0 == m_schedule);
  CHECK(0 == m_cancel);

  m_backend->SetBool("/command/running", false);
  prosdk::tunables::TunableRegistry::Update();
  GetScheduler().Run();
  CHECK_FALSE(m_command->IsScheduled());
  CHECK(0 == m_schedule);
  CHECK(0 == m_cancel);
}

TEST_CASE_METHOD(CommandTunableButtonTest,
                 "CommandTunableButtonTest falseAndScheduledCancel",
                 "[commandsv2][command]") {
  // Scheduled and false -> cancel
  prosdk::cmd::CommandScheduler::GetInstance().Schedule(m_command.value());
  GetScheduler().Run();
  CHECK(m_command->IsScheduled());
  CHECK(1 == m_schedule);
  CHECK(0 == m_cancel);

  m_backend->SetBool("/command/running", false);
  prosdk::tunables::TunableRegistry::Update();
  GetScheduler().Run();
  CHECK_FALSE(m_command->IsScheduled());
  CHECK(1 == m_schedule);
  CHECK(1 == m_cancel);
}

// prosdk: "CommandTunableButtonTest NetworkTablesLifecycleMatchesGlassRunCancel"
// needs the NetworkTables tunable backend (ntcore is not ported). The same
// lifecycle on the Driver Station backend:
// test/prosdk/commands2/DashboardCommandsTest.cpp.
