// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// The command framework on the probot-core dashboard backends (fake core store): what the Driver
// Station sees of the CommandScheduler, of a Command and of a NetworkButton, and what its writes
// do. Replaces the two upstream test cases that need the NetworkTables tunable backend
// (CommandScheduleTest NetworkTablesTunableCancelClearsValue, CommandTunableButtonTest
// NetworkTablesLifecycleMatchesGlassRunCancel); a Driver Station write is fake::PushWrite.

#include <stdint.h>

#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "DashboardFixture.hpp"
#include "prosdk/commands2/CommandScheduler.hpp"
#include "prosdk/commands2/Commands.hpp"
#include "prosdk/commands2/FunctionalCommand.hpp"
#include "prosdk/commands2/WaitCommand.hpp"
#include "prosdk/commands2/button/NetworkButton.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/tunables/Tunables.hpp"

using dashboard_test::DashboardFixture;
namespace dash = prosdk::platform::dashboard;
using namespace prosdk::cmd;

namespace {

uint8_t VT(dash::ValueType vt) {
  return static_cast<uint8_t>(vt);
}

std::vector<uint8_t> EncodeIntArray(const std::vector<int64_t>& values) {
  std::vector<uint8_t> out;
  for (int64_t v : values) {
    auto p = reinterpret_cast<const uint8_t*>(&v);
    out.insert(out.end(), p, p + sizeof(v));
  }
  return out;
}

int64_t IdOf(const Command& command) {
  return static_cast<int64_t>(reinterpret_cast<uintptr_t>(&command));
}

class CommandsDashboardFixture : public DashboardFixture {
 public:
  CommandsDashboardFixture() {
    auto& scheduler = CommandScheduler::GetInstance();
    scheduler.CancelAll();
    scheduler.Enable();
    scheduler.GetActiveButtonLoop()->Clear();
    scheduler.UnregisterAllSubsystems();
    SetEnabled(true);
  }

  ~CommandsDashboardFixture() {
    auto& scheduler = CommandScheduler::GetInstance();
    scheduler.CancelAll();
    scheduler.GetActiveButtonLoop()->Clear();
    scheduler.UnregisterAllSubsystems();
  }

  static void SetEnabled(bool enabled) {
    prosdk::sim::DriverStationSim::SetDsAttached(true);
    prosdk::sim::DriverStationSim::SetEnabled(enabled);
    prosdk::sim::DriverStationSim::NotifyNewData();
  }
};

FunctionalCommand Forever() {
  return FunctionalCommand{[] {}, [] {}, [](bool) {}, [] { return false; }};
}

}  // namespace

TEST_CASE_METHOD(CommandsDashboardFixture, "Scheduler telemetry reaches the Driver Station",
                 "[prosdk][commands2][dashboard]") {
  auto& scheduler = CommandScheduler::GetInstance();
  auto command = Forever();
  command.SetName("Drive");
  scheduler.Schedule(&command);
  scheduler.Run();

  prosdk::telemetry::Log("Scheduler", scheduler);
  Cycle();

  CHECK(fake::GetString("/Telemetry/Scheduler/.type") == "Scheduler");
  CHECK(fake::GetStringArray("/Telemetry/Scheduler/Names") == std::vector<std::string>{"Drive"});
  CHECK(fake::GetIntArray("/Telemetry/Scheduler/Ids") == std::vector<int64_t>{IdOf(command)});

  scheduler.Cancel(&command);
  prosdk::telemetry::Log("Scheduler", scheduler);
  Cycle();
  CHECK(fake::GetStringArray("/Telemetry/Scheduler/Names") == std::vector<std::string>{});
}

TEST_CASE_METHOD(CommandsDashboardFixture,
                 "Scheduler tunable: Driver Station cancel by id clears the value",
                 "[prosdk][commands2][dashboard]") {
  auto& scheduler = CommandScheduler::GetInstance();
  // Cancel holds raw Command* addresses: robust, never persistent (a stored record is ignored)
  fake::SetPersisted("/Tunables/Scheduler/Cancel", VT(dash::ValueType::kIntArray), "int[]",
                     EncodeIntArray({12345}));
  REQUIRE(prosdk::tunables::Publish("Scheduler", scheduler));
  CHECK(fake::GetString("/Tunables/Scheduler/.type") == "Scheduler");
  auto cancel = fake::Find("/Tunables/Scheduler/Cancel");
  REQUIRE(cancel);
  CHECK(cancel->valueType == VT(dash::ValueType::kIntArray));
  CHECK((cancel->flags & (dash::kFlagTunable | dash::kFlagMutable | dash::kFlagRobust)) ==
        (dash::kFlagTunable | dash::kFlagMutable | dash::kFlagRobust));
  CHECK((cancel->flags & dash::kFlagPersistent) == 0);
  CHECK(fake::GetIntArray("/Tunables/Scheduler/Cancel") == std::vector<int64_t>{});

  auto command = Forever();
  command.SetName("Intake");
  scheduler.Schedule(&command);
  scheduler.Run();
  Cycle();
  CHECK(fake::GetStringArray("/Tunables/Scheduler/Names") == std::vector<std::string>{"Intake"});
  CHECK(fake::GetIntArray("/Tunables/Scheduler/Ids") == std::vector<int64_t>{IdOf(command)});

  fake::PushWrite("/Tunables/Scheduler/Cancel", VT(dash::ValueType::kIntArray),
                  EncodeIntArray({IdOf(command)}));
  Cycle();
  CHECK_FALSE(scheduler.IsScheduled(&command));

  Cycle();
  CHECK(fake::GetIntArray("/Tunables/Scheduler/Cancel") == std::vector<int64_t>{});
  CHECK(fake::GetStringArray("/Tunables/Scheduler/Names") == std::vector<std::string>{});
  prosdk::tunables::Remove("Scheduler");
}

TEST_CASE_METHOD(CommandsDashboardFixture,
                 "Command tunable: Driver Station run and cancel follow the lifecycle",
                 "[prosdk][commands2][dashboard]") {
  int initializeCount = 0;
  int executeCount = 0;
  int interruptedEndCount = 0;
  int finishedEndCount = 0;
  FunctionalCommand command{[&] { ++initializeCount; }, [&] { ++executeCount; },
                            [&](bool interrupted) {
                              if (interrupted) {
                                ++interruptedEndCount;
                              } else {
                                ++finishedEndCount;
                              }
                            },
                            [] { return false; }};
  command.SetName("DS Label");
  REQUIRE(prosdk::tunables::Publish("command", command));

  // the Driver Station "Command" widget layout (core-requests.md R5)
  CHECK(fake::GetString("/Tunables/command/.type") == "Command");
  auto name = fake::Find("/Tunables/command/name");
  REQUIRE(name);
  CHECK((name->flags & dash::kFlagMutable) == 0);
  CHECK(fake::GetString("/Tunables/command/name") == "DS Label");
  auto running = fake::Find("/Tunables/command/running");
  REQUIRE(running);
  CHECK(running->valueType == VT(dash::ValueType::kBoolean));
  CHECK((running->flags & (dash::kFlagTunable | dash::kFlagMutable)) ==
        (dash::kFlagTunable | dash::kFlagMutable));
  CHECK(fake::GetBoolean("/Tunables/command/running") == false);

  fake::PushWrite("/Tunables/command/running", VT(dash::ValueType::kBoolean),
                  fake::EncodeBoolean(true));
  Cycle();
  CHECK(command.IsScheduled());
  CHECK(fake::GetBoolean("/Tunables/command/running") == true);
  CHECK(1 == initializeCount);
  CHECK(0 == executeCount);

  CommandScheduler::GetInstance().Run();
  CHECK(1 == executeCount);
  Cycle();
  CHECK(fake::GetBoolean("/Tunables/command/running") == true);

  command.SetName("Updated Label");
  Cycle();
  CHECK(fake::GetString("/Tunables/command/name") == "Updated Label");

  fake::PushWrite("/Tunables/command/running", VT(dash::ValueType::kBoolean),
                  fake::EncodeBoolean(false));
  Cycle();
  CHECK_FALSE(command.IsScheduled());
  CHECK(fake::GetBoolean("/Tunables/command/running") == false);
  CHECK(1 == initializeCount);
  CHECK(1 == interruptedEndCount);
  CHECK(0 == finishedEndCount);

  // the command stops on its own: the Driver Station sees running = false
  fake::PushWrite("/Tunables/command/running", VT(dash::ValueType::kBoolean),
                  fake::EncodeBoolean(true));
  Cycle();
  REQUIRE(command.IsScheduled());
  CommandScheduler::GetInstance().Cancel(&command);
  Cycle();
  CHECK(fake::GetBoolean("/Tunables/command/running") == false);
  prosdk::tunables::Remove("command");
}

TEST_CASE_METHOD(CommandsDashboardFixture, "NetworkButton is a Driver Station tunable",
                 "[prosdk][commands2][dashboard]") {
  auto& scheduler = CommandScheduler::GetInstance();
  int runs = 0;
  auto command = RunOnce([&] { ++runs; });
  {
    NetworkButton("SmartDashboard", "Shoot").OnTrue(std::move(command));
  }
  auto topic = fake::Find("/Tunables/SmartDashboard/Shoot");
  REQUIRE(topic);
  CHECK(topic->valueType == VT(dash::ValueType::kBoolean));
  CHECK((topic->flags & (dash::kFlagTunable | dash::kFlagMutable)) ==
        (dash::kFlagTunable | dash::kFlagMutable));
  CHECK(fake::GetBoolean("/Tunables/SmartDashboard/Shoot") == false);

  scheduler.Run();
  CHECK(0 == runs);
  fake::PushWrite("/Tunables/SmartDashboard/Shoot", VT(dash::ValueType::kBoolean),
                  fake::EncodeBoolean(true));
  Cycle();
  scheduler.Run();
  CHECK(1 == runs);
  scheduler.Run();
  CHECK(1 == runs);  // OnTrue: rising edge only

  // the binding owns the button: clearing the loop removes the topic
  scheduler.GetActiveButtonLoop()->Clear();
  Cycle();
  CHECK(fake::Find("/Tunables/SmartDashboard/Shoot") == nullptr);
}
