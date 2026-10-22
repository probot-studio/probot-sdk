// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/CommandScheduler.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "prosdk/commands2/CommandPtr.hpp"
#include "prosdk/commands2/Subsystem.hpp"
#include "prosdk/driverstation/RobotState.hpp"
#include "prosdk/framework/RobotBase.hpp"
#include "prosdk/framework/TimedRobot.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/tunables/TunableConfig.hpp"
#include "prosdk/tunables/TunableTable.hpp"
#include "prosdk/util/DenseMap.hpp"
#include "prosdk/util/SmallVector.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk::cmd;

class CommandScheduler::Impl {
 public:
  // A set of the currently-running commands.
  prosdk::util::SmallSet<Command*, 12> scheduledCommands;

  // A map from required subsystems to their requiring commands.  Also used as a
  // set of the currently-required subsystems.
  prosdk::util::DenseMap<Subsystem*, Command*> requirements;

  // A map from subsystems registered with the scheduler to their default
  // commands.  Also used as a list of currently-registered subsystems.
  prosdk::util::DenseMap<Subsystem*, std::unique_ptr<Command>> subsystems;

  prosdk::EventLoop defaultButtonLoop;
  // The set of currently-registered buttons that will be polled every
  // iteration.
  prosdk::EventLoop* activeButtonLoop{&defaultButtonLoop};

  bool disabled{false};

  // Lists of user-supplied actions to be executed on scheduling events for
  // every command.
  prosdk::util::SmallVector<Action, 4> initActions;
  prosdk::util::SmallVector<Action, 4> executeActions;
  prosdk::util::SmallVector<InterruptAction, 4> interruptActions;
  prosdk::util::SmallVector<Action, 4> finishActions;

  // Map of Command* -> CommandPtr for CommandPtrs transferred to the scheduler
  // via Schedule(CommandPtr&&). These are erased (destroyed) at the very end of
  // the loop cycle when the command lifecycle is complete.
  prosdk::util::DenseMap<Command*, CommandPtr> ownedCommands;

  // prosdk: a watchdog epoch name "<name><suffix>" is built in this buffer, reused across loops,
  // instead of a new std::string per epoch per loop (a heap allocation whenever it is longer than
  // the 15-character small-string buffer). AddEpoch() does not keep the view it is given.
  std::string epochName;

  std::string_view EpochName(std::string_view name, std::string_view suffix) {
    epochName.assign(name);
    epochName.append(suffix);
    return epochName;
  }
};

template <typename TMap, typename TKey>
static bool ContainsKey(const TMap& map, TKey keyToCheck) {
  return map.find(keyToCheck) != map.end();
}

CommandScheduler::CommandScheduler()
    : m_impl(new Impl), m_watchdog(prosdk::TimedRobot::DEFAULT_PERIOD, [] {
        prosdk::platform::ConsoleWrite("CommandScheduler loop time overrun.\n");
      }) {
  prosdk::util::ReportUsage("CommandScheduler", "");
}

CommandScheduler::~CommandScheduler() {
  std::unique_ptr<Impl>().swap(m_impl);
}

CommandScheduler& CommandScheduler::GetInstance() {
  static CommandScheduler scheduler;
  return scheduler;
}

void CommandScheduler::SetPeriod(prosdk::units::second_t period) {
  m_watchdog.SetTimeout(period);
}

prosdk::EventLoop* CommandScheduler::GetActiveButtonLoop() const {
  return m_impl->activeButtonLoop;
}

void CommandScheduler::SetActiveButtonLoop(prosdk::EventLoop* loop) {
  m_impl->activeButtonLoop = loop;
}

prosdk::EventLoop* CommandScheduler::GetDefaultButtonLoop() const {
  return &(m_impl->defaultButtonLoop);
}

void CommandScheduler::Schedule(Command* command) {
  RequireUngrouped(command);

  if (m_impl->disabled || m_impl->scheduledCommands.contains(command) ||
      (prosdk::RobotState::IsDisabled() && !command->RunsWhenDisabled())) {
    return;
  }

  const auto& requirements = command->GetRequirements();

  prosdk::util::SmallVector<Command*, 8> intersection;

  bool isDisjoint = true;
  bool allInterruptible = true;
  for (auto&& i1 : m_impl->requirements) {
    if (requirements.find(i1.first) != requirements.end()) {
      isDisjoint = false;
      allInterruptible &= (i1.second->GetInterruptionBehavior() ==
                           Command::InterruptionBehavior::CANCEL_SELF);
      intersection.emplace_back(i1.second);
    }
  }

  if (isDisjoint || allInterruptible) {
    if (allInterruptible) {
      for (auto&& cmdToCancel : intersection) {
        Cancel(cmdToCancel, std::make_optional(command));
      }
    }
    m_impl->scheduledCommands.insert(command);
    for (auto&& requirement : requirements) {
      m_impl->requirements[requirement] = command;
    }
    command->Initialize();
    for (auto&& action : m_impl->initActions) {
      action(*command);
    }
    m_watchdog.AddEpoch(m_impl->EpochName(command->GetName(), ".Initialize()"));
  }
}

void CommandScheduler::Schedule(std::span<Command* const> commands) {
  for (auto command : commands) {
    Schedule(command);
  }
}

void CommandScheduler::Schedule(std::initializer_list<Command*> commands) {
  for (auto command : commands) {
    Schedule(command);
  }
}

void CommandScheduler::Schedule(const CommandPtr& command) {
  Schedule(command.get());
}

void CommandScheduler::Schedule(CommandPtr&& command) {
  auto ptr = command.get();
  m_impl->ownedCommands.try_emplace(ptr, std::move(command));
  Schedule(ptr);
}

void CommandScheduler::Run() {
  if (m_impl->disabled) {
    return;
  }

  m_watchdog.Reset();

  // Run the periodic method of all registered subsystems.
  for (auto&& subsystem : m_impl->subsystems) {
    subsystem.getFirst()->Periodic();
    if constexpr (prosdk::RobotBase::IsSimulation()) {
      subsystem.getFirst()->SimulationPeriodic();
    }
    m_watchdog.AddEpoch(
        m_impl->EpochName(subsystem.getFirst()->GetName(), ".Periodic()"));
  }

  // Cache the active instance to avoid concurrency problems if SetActiveLoop()
  // is called from inside the button bindings.
  prosdk::EventLoop* loopCache = m_impl->activeButtonLoop;
  // Poll buttons for new commands to add.
  loopCache->Poll();
  m_watchdog.AddEpoch("buttons.Run()");

  bool isDisabled = prosdk::RobotState::IsDisabled();
  // create a new set to avoid iterator invalidation.
  for (Command* command : prosdk::util::SmallSet(m_impl->scheduledCommands)) {
    if (!IsScheduled(command)) {
      continue;  // skip as the normal scheduledCommands was modified
    }

    if (isDisabled && !command->RunsWhenDisabled()) {
      Cancel(command, std::nullopt);
      continue;
    }

    command->Execute();
    for (auto&& action : m_impl->executeActions) {
      action(*command);
    }
    m_watchdog.AddEpoch(m_impl->EpochName(command->GetName(), ".Execute()"));

    if (command->IsFinished()) {
      m_impl->scheduledCommands.erase(command);
      command->End(false);
      for (auto&& action : m_impl->finishActions) {
        action(*command);
      }

      for (auto&& requirement : command->GetRequirements()) {
        m_impl->requirements.erase(requirement);
      }

      m_watchdog.AddEpoch(m_impl->EpochName(command->GetName(), ".End(false)"));
      // remove owned commands after everything else is done
      m_impl->ownedCommands.erase(command);
    }
  }

  // Add default commands for un-required registered subsystems.
  for (auto&& subsystem : m_impl->subsystems) {
    auto s = m_impl->requirements.find(subsystem.getFirst());
    if (s == m_impl->requirements.end() && subsystem.getSecond()) {
      Schedule({subsystem.getSecond().get()});
    }
  }

  m_watchdog.Disable();
  if (m_watchdog.IsExpired()) {
    m_watchdog.PrintEpochs();
  }
}

void CommandScheduler::RegisterSubsystem(Subsystem* subsystem) {
  if (m_impl->subsystems.find(subsystem) != m_impl->subsystems.end()) {
    prosdk::platform::ConsoleWrite(
        "Tried to register an already-registered subsystem\n");
    return;
  }

  m_impl->subsystems[subsystem] = nullptr;
}

void CommandScheduler::UnregisterSubsystem(Subsystem* subsystem) {
  auto s = m_impl->subsystems.find(subsystem);
  if (s != m_impl->subsystems.end()) {
    m_impl->subsystems.erase(s);
  }
}

void CommandScheduler::RegisterSubsystem(
    std::initializer_list<Subsystem*> subsystems) {
  for (auto* subsystem : subsystems) {
    RegisterSubsystem(subsystem);
  }
}

void CommandScheduler::RegisterSubsystem(
    std::span<Subsystem* const> subsystems) {
  for (auto* subsystem : subsystems) {
    RegisterSubsystem(subsystem);
  }
}

void CommandScheduler::UnregisterSubsystem(
    std::initializer_list<Subsystem*> subsystems) {
  for (auto* subsystem : subsystems) {
    UnregisterSubsystem(subsystem);
  }
}

void CommandScheduler::UnregisterSubsystem(
    std::span<Subsystem* const> subsystems) {
  for (auto* subsystem : subsystems) {
    UnregisterSubsystem(subsystem);
  }
}

void CommandScheduler::UnregisterAllSubsystems() {
  m_impl->subsystems.clear();
}

void CommandScheduler::SetDefaultCommand(Subsystem* subsystem,
                                         CommandPtr&& defaultCommand) {
  if (!defaultCommand.get()->HasRequirement(subsystem)) {
    throw WPILIB_MakeError(prosdk::err::CommandIllegalUse, "{}",
                           "Default commands must require their subsystem!");
  }
  RequireUngrouped(defaultCommand.get());

  SetDefaultCommandImpl(subsystem, std::move(defaultCommand).Unwrap());
}

void CommandScheduler::RemoveDefaultCommand(Subsystem* subsystem) {
  m_impl->subsystems[subsystem] = nullptr;
}

Command* CommandScheduler::GetDefaultCommand(const Subsystem* subsystem) const {
  auto&& find = m_impl->subsystems.find(subsystem);
  if (find != m_impl->subsystems.end()) {
    return find->second.get();
  } else {
    return nullptr;
  }
}

void CommandScheduler::Cancel(Command* command,
                              std::optional<Command*> interruptor) {
  if (!m_impl) {
    return;
  }
  if (!IsScheduled(command)) {
    return;
  }
  m_impl->scheduledCommands.erase(command);
  command->End(true);
  for (auto&& action : m_impl->interruptActions) {
    action(*command, interruptor);
  }
  prosdk::util::SmallVector<Subsystem*, 8> toErase;
  for (auto&& requirement : m_impl->requirements) {
    if (requirement.second == command) {
      toErase.push_back(requirement.first);
    }
  }
  for (auto* subsystem : toErase) {
    m_impl->requirements.erase(subsystem);
  }
  m_watchdog.AddEpoch(m_impl->EpochName(command->GetName(), ".End(true)"));
}

void CommandScheduler::Cancel(Command* command) {
  Cancel(command, std::nullopt);
}

void CommandScheduler::Cancel(const CommandPtr& command) {
  Cancel(command.get());
}

void CommandScheduler::Cancel(std::span<Command* const> commands) {
  for (auto command : commands) {
    Cancel(command);
  }
}

void CommandScheduler::Cancel(std::initializer_list<Command*> commands) {
  for (auto command : commands) {
    Cancel(command);
  }
}

void CommandScheduler::CancelAll() {
  prosdk::util::SmallVector<Command*, 16> commands;
  for (auto&& command : m_impl->scheduledCommands) {
    commands.emplace_back(command);
  }
  Cancel(commands);
}

bool CommandScheduler::IsScheduled(
    std::span<const Command* const> commands) const {
  for (auto command : commands) {
    if (!IsScheduled(command)) {
      return false;
    }
  }
  return true;
}

bool CommandScheduler::IsScheduled(
    std::initializer_list<const Command*> commands) const {
  for (auto command : commands) {
    if (!IsScheduled(command)) {
      return false;
    }
  }
  return true;
}

bool CommandScheduler::IsScheduled(const Command* command) const {
  return m_impl->scheduledCommands.contains(command);
}

bool CommandScheduler::IsScheduled(const CommandPtr& command) const {
  return m_impl->scheduledCommands.contains(command.get());
}

Command* CommandScheduler::Requiring(const Subsystem* subsystem) const {
  auto find = m_impl->requirements.find(subsystem);
  if (find != m_impl->requirements.end()) {
    return find->second;
  } else {
    return nullptr;
  }
}

void CommandScheduler::Disable() {
  m_impl->disabled = true;
}

void CommandScheduler::Enable() {
  m_impl->disabled = false;
}

void CommandScheduler::PrintWatchdogEpochs() {
  m_watchdog.PrintEpochs();
}

void CommandScheduler::OnCommandInitialize(Action action) {
  m_impl->initActions.emplace_back(std::move(action));
}

void CommandScheduler::OnCommandExecute(Action action) {
  m_impl->executeActions.emplace_back(std::move(action));
}

void CommandScheduler::OnCommandInterrupt(Action action) {
  m_impl->interruptActions.emplace_back(
      [action = std::move(action)](const Command& command,
                                   const std::optional<Command*>& interruptor) {
        action(command);
      });
}

void CommandScheduler::OnCommandInterrupt(InterruptAction action) {
  m_impl->interruptActions.emplace_back(std::move(action));
}

void CommandScheduler::OnCommandFinish(Action action) {
  m_impl->finishActions.emplace_back(std::move(action));
}

void CommandScheduler::RequireUngrouped(const Command* command) {
  auto stacktrace = command->GetPreviousCompositionSite();
  if (stacktrace.has_value()) {
    throw WPILIB_MakeError(
        prosdk::err::CommandIllegalUse,
        "Commands that have been composed may not be added to "
        "another composition or scheduled individually!"
        "\nOriginally composed at:\n{}",
        stacktrace.value());
  }
}

void CommandScheduler::RequireUngrouped(
    std::span<const std::unique_ptr<Command>> commands) {
  for (auto&& command : commands) {
    RequireUngrouped(command.get());
  }
}

void CommandScheduler::RequireUngrouped(
    std::initializer_list<const Command*> commands) {
  for (auto&& command : commands) {
    RequireUngrouped(command);
  }
}

void CommandScheduler::RequireUngroupedAndUnscheduled(const Command* command) {
  if (IsScheduled(command)) {
    throw WPILIB_MakeError(prosdk::err::CommandIllegalUse,
                           "Commands that have been scheduled individually may "
                           "not be added to another composition!");
  }
  RequireUngrouped(command);
}

void CommandScheduler::RequireUngroupedAndUnscheduled(
    std::span<const std::unique_ptr<Command>> commands) {
  for (auto&& command : commands) {
    RequireUngroupedAndUnscheduled(command.get());
  }
}

void CommandScheduler::RequireUngroupedAndUnscheduled(
    std::initializer_list<const Command*> commands) {
  for (auto&& command : commands) {
    RequireUngroupedAndUnscheduled(command);
  }
}

std::vector<std::string> CommandScheduler::GetScheduledCommandNames() const {
  std::vector<std::string> names;
  for (Command* command : m_impl->scheduledCommands) {
    names.emplace_back(command->GetName());
  }
  return names;
}

std::vector<int64_t> CommandScheduler::GetScheduledCommandIds() const {
  std::vector<int64_t> ids;
  for (Command* command : m_impl->scheduledCommands) {
    uintptr_t ptrTmp = reinterpret_cast<uintptr_t>(command);
    ids.emplace_back(static_cast<int64_t>(ptrTmp));
  }
  return ids;
}

void CommandScheduler::LogTo(prosdk::telemetry::TelemetryTable& table) const {
  std::vector<std::string> names = GetScheduledCommandNames();
  table.Log("Names", names);

  std::vector<int64_t> ids = GetScheduledCommandIds();
  table.Log("Ids", ids);
}

std::string_view CommandScheduler::GetTelemetryType() const {
  return "Scheduler";
}

void CommandScheduler::PublishTunable(prosdk::tunables::TunableTable& table) {
  UpdateTunable();
  table.Publish(
      "Names", this, &CommandScheduler::m_scheduledCommandNames,
      prosdk::tunables::TunableConfig{
          .isMutable = false,
          .parent = this,
          .polling = prosdk::tunables::TunableConfig::Polling::kAlwaysGet});
  table.Publish(
      "Ids", this, &CommandScheduler::m_scheduledCommandIds,
      prosdk::tunables::TunableConfig{
          .isMutable = false,
          .parent = this,
          .polling = prosdk::tunables::TunableConfig::Polling::kAlwaysGet});
  table.Publish(
      "Cancel", this, &CommandScheduler::m_toCancel,
      prosdk::tunables::TunableConfig{
          .robust = true,
          .onTune =
              [](TunableBase&, prosdk::tunables::ComplexTunable* self) {
                auto scheduler = static_cast<CommandScheduler*>(self);
                for (auto cancel : scheduler->m_toCancel) {
                  uintptr_t ptrTmp = static_cast<uintptr_t>(cancel);
                  Command* command = reinterpret_cast<Command*>(ptrTmp);
                  if (scheduler->m_impl->scheduledCommands.find(command) !=
                      scheduler->m_impl->scheduledCommands.end()) {
                    scheduler->Cancel(command);
                  }
                }
                scheduler->m_toCancel.clear();
                scheduler->SetChildTunableChanged("Cancel");
              },
          .parent = this});
}

void CommandScheduler::UpdateTunable() const {
  m_scheduledCommandNames = GetScheduledCommandNames();
  m_scheduledCommandIds = GetScheduledCommandIds();
}

std::string_view CommandScheduler::GetTunableType() const {
  return "Scheduler";
}

void CommandScheduler::SetDefaultCommandImpl(Subsystem* subsystem,
                                             std::unique_ptr<Command> command) {
  if (command->GetInterruptionBehavior() ==
      Command::InterruptionBehavior::CANCEL_INCOMING) {
    prosdk::platform::ConsoleWrite(
        "Registering a non-interruptible default command!\n"
        "This will likely prevent any other commands from "
        "requiring this subsystem.\n");
    // Warn, but allow -- there might be a use case for this.
  }
  m_impl->subsystems[subsystem] = std::move(command);
}
