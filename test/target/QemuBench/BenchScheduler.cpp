// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (b) CommandScheduler::Run() cost against the number of scheduled commands (0, 10, 50, 200) and
// with subsystems that have a periodic method and a default command. Every command runs forever
// (cmd::Run) and ignores the disabled state, so each Run() executes all of them; per_op is one
// Run(). Run() also polls the button loop, runs the subsystems' Periodic() and keeps a watchdog
// epoch per command, as on a robot. heap_bytes is the heap the case's subsystems and commands
// hold once scheduled.

#include <format>
#include <memory>
#include <string>
#include <vector>

#include <prosdk/commands2/CommandScheduler.hpp>
#include <prosdk/commands2/Commands.hpp>
#include <prosdk/commands2/SubsystemBase.hpp>

#include "Bench.hpp"

namespace qemu::bench {

namespace {

namespace cmd = prosdk::cmd;

class BenchSubsystem : public cmd::SubsystemBase {
 public:
  explicit BenchSubsystem(std::string name) : SubsystemBase{std::move(name)} {}
  void Periodic() override { ++periodic; }
  int periodic = 0;
};

void SchedulerCase(const char* name, int commands, int subsystems, uint32_t iters) {
  auto& scheduler = cmd::CommandScheduler::GetInstance();
  int executes = 0;
  const size_t heapBefore = HeapFree();
  {
    std::vector<std::unique_ptr<BenchSubsystem>> subs;
    std::vector<cmd::CommandPtr> cmds;
    for (int i = 0; i < subsystems; ++i) {
      subs.push_back(std::make_unique<BenchSubsystem>("Sub" + std::to_string(i)));
      BenchSubsystem* sub = subs.back().get();
      scheduler.SetDefaultCommand(sub, sub->Run([&executes] { ++executes; })
                                           .IgnoringDisable(true)
                                           .WithName("Default" + std::to_string(i)));
    }
    for (int i = 0; i < commands; ++i) {
      cmds.push_back(cmd::Run([&executes] { ++executes; })
                         .IgnoringDisable(true)
                         .WithName("Cmd" + std::to_string(i)));
      scheduler.Schedule(cmds.back());
    }
    scheduler.Run();  // schedules the default commands
    // heap held by the subsystems, the commands and the scheduler's bookkeeping of them
    const long heapBytes = static_cast<long>(heapBefore) - static_cast<long>(HeapFree());
    executes = 0;
    Run(
        name, iters,
        [&] {
          scheduler.Run();
          DoNotOptimize(executes);
        },
        std::format("heap_bytes={}", heapBytes));
    // every command ran on every Run() (plus Run's warm-up call)
    const int expected = static_cast<int>(iters + 1) * (commands + subsystems);
    if (executes != expected) {
      Linef("BENCH-FAIL {} executes={} expected={}", name, executes, expected);
    }
    scheduler.CancelAll();
    scheduler.UnregisterAllSubsystems();
  }
  scheduler.Run();
}

}  // namespace

void BenchScheduler() {
  SchedulerCase("scheduler_run_0cmd", 0, 0, 2000);
  SchedulerCase("scheduler_run_10cmd", 10, 0, 1000);
  SchedulerCase("scheduler_run_50cmd", 50, 0, 400);
  SchedulerCase("scheduler_run_200cmd", 200, 0, 100);
  SchedulerCase("scheduler_run_10sub_default", 0, 10, 1000);
  SchedulerCase("scheduler_run_200cmd_10sub_default", 200, 10, 100);
}

}  // namespace qemu::bench
