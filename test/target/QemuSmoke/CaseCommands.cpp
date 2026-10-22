// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (g) commandsv2: CommandScheduler::Run() driven by hand (20 ms steps) with a subsystem and its
// default command, a sequence with a timed wait, a parallel race, requirement interruption and
// the disabled rule (the robot is disabled here: only IgnoringDisable commands run).

#include <string>
#include <vector>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <prosdk/commands2/CommandScheduler.hpp>
#include <prosdk/commands2/Commands.hpp>
#include <prosdk/commands2/SubsystemBase.hpp>
#include <prosdk/units/time.hpp>

#include "Harness.hpp"

namespace qemu {

namespace {

class Arm : public prosdk::cmd::SubsystemBase {
 public:
  Arm() : SubsystemBase{"QemuArm"} {}
  void Periodic() override { ++periodic; }
  int periodic = 0;
};

}  // namespace

void CaseCommands(Ctx& ctx) {
  namespace cmd = prosdk::cmd;
  auto& scheduler = cmd::CommandScheduler::GetInstance();
  std::vector<std::string> events;
  int defaultRuns = 0;
  {
    Arm arm;
    scheduler.SetDefaultCommand(
        &arm, arm.Run([&] { ++defaultRuns; }).IgnoringDisable(true).WithName("ArmDefault"));

    // the scheduler is a singleton and keeps these actions: no captures of locals
    static int inits, finishes, interrupts;
    inits = finishes = interrupts = 0;
    scheduler.OnCommandInitialize([](const cmd::Command&) { ++inits; });
    scheduler.OnCommandFinish([](const cmd::Command&) { ++finishes; });
    scheduler.OnCommandInterrupt([](const cmd::Command&) { ++interrupts; });

    // sequence with a 100 ms wait, running while disabled
    auto sequence = cmd::Sequence(cmd::RunOnce([&] { events.push_back("seq-start"); }),
                                  cmd::Wait(100_ms),
                                  cmd::RunOnce([&] { events.push_back("seq-end"); }))
                        .IgnoringDisable(true)
                        .WithName("Sequence");
    // race: ends when the 5th step is reached
    int steps = 0;
    auto race = cmd::Race(cmd::Run([&] { ++steps; }), cmd::WaitUntil([&] { return steps >= 5; }))
                    .IgnoringDisable(true);
    // requires the arm: interrupts the default command, then the default command resumes
    auto armCommand = arm.RunOnce([&] { events.push_back("arm-once"); }).IgnoringDisable(true);
    // not allowed while disabled
    auto blocked = cmd::RunOnce([&] { events.push_back("blocked-ran"); });

    scheduler.Schedule(sequence);
    scheduler.Schedule(race);
    scheduler.Schedule(blocked);
    QCHECK(ctx, scheduler.IsScheduled(sequence));
    QCHECK(ctx, !scheduler.IsScheduled(blocked));

    int64_t t0 = esp_timer_get_time();
    int64_t seqDone = 0;
    for (int i = 0; i < 25; ++i) {
      if (i == 3) {
        scheduler.Schedule(armCommand);
      }
      scheduler.Run();
      if (seqDone == 0 && !scheduler.IsScheduled(sequence)) {
        seqDone = esp_timer_get_time() - t0;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    QCHECK_MSG(ctx, !events.empty() && events.front() == "seq-start", "events {}", events.size());
    bool seqEnd = false, armOnce = false, blockedRan = false;
    for (const auto& e : events) {
      seqEnd |= e == "seq-end";
      armOnce |= e == "arm-once";
      blockedRan |= e == "blocked-ran";
    }
    QCHECK(ctx, seqEnd);
    QCHECK(ctx, armOnce);
    QCHECK(ctx, !blockedRan);
    QCHECK_MSG(ctx, seqDone >= 100000 && seqDone < 200000, "sequence took {} us", seqDone);
    QCHECK_MSG(ctx, steps == 5, "race steps {}", steps);
    QCHECK_MSG(ctx, arm.periodic == 25, "subsystem periodic {}", arm.periodic);
    QCHECK_MSG(ctx, defaultRuns >= 20 && defaultRuns <= 24, "default runs {}", defaultRuns);
    QCHECK_MSG(ctx, interrupts >= 1, "interrupts {}", interrupts);  // default cmd by armCommand
    QCHECK_MSG(ctx, finishes >= 3, "finishes {}", finishes);
    ctx.Note("inits={} finishes={} interrupts={} default_runs={} seq_us={}", inits, finishes,
             interrupts, defaultRuns, seqDone);

    scheduler.CancelAll();
    scheduler.UnregisterAllSubsystems();
  }
  scheduler.Run();
}

}  // namespace qemu
