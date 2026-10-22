// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (d) The hosted robot program (design/PORT.md section 8.1) on the target: PROSDK_ROBOT_MAIN
// registers a TimedRobot (20 ms loop + a 10 ms AddPeriodic) at static initialization; this case
// task plays probot-core's user task: it calls core's boot hook (detail::Boot: HAL, robot
// construction, StartHosted, loop period), then one pre/post cycle per core pass on a FreeRTOS
// fixed-rate schedule at the period the robot asked for, while the fake core walks through
// disabled -> autonomous -> disabled -> teleop -> e-stop. The e-stop hook runs on its own task,
// as core does it, and latches: this case runs last.

#include <algorithm>
#include <string>
#include <vector>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <prosdk/driverstation/RobotState.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hal/HAL.h>
#include <prosdk/platform/CoreOps.hpp>
#include <prosdk/platform/RobotHost.hpp>
#include <prosdk/platform/RobotMain.hpp>
#include <prosdk/units/time.hpp>

#include "FakeCore.hpp"
#include "Harness.hpp"

namespace pf = prosdk::platform;

namespace {

struct Stamps {
  std::vector<int64_t> t;
  void Add() {
    if (t.size() < 400) {
      t.push_back(esp_timer_get_time());
    }
  }
};

class SmokeRobot : public prosdk::TimedRobot {
 public:
  static inline SmokeRobot* instance = nullptr;

  SmokeRobot() : TimedRobot(20_ms) {
    instance = this;
    Event("ctor");
    AddPeriodic([this] { fast.Add(); }, 10_ms);
  }

  void DriverStationConnected() override { Event("DriverStationConnected"); }
  void DisabledInit() override { Event("DisabledInit"); }
  void DisabledExit() override { Event("DisabledExit"); }
  void AutonomousInit() override { Event("AutonomousInit"); }
  void AutonomousPeriodic() override {
    ++autoPeriodic;
    autoSawEnabled &= prosdk::RobotState::IsAutonomousEnabled();
    int32_t status = 0;
    autoSawGateOpen &= HAL_GetSystemActive(&status) != 0;
  }
  void AutonomousExit() override { Event("AutonomousExit"); }
  void TeleopInit() override { Event("TeleopInit"); }
  void TeleopPeriodic() override {
    ++teleopPeriodic;
    teleopSawEnabled &= prosdk::RobotState::IsTeleopEnabled();
  }
  void TeleopExit() override { Event("TeleopExit"); }
  void DisabledPeriodic() override {
    ++disabledPeriodic;
    int32_t status = 0;
    disabledSawGateClosed &= HAL_GetSystemActive(&status) == 0;
  }
  void RobotPeriodic() override { loop.Add(); }

  void Event(const char* name) {
    if (events.empty() || events.back() != name) {
      events.emplace_back(name);
    }
  }

  std::vector<std::string> events;
  Stamps loop;
  Stamps fast;
  int autoPeriodic = 0;
  int teleopPeriodic = 0;
  int disabledPeriodic = 0;
  bool autoSawEnabled = true;
  bool autoSawGateOpen = true;
  bool teleopSawEnabled = true;
  bool disabledSawGateClosed = true;
};

struct EStopRun {
  TaskHandle_t waiter;
  int64_t us;
};

void EStopTask(void* arg) {
  auto* run = static_cast<EStopRun*>(arg);
  int64_t t0 = esp_timer_get_time();
  pf::detail::EmergencyStop();
  run->us = esp_timer_get_time() - t0;
  xTaskNotifyGive(run->waiter);
  vTaskDelete(nullptr);
}

// mean spacing (us) of the stamps taken in [from, to)
double MeanSpacing(const std::vector<int64_t>& t, int64_t from, int64_t to, int64_t* maxDev,
                   int64_t nominal, int* count) {
  std::vector<int64_t> s;
  for (int64_t x : t) {
    if (x >= from && x < to) {
      s.push_back(x);
    }
  }
  *count = static_cast<int>(s.size());
  *maxDev = 0;
  for (size_t i = 1; i < s.size(); ++i) {
    *maxDev = std::max(*maxDev, std::abs((s[i] - s[i - 1]) - nominal));
  }
  return s.size() > 1 ? double(s.back() - s.front()) / double(s.size() - 1) : 0.0;
}

}  // namespace

PROSDK_ROBOT_MAIN(SmokeRobot)

namespace qemu {

void CaseRobot(Ctx& ctx) {
  auto& core = fakecore::Get();
  QCHECK(ctx, pf::GetRobotProgramState() == pf::RobotProgramState::kRegistered);
  core.selectedMode.store(pf::kCoreModeAuto);

  // core's boot hook, then its passes
  int64_t bootStart = esp_timer_get_time();
  pf::detail::Boot();
  int64_t bootUs = esp_timer_get_time() - bootStart;
  SmokeRobot* robot = SmokeRobot::instance;
  QCHECK(ctx, robot != nullptr);
  QCHECK(ctx, pf::GetRobotProgramState() == pf::RobotProgramState::kRunning);
  if (robot == nullptr) {
    return;
  }
  // 20 ms loop + 10 ms AddPeriodic -> core passes every 10 ms (gcd)
  QCHECK_MSG(ctx, core.periodMs.load() == 10, "core period {} ms", core.periodMs.load());

  struct Step {
    int64_t atMs;
    uint8_t phase;
    bool outputs;
  };
  // the timeline of the fake core (ms after boot); INIT phases last one pass
  const Step steps[] = {
      {0, pf::kCorePhaseStopped, false},       {400, pf::kCorePhaseAutoInit, false},
      {410, pf::kCorePhaseAutoRun, true},      {1410, pf::kCorePhaseTransition, false},
      {1420, pf::kCorePhaseStopped, false},    {1600, pf::kCorePhaseTeleopInit, false},
      {1610, pf::kCorePhaseTeleopRun, true},   {2610, pf::kCorePhaseStopped, false},
  };
  constexpr int64_t kEStopAtMs = 2800;

  int64_t passMaxUs = 0;
  int64_t passSumUs = 0;
  int passes = 0;
  int64_t autoFrom = 0, autoTo = 0, teleFrom = 0, teleTo = 0;
  bool gateOpenInAuto = false;
  bool gateClosedWhenStopped = true;
  const int64_t origin = esp_timer_get_time();
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const int64_t nowMs = (esp_timer_get_time() - origin) / 1000;
    if (nowMs >= kEStopAtMs) {
      break;
    }
    const Step* step = &steps[0];
    for (const Step& s : steps) {
      if (nowMs >= s.atMs) {
        step = &s;
      }
    }
    core.packetSeq.fetch_add(1);  // a Driver Station packet every pass
    if (core.phase.load() != step->phase) {
      int64_t now = esp_timer_get_time();
      if (step->phase == pf::kCorePhaseAutoRun) autoFrom = now + 100000;
      if (step->phase == pf::kCorePhaseTransition) autoTo = now;
      if (step->phase == pf::kCorePhaseTeleopRun) teleFrom = now + 100000;
      if (step->phase == pf::kCorePhaseStopped && teleFrom != 0) teleTo = now;
    }
    core.phase.store(step->phase);
    core.outputs.store(step->outputs);
    core.selectedMode.store(nowMs < 1420 ? pf::kCoreModeAuto : pf::kCoreModeTeleop);

    int64_t t0 = esp_timer_get_time();
    pf::detail::PreCycle();
    pf::detail::PostCycle();
    int64_t dt = esp_timer_get_time() - t0;
    passMaxUs = std::max(passMaxUs, dt);
    passSumUs += dt;
    ++passes;

    int32_t status = 0;
    bool gate = HAL_GetSystemActive(&status) != 0;
    if (step->phase == pf::kCorePhaseAutoRun) gateOpenInAuto |= gate;
    if (step->phase == pf::kCorePhaseStopped) gateClosedWhenStopped &= !gate;

    // core's fixed-rate grid at the period the robot asked for
    const uint32_t period = std::max<uint32_t>(core.periodMs.load(), 1);
    if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(period)) == pdFALSE) {
      core.overruns.fetch_add(1);
      wake = xTaskGetTickCount();
    }
  }

  // ---- e-stop: core latches it, deletes the user task, runs the hook on its own task ----
  core.estop.store(true);
  core.phase.store(pf::kCorePhaseStopped);
  core.outputs.store(false);
  EStopRun run{xTaskGetCurrentTaskHandle(), -1};
  xTaskCreatePinnedToCore(EStopTask, "estop", 4096, &run, 4, nullptr, 1);
  QCHECK(ctx, ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) != 0);
  const int loopsBefore = static_cast<int>(robot->loop.t.size());
  const int fastBefore = static_cast<int>(robot->fast.t.size());
  // one more pass (on the robot core deleted the task; the robot host must stop by itself)
  pf::detail::PreCycle();
  pf::detail::PostCycle();
  vTaskDelay(pdMS_TO_TICKS(30));
  pf::detail::PreCycle();
  pf::detail::PostCycle();
  int32_t status = 0;
  QCHECK(ctx, pf::GetRobotProgramState() == pf::RobotProgramState::kEStopped);
  QCHECK(ctx, pf::EmergencyStopped());
  QCHECK(ctx, HAL_GetSystemActive(&status) == 0);
  QCHECK_MSG(ctx, static_cast<int>(robot->loop.t.size()) == loopsBefore &&
                      static_cast<int>(robot->fast.t.size()) == fastBefore,
             "robot ran after the e-stop ({} -> {})", loopsBefore, robot->loop.t.size());

  // ---- lifecycle ----
  const std::vector<std::string> want{"ctor",          "DriverStationConnected", "DisabledInit",
                                      "DisabledExit",  "AutonomousInit",         "AutonomousExit",
                                      "DisabledInit",  "DisabledExit",           "TeleopInit",
                                      "TeleopExit",    "DisabledInit"};
  std::string got;
  for (const auto& e : robot->events) {
    got += e;
    got += ',';
  }
  // DriverStationConnected may come before or after the first DisabledInit
  std::vector<std::string> ev = robot->events;
  std::vector<std::string> w = want;
  std::erase(ev, "DriverStationConnected");
  std::erase(w, "DriverStationConnected");
  QCHECK_MSG(ctx, ev == w, "events {}", got);
  QCHECK_MSG(ctx, got.find("DriverStationConnected") != std::string::npos, "events {}", got);
  QCHECK_MSG(ctx, robot->autoPeriodic >= 40 && robot->autoSawEnabled && robot->autoSawGateOpen,
             "auto periodic {} enabled {} gate {}", robot->autoPeriodic, robot->autoSawEnabled,
             robot->autoSawGateOpen);
  QCHECK_MSG(ctx, robot->teleopPeriodic >= 40 && robot->teleopSawEnabled, "teleop periodic {}",
             robot->teleopPeriodic);
  QCHECK(ctx, robot->disabledPeriodic > 10 && robot->disabledSawGateClosed);
  QCHECK(ctx, gateOpenInAuto && gateClosedWhenStopped);

  // ---- timing: robot loop 20 ms, AddPeriodic 10 ms (inside the enabled windows) ----
  int64_t loopDev = 0, fastDev = 0;
  int loopN = 0, fastN = 0;
  double loopMean = MeanSpacing(robot->loop.t, autoFrom, autoTo, &loopDev, 20000, &loopN);
  double fastMean = MeanSpacing(robot->fast.t, autoFrom, autoTo, &fastDev, 10000, &fastN);
  int64_t teleDev = 0;
  int teleN = 0;
  double teleMean = MeanSpacing(robot->loop.t, teleFrom, teleTo, &teleDev, 20000, &teleN);
  QCHECK_MSG(ctx, loopMean > 19500 && loopMean < 20500, "loop mean {} us", loopMean);
  QCHECK_MSG(ctx, fastMean > 9750 && fastMean < 10250, "AddPeriodic mean {} us", fastMean);
  QCHECK_MSG(ctx, teleMean > 19500 && teleMean < 20500, "teleop loop mean {} us", teleMean);
  QCHECK_MSG(ctx, fastN >= 2 * loopN - 3 && fastN <= 2 * loopN + 3, "fast {} loop {}", fastN,
             loopN);

  ctx.Note("boot_us={} passes={} pass_mean_us={} pass_max_us={} overruns={}", bootUs, passes,
           passes ? passSumUs / passes : 0, passMaxUs, core.overruns.load());
  ctx.Note("loop_mean_us={:.0f} loop_maxdev_us={} addperiodic_mean_us={:.0f} "
           "addperiodic_maxdev_us={} estop_hook_us={}",
           loopMean, loopDev, fastMean, fastDev, run.us);
  ctx.Note("dash_topics={} dash_sets={}", core.topics.load(), core.sets.load());
}

}  // namespace qemu
