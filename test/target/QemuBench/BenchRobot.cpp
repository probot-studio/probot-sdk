// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (a) The hosted TimedRobot under a realistic load, at 20, 10, 5 and 2 ms. Every loop runs what a
// command-based differential-drive robot touches: CommandScheduler::Run() with a default drive
// command, a status command, a repeating sequence and a trigger binding; differential kinematics,
// two PIDControllers and a SimpleMotorFeedforward; differential odometry; telemetry (pose, wheel
// distances and voltages, the scheduler) and tunables (both PIDs and a speed, published once).
// No motor or encoder hardware (QEMU has no MCPWM or PCNT): the wheels follow their setpoints.
//
// This group task plays probot-core's user task as QemuSmoke's case d does: core's boot hook,
// then one pre/post cycle per core pass on a FreeRTOS fixed-rate schedule at the period the robot
// asked for. Each period gets a fresh robot (the test-only robot host reset deletes the previous
// one): 200 ms disabled, one teleop-init pass, 2.2 s teleop; the numbers cover the 2 s of teleop
// after a 200 ms settle.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <format>
#include <vector>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <prosdk/commands2/CommandScheduler.hpp>
#include <prosdk/commands2/Commands.hpp>
#include <prosdk/commands2/SubsystemBase.hpp>
#include <prosdk/commands2/button/Trigger.hpp>
#include <prosdk/event/EventLoop.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/math/kinematics/DifferentialDriveKinematics.hpp>
#include <prosdk/math/kinematics/DifferentialDriveOdometry.hpp>
#include <prosdk/platform/CoreOps.hpp>
#include <prosdk/platform/RobotHost.hpp>
#include <prosdk/system/RobotController.hpp>
#include <prosdk/telemetry/Telemetry.hpp>
#include <prosdk/tunables/Tunable.hpp>
#include <prosdk/tunables/Tunables.hpp>
#include <prosdk/units/angle.hpp>
#include <prosdk/units/angular_velocity.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/time.hpp>
#include <prosdk/units/velocity.hpp>
#include <prosdk/units/voltage.hpp>

#include "Bench.hpp"
#include "FakeCore.hpp"

namespace qemu::bench {

namespace {

namespace cmd = prosdk::cmd;
namespace pf = prosdk::platform;
namespace pm = prosdk::math;
namespace pu = prosdk::units;

uint32_t gPeriodMs = 20;  // the next robot's loop period

// Published once and never destroyed: the tunable registry keeps pointers to them, and every
// robot of the run uses the same ones.
struct Tuning {
  pm::PIDController leftPid{1.0, 0.0, 0.01};
  pm::PIDController rightPid{1.0, 0.0, 0.01};
  prosdk::tunables::Tunable<double> speed{1.5};  // m/s

  Tuning() {
    prosdk::tunables::Publish("Drive/Left PID", leftPid);
    prosdk::tunables::Publish("Drive/Right PID", rightPid);
    prosdk::tunables::Publish("Drive/Speed", speed);
  }
};

Tuning& GetTuning() {
  static Tuning* tuning = new Tuning;
  return *tuning;
}

class BenchDrive : public cmd::SubsystemBase {
 public:
  BenchDrive() : SubsystemBase{"Drive"} {}

  // odometry from the simulated wheels, then telemetry
  void Periodic() override {
    const double dt = gPeriodMs / 1000.0;
    m_left += m_leftVel * dt;
    m_right += m_rightVel * dt;
    m_heading += (m_rightVel - m_leftVel) / kTrackWidth * dt;
    m_odometry.Update(pm::Rotation2d{pu::radian_t{m_heading}}, pu::meter_t{m_left},
                      pu::meter_t{m_right});
    prosdk::telemetry::Log("Drive/Pose", m_odometry.GetPose());
    prosdk::telemetry::Log("Drive/Left (m)", m_left);
    prosdk::telemetry::Log("Drive/Right (m)", m_right);
  }

  // kinematics, feedforward plus PID per wheel
  void Drive(pu::meters_per_second_t vx, pu::radians_per_second_t omega) {
    auto wheels = m_kinematics.ToWheelVelocities({vx, 0_mps, omega});
    Tuning& tuning = GetTuning();
    const pu::volt_t left =
        m_ff.Calculate(wheels.left) +
        pu::volt_t{tuning.leftPid.Calculate(m_leftVel, wheels.left.value())};
    const pu::volt_t right =
        m_ff.Calculate(wheels.right) +
        pu::volt_t{tuning.rightPid.Calculate(m_rightVel, wheels.right.value())};
    // no motors: the wheels follow the setpoint one loop later
    m_leftVel = wheels.left.value();
    m_rightVel = wheels.right.value();
    prosdk::telemetry::Log("Drive/Left (V)", left.value());
    prosdk::telemetry::Log("Drive/Right (V)", right.value());
  }

 private:
  static constexpr double kTrackWidth = 0.6;  // m
  pm::DifferentialDriveKinematics m_kinematics{pu::meter_t{kTrackWidth}};
  pm::DifferentialDriveOdometry m_odometry{0_deg, 0_m, 0_m};
  pm::SimpleMotorFeedforward<pu::meters> m_ff{1_V, 3_V / 1_mps};
  double m_left = 0.0;
  double m_right = 0.0;
  double m_heading = 0.0;
  double m_leftVel = 0.0;
  double m_rightVel = 0.0;
};

class BenchRobot : public prosdk::TimedRobot {
 public:
  static inline BenchRobot* instance = nullptr;

  BenchRobot() : TimedRobot(pu::millisecond_t{static_cast<double>(gPeriodMs)}) {
    instance = this;
    loopStart.reserve(kMaxStamps);
    loopUserNs.reserve(kMaxStamps);
    GetTuning();
    m_drive.SetDefaultCommand(m_drive.Run([this] {
      const double speed = GetTuning().speed;
      m_drive.Drive(speed * 1_mps, 0.5_rad_per_s);
    }).WithName("Arcade"));
    // a button-style binding, polled every loop: true for 0.5 s out of every second
    cmd::Trigger{[this] {
      return (prosdk::RobotController::GetMonotonicTime() / 500'000'000) % 2 == 1;
    }}.OnTrue(cmd::RunOnce([this] { ++m_triggered; }).WithName("Triggered"));
  }

  ~BenchRobot() override {
    auto& scheduler = cmd::CommandScheduler::GetInstance();
    scheduler.CancelAll();
    scheduler.UnregisterAllSubsystems();
    scheduler.GetDefaultButtonLoop()->Clear();  // the trigger above captures this robot
    instance = nullptr;
  }

  void DisabledPeriodic() override {}
  void TeleopPeriodic() override {}

  void TeleopInit() override {
    auto& scheduler = cmd::CommandScheduler::GetInstance();
    scheduler.Schedule(m_status);
    scheduler.Schedule(m_blink);
  }

  void RobotPeriodic() override {
    Stopwatch watch;
    const int64_t start = esp_timer_get_time();
    auto& scheduler = cmd::CommandScheduler::GetInstance();
    scheduler.Run();
    prosdk::telemetry::Log("Scheduler", scheduler);
    if (loopStart.size() < kMaxStamps) {
      loopStart.push_back(start);
      loopOffset.push_back(start - pf::CycleStartTime() / 1000);
      loopUserNs.push_back(watch.ElapsedNs());
    }
  }

  static constexpr size_t kMaxStamps = 1600;
  std::vector<int64_t> loopStart;    // esp_timer us at the start of RobotPeriodic
  std::vector<int64_t> loopOffset;   // us from the pass start (core's pre-cycle hook) to it
  std::vector<uint64_t> loopUserNs;  // RobotPeriodic (scheduler and its commands, telemetry)

 private:
  BenchDrive m_drive;
  int m_loops = 0;
  int m_triggered = 0;
  cmd::CommandPtr m_status = cmd::Run([this] {
                               prosdk::telemetry::Log("Status/Loops", ++m_loops);
                             }).WithName("Status");
  cmd::CommandPtr m_blink =
      cmd::Sequence(cmd::Wait(250_ms), cmd::RunOnce([this] { ++m_triggered; }))
          .Repeatedly()
          .WithName("Blink");
};

prosdk::RobotBase* NewBenchRobot() {
  return new BenchRobot();
}

void RunPeriod(uint32_t periodMs, bool first) {
  auto& core = fakecore::Get();
  core.phase.store(pf::kCorePhaseStopped);
  core.outputs.store(false);
  core.selectedMode.store(pf::kCoreModeTeleop);
  gPeriodMs = periodMs;
  if (!first) {
    // test-only robot host reset: deletes the previous robot; the next pre-cycle boots this one
    pf::detail::ResetRobotMainForTest();
  }
  pf::SetRobotMain(&NewBenchRobot, "BenchRobot");
  if (first) {
    pf::detail::Boot();  // core's boot hook: HAL, robot, loop period
  }
  // later robots are registered after core's boot: the first pass below constructs them (the
  // robot host aligns its schedule on the pass after that one)

  // the fake core's timeline (ms): disabled, one teleop-init pass, teleop
  constexpr int64_t kTeleopAtMs = 200;
  constexpr int64_t kWindowFromMs = 400;
  constexpr int64_t kWindowToMs = 2400;
  int64_t passes = 0;
  uint64_t passSumNs = 0;
  uint64_t passMaxNs = 0;
  int64_t windowFrom = 0;
  int64_t windowTo = 0;
  bool initDone = false;
  const uint32_t overrunsBefore = core.overruns.load();
  uint32_t overrunsInWindow = 0;
  int64_t lastPassStart = 0;
  int64_t passJitterMax = 0;  // pass start spacing vs the period (wake-up), in the window
  const int64_t origin = esp_timer_get_time();
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const int64_t now = esp_timer_get_time();
    const int64_t nowMs = (now - origin) / 1000;
    if (nowMs >= kWindowToMs) {
      windowTo = now;
      break;
    }
    const bool inWindow = nowMs >= kWindowFromMs;
    if (inWindow && windowFrom == 0) {
      windowFrom = now;
    }
    if (inWindow && lastPassStart != 0) {
      const int64_t dev = now - lastPassStart - int64_t{core.periodMs.load()} * 1000;
      passJitterMax = std::max(passJitterMax, std::abs(dev));
    }
    lastPassStart = now;
    core.packetSeq.fetch_add(1);  // a Driver Station packet every pass
    if (nowMs < kTeleopAtMs) {
      core.phase.store(pf::kCorePhaseStopped);
      core.outputs.store(false);
    } else if (!initDone) {
      core.phase.store(pf::kCorePhaseTeleopInit);
      core.outputs.store(false);
      initDone = true;
    } else {
      core.phase.store(pf::kCorePhaseTeleopRun);
      core.outputs.store(true);
    }

    Stopwatch watch;
    pf::detail::PreCycle();
    pf::detail::PostCycle();
    const uint64_t ns = watch.ElapsedNs();
    if (inWindow) {
      ++passes;
      passSumNs += ns;
      passMaxNs = std::max(passMaxNs, ns);
    }

    // core's fixed-rate grid at the period the robot asked for
    const uint32_t period = std::max<uint32_t>(core.periodMs.load(), 1);
    if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(period)) == pdFALSE) {
      core.overruns.fetch_add(1);
      overrunsInWindow += inWindow ? 1 : 0;
      wake = xTaskGetTickCount();
    }
  }

  BenchRobot* robot = BenchRobot::instance;
  if (robot == nullptr) {
    Linef("BENCH-FAIL robot_loop_{}ms no robot (state {})", periodMs,
          static_cast<int>(pf::GetRobotProgramState()));
    return;
  }
  // robot loops inside the window: spacing (jitter) and the user code time
  const int64_t periodUs = int64_t{periodMs} * 1000;
  std::vector<int64_t> starts;
  uint64_t userSumNs = 0;
  uint64_t userMaxNs = 0;
  int64_t offsetMin = 0;
  int64_t offsetMax = 0;
  for (size_t i = 0; i < robot->loopStart.size(); ++i) {
    const int64_t t = robot->loopStart[i];
    if (t >= windowFrom && t < windowTo) {
      const int64_t offset = robot->loopOffset[i];
      offsetMin = starts.empty() ? offset : std::min(offsetMin, offset);
      offsetMax = starts.empty() ? offset : std::max(offsetMax, offset);
      starts.push_back(t);
      userSumNs += robot->loopUserNs[i];
      userMaxNs = std::max(userMaxNs, robot->loopUserNs[i]);
    }
  }
  int64_t jitterMax = 0;
  double jitterSq = 0.0;
  for (size_t i = 1; i < starts.size(); ++i) {
    const int64_t dev = (starts[i] - starts[i - 1]) - periodUs;
    jitterMax = std::max(jitterMax, std::abs(dev));
    jitterSq += double(dev) * double(dev);
  }
  const double jitterRms = starts.size() > 1 ? std::sqrt(jitterSq / double(starts.size() - 1)) : 0;
  const double meanUs = passes ? double(passSumNs) / 1000.0 / double(passes) : 0.0;
  const double periodD = double(periodUs);
  Report(std::format("robot_loop_{}ms", periodMs), static_cast<uint64_t>(passes), passSumNs,
         std::format("period_ms={} core_period_ms={} max_us={:.1f} loops={} user_mean_us={:.1f} "
                     "user_max_us={:.1f} jitter_max_us={} jitter_rms_us={:.1f} overruns={} "
                     "overruns_total={} headroom_pct={:.1f} headroom_min_pct={:.1f} "
                     "pass_jitter_max_us={} loop_offset_min_us={} loop_offset_max_us={}",
                     periodMs, core.periodMs.load(), double(passMaxNs) / 1000.0, starts.size(),
                     starts.empty() ? 0.0 : double(userSumNs) / 1000.0 / double(starts.size()),
                     double(userMaxNs) / 1000.0, jitterMax, jitterRms, overrunsInWindow,
                     core.overruns.load() - overrunsBefore, 100.0 * (periodD - meanUs) / periodD,
                     100.0 * (periodD - double(passMaxNs) / 1000.0) / periodD, passJitterMax,
                     offsetMin, offsetMax));
}

}  // namespace

void BenchRobot() {
  bool first = true;
  for (uint32_t periodMs : {20u, 10u, 5u, 2u}) {
    RunPeriod(periodMs, first);
    first = false;
  }
  // delete the last robot (clears the scheduler) and let the HAL see a disabled robot again
  pf::detail::ResetRobotMainForTest();
  auto& core = fakecore::Get();
  core.phase.store(pf::kCorePhaseStopped);
  core.outputs.store(false);
  core.packetSeq.fetch_add(1);
  pf::detail::PreCycle();
  pf::detail::PostCycle();
  Linef("BENCH-INFO robot dash_topics={} dash_sets={} dash_schemas={}", core.topics.load(),
        core.sets.load(), core.schemas.load());
}

}  // namespace qemu::bench
