// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// The hosted robot program (prosdk/platform/RobotHost.hpp, design/PORT.md section 8): constructs
// the robot on probot-core's user task at boot and runs its periodic callbacks from core's
// pre-cycle callback.
//
// Scheduling: core runs one loop pass per period on a fixed 1 ms tick grid (R6). The robot's
// callbacks (TimedRobot/OpModeRobot main loop, AddPeriodic, TimesliceRobot slots, PeriodicOpMode
// callbacks) keep their upstream schedule in PeriodicPriorityQueue; the host
//   - sets core's pass period to the greatest common divisor (whole milliseconds, at most
//     1000 ms) of the callback periods and of their phases relative to the robot's start time, so
//     every expiration falls on a pass, and follows it when callbacks are added or removed. A
//     callback whose phase is not a whole number of milliseconds (a PeriodicOpMode callback is
//     timed from the opmode's construction, somewhere inside a pass) is moved to the nearest
//     pass instead of shrinking the pass period; one with a period below 1 ms runs once per pass;
//   - aligns the schedule with core's passes (it shifts the robot's start time and every
//     expiration by less than half a pass). It re-aligns after a period change and after a late
//     pass (core restarts its grid; no catch-up, like upstream's "skip the periods it is behind");
//   - runs every callback that is due by the middle of the next pass (PeriodicPriorityQueue::
//     RunDue with half a pass of lookahead: upstream order and rescheduling), so that the run-time
//     jitter of the earlier pre-cycle callbacks never makes a due callback miss its pass.

#include <stdint.h>

#include <atomic>
#include <chrono>
#include <exception>
#include <numeric>

#include "prosdk/framework/RobotBase.hpp"
#include "prosdk/hal/DriverStation.hpp"
#include "prosdk/internal/PeriodicPriorityQueue.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/RobotHost.hpp"
#include "prosdk/platform/dashboard/Dashboard.hpp"
#include "prosdk/platform/dashboard/DashboardStore.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/system/RobotController.hpp"

namespace prosdk::platform {

namespace {

using std::chrono::nanoseconds;
constexpr int64_t kNsPerMs = 1000000;

constinit std::atomic<RobotFactory> gFactory{nullptr};
constinit std::atomic<const char*> gName{""};
constinit std::atomic<RobotProgramState> gState{RobotProgramState::kNone};

// Everything below is only touched on core's user task (boot and pre-cycle callbacks).
struct Host {
  prosdk::RobotBase* robot = nullptr;
  prosdk::internal::PeriodicPriorityQueue* callbacks = nullptr;
  nanoseconds* startTime = nullptr;

  uint64_t version = ~uint64_t{0};  // queue version the pass period was computed for
  uint32_t passMs = 0;              // core's pass period (what the schedule is aligned to)
  uint64_t passes = 0;              // passes started
  uint64_t alignPass = 0;           // align the schedule at this pass
  uint32_t overruns = 0;            // core's overrun count at the previous pass
  int64_t lastOverrunReport = 0;    // monotonic ns
  bool overrunReported = false;
  bool warnedSubMs = false;
  bool warnedFraction = false;
  bool warnedNoLoopRate = false;
};
Host gHost;

void RobotPass(void*);

// Telemetry is written to the store as it is logged; this publishes what waited for the store.
void FlushDashboard() {
  dashboard::FlushPending();
}

void Halt(const char* why) {
  RemoveCycleCallback(CyclePhase::kPre, RobotPass, nullptr);
  gState.store(RobotProgramState::kHalted, std::memory_order_release);
  ConsolePrint("[prosdk] robot program {} halted: {}. Outputs stay disabled until reboot.\n",
               gName.load(std::memory_order_relaxed), why);
}

// Runs fn like upstream StartRobot runs the robot: exceptions are reported, not propagated.
template <typename Fn>
bool Guard(Fn&& fn) {
  try {
    fn();
    return true;
  } catch (const prosdk::RuntimeError& e) {
    e.Report();
    WPILIB_ReportError(
        err::Error,
        "The robot program quit unexpectedly."
        " This is usually due to a code error.\n"
        "  The above stacktrace can help determine where the error occurred.\n"
        "  See https://wpilib.org/stacktrace for more information.\n");
  } catch (const std::exception& e) {
    prosdk::hal::SendError(1, err::Error, e.what(), "", "", 1);
  } catch (...) {
    // upstream lets this terminate the program; on the robot that would reboot it in a loop
    WPILIB_ReportError(err::Error,
                       "The robot program quit unexpectedly (an exception that is not a "
                       "std::exception).");
  }
  return false;
}

uint32_t LargestDivisorUpTo(uint32_t n, uint32_t limit) {
  if (n <= limit) {
    return n;
  }
  for (uint32_t d = limit; d > 1; --d) {
    if (n % d == 0) {
      return d;
    }
  }
  return 1;
}

int64_t RoundToMs(int64_t ns) {
  return (ns + kNsPerMs / 2) / kNsPerMs;
}

// Positive remainder.
int64_t Mod(int64_t a, int64_t m) {
  const int64_t r = a % m;
  return r < 0 ? r + m : r;
}

// Moves every expiration that is not on the pass grid (startTime + k * pass) to the nearest grid
// point (on the robot nothing can run between two passes anyway) and gives a callback with a
// period below 1 ms the pass period.
void SnapToGrid() {
  const int64_t pass = static_cast<int64_t>(gHost.passMs) * kNsPerMs;
  const int64_t base = gHost.startTime->count();
  gHost.callbacks->Update([&](prosdk::internal::PeriodicPriorityQueue::Callback& cb) {
    if (cb.period.count() < kNsPerMs) {
      cb.period = nanoseconds{pass};
    }
    const int64_t off = Mod(cb.expirationTime.count() - base, pass);
    if (off != 0) {
      cb.expirationTime += nanoseconds{off <= pass / 2 ? -off : pass - off};
    }
  });
}

// Pass period from the callback periods and phases; tells core when it changed.
void UpdatePassPeriod() {
  auto& queue = *gHost.callbacks;
  gHost.version = queue.GetVersion();
  const int64_t base = gHost.startTime->count();

  uint64_t gcdMs = 0;
  bool subMs = false;
  bool fraction = false;
  queue.Update([&](prosdk::internal::PeriodicPriorityQueue::Callback& cb) {
    const int64_t period = cb.period.count();
    if (period < kNsPerMs) {
      subMs = true;  // runs once per pass
      return;
    }
    if (period % kNsPerMs != 0) {
      fraction = true;
    }
    gcdMs = std::gcd<uint64_t, uint64_t>(gcdMs, static_cast<uint64_t>(RoundToMs(period)));
    // phase relative to the robot's start time; one that is not whole milliseconds is snapped
    const int64_t phase = Mod(cb.expirationTime.count() - base, period);
    if (phase % kNsPerMs == 0) {
      gcdMs = std::gcd<uint64_t, uint64_t>(gcdMs, static_cast<uint64_t>(phase / kNsPerMs));
    }
  });

  if (subMs && !gHost.warnedSubMs) {
    gHost.warnedSubMs = true;
    WPILIB_ReportError(err::Error,
                       "A periodic callback has a period below 1 ms; probot-core's loop runs at "
                       "most every 1 ms, so it runs once per loop pass instead.");
  }
  if (fraction && !gHost.warnedFraction) {
    gHost.warnedFraction = true;
    WPILIB_ReportWarning(
        "A periodic callback period is not a whole number of milliseconds; probot-core's loop "
        "runs on a 1 ms grid, so it runs up to one loop pass late.");
  }

  if (gcdMs > UINT32_MAX) {
    gcdMs = kCoreLoopPeriodMaxMs;  // periods of weeks: anything that divides them is fine
  }
  uint32_t passMs = gcdMs == 0 ? (subMs ? kCoreLoopPeriodMinMs : kCoreLoopPeriodDefaultMs)
                               : LargestDivisorUpTo(static_cast<uint32_t>(gcdMs),
                                                    kCoreLoopPeriodMaxMs);
  if (passMs != gHost.passMs) {
    if (!SetLoopPeriodMs(passMs)) {
      const uint32_t corePeriod = LoopPeriodMs();
      if (!gHost.warnedNoLoopRate) {
        gHost.warnedNoLoopRate = true;
        WPILIB_ReportError(err::Error,
                           "probot-core refused the loop period {} ms (it needs probot-core with "
                           "setLoopPeriodMs, design/core-requests.md R6); the robot loop runs on "
                           "core's {} ms passes instead.",
                           passMs, corePeriod);
      }
      passMs = corePeriod;
    }
    if (passMs != gHost.passMs) {
      gHost.passMs = passMs;
      gHost.alignPass = gHost.passes;  // the new period applies from the next pass
    }
  }
  SnapToGrid();
}

// Moves the schedule (and the robot's start time, which later AddPeriodic() calls use) so that
// the pass grid falls on core's passes.
void AlignSchedule(int64_t now) {
  const int64_t pass = static_cast<int64_t>(gHost.passMs) * kNsPerMs;
  int64_t delta = Mod(now - gHost.startTime->count(), pass);
  if (delta > pass / 2) {
    delta -= pass;
  }
  if (delta != 0) {
    *gHost.startTime += nanoseconds{delta};
    gHost.callbacks->Update([&](prosdk::internal::PeriodicPriorityQueue::Callback& cb) {
      cb.expirationTime += nanoseconds{delta};
    });
  }
}

void ReportOverruns(int64_t now, uint32_t overruns) {
  constexpr int64_t kReportInterval = 1000 * kNsPerMs;
  if (gHost.overrunReported && now - gHost.lastOverrunReport < kReportInterval) {
    return;
  }
  gHost.overrunReported = true;
  gHost.lastOverrunReport = now;
  WPILIB_ReportWarning("Loop pass of {} ms overrun ({} late passes since boot)", gHost.passMs,
                       overruns);
}

void RobotPass(void*) {
  if (gState.load(std::memory_order_acquire) != RobotProgramState::kRunning) {
    return;
  }
  if (EmergencyStopped()) {
    // never touch the robot again: on the robot core already deleted this task
    gState.store(RobotProgramState::kEStopped, std::memory_order_release);
    RemoveCycleCallback(CyclePhase::kPre, RobotPass, nullptr);
    return;
  }

  const uint64_t pass = gHost.passes++;
  if (gHost.callbacks->GetVersion() != gHost.version) {
    UpdatePassPeriod();
  }
  const int64_t now = prosdk::RobotController::GetMonotonicTime();
  const uint32_t overruns = LoopOverruns();
  if (overruns != gHost.overruns) {
    gHost.overruns = overruns;
    ReportOverruns(now, overruns);
    gHost.alignPass = pass;  // core restarted its pass grid with this pass
  }
  if (pass == gHost.alignPass) {
    AlignSchedule(now);
  }

  const nanoseconds lookahead{static_cast<int64_t>(gHost.passMs) * kNsPerMs / 2};
  if (!Guard([&] { gHost.callbacks->RunDue(nanoseconds{now}, lookahead); })) {
    Halt("its loop threw an exception");
  }
}

void BootRobot(void*) {
  if (gState.load(std::memory_order_acquire) != RobotProgramState::kRegistered) {
    return;
  }
  if (EmergencyStopped()) {
    gState.store(RobotProgramState::kEStopped, std::memory_order_release);
    return;
  }
  const RobotFactory factory = gFactory.load(std::memory_order_acquire);

  // upstream RobotBase registers the NetworkTables backends and flushes NetworkTables here
  SetRobotHostHooks({&RegisterDashboardBackends, &FlushDashboard});

  static bool halInitialized = false;
  if (!halInitialized) {
    if (prosdk::RunHALInitialization() != 0) {
      Halt("the HAL could not be initialized");
      return;
    }
    halInitialized = true;
  }

  prosdk::RobotBase* robot = nullptr;
  if (!Guard([&] { robot = factory(); }) || robot == nullptr) {
    Halt("its constructor threw an exception");
    return;
  }
  gHost.robot = robot;

  prosdk::RobotBase::HostedLoop loop;
  if (!Guard([&] { loop = robot->StartHosted(); })) {
    Halt("its startup threw an exception");
    return;
  }
  if (loop.callbacks == nullptr || loop.startTime == nullptr) {
    WPILIB_ReportError(err::Error,
                       "{} derives from RobotBase directly; on the robot the program must be a "
                       "TimedRobot, TimesliceRobot or OpModeRobot (StartCompetition() cannot "
                       "block probot-core's loop).",
                       gName.load(std::memory_order_relaxed));
    Halt("its robot class has no periodic loop");
    return;
  }
  const double periodMs = loop.period.value() * 1000.0;
  if (!(periodMs >= kCoreLoopPeriodMinMs && periodMs <= kCoreLoopPeriodMaxMs)) {
    WPILIB_ReportError(err::Error,
                       "Robot loop period {} s is not supported: probot-core runs its loop every "
                       "1 ms to 1000 ms.",
                       loop.period.value());
    Halt("its loop period is outside 1 ms..1000 ms");
    return;
  }
  gHost.callbacks = loop.callbacks;
  gHost.startTime = loop.startTime;
  gHost.passes = 0;
  UpdatePassPeriod();
  gHost.alignPass = 0;
  gHost.overruns = LoopOverruns();

  gState.store(RobotProgramState::kRunning, std::memory_order_release);
  // after the HAL's (output gate, Driver Station pump) and the dashboard's pre-cycle callbacks
  if (!AddCycleCallback(CyclePhase::kPre, RobotPass, nullptr)) {
    Halt("probot-core's cycle callback table is full");
  }
}

}  // namespace

bool SetRobotMain(RobotFactory factory, const char* name) noexcept {
  if (factory == nullptr) {
    return false;
  }
  RobotFactory expected = nullptr;
  if (!gFactory.compare_exchange_strong(expected, factory, std::memory_order_acq_rel)) {
    return false;
  }
  gName.store(name != nullptr ? name : "", std::memory_order_relaxed);
  gState.store(RobotProgramState::kRegistered, std::memory_order_release);
  if (!AddBootCallback(BootRobot, nullptr)) {
    gState.store(RobotProgramState::kHalted, std::memory_order_release);
    ConsoleWrite("[prosdk] robot program not started: probot-core's boot callback table is full\n");
    return false;
  }
  return true;
}

RobotProgramState GetRobotProgramState() noexcept {
  const RobotProgramState state = gState.load(std::memory_order_acquire);
  // on the robot core deletes the user task at an e-stop, so RobotPass never records it
  if ((state == RobotProgramState::kRunning || state == RobotProgramState::kRegistered) &&
      EmergencyStopped()) {
    return RobotProgramState::kEStopped;
  }
  return state;
}

bool RobotProgramHalted() noexcept {
  return gState.load(std::memory_order_acquire) == RobotProgramState::kHalted;
}

namespace detail {

void ResetRobotMainForTest() {
  RemoveCycleCallback(CyclePhase::kPre, RobotPass, nullptr);
  delete gHost.robot;
  gHost = Host{};
  gFactory.store(nullptr, std::memory_order_release);
  gName.store("", std::memory_order_relaxed);
  gState.store(RobotProgramState::kNone, std::memory_order_release);
}

}  // namespace detail

}  // namespace prosdk::platform
