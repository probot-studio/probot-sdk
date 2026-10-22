// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// The hosted robot program (prosdk/platform/RobotHost.hpp) against the HAL sim backend and a
// fake probot-core table: the test drives core's hooks (detail::Boot/PreCycle/PostCycle/
// EmergencyStop) on simulated time, and a pre-cycle callback registered before the robot host's
// mirrors the fake core state into DriverStationSim the way the ESP32 Driver Station backend maps
// it (enabled = outputs enabled, mode from the phase, active opmode in INIT/RUN, selected one
// while idle).
//
// Own executable: the fake table is installed once per process and the e-stop latches, so the
// e-stop test case is the last one (main.cpp runs the test cases in declaration order).

#include <stdint.h>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/framework/OpModeRobot.hpp"
#include "prosdk/framework/TimedRobot.hpp"
#include "prosdk/framework/TimesliceRobot.hpp"
#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/hal/HAL.h"
#include "prosdk/opmode/PeriodicOpMode.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/RobotHost.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/system/RobotController.hpp"
#include "prosdk/util/string.hpp"

namespace pf = prosdk::platform;
using prosdk::hal::RobotMode;

namespace {

// ---- fake probot-core ----

struct FakeCore {
  uint8_t phase = pf::kCorePhaseStopped;
  uint8_t selectedMode = pf::kCoreModeAuto;
  bool outputs = false;
  bool estop = false;
  bool attached = true;
  uint64_t selectedId = 0;
  uint64_t activeId = 0;
  uint32_t period = 20;
  uint32_t overruns = 0;
  std::vector<uint32_t> periodRequests;
};
FakeCore gCore;

std::mutex gConsoleMutex;
std::string gConsole;

uint8_t FakePhase() { return gCore.phase; }
uint8_t FakeSelectedMode() { return gCore.selectedMode; }
bool FakeOutputs() { return gCore.outputs && !gCore.estop; }
bool FakeEStopped() { return gCore.estop; }
bool FakeAttached() { return gCore.attached; }
bool FakeSetPeriod(uint32_t ms) {
  if (ms > pf::kCoreLoopPeriodMaxMs) {
    return false;
  }
  gCore.periodRequests.push_back(ms);
  gCore.period = ms == 0 ? 20 : ms;
  return true;
}
uint32_t FakePeriod() { return gCore.period; }
uint32_t FakeOverruns() { return gCore.overruns; }
uint64_t FakeSelectedId() { return gCore.selectedId; }
uint64_t FakeActiveId() { return gCore.activeId; }
void FakeConsole(const char* text, size_t len) {
  std::scoped_lock lock{gConsoleMutex};
  gConsole.append(text, len);
}

RobotMode ModeOfPhase(uint8_t phase) {
  switch (phase) {
    case pf::kCorePhaseAutoInit:
    case pf::kCorePhaseAutoRun:
      return RobotMode::AUTONOMOUS;
    case pf::kCorePhaseTeleopInit:
    case pf::kCorePhaseTeleopRun:
      return RobotMode::TELEOPERATED;
    case pf::kCorePhaseUtilityInit:
    case pf::kCorePhaseUtilityRun:
      return RobotMode::UTILITY;
    default:
      return gCore.selectedMode == pf::kCoreModeAuto     ? RobotMode::AUTONOMOUS
             : gCore.selectedMode == pf::kCoreModeUtility ? RobotMode::UTILITY
                                                          : RobotMode::TELEOPERATED;
  }
}

// core state -> HAL sim, like the ESP32 Driver Station pump (first pre-cycle callback)
void MirrorCore(void*) {
  namespace ds = prosdk::sim;
  const bool active = pf::IsCoreInitPhase(gCore.phase) || pf::IsCoreRunPhase(gCore.phase);
  ds::DriverStationSim::SetDsAttached(gCore.attached);
  ds::DriverStationSim::SetEnabled(FakeOutputs());
  ds::DriverStationSim::SetEStop(gCore.estop);
  const RobotMode mode = ModeOfPhase(gCore.phase);
  ds::DriverStationSim::SetRobotMode(mode);
  // an id of another mode is never reported (ESP32 backend)
  const int64_t id = static_cast<int64_t>(active ? gCore.activeId : gCore.selectedId);
  ds::DriverStationSim::SetOpMode(
      id != 0 && HAL_OpMode_GetRobotMode(id) == static_cast<HAL_RobotMode>(mode) ? id : 0);
  ds::DriverStationSim::NotifyNewData();
}

bool gBooted = false;

void InstallOnce() {
  static bool installed = [] {
    // the sim HAL first (the robot host's RunHALInitialization() finds it initialized)
    REQUIRE(HAL_Initialize());
    pf::CoreOps ops{};
    ops.abiVersion = pf::kCoreOpsAbiVersion;
    ops.size = sizeof(pf::CoreOps);
    ops.phase = FakePhase;
    ops.selectedMode = FakeSelectedMode;
    ops.outputsEnabled = FakeOutputs;
    ops.isEStopped = FakeEStopped;
    ops.isAttached = FakeAttached;
    ops.consoleWrite = FakeConsole;
    ops.setLoopPeriodMs = FakeSetPeriod;
    ops.loopPeriodMs = FakePeriod;
    ops.loopOverruns = FakeOverruns;
    ops.selectedOpModeId = FakeSelectedId;
    ops.activeOpModeId = FakeActiveId;
    REQUIRE(pf::InstallCoreOps(&ops));
    REQUIRE(pf::AddCycleCallback(pf::CyclePhase::kPre, MirrorCore, nullptr));
    return true;
  }();
  (void)installed;
}

// Starts a hosted program: what PROSDK_ROBOT_MAIN and core's boot do on the robot.
template <class Robot>
void StartProgram() {
  InstallOnce();
  pf::detail::ResetRobotMainForTest();
  prosdk::sim::PauseTiming();
  prosdk::sim::DriverStationSim::ResetData();
  gCore = FakeCore{};
  {
    std::scoped_lock lock{gConsoleMutex};
    gConsole.clear();
  }
  REQUIRE(pf::SetRobotMain([]() -> prosdk::RobotBase* { return new Robot(); }, "TestRobot"));
  REQUIRE_FALSE(pf::SetRobotMain([]() -> prosdk::RobotBase* { return new Robot(); }, "Twice"));
  if (!gBooted) {
    gBooted = true;
    pf::detail::Boot();  // first program: core's boot hook
  }
  // later programs: the boot callback runs at the start of the next cycle (late registration)
}

// One core loop pass, then simulated time advances by the current pass period.
void Pass() {
  pf::detail::PreCycle();
  pf::detail::PostCycle();
  prosdk::sim::StepTiming(prosdk::units::millisecond_t{static_cast<double>(gCore.period)});
}

void Passes(int n) {
  for (int i = 0; i < n; ++i) {
    Pass();
  }
}

// monotonic time in microseconds (sim time; whole for whole-millisecond steps)
int64_t NowUs() {
  return prosdk::RobotController::GetMonotonicTime() / 1000;
}

std::string Console() {
  std::scoped_lock lock{gConsoleMutex};
  return gConsole;
}

// Event log without the repeats of one periodic function.
struct EventLog {
  std::vector<std::string> events;
  std::string lastPeriodic;

  void Event(std::string_view name) {
    events.emplace_back(name);
    lastPeriodic.clear();
  }
  void Periodic(std::string_view name) {
    if (lastPeriodic != name) {
      events.emplace_back(name);
      lastPeriodic = name;
    }
  }
};

// ---- TimedRobot ----

class EventRobot : public prosdk::TimedRobot {
 public:
  static inline EventRobot* instance = nullptr;
  EventLog log;
  std::vector<int64_t> loopTimes;
  std::vector<int64_t> fastTimes;

  EventRobot() : TimedRobot(20_ms) {
    instance = this;
    log.Event("ctor");
    AddPeriodic([this] { fastTimes.push_back(NowUs()); }, 10_ms);
  }
  ~EventRobot() override { instance = nullptr; }

  void SimulationInit() override { log.Event("SimulationInit"); }
  void DriverStationConnected() override { log.Event("DriverStationConnected"); }
  void DisabledInit() override { log.Event("DisabledInit"); }
  void DisabledPeriodic() override { log.Periodic("DisabledPeriodic"); }
  void DisabledExit() override { log.Event("DisabledExit"); }
  void AutonomousInit() override { log.Event("AutonomousInit"); }
  void AutonomousPeriodic() override { log.Periodic("AutonomousPeriodic"); }
  void AutonomousExit() override { log.Event("AutonomousExit"); }
  void TeleopInit() override { log.Event("TeleopInit"); }
  void TeleopPeriodic() override { log.Periodic("TeleopPeriodic"); }
  void TeleopExit() override { log.Event("TeleopExit"); }
  void UtilityInit() override { log.Event("UtilityInit"); }
  void UtilityPeriodic() override { log.Periodic("UtilityPeriodic"); }
  void UtilityExit() override { log.Event("UtilityExit"); }
  void RobotPeriodic() override { loopTimes.push_back(NowUs()); }
};

void CheckSpacing(const std::vector<int64_t>& times, int64_t periodUs) {
  REQUIRE(times.size() >= 2u);
  for (size_t i = 1; i < times.size(); ++i) {
    INFO("index " << i);
    CHECK(times[i] - times[i - 1] == periodUs);
  }
}

}  // namespace

TEST_CASE("RobotHost TimedRobot lifecycle and AddPeriodic timing", "[robothost]") {
  StartProgram<EventRobot>();
  Pass();  // boot (late registration) and the first pass
  auto* robot = EventRobot::instance;
  REQUIRE(robot != nullptr);
  CHECK(pf::GetRobotProgramState() == pf::RobotProgramState::kRunning);
  // 20 ms loop + 10 ms AddPeriodic -> 10 ms core passes
  REQUIRE_FALSE(gCore.periodRequests.empty());
  CHECK(gCore.periodRequests.back() == 10u);
  CHECK(gCore.period == 10u);

  // disabled, autonomous INIT (outputs still off) -> disabled
  Passes(5);
  gCore.phase = pf::kCorePhaseAutoInit;
  Passes(4);
  // autonomous RUN
  gCore.phase = pf::kCorePhaseAutoRun;
  gCore.outputs = true;
  Passes(6);
  // transition: disabled
  gCore.phase = pf::kCorePhaseTransition;
  gCore.outputs = false;
  gCore.selectedMode = pf::kCoreModeTeleop;
  Passes(4);
  // teleop
  gCore.phase = pf::kCorePhaseTeleopRun;
  gCore.outputs = true;
  Passes(6);
  // utility (straight from teleop: a mode change while enabled)
  gCore.phase = pf::kCorePhaseUtilityRun;
  Passes(6);
  // stop
  gCore.phase = pf::kCorePhaseStopped;
  gCore.outputs = false;
  Passes(4);

  const std::vector<std::string> expected{
      "ctor",           "SimulationInit",   "DriverStationConnected", "DisabledInit",
      "DisabledPeriodic", "DisabledExit",   "AutonomousInit",         "AutonomousPeriodic",
      "AutonomousExit", "DisabledInit",     "DisabledPeriodic",       "DisabledExit",
      "TeleopInit",     "TeleopPeriodic",   "TeleopExit",             "UtilityInit",
      "UtilityPeriodic", "UtilityExit",     "DisabledInit",           "DisabledPeriodic",
  };
  CHECK(robot->log.events == expected);

  // upstream schedule: the main loop every 20 ms, the AddPeriodic callback every 10 ms, each on
  // a core pass (time only advances between passes)
  CheckSpacing(robot->loopTimes, 20000);
  CheckSpacing(robot->fastTimes, 10000);
  CHECK(robot->fastTimes.size() >= 2 * robot->loopTimes.size() - 1);
}

TEST_CASE("RobotHost re-aligns after a late pass", "[robothost]") {
  StartProgram<EventRobot>();
  Pass();
  auto* robot = EventRobot::instance;
  REQUIRE(robot != nullptr);
  Passes(4);
  // core ran late by 3 ms and restarted its grid
  prosdk::sim::StepTiming(3_ms);
  ++gCore.overruns;
  Passes(6);
  CHECK(Console().find("overrun") != std::string::npos);
  // after the late pass the callbacks keep their periods on the new grid
  const auto& t = robot->fastTimes;
  REQUIRE(t.size() >= 6u);
  for (size_t i = t.size() - 4; i < t.size(); ++i) {
    CHECK(t[i] - t[i - 1] == 10000);
  }
}

TEST_CASE("RobotHost runs each callback once per pass despite pass jitter", "[robothost]") {
  StartProgram<EventRobot>();
  Pass();
  auto* robot = EventRobot::instance;
  REQUIRE(robot != nullptr);
  Passes(3);
  // passes start up to 4 ms (less than half a 10 ms pass) after their grid time
  const int jitterMs[] = {4, 0, 3, 1, 4, 4, 0, 2, 3, 0, 4, 1};
  size_t fast = robot->fastTimes.size();
  size_t loops = robot->loopTimes.size();
  for (int j : jitterMs) {
    prosdk::sim::StepTiming(prosdk::units::millisecond_t{static_cast<double>(j)});
    pf::detail::PreCycle();
    pf::detail::PostCycle();
    prosdk::sim::StepTiming(prosdk::units::millisecond_t{static_cast<double>(10 - j)});
    CHECK(robot->fastTimes.size() == ++fast);
  }
  CHECK(robot->loopTimes.size() == loops + 6);
}

namespace {

class SliceRobot : public prosdk::TimesliceRobot {
 public:
  static inline SliceRobot* instance = nullptr;
  std::vector<int64_t> loop, slice1, slice2;

  SliceRobot() : TimesliceRobot{2_ms, 10_ms} {
    instance = this;
    Schedule([this] { slice1.push_back(NowUs()); }, 3_ms);  // offset 2 ms
    Schedule([this] { slice2.push_back(NowUs()); }, 1_ms);  // offset 5 ms
  }
  ~SliceRobot() override { instance = nullptr; }
  void RobotPeriodic() override { loop.push_back(NowUs()); }
};

}  // namespace

TEST_CASE("RobotHost TimesliceRobot keeps its time slices", "[robothost]") {
  StartProgram<SliceRobot>();
  Pass();
  auto* robot = SliceRobot::instance;
  REQUIRE(robot != nullptr);
  // main loop 20 ms (TimedRobot default), slices at offsets 2 and 5 ms in a 10 ms period -> 1 ms
  // core passes
  CHECK(gCore.period == 1u);
  Passes(100);
  REQUIRE(robot->loop.size() >= 4u);
  REQUIRE(robot->slice1.size() >= 4u);
  REQUIRE(robot->slice2.size() >= 4u);
  CheckSpacing(robot->loop, 20000);  // TimedRobot's default period (upstream)
  CheckSpacing(robot->slice1, 10000);
  CheckSpacing(robot->slice2, 10000);
  // each slice at its offset after the robot loop of the same period
  const int64_t l0 = robot->loop.front();
  auto after = [&](const std::vector<int64_t>& v) -> int64_t {
    for (int64_t t : v) {
      if (t >= l0) {
        return t - l0;
      }
    }
    return -1;
  };
  CHECK(after(robot->slice1) == 2000);
  CHECK(after(robot->slice2) == 5000);
}

namespace {

class OpRobot;
EventLog* gOpLog = nullptr;

// logs into the robot's log; nothing after the robot's members are gone (its base destroys the
// current opmode)
void OpEvent(const std::string& name) {
  if (gOpLog != nullptr) {
    gOpLog->Event(name);
  }
}
void OpPeriodic(const std::string& name) {
  if (gOpLog != nullptr) {
    gOpLog->Periodic(name);
  }
}

class LoggedOpMode : public prosdk::PeriodicOpMode {
 public:
  explicit LoggedOpMode(std::string name) : m_name{std::move(name)} { OpEvent(m_name + " ctor"); }
  ~LoggedOpMode() override { OpEvent(m_name + " dtor"); }
  void DisabledPeriodic() override { OpPeriodic(m_name + " DisabledPeriodic"); }
  void Start() override { OpEvent(m_name + " Start"); }
  void Periodic() override { OpPeriodic(m_name + " Periodic"); }
  void End() override { OpEvent(m_name + " End"); }

 private:
  std::string m_name;
};

class AutoA : public LoggedOpMode {
 public:
  explicit AutoA(OpRobot&) : LoggedOpMode{"AutoA"} {}
};
class AutoB : public LoggedOpMode {
 public:
  explicit AutoB(OpRobot&) : LoggedOpMode{"AutoB"} {}
};
std::vector<int64_t> gDriveExtra;

class Drive : public LoggedOpMode {
 public:
  // a PeriodicOpMode callback is timed from the opmode's construction
  explicit Drive(OpRobot&) : LoggedOpMode{"Drive"} {
    AddPeriodic([] { gDriveExtra.push_back(NowUs()); }, 20_ms);
  }
};
class Calibrate : public LoggedOpMode {
 public:
  explicit Calibrate(OpRobot&) : LoggedOpMode{"Calibrate"} {}
};

class OpRobot : public prosdk::OpModeRobot<OpRobot> {
 public:
  static inline OpRobot* instance = nullptr;
  EventLog log;

  OpRobot() {
    instance = this;
    gOpLog = &log;
    AddOpMode<AutoA>(prosdk::RobotMode::AUTONOMOUS, "Auto A");
    AddOpMode<AutoB>(prosdk::RobotMode::AUTONOMOUS, "Auto B");
    AddOpMode<Drive>(prosdk::RobotMode::TELEOPERATED, "Drive");
    AddOpMode<Calibrate>(prosdk::RobotMode::UTILITY, "Calibrate");
    PublishOpModes();
  }
  ~OpRobot() override {
    instance = nullptr;
    gOpLog = nullptr;
  }

  void DisabledInit() override { log.Event("DisabledInit"); }
  void DisabledExit() override { log.Event("DisabledExit"); }
};

uint64_t OpModeId(std::string_view name) {
  for (const auto& option : prosdk::sim::DriverStationSim::GetOpModeOptions()) {
    if (prosdk::util::to_string_view(&option.name) == name) {
      return static_cast<uint64_t>(option.id);
    }
  }
  return 0;
}

}  // namespace

TEST_CASE("RobotHost OpModeRobot selection and dispatch", "[robothost]") {
  gDriveExtra.clear();
  StartProgram<OpRobot>();
  Pass();
  auto* robot = OpRobot::instance;
  REQUIRE(robot != nullptr);
  CHECK(gCore.period == 20u);
  const uint64_t autoA = OpModeId("Auto A");
  const uint64_t autoB = OpModeId("Auto B");
  const uint64_t drive = OpModeId("Drive");
  const uint64_t calibrate = OpModeId("Calibrate");
  REQUIRE(autoA != 0);
  REQUIRE(autoB != 0);
  REQUIRE(drive != 0);
  REQUIRE(calibrate != 0);
  // stable ids: the same name and mode gives the same id
  CHECK(OpModeId("Auto A") == autoA);

  // select Auto A while idle, then change to Auto B (still idle)
  gCore.selectedId = autoA;
  Passes(3);
  gCore.selectedId = autoB;
  Passes(3);
  // run autonomous with Auto B
  gCore.phase = pf::kCorePhaseAutoInit;
  gCore.activeId = autoB;
  Passes(2);
  gCore.phase = pf::kCorePhaseAutoRun;
  gCore.outputs = true;
  Passes(3);
  // autonomous over; Drive preselected
  gCore.phase = pf::kCorePhaseTransition;
  gCore.outputs = false;
  gCore.selectedMode = pf::kCoreModeTeleop;
  gCore.selectedId = drive;
  Passes(3);
  gCore.phase = pf::kCorePhaseTeleopRun;
  gCore.activeId = drive;
  gCore.outputs = true;
  Passes(3);
  // stop, then utility
  gCore.phase = pf::kCorePhaseStopped;
  gCore.outputs = false;
  gCore.selectedMode = pf::kCoreModeUtility;
  gCore.selectedId = calibrate;
  Passes(3);
  gCore.phase = pf::kCorePhaseUtilityRun;
  gCore.activeId = calibrate;
  gCore.outputs = true;
  Passes(3);

  // upstream OpModeRobotBase::LoopFunc semantics: the selected opmode is constructed while
  // disabled (and replaced when the selection changes), started on enable, ended and destroyed on
  // disable; a new selection is constructed on the next loop
  const std::vector<std::string> expected{
      "AutoA ctor",     "AutoA DisabledPeriodic", "AutoA dtor",
      "AutoB ctor",     "AutoB DisabledPeriodic",
      "DisabledExit",   "AutoB Start",            "AutoB Periodic",
      "AutoB End",      "AutoB dtor",
      "Drive ctor",     "Drive DisabledPeriodic", "DisabledInit",   "Drive DisabledPeriodic",
      "DisabledExit",   "Drive Start",            "Drive Periodic",
      "Drive End",      "Drive dtor",
      "Calibrate ctor", "Calibrate DisabledPeriodic", "DisabledInit", "Calibrate DisabledPeriodic",
      "DisabledExit",   "Calibrate Start",        "Calibrate Periodic",
  };
  CHECK(robot->log.events == expected);

  // Drive's own 20 ms callback ran on core passes without changing the 20 ms pass period
  for (uint32_t ms : gCore.periodRequests) {
    CHECK(ms == 20u);
  }
  CHECK(gDriveExtra.size() >= 3u);
  CheckSpacing(gDriveExtra, 20000);
}

namespace {

class ThrowingRobot : public prosdk::TimedRobot {
 public:
  ThrowingRobot() { throw std::runtime_error("constructor failure for the test"); }
};

class DirectRobot : public prosdk::RobotBase {
 public:
  void StartCompetition() override {}
  void EndCompetition() override {}
};

class FastRobot : public prosdk::TimedRobot {
 public:
  FastRobot() : TimedRobot(0.5_ms) {}
};

class LoopThrowRobot : public prosdk::TimedRobot {
 public:
  static inline int calls = 0;
  void TeleopPeriodic() override {
    ++calls;
    throw std::runtime_error("loop failure for the test");
  }
};

}  // namespace

TEST_CASE("RobotHost halts a program that cannot run", "[robothost]") {
  SECTION("constructor exception") {
    StartProgram<ThrowingRobot>();
    Pass();
    CHECK(pf::GetRobotProgramState() == pf::RobotProgramState::kHalted);
    CHECK(pf::RobotProgramHalted());
    CHECK(Console().find("constructor failure for the test") != std::string::npos);
    CHECK(Console().find("halted") != std::string::npos);
  }
  SECTION("robot class without a periodic loop") {
    StartProgram<DirectRobot>();
    Pass();
    CHECK(pf::GetRobotProgramState() == pf::RobotProgramState::kHalted);
    CHECK(Console().find("derives from RobotBase directly") != std::string::npos);
  }
  SECTION("loop period below 1 ms") {
    StartProgram<FastRobot>();
    Pass();
    CHECK(pf::GetRobotProgramState() == pf::RobotProgramState::kHalted);
    CHECK(Console().find("is not supported") != std::string::npos);
  }
  SECTION("loop exception") {
    LoopThrowRobot::calls = 0;
    StartProgram<LoopThrowRobot>();
    Pass();
    gCore.phase = pf::kCorePhaseTeleopRun;
    gCore.outputs = true;
    Passes(5);
    CHECK(pf::GetRobotProgramState() == pf::RobotProgramState::kHalted);
    CHECK(LoopThrowRobot::calls == 1);
    CHECK(Console().find("loop failure for the test") != std::string::npos);
  }
  pf::detail::ResetRobotMainForTest();
  CHECK_FALSE(pf::RobotProgramHalted());
}

// last: the e-stop latches for the rest of the process
TEST_CASE("RobotHost never runs the robot after an e-stop", "[robothost]") {
  StartProgram<EventRobot>();
  Pass();
  auto* robot = EventRobot::instance;
  REQUIRE(robot != nullptr);
  gCore.phase = pf::kCorePhaseTeleopRun;
  gCore.outputs = true;
  Passes(6);
  REQUIRE(robot->log.events.back() == "TeleopPeriodic");
  const size_t loops = robot->loopTimes.size();
  const size_t fast = robot->fastTimes.size();

  gCore.estop = true;
  gCore.phase = pf::kCorePhaseStopped;
  pf::detail::EmergencyStop();
  CHECK(pf::EmergencyStopped());
  // on the robot core deleted the user task; even if a cycle ran, the robot is not touched
  Passes(6);
  CHECK(pf::GetRobotProgramState() == pf::RobotProgramState::kEStopped);
  CHECK(robot->loopTimes.size() == loops);
  CHECK(robot->fastTimes.size() == fast);
  CHECK(robot->log.events.back() == "TeleopPeriodic");
}
