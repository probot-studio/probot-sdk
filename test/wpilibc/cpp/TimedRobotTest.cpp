// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/framework/TimedRobot.hpp"

#include <stdint.h>

#include <atomic>
#include <string>
#include <thread>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/platform/RobotHost.hpp"
#include "prosdk/simulation/AlertSim.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/util/timestamp.hpp"

using namespace prosdk;

inline constexpr auto PERIOD = 20_ms;

namespace {
class TimedRobotTest {
 public:
  TimedRobotTest() {
    prosdk::telemetry::TelemetryRegistry::Reset();
    prosdk::tunables::TunableRegistry::Reset();
    prosdk::sim::AlertSim::ResetData();
    prosdk::sim::PauseTiming();
    prosdk::sim::SetProgramStarted(false);
  }

  ~TimedRobotTest() {
    prosdk::telemetry::TelemetryRegistry::Reset();
    prosdk::tunables::TunableRegistry::Reset();
    prosdk::sim::AlertSim::ResetData();
    prosdk::sim::ResumeTiming();
  }
};

class MockRobot : public TimedRobot {
 public:
  std::atomic<uint32_t> m_simulationInitCount{0};
  std::atomic<uint32_t> m_disabledInitCount{0};
  std::atomic<uint32_t> m_autonomousInitCount{0};
  std::atomic<uint32_t> m_teleopInitCount{0};
  std::atomic<uint32_t> m_utilityInitCount{0};

  std::atomic<uint32_t> m_disabledExitCount{0};
  std::atomic<uint32_t> m_autonomousExitCount{0};
  std::atomic<uint32_t> m_teleopExitCount{0};
  std::atomic<uint32_t> m_utilityExitCount{0};

  std::atomic<uint32_t> m_robotPeriodicCount{0};
  std::atomic<uint32_t> m_simulationPeriodicCount{0};
  std::atomic<uint32_t> m_disabledPeriodicCount{0};
  std::atomic<uint32_t> m_autonomousPeriodicCount{0};
  std::atomic<uint32_t> m_teleopPeriodicCount{0};
  std::atomic<uint32_t> m_utilityPeriodicCount{0};

  MockRobot() : TimedRobot{PERIOD} {}

  void SimulationInit() override { m_simulationInitCount++; }

  void DisabledInit() override { m_disabledInitCount++; }

  void AutonomousInit() override { m_autonomousInitCount++; }

  void TeleopInit() override { m_teleopInitCount++; }

  void UtilityInit() override { m_utilityInitCount++; }

  void RobotPeriodic() override { m_robotPeriodicCount++; }

  void SimulationPeriodic() override { m_simulationPeriodicCount++; }

  void DisabledPeriodic() override { m_disabledPeriodicCount++; }

  void AutonomousPeriodic() override { m_autonomousPeriodicCount++; }

  void TeleopPeriodic() override { m_teleopPeriodicCount++; }

  void UtilityPeriodic() override { m_utilityPeriodicCount++; }

  void DisabledExit() override { m_disabledExitCount++; }

  void AutonomousExit() override { m_autonomousExitCount++; }

  void TeleopExit() override { m_teleopExitCount++; }

  void UtilityExit() override { m_utilityExitCount++; }
};
}  // namespace

TEST_CASE_METHOD(TimedRobotTest, "TimedRobotTest DisabledMode", "[wpilibc]") {
  MockRobot robot;

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetEnabled(false);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(0u == robot.m_robotPeriodicCount);
  CHECK(0u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(1u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(1u == robot.m_robotPeriodicCount);
  CHECK(1u == robot.m_simulationPeriodicCount);
  CHECK(1u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(1u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(2u == robot.m_robotPeriodicCount);
  CHECK(2u == robot.m_simulationPeriodicCount);
  CHECK(2u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  robot.EndCompetition();
  robotThread.join();
}

// prosdk: upstream ConstructorPublishesProgramStartTime and
// ConstructorRegistersTelemetryAndTunableBackends check the NetworkTables topics
// /Robot/ProgramStartTime, /Telemetry and /Tunables; ntcore is not ported. RobotBase calls the
// robot host hooks there instead (prosdk/platform/RobotHost.hpp), which this test checks.
namespace {
std::atomic<uint32_t> gRobotBaseConstructedCount{0};
std::atomic<uint32_t> gFlushTelemetryCount{0};
}  // namespace

TEST_CASE_METHOD(TimedRobotTest, "TimedRobotTest RobotHostHooks",
                 "[wpilibc]") {
  gRobotBaseConstructedCount = 0;
  gFlushTelemetryCount = 0;
  prosdk::platform::SetRobotHostHooks(
      {.robotBaseConstructed = [] { ++gRobotBaseConstructedCount; },
       .flushTelemetry = [] { ++gFlushTelemetryCount; }});
  {
    MockRobot robot;
    CHECK(1u == gRobotBaseConstructedCount);
    CHECK(0u == gFlushTelemetryCount);

    std::thread robotThread{[&] { robot.StartCompetition(); }};
    prosdk::sim::WaitForProgramStart();

    prosdk::sim::DriverStationSim::SetEnabled(false);
    prosdk::sim::DriverStationSim::NotifyNewData();

    prosdk::sim::StepTiming(PERIOD);
    CHECK(1u == robot.m_robotPeriodicCount);
    CHECK(1u == gFlushTelemetryCount);

    prosdk::sim::StepTiming(PERIOD);
    CHECK(2u == robot.m_robotPeriodicCount);
    CHECK(2u == gFlushTelemetryCount);

    robot.EndCompetition();
    robotThread.join();
  }
  prosdk::platform::SetRobotHostHooks({});
}

TEST_CASE_METHOD(TimedRobotTest,
                 "TimedRobotTest ConstructorMapsWarningsToAlerts",
                 "[wpilibc]") {
  {
    MockRobot robot;

    prosdk::telemetry::TelemetryRegistry::ReportWarning("/bad",
                                                     "telemetry test warning");
    prosdk::tunables::TunableRegistry::ReportWarning("tunable test warning");

    auto alerts = prosdk::sim::AlertSim::GetActive();
    CHECK(alerts.size() == 2);

    bool sawTelemetry = false;
    bool sawTunable = false;
    for (const auto& alert : alerts) {
      if (alert.group == "Telemetry" &&
          alert.text.find("telemetry test warning") != std::string::npos) {
        sawTelemetry = true;
      }
      if (alert.group == "Tunables" &&
          alert.text.find("tunable test warning") != std::string::npos) {
        sawTunable = true;
      }
    }
    CHECK(sawTelemetry);
    CHECK(sawTunable);
  }

  CHECK(prosdk::sim::AlertSim::GetActive().empty());
}

TEST_CASE_METHOD(TimedRobotTest, "TimedRobotTest AutonomousMode", "[wpilibc]") {
  MockRobot robot;

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::SetRobotMode(hal::RobotMode::AUTONOMOUS);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(0u == robot.m_robotPeriodicCount);
  CHECK(0u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(1u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(1u == robot.m_robotPeriodicCount);
  CHECK(1u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(1u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(1u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(2u == robot.m_robotPeriodicCount);
  CHECK(2u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(2u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(TimedRobotTest, "TimedRobotTest TeleopMode", "[wpilibc]") {
  MockRobot robot;

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::SetRobotMode(hal::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(0u == robot.m_robotPeriodicCount);
  CHECK(0u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(1u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(1u == robot.m_robotPeriodicCount);
  CHECK(1u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(1u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(1u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(2u == robot.m_robotPeriodicCount);
  CHECK(2u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(2u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(TimedRobotTest, "TimedRobotTest UtilityMode", "[wpilibc]") {
  MockRobot robot;
  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::SetRobotMode(hal::RobotMode::UTILITY);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(0u == robot.m_robotPeriodicCount);
  CHECK(0u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(0u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(1u == robot.m_utilityInitCount);

  CHECK(1u == robot.m_robotPeriodicCount);
  CHECK(1u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(1u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(1u == robot.m_utilityInitCount);

  CHECK(2u == robot.m_robotPeriodicCount);
  CHECK(2u == robot.m_simulationPeriodicCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(2u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::DriverStationSim::SetEnabled(false);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(20_ms);  // Wait for Notifiers

  CHECK(1u == robot.m_simulationInitCount);
  CHECK(1u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(1u == robot.m_utilityInitCount);

  CHECK(3u == robot.m_robotPeriodicCount);
  CHECK(3u == robot.m_simulationPeriodicCount);
  CHECK(1u == robot.m_disabledPeriodicCount);
  CHECK(0u == robot.m_autonomousPeriodicCount);
  CHECK(0u == robot.m_teleopPeriodicCount);
  CHECK(2u == robot.m_utilityPeriodicCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(1u == robot.m_utilityExitCount);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(TimedRobotTest, "TimedRobotTest ModeChange", "[wpilibc]") {
  MockRobot robot;

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  // Start in disabled
  prosdk::sim::DriverStationSim::SetEnabled(false);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(0u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  // Transition to autonomous
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::SetRobotMode(hal::RobotMode::AUTONOMOUS);
  prosdk::sim::DriverStationSim::NotifyNewData();

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_disabledInitCount);
  CHECK(1u == robot.m_autonomousInitCount);
  CHECK(0u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(1u == robot.m_disabledExitCount);
  CHECK(0u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  // Transition to teleop
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::SetRobotMode(hal::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::NotifyNewData();

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_disabledInitCount);
  CHECK(1u == robot.m_autonomousInitCount);
  CHECK(1u == robot.m_teleopInitCount);
  CHECK(0u == robot.m_utilityInitCount);

  CHECK(1u == robot.m_disabledExitCount);
  CHECK(1u == robot.m_autonomousExitCount);
  CHECK(0u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  // Transition to utility
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::SetRobotMode(hal::RobotMode::UTILITY);
  prosdk::sim::DriverStationSim::NotifyNewData();

  prosdk::sim::StepTiming(PERIOD);

  CHECK(1u == robot.m_disabledInitCount);
  CHECK(1u == robot.m_autonomousInitCount);
  CHECK(1u == robot.m_teleopInitCount);
  CHECK(1u == robot.m_utilityInitCount);

  CHECK(1u == robot.m_disabledExitCount);
  CHECK(1u == robot.m_autonomousExitCount);
  CHECK(1u == robot.m_teleopExitCount);
  CHECK(0u == robot.m_utilityExitCount);

  // Transition to disabled
  prosdk::sim::DriverStationSim::SetEnabled(false);
  prosdk::sim::DriverStationSim::NotifyNewData();

  prosdk::sim::StepTiming(PERIOD);

  CHECK(2u == robot.m_disabledInitCount);
  CHECK(1u == robot.m_autonomousInitCount);
  CHECK(1u == robot.m_teleopInitCount);
  CHECK(1u == robot.m_utilityInitCount);

  CHECK(1u == robot.m_disabledExitCount);
  CHECK(1u == robot.m_autonomousExitCount);
  CHECK(1u == robot.m_teleopExitCount);
  CHECK(1u == robot.m_utilityExitCount);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(TimedRobotTest, "TimedRobotTest AddPeriodic", "[wpilibc]") {
  MockRobot robot;

  std::atomic<uint32_t> callbackCount{0};
  robot.AddPeriodic([&] { callbackCount++; }, PERIOD / 2.0);

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetEnabled(false);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == callbackCount);

  prosdk::sim::StepTiming(PERIOD / 2.0);

  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(1u == callbackCount);

  prosdk::sim::StepTiming(PERIOD / 2.0);

  CHECK(1u == robot.m_disabledInitCount);
  CHECK(1u == robot.m_disabledPeriodicCount);
  CHECK(2u == callbackCount);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(TimedRobotTest, "TimedRobotTest AddPeriodicWithOffset",
                 "[wpilibc]") {
  MockRobot robot;

  std::atomic<uint32_t> callbackCount{0};
  robot.AddPeriodic([&] { callbackCount++; }, PERIOD / 2.0, PERIOD / 4.0);

  // Expirations in this test (ms)
  //
  // Robot | Callback
  // ================
  //     p |    0.75p
  //    2p |    1.25p

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetEnabled(false);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == callbackCount);

  prosdk::sim::StepTiming(PERIOD * 3.0 / 8.0);

  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(0u == callbackCount);

  prosdk::sim::StepTiming(PERIOD * 3.0 / 8.0);

  CHECK(0u == robot.m_disabledInitCount);
  CHECK(0u == robot.m_disabledPeriodicCount);
  CHECK(1u == callbackCount);

  prosdk::sim::StepTiming(PERIOD / 4.0);

  CHECK(1u == robot.m_disabledInitCount);
  CHECK(1u == robot.m_disabledPeriodicCount);
  CHECK(1u == callbackCount);

  prosdk::sim::StepTiming(PERIOD / 4.0);

  CHECK(1u == robot.m_disabledInitCount);
  CHECK(1u == robot.m_disabledPeriodicCount);
  CHECK(2u == callbackCount);

  robot.EndCompetition();
  robotThread.join();
}
