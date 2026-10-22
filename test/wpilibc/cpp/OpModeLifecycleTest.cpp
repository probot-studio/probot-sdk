// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/driverstation/RobotState.hpp"
#include "prosdk/framework/OpModeRobot.hpp"
#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/util/string.hpp"

using namespace prosdk::units;

namespace {
struct Counts {
  std::atomic<uint32_t> constructed{0};
  std::atomic<uint32_t> disabledPeriodic{0};
  std::atomic<uint32_t> start{0};
  std::atomic<uint32_t> periodic{0};
  std::atomic<uint32_t> end{0};
  std::atomic<uint32_t> destructed{0};
};

class LifecycleOpMode : public prosdk::OpMode {
 public:
  explicit LifecycleOpMode(Counts& counts) : m_counts(counts) {
    m_counts.constructed++;
  }

  ~LifecycleOpMode() override { m_counts.destructed++; }

  void DisabledPeriodic() override { m_counts.disabledPeriodic++; }
  void Start() override { m_counts.start++; }
  void Periodic() override { m_counts.periodic++; }
  void End() override { m_counts.end++; }

 private:
  Counts& m_counts;
};

class LifecycleRobot : public prosdk::OpModeRobot<LifecycleRobot> {
 public:
  LifecycleRobot() = default;
};

// OpMode whose GetCallbacks() returns a single callback with no enabled check,
// used to verify that getCallbacks() callbacks are registered immediately and
// run even while disabled.
class CallbackOpMode : public prosdk::OpMode {
 public:
  explicit CallbackOpMode(std::atomic<uint32_t>& callbackCount)
      : m_callbackCount(callbackCount) {}

  std::vector<prosdk::internal::PeriodicPriorityQueue::Callback> GetCallbacks() {
    std::vector<prosdk::internal::PeriodicPriorityQueue::Callback> callbacks;
    callbacks.emplace_back([&count = m_callbackCount] { count++; },
                           std::chrono::nanoseconds{0}, 20_ms);
    return callbacks;
  }

 private:
  std::atomic<uint32_t>& m_callbackCount;
};

class OpModeLifecycleTest {
 public:
  OpModeLifecycleTest() {
    prosdk::sim::PauseTiming();
    prosdk::sim::SetProgramStarted(false);
    prosdk::sim::DriverStationSim::ResetData();
    prosdk::sim::DriverStationSim::SetDsAttached(true);
    prosdk::sim::DriverStationSim::SetEnabled(false);
    prosdk::RobotState::ClearOpModes();
  }

  ~OpModeLifecycleTest() {
    prosdk::sim::ResumeTiming();
  }

 protected:
  static int64_t MakeOpModeId(prosdk::RobotMode mode, std::string_view name) {
    return HAL_MakeOpModeId(static_cast<HAL_RobotMode>(mode),
                            std::hash<std::string_view>{}(name));
  }
};

TEST_CASE_METHOD(OpModeLifecycleTest, "OpModeLifecycleTest EnabledTransition",
                 "[wpilibc]") {
  Counts counts;
  LifecycleRobot robot;
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "TestOpMode", [&] {
    return std::make_unique<LifecycleOpMode>(counts);
  });
  robot.PublishOpModes();

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  // Ensure DS is attached after program starts
  prosdk::sim::DriverStationSim::SetDsAttached(true);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(20_ms);  // Let the DS attached state propagate

  // 1. Selected, but disabled
  prosdk::sim::DriverStationSim::SetRobotMode(prosdk::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "TestOpMode"));
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(20_ms);
  CHECK(counts.constructed.load() == 1u);
  CHECK(counts.disabledPeriodic.load() == 1u);
  CHECK(counts.periodic.load() == 0u);

  // 2. Transition to enabled
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(
      40_ms);  // Step twice like Java test to get periodic callback
  CHECK(counts.start.load() == 1u);
  CHECK(counts.periodic.load() == 1u);

  // 3. Transition to disabled
  prosdk::sim::DriverStationSim::SetEnabled(false);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(20_ms);
  CHECK(counts.end.load() == 1u);
  CHECK(counts.destructed.load() == 1u);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(OpModeLifecycleTest,
                 "OpModeLifecycleTest OpModeChangeWhileEnabled", "[wpilibc]") {
  Counts counts1;
  Counts counts2;
  LifecycleRobot robot;
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "OpMode1", [&] {
    return std::make_unique<LifecycleOpMode>(counts1);
  });
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "OpMode2", [&] {
    return std::make_unique<LifecycleOpMode>(counts2);
  });
  robot.PublishOpModes();

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  // Ensure DS is attached after program starts
  prosdk::sim::DriverStationSim::SetDsAttached(true);
  prosdk::sim::DriverStationSim::NotifyNewData();

  // 1. Select OpMode1 and enable
  prosdk::sim::DriverStationSim::SetRobotMode(prosdk::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "OpMode1"));
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(40_ms);  // Need two iterations for periodic callback
  CHECK(counts1.constructed.load() == 1u);
  CHECK(counts1.start.load() == 1u);
  CHECK(counts1.periodic.load() == 1u);

  // 2. Switch to OpMode2 while enabled. Selecting a different opmode while
  // enabled disables the robot first, so the DS sends disabled + new opmode.
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "OpMode2"));
  prosdk::sim::DriverStationSim::SetEnabled(false);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(20_ms);
  // OpMode1 should be ended and destructed
  CHECK(counts1.end.load() == 1u);
  CHECK(counts1.destructed.load() == 1u);
  // OpMode2 should be constructed exactly once and persist while disabled
  CHECK(counts2.constructed.load() == 1u);
  CHECK(counts2.start.load() == 0u);

  // 3. Re-enable. The same OpMode2 instance is started; it is not
  // reconstructed.
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(40_ms);
  CHECK(counts2.constructed.load() == 1u);
  CHECK(counts2.start.load() == 1u);
  CHECK(counts2.periodic.load() == 1u);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(OpModeLifecycleTest,
                 "OpModeLifecycleTest OpModeChangeWhileDisabled", "[wpilibc]") {
  Counts counts1;
  Counts counts2;
  LifecycleRobot robot;
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "OpMode1", [&] {
    return std::make_unique<LifecycleOpMode>(counts1);
  });
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "OpMode2", [&] {
    return std::make_unique<LifecycleOpMode>(counts2);
  });
  robot.PublishOpModes();

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  // Ensure DS is attached after program starts
  prosdk::sim::DriverStationSim::SetDsAttached(true);

  // 1. Select OpMode1 while disabled
  prosdk::sim::DriverStationSim::SetRobotMode(prosdk::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "OpMode1"));
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(20_ms);
  CHECK(counts1.constructed.load() == 1u);
  CHECK(counts1.disabledPeriodic.load() == 1u);

  // 2. Change to OpMode2 while disabled
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "OpMode2"));
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(20_ms);
  // OpMode1 should be destructed, but NOT ended
  CHECK(counts1.destructed.load() == 1u);
  CHECK(counts1.end.load() == 0u);
  // OpMode2 should be selected
  CHECK(counts2.constructed.load() == 1u);
  CHECK(counts2.disabledPeriodic.load() == 1u);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(OpModeLifecycleTest,
                 "OpModeLifecycleTest GetCallbacksRunImmediatelyWhileDisabled",
                 "[wpilibc]") {
  std::atomic<uint32_t> callbackCount{0};
  LifecycleRobot robot;
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "CallbackOpMode", [&] {
    return std::make_unique<CallbackOpMode>(callbackCount);
  });
  robot.PublishOpModes();

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetDsAttached(true);

  // Select the opmode while disabled. getCallbacks() callbacks are registered
  // immediately on construction and run even while the robot is disabled.
  prosdk::sim::DriverStationSim::SetRobotMode(prosdk::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "CallbackOpMode"));
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(100_ms);
  CHECK(callbackCount.load() >= 1u);

  // Deselecting the opmode tears it down and removes its callbacks, so the
  // callback must stop running.
  prosdk::sim::DriverStationSim::SetOpMode(0);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(100_ms);  // let teardown settle
  uint32_t countAfterTeardown = callbackCount.load();
  prosdk::sim::StepTiming(100_ms);
  CHECK(callbackCount.load() == countAfterTeardown);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(OpModeLifecycleTest, "OpModeLifecycleTest InitialEnabledState",
                 "[wpilibc]") {
  Counts counts;
  LifecycleRobot robot;
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "TestOpMode", [&] {
    return std::make_unique<LifecycleOpMode>(counts);
  });
  robot.PublishOpModes();

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  // The very first DS packet is fully enabled and has an opmode selected
  prosdk::sim::DriverStationSim::SetDsAttached(true);
  prosdk::sim::DriverStationSim::SetRobotMode(prosdk::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "TestOpMode"));
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(40_ms);  // Step twice to get periodic callback

  // Should construct, call disabledPeriodic once (since it's a new opmode),
  // then start and periodic
  CHECK(counts.constructed.load() == 1u);
  CHECK(counts.disabledPeriodic.load() == 1u);
  CHECK(counts.start.load() == 1u);
  CHECK(counts.periodic.load() == 1u);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(OpModeLifecycleTest,
                 "OpModeLifecycleTest ReconstructionOnDisable", "[wpilibc]") {
  Counts counts;
  LifecycleRobot robot;
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "TestOpMode", [&] {
    return std::make_unique<LifecycleOpMode>(counts);
  });
  robot.PublishOpModes();

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetDsAttached(true);

  // 1. Enable
  prosdk::sim::DriverStationSim::SetRobotMode(prosdk::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "TestOpMode"));
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(40_ms);

  CHECK(counts.constructed.load() == 1u);
  CHECK(counts.start.load() == 1u);

  // 2. Disable
  prosdk::sim::DriverStationSim::SetEnabled(false);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(40_ms);

  CHECK(counts.end.load() == 1u);
  CHECK(counts.destructed.load() == 1u);
  CHECK(counts.constructed.load() == 2u);
  CHECK(counts.disabledPeriodic.load() >= 1u);
  CHECK(counts.start.load() == 1u);

  // 3. Re-enable
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(40_ms);

  CHECK(counts.constructed.load() == 2u);
  CHECK(counts.start.load() == 2u);
  CHECK(counts.end.load() == 1u);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(OpModeLifecycleTest, "OpModeLifecycleTest DeselectOpMode",
                 "[wpilibc]") {
  Counts counts;
  LifecycleRobot robot;
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "TestOpMode", [&] {
    return std::make_unique<LifecycleOpMode>(counts);
  });
  robot.PublishOpModes();

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetDsAttached(true);

  prosdk::sim::DriverStationSim::SetRobotMode(prosdk::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "TestOpMode"));
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(20_ms);

  CHECK(counts.constructed.load() == 1u);

  // Deselect opmode
  prosdk::sim::DriverStationSim::SetOpMode(0);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(20_ms);

  CHECK(counts.destructed.load() == 1u);
  CHECK(counts.constructed.load() == 1u);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(OpModeLifecycleTest, "OpModeLifecycleTest DsDisconnect",
                 "[wpilibc]") {
  Counts counts;
  LifecycleRobot robot;
  robot.AddOpModeFactory(prosdk::RobotMode::TELEOPERATED, "TestOpMode", [&] {
    return std::make_unique<LifecycleOpMode>(counts);
  });
  robot.PublishOpModes();

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  prosdk::sim::DriverStationSim::SetDsAttached(true);
  prosdk::sim::DriverStationSim::SetRobotMode(prosdk::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::SetOpMode(
      MakeOpModeId(prosdk::RobotMode::TELEOPERATED, "TestOpMode"));
  prosdk::sim::DriverStationSim::SetEnabled(true);
  prosdk::sim::DriverStationSim::NotifyNewData();
  prosdk::sim::StepTiming(40_ms);

  CHECK(counts.constructed.load() == 1u);
  CHECK(counts.start.load() == 1u);

  // DS Disconnect
  prosdk::sim::DriverStationSim::SetDsAttached(false);
  // prosdk::sim::DriverStationSim::NotifyNewData(); // DON'T DO THIS
  prosdk::sim::StepTiming(40_ms);

  CHECK(counts.end.load() == 1u);
  CHECK(counts.destructed.load() == 1u);
  CHECK(counts.constructed.load() == 1u);

  robot.EndCompetition();
  robotThread.join();
}

}  // namespace
