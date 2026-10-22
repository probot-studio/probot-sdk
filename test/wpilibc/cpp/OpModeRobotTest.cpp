// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/framework/OpModeRobot.hpp"

#include <sys/types.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/driverstation/RobotState.hpp"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/util/Color.hpp"
#include "prosdk/util/string.hpp"

inline constexpr auto PERIOD = 20_ms;

namespace {
class OpModeRobotTest {
 public:
  OpModeRobotTest() {
    prosdk::sim::PauseTiming();
    prosdk::sim::SetProgramStarted(false);
    prosdk::sim::DriverStationSim::ResetData();
    prosdk::RobotState::ClearOpModes();
  }

  ~OpModeRobotTest() {
    prosdk::sim::ResumeTiming();
  }
};

class MockRobot;

class MockOpMode : public prosdk::OpMode {
 public:
  std::atomic<uint32_t> m_disabledPeriodicCount{0};
  std::atomic<uint32_t> m_startCount{0};
  std::atomic<uint32_t> m_periodicCount{0};
  std::atomic<uint32_t> m_endCount{0};
  std::atomic<uint32_t> m_closeCount{0};

  MockOpMode() = default;
  ~MockOpMode() override { m_closeCount++; }
  void DisabledPeriodic() override { m_disabledPeriodicCount++; }
  void Start() override { m_startCount++; }
  void Periodic() override { m_periodicCount++; }
  void End() override { m_endCount++; }
};

class OneArgOpMode : public prosdk::OpMode {
 public:
  explicit OneArgOpMode(MockRobot& robot) {}
  void Start() override {}
  void End() override {}
};

class MockRobot : public prosdk::OpModeRobot<MockRobot> {
 public:
  std::atomic<uint32_t> m_driverStationConnectedCount{0};
  std::atomic<uint32_t> m_nonePeriodicCount{0};

  // RobotPeriodic method counter
  std::atomic<uint32_t> m_robotPeriodicCount{0};

  MockRobot() = default;

  void DriverStationConnected() { m_driverStationConnectedCount++; }

  void NonePeriodic() { m_nonePeriodicCount++; }

  void RobotPeriodic() { m_robotPeriodicCount++; }
};
}  // namespace

static_assert(prosdk::ConstructibleOpMode<MockOpMode, MockRobot>);
static_assert(prosdk::ConstructibleOpMode<OneArgOpMode, MockRobot>);

TEST_CASE_METHOD(OpModeRobotTest, "OpModeRobotTest AddOpMode", "[wpilibc]") {
  struct MyMockRobot : public MockRobot {
    MyMockRobot() {
      AddOpMode<MockOpMode>(prosdk::RobotMode::AUTONOMOUS, "NoArgOpMode-Auto",
                            "Group", "Description", prosdk::util::Color::WHITE,
                            prosdk::util::Color::BLACK);
      AddOpMode<OneArgOpMode>(prosdk::RobotMode::UTILITY, "OneArgOpMode-Test",
                              "Group", "Description", prosdk::util::Color::WHITE,
                              prosdk::util::Color::BLACK);
      AddOpMode<MockOpMode>(prosdk::RobotMode::TELEOPERATED, "NoArgOpMode");
      AddOpMode<OneArgOpMode>(prosdk::RobotMode::TELEOPERATED, "OneArgOpMode");
      PublishOpModes();
    }
  };
  MyMockRobot robot;

  auto options = prosdk::sim::DriverStationSim::GetOpModeOptions();
  REQUIRE(options.size() == 4u);
  int indexes[4] = {-1, -1, -1, -1};
  for (size_t i = 0; i < options.size(); ++i) {
    auto name = prosdk::util::to_string_view(&options[i].name);
    if (name == "NoArgOpMode-Auto") {
      indexes[0] = i;
    } else if (name == "OneArgOpMode-Test") {
      indexes[1] = i;
    } else if (name == "NoArgOpMode") {
      indexes[2] = i;
    } else if (name == "OneArgOpMode") {
      indexes[3] = i;
    }
  }

  int i = indexes[0];
  REQUIRE(i != -1);
  CHECK(prosdk::util::to_string_view(&options[i].group) == "Group");
  CHECK(prosdk::util::to_string_view(&options[i].description) == "Description");
  CHECK(options[i].textColor == 0xffffff);
  CHECK(options[i].backgroundColor == 0x000000);

  i = indexes[1];
  REQUIRE(i != -1);
  CHECK(prosdk::util::to_string_view(&options[i].group) == "Group");
  CHECK(prosdk::util::to_string_view(&options[i].description) == "Description");
  CHECK(options[i].textColor == 0xffffff);
  CHECK(options[i].backgroundColor == 0x000000);

  i = indexes[2];
  REQUIRE(i != -1);
  CHECK(prosdk::util::to_string_view(&options[i].group) == "");
  CHECK(prosdk::util::to_string_view(&options[i].description) == "");
  CHECK(options[i].textColor == -1);
  CHECK(options[i].backgroundColor == -1);

  i = indexes[3];
  REQUIRE(i != -1);
  CHECK(prosdk::util::to_string_view(&options[i].group) == "");
  CHECK(prosdk::util::to_string_view(&options[i].description) == "");
  CHECK(options[i].textColor == -1);
  CHECK(options[i].backgroundColor == -1);
}

TEST_CASE_METHOD(OpModeRobotTest, "OpModeRobotTest ClearOpModes", "[wpilibc]") {
  struct MyMockRobot : public MockRobot {
    MyMockRobot() {
      AddOpMode<MockOpMode>(prosdk::RobotMode::TELEOPERATED, "NoArgOpMode");
      AddOpMode<OneArgOpMode>(prosdk::RobotMode::TELEOPERATED, "OneArgOpMode");
      PublishOpModes();
    }
  };
  MyMockRobot robot;

  robot.ClearOpModes();
  auto options = prosdk::sim::DriverStationSim::GetOpModeOptions();
  CHECK(options.empty());
}

TEST_CASE_METHOD(OpModeRobotTest, "OpModeRobotTest RemoveOpMode", "[wpilibc]") {
  struct MyMockRobot : public MockRobot {
    MyMockRobot() {
      AddOpMode<MockOpMode>(prosdk::RobotMode::TELEOPERATED, "NoArgOpMode");
      AddOpMode<OneArgOpMode>(prosdk::RobotMode::TELEOPERATED, "OneArgOpMode");
      PublishOpModes();
    }
  };
  MyMockRobot robot;

  robot.RemoveOpMode(prosdk::RobotMode::TELEOPERATED, "NoArgOpMode");
  robot.PublishOpModes();
  auto options = prosdk::sim::DriverStationSim::GetOpModeOptions();
  REQUIRE(options.size() == 1u);
  CHECK(prosdk::util::to_string_view(&options[0].name) == "OneArgOpMode");
}

TEST_CASE_METHOD(OpModeRobotTest, "OpModeRobotTest NonePeriodic", "[wpilibc]") {
  struct MyMockRobot : public MockRobot {
    MyMockRobot() {
      AddOpMode<MockOpMode>(prosdk::RobotMode::TELEOPERATED, "NoArgOpMode");
      AddOpMode<OneArgOpMode>(prosdk::RobotMode::TELEOPERATED, "OneArgOpMode");
      PublishOpModes();
    }
  };
  MyMockRobot robot;

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  // Time step to get periodic calls on 20 ms robot loop
  prosdk::sim::StepTiming(110_ms);
  CHECK(robot.m_nonePeriodicCount.load() == 5u);

  robot.EndCompetition();
  robotThread.join();
}

TEST_CASE_METHOD(OpModeRobotTest, "OpModeRobotTest RobotPeriodic",
                 "[wpilibc]") {
  struct MyMockRobot : public MockRobot {
    MyMockRobot() {
      AddOpMode<MockOpMode>(prosdk::RobotMode::TELEOPERATED, "TestOpMode");
      PublishOpModes();
    }
  };
  MyMockRobot robot;

  std::thread robotThread{[&] { robot.StartCompetition(); }};
  prosdk::sim::WaitForProgramStart();

  // RobotPeriodic should be called regardless of state
  CHECK(robot.m_robotPeriodicCount.load() == 0u);

  // Step timing to allow callbacks to execute
  prosdk::sim::StepTiming(PERIOD);
  CHECK(robot.m_robotPeriodicCount.load() == 1u);

  // Additional time steps should continue calling RobotPeriodic
  prosdk::sim::StepTiming(PERIOD);
  CHECK(robot.m_robotPeriodicCount.load() == 2u);

  robot.EndCompetition();
  robotThread.join();
}
