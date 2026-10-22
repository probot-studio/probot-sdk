// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Upstream wpilibcExamples/src/test/cpp/snippets/DigitalCommunication, ported to prosdk: runs the
// examples/SnippetDigitalCommunication robot in the host simulator (tools/build/modules/examples.cmake).

#include <optional>
#include <string>
#include <thread>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "Robot.hpp"
#include <prosdk/hal/DriverStationTypes.h>
#include <prosdk/simulation/DIOSim.hpp>
#include <prosdk/simulation/DriverStationSim.hpp>
#include <prosdk/simulation/SimHooks.hpp>
#include <prosdk/units/time.hpp>

class DigitalCommunicationTest {
 public:
  prosdk::sim::DIOSim allianceOutput{Robot::ALLIANCE_PORT};
  prosdk::sim::DIOSim enabledOutput{Robot::ENABLED_PORT};
  prosdk::sim::DIOSim autonomousOutput{Robot::AUTONOMOUS_PORT};
  prosdk::sim::DIOSim alertOutput{Robot::ALERT_PORT};
  Robot robot;
  std::optional<std::thread> thread;

  DigitalCommunicationTest() {
    prosdk::sim::PauseTiming();
    prosdk::sim::SetProgramStarted(false);
    prosdk::sim::DriverStationSim::ResetData();

    thread = std::thread([&] { robot.StartCompetition(); });
    prosdk::sim::WaitForProgramStart();
  }

  ~DigitalCommunicationTest() {
    robot.EndCompetition();
    thread->join();
    allianceOutput.ResetData();
    enabledOutput.ResetData();
    autonomousOutput.ResetData();
    alertOutput.ResetData();
  }
};

TEST_CASE_METHOD(DigitalCommunicationTest, "DigitalCommunication alliance",
                 "[wpilibcExamples][snippets][dio]") {
  auto alliance = GENERATE(
      prosdk::hal::AllianceStationID::RED_1, prosdk::hal::AllianceStationID::RED_2,
      prosdk::hal::AllianceStationID::RED_3, prosdk::hal::AllianceStationID::BLUE_1,
      prosdk::hal::AllianceStationID::BLUE_2, prosdk::hal::AllianceStationID::BLUE_3,
      prosdk::hal::AllianceStationID::UNKNOWN);
  prosdk::sim::DriverStationSim::SetAllianceStationId(alliance);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(allianceOutput.GetInitialized());
  CHECK_FALSE(allianceOutput.GetIsInput());

  prosdk::sim::StepTiming(20_ms);

  bool isRed = false;
  switch (alliance) {
    case prosdk::hal::AllianceStationID::BLUE_1:
    case prosdk::hal::AllianceStationID::BLUE_2:
    case prosdk::hal::AllianceStationID::BLUE_3:
    case prosdk::hal::AllianceStationID::UNKNOWN:
      isRed = false;
      break;
    case prosdk::hal::AllianceStationID::RED_1:
    case prosdk::hal::AllianceStationID::RED_2:
    case prosdk::hal::AllianceStationID::RED_3:
      isRed = true;
      break;
  }
  CHECK(isRed == allianceOutput.GetValue());
}

TEST_CASE_METHOD(DigitalCommunicationTest, "DigitalCommunication enabled",
                 "[wpilibcExamples][snippets][dio]") {
  auto enabled = GENERATE(false, true);
  prosdk::sim::DriverStationSim::SetEnabled(enabled);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(enabledOutput.GetInitialized());
  CHECK_FALSE(enabledOutput.GetIsInput());

  prosdk::sim::StepTiming(20_ms);

  CHECK(enabled == enabledOutput.GetValue());
}

TEST_CASE_METHOD(DigitalCommunicationTest, "DigitalCommunication autonomous",
                 "[wpilibcExamples][snippets][dio]") {
  auto autonomous = GENERATE(false, true);
  prosdk::sim::DriverStationSim::SetRobotMode(
      autonomous ? prosdk::hal::RobotMode::AUTONOMOUS
                 : prosdk::hal::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(autonomousOutput.GetInitialized());
  CHECK_FALSE(autonomousOutput.GetIsInput());

  prosdk::sim::StepTiming(20_ms);

  CHECK(autonomous == autonomousOutput.GetValue());
}

TEST_CASE_METHOD(DigitalCommunicationTest, "DigitalCommunication alert",
                 "[wpilibcExamples][snippets][dio]") {
  auto matchTime = GENERATE(45.0, 27.0, 23.0);
  prosdk::sim::DriverStationSim::SetMatchTime(matchTime);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(alertOutput.GetInitialized());
  CHECK_FALSE(alertOutput.GetIsInput());

  prosdk::sim::StepTiming(20_ms);

  CHECK((matchTime <= 30 && matchTime >= 25) == alertOutput.GetValue());
}
