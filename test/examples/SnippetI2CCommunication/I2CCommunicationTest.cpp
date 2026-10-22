// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Upstream wpilibcExamples/src/test/cpp/snippets/I2CCommunication, ported to prosdk: runs the
// examples/SnippetI2CCommunication robot in the host simulator (tools/build/modules/examples.cmake).

#include <format>
#include <optional>
#include <string>
#include <thread>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "Robot.hpp"
#include <prosdk/hal/DriverStationTypes.h>
#include <prosdk/hal/simulation/I2CData.h>
#include <prosdk/simulation/DriverStationSim.hpp>
#include <prosdk/simulation/SimHooks.hpp>
#include <prosdk/units/time.hpp>

static std::string gString;

void callback(const char* name, void* param, const unsigned char* buffer,
              unsigned int count) {
  gString.assign(reinterpret_cast<const char*>(buffer),
                 static_cast<int>(count));
}

class I2CCommunicationTest {
 public:
  Robot robot;
  std::optional<std::thread> thread;
  int32_t callbackHandle;
  int32_t port;

  I2CCommunicationTest() {
    gString = std::string();
    prosdk::sim::PauseTiming();
    prosdk::sim::SetProgramStarted(false);
    prosdk::sim::DriverStationSim::ResetData();
    port = static_cast<int32_t>(Robot::PORT);

    callbackHandle = HALSIM_RegisterI2CWriteCallback(port, &callback, nullptr);

    thread = std::thread([&] { robot.StartCompetition(); });
    prosdk::sim::WaitForProgramStart();
  }

  ~I2CCommunicationTest() {
    robot.EndCompetition();
    thread->join();

    HALSIM_CancelI2CWriteCallback(port, callbackHandle);
    HALSIM_ResetI2CData(port);
  }
};

TEST_CASE_METHOD(I2CCommunicationTest, "I2CCommunication alliance",
                 "[wpilibcExamples][snippets][i2c]") {
  auto alliance = GENERATE(
      prosdk::hal::AllianceStationID::RED_1, prosdk::hal::AllianceStationID::RED_2,
      prosdk::hal::AllianceStationID::RED_3, prosdk::hal::AllianceStationID::BLUE_1,
      prosdk::hal::AllianceStationID::BLUE_2, prosdk::hal::AllianceStationID::BLUE_3,
      prosdk::hal::AllianceStationID::UNKNOWN);
  prosdk::sim::DriverStationSim::SetAllianceStationId(alliance);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(HALSIM_GetI2CInitialized(port));

  prosdk::sim::StepTiming(20_ms);

  char expected = 'U';
  switch (alliance) {
    case prosdk::hal::AllianceStationID::BLUE_1:
    case prosdk::hal::AllianceStationID::BLUE_2:
    case prosdk::hal::AllianceStationID::BLUE_3:
      expected = 'B';
      break;
    case prosdk::hal::AllianceStationID::RED_1:
    case prosdk::hal::AllianceStationID::RED_2:
    case prosdk::hal::AllianceStationID::RED_3:
      expected = 'R';
      break;
    case prosdk::hal::AllianceStationID::UNKNOWN:
      expected = 'U';
      break;
  }
  CHECK(expected == gString.at(0));
}

TEST_CASE_METHOD(I2CCommunicationTest, "I2CCommunication enabled",
                 "[wpilibcExamples][snippets][i2c]") {
  auto enabled = GENERATE(false, true);
  prosdk::sim::DriverStationSim::SetEnabled(enabled);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(HALSIM_GetI2CInitialized(port));

  prosdk::sim::StepTiming(20_ms);

  char expected = enabled ? 'E' : 'D';
  CHECK(expected == gString.at(1));
}

TEST_CASE_METHOD(I2CCommunicationTest, "I2CCommunication autonomous",
                 "[wpilibcExamples][snippets][i2c]") {
  auto autonomous = GENERATE(false, true);
  prosdk::sim::DriverStationSim::SetRobotMode(
      autonomous ? prosdk::hal::RobotMode::AUTONOMOUS
                 : prosdk::hal::RobotMode::TELEOPERATED);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(HALSIM_GetI2CInitialized(port));

  prosdk::sim::StepTiming(20_ms);

  char expected = autonomous ? 'A' : 'T';
  CHECK(expected == gString.at(2));
}

TEST_CASE_METHOD(I2CCommunicationTest, "I2CCommunication alert",
                 "[wpilibcExamples][snippets][i2c]") {
  auto matchTime = GENERATE(112, 45, 27, 23, 3);
  prosdk::sim::DriverStationSim::SetMatchTime(matchTime);
  prosdk::sim::DriverStationSim::NotifyNewData();

  CHECK(HALSIM_GetI2CInitialized(port));

  prosdk::sim::StepTiming(20_ms);

  std::string expected = std::format("{:03}", matchTime);
  CHECK(expected == gString.substr(3));
}
