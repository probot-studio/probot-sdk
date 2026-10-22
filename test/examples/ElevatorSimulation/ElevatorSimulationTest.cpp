// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Upstream wpilibcExamples/src/test/cpp/examples/ElevatorSimulation, ported to prosdk: runs the
// examples/ElevatorSimulation robot in the host simulator (tools/build/modules/examples.cmake).

#include <optional>
#include <thread>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "Constants.hpp"
#include "Robot.hpp"
#include <prosdk/hal/DriverStationTypes.h>
#include <prosdk/hal/simulation/MockHooks.h>
#include <prosdk/simulation/DriverStationSim.hpp>
#include <prosdk/simulation/JoystickSim.hpp>
#include <prosdk/simulation/PWMMotorControllerSim.hpp>
#include <prosdk/simulation/SimHooks.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/mass.hpp>
#include <prosdk/units/time.hpp>

using namespace Constants;

class ElevatorSimulationTest {
  Robot robot;
  std::optional<std::thread> thread;

 public:
  prosdk::sim::PWMMotorControllerSim motorSim{Constants::MOTOR_PORT};
  prosdk::sim::EncoderSim encoderSim =
      prosdk::sim::EncoderSim::CreateForChannel(Constants::ENCODER_A_CHANNEL);
  prosdk::sim::JoystickSim joystickSim{Constants::JOYSTICK_PORT};

  ElevatorSimulationTest() {
    prosdk::sim::PauseTiming();
    prosdk::sim::SetProgramStarted(false);

    thread = std::thread([&] { robot.StartCompetition(); });
    prosdk::sim::WaitForProgramStart();
  }

  ~ElevatorSimulationTest() {
    robot.EndCompetition();
    thread->join();

    encoderSim.ResetData();
    prosdk::sim::DriverStationSim::ResetData();
  }
};

TEST_CASE_METHOD(ElevatorSimulationTest, "ElevatorSimulationTest teleop",
                 "[wpilibcExamples][examples][simulation][elevator]") {
  // teleop init
  {
    prosdk::sim::DriverStationSim::SetRobotMode(prosdk::hal::RobotMode::TELEOPERATED);
    prosdk::sim::DriverStationSim::SetEnabled(true);
    prosdk::sim::DriverStationSim::NotifyNewData();

    CHECK(encoderSim.GetInitialized());
  }

  {
    // advance 50 timesteps
    prosdk::sim::StepTiming(1_s);

    // Ensure elevator is still at 0.
    CHECK_THAT(encoderSim.GetDistance(), Catch::Matchers::WithinAbs(0.0, 0.05));
  }

  {
    // Press button to reach setpoint
    joystickSim.SetTrigger(true);
    joystickSim.NotifyNewData();

    // advance 75 timesteps
    prosdk::sim::StepTiming(1.5_s);

    CHECK_THAT(encoderSim.GetDistance(),
               Catch::Matchers::WithinAbs(SETPOINT.value(), 0.05));

    // advance 25 timesteps to see setpoint is held.
    prosdk::sim::StepTiming(0.5_s);

    CHECK_THAT(encoderSim.GetDistance(),
               Catch::Matchers::WithinAbs(SETPOINT.value(), 0.05));
  }

  {
    // Unpress the button to go back down
    joystickSim.SetTrigger(false);
    joystickSim.NotifyNewData();

    // advance 75 timesteps
    prosdk::sim::StepTiming(1.5_s);

    CHECK_THAT(encoderSim.GetDistance(), Catch::Matchers::WithinAbs(0.0, 0.05));
  }

  {
    // Press button to go back up
    joystickSim.SetTrigger(true);
    joystickSim.NotifyNewData();

    // advance 75 timesteps
    prosdk::sim::StepTiming(1.5_s);

    CHECK_THAT(encoderSim.GetDistance(),
               Catch::Matchers::WithinAbs(SETPOINT.value(), 0.05));

    // advance 25 timesteps to see setpoint is held.
    prosdk::sim::StepTiming(0.5_s);

    CHECK_THAT(encoderSim.GetDistance(),
               Catch::Matchers::WithinAbs(SETPOINT.value(), 0.05));
  }

  {
    // Disable
    prosdk::sim::DriverStationSim::SetEnabled(false);
    prosdk::sim::DriverStationSim::NotifyNewData();

    // advance 75 timesteps
    prosdk::sim::StepTiming(1.5_s);

    REQUIRE_THAT(motorSim.GetThrottle(), Catch::Matchers::WithinAbs(0.0, 0.05));
    REQUIRE_THAT(encoderSim.GetDistance(),
                 Catch::Matchers::WithinAbs(0.0, 0.05));
  }
}
