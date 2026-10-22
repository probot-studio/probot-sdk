// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Upstream wpilibcExamples/src/test/cpp/examples/ArmSimulation, ported to prosdk: runs the
// examples/ArmSimulation robot in the host simulator (tools/build/modules/examples.cmake).

#include <cmath>
#include <optional>
#include <thread>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "Constants.hpp"
#include "Robot.hpp"
#include <prosdk/hal/DriverStationTypes.h>
#include <prosdk/preferences/Preferences.hpp>
#include <prosdk/simulation/DriverStationSim.hpp>
#include <prosdk/simulation/JoystickSim.hpp>
#include <prosdk/simulation/PWMMotorControllerSim.hpp>
#include <prosdk/simulation/SimHooks.hpp>
#include <prosdk/units/time.hpp>

class ArmSimulationTest {
  Robot robot;
  std::optional<std::thread> thread;

 public:
  prosdk::sim::PWMMotorControllerSim motorSim{MOTOR_PORT};
  prosdk::sim::EncoderSim encoderSim =
      prosdk::sim::EncoderSim::CreateForChannel(ENCODER_A_CHANNEL);
  prosdk::sim::JoystickSim joystickSim{JOYSTICK_PORT};

  ArmSimulationTest() {
    prosdk::sim::PauseTiming();
    prosdk::sim::SetProgramStarted(false);

    thread = std::thread([&] { robot.StartCompetition(); });
    prosdk::sim::WaitForProgramStart();
  }

  ~ArmSimulationTest() {
    robot.EndCompetition();
    thread->join();

    encoderSim.ResetData();
    prosdk::sim::DriverStationSim::ResetData();
    prosdk::Preferences::RemoveAll();
  }
};

TEST_CASE_METHOD(ArmSimulationTest, "ArmSimulationTest teleop",
                 "[wpilibcExamples][examples][simulation][arm]") {
  prosdk::units::degree_t setpoint =
      GENERATE(DEFAULT_ARM_SETPOINT, 25.0_deg, 50.0_deg);

  CHECK(prosdk::Preferences::ContainsKey(ARM_POSITION_KEY));
  CHECK(prosdk::Preferences::ContainsKey(ARM_P_KEY));
  prosdk::Preferences::SetDouble(ARM_POSITION_KEY, setpoint.value());
  CHECK_THAT(prosdk::Preferences::GetDouble(ARM_POSITION_KEY, NAN),
             Catch::Matchers::WithinULP(setpoint.value(), 4));

  // teleop init
  {
    prosdk::sim::DriverStationSim::SetRobotMode(prosdk::hal::RobotMode::TELEOPERATED);
    prosdk::sim::DriverStationSim::SetEnabled(true);
    prosdk::sim::DriverStationSim::NotifyNewData();

    CHECK(encoderSim.GetInitialized());
  }

  {
    prosdk::sim::StepTiming(3_s);

    // Ensure arm is still at minimum angle.
    CHECK_THAT(encoderSim.GetDistance(),
               Catch::Matchers::WithinAbs(MIN_ANGLE.value(), 2.0));
  }

  {
    // Press button to reach setpoint
    joystickSim.SetTrigger(true);
    joystickSim.NotifyNewData();

    prosdk::sim::StepTiming(1.5_s);

    CHECK_THAT(prosdk::units::radian_t{encoderSim.GetDistance()}
                   .convert<prosdk::units::degree>()
                   .value(),
               Catch::Matchers::WithinAbs(setpoint.value(), 2.0));

    // see setpoint is held.
    prosdk::sim::StepTiming(0.5_s);

    CHECK_THAT(prosdk::units::radian_t{encoderSim.GetDistance()}
                   .convert<prosdk::units::degree>()
                   .value(),
               Catch::Matchers::WithinAbs(setpoint.value(), 2.0));
  }

  {
    // Unpress the button to go back down
    joystickSim.SetTrigger(false);
    joystickSim.NotifyNewData();

    prosdk::sim::StepTiming(3_s);

    CHECK_THAT(encoderSim.GetDistance(),
               Catch::Matchers::WithinAbs(MIN_ANGLE.value(), 2.0));
  }

  {
    // Press button to go back up
    joystickSim.SetTrigger(true);
    joystickSim.NotifyNewData();

    // advance 75 timesteps
    prosdk::sim::StepTiming(1.5_s);

    CHECK_THAT(prosdk::units::radian_t{encoderSim.GetDistance()}
                   .convert<prosdk::units::degree>()
                   .value(),
               Catch::Matchers::WithinAbs(setpoint.value(), 2.0));

    // advance 25 timesteps to see setpoint is held.
    prosdk::sim::StepTiming(0.5_s);

    CHECK_THAT(prosdk::units::radian_t{encoderSim.GetDistance()}
                   .convert<prosdk::units::degree>()
                   .value(),
               Catch::Matchers::WithinAbs(setpoint.value(), 2.0));
  }

  {
    // Disable
    prosdk::sim::DriverStationSim::SetEnabled(false);
    prosdk::sim::DriverStationSim::NotifyNewData();

    prosdk::sim::StepTiming(3_s);

    REQUIRE_THAT(motorSim.GetThrottle(), Catch::Matchers::WithinAbs(0.0, 0.05));
    CHECK_THAT(encoderSim.GetDistance(),
               Catch::Matchers::WithinAbs(MIN_ANGLE.value(), 2.0));
  }
}
