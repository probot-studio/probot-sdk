// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/DCMotorSim.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "motorcontrol/TestPWMMotorController.hpp"
#include "prosdk/hardware/rotation/Encoder.hpp"
#include "prosdk/math/controller/PIDController.hpp"
#include "prosdk/math/system/Models.hpp"
#include "prosdk/simulation/BatterySim.hpp"
#include "prosdk/simulation/EncoderSim.hpp"
#include "prosdk/simulation/RoboRioSim.hpp"
#include "prosdk/system/RobotController.hpp"

TEST_CASE("DCMotorSimTest VoltageSteadyState", "[wpilibc][simulation]") {
  prosdk::math::DCMotor gearbox = prosdk::math::DCMotor::NEO(1);
  auto plant = prosdk::math::Models::SingleJointedArmFromPhysicalConstants(
      prosdk::math::DCMotor::NEO(1), prosdk::units::kilogram_square_meter_t{0.0005},
      1.0);
  prosdk::sim::DCMotorSim sim{plant, gearbox};

  prosdk::Encoder encoder{0, 1};
  prosdk::sim::EncoderSim encoderSim{encoder};
  prosdk::TestPWMMotorController motor{0};

  prosdk::sim::RoboRioSim::ResetData();
  encoderSim.ResetData();

  // Spin-up
  for (int i = 0; i < 100; i++) {
    // RobotPeriodic runs first
    motor.SetVoltage(12_V);

    // Then, SimulationPeriodic runs
    prosdk::sim::RoboRioSim::SetVInVoltage(
        prosdk::sim::BatterySim::Calculate({sim.GetCurrentDraw()}));
    sim.SetInputVoltage(motor.GetThrottle() *
                        prosdk::RobotController::GetBatteryVoltage());
    sim.Update(20_ms);
    encoderSim.SetRate(sim.GetAngularVelocity().value());
  }

  CHECK_THAT((gearbox.Kv * 12_V).value(),
             Catch::Matchers::WithinAbs(encoder.GetRate(), 0.1));

  // Decay
  for (int i = 0; i < 100; i++) {
    // RobotPeriodic runs first
    motor.SetVoltage(0_V);

    // Then, SimulationPeriodic runs
    prosdk::sim::RoboRioSim::SetVInVoltage(
        prosdk::sim::BatterySim::Calculate({sim.GetCurrentDraw()}));
    sim.SetInputVoltage(motor.GetThrottle() *
                        prosdk::RobotController::GetBatteryVoltage());
    sim.Update(20_ms);
    encoderSim.SetRate(sim.GetAngularVelocity().value());
  }

  CHECK_THAT(0, Catch::Matchers::WithinAbs(encoder.GetRate(), 0.1));
}

TEST_CASE("DCMotorSimTest PositionFeedbackControl", "[wpilibc][simulation]") {
  prosdk::math::DCMotor gearbox = prosdk::math::DCMotor::NEO(1);
  auto plant = prosdk::math::Models::SingleJointedArmFromPhysicalConstants(
      prosdk::math::DCMotor::NEO(1), prosdk::units::kilogram_square_meter_t{0.0005},
      1.0);
  prosdk::sim::DCMotorSim sim{plant, gearbox};

  prosdk::math::PIDController controller{0.04, 0.0, 0.001};

  prosdk::Encoder encoder{0, 1};
  prosdk::sim::EncoderSim encoderSim{encoder};
  prosdk::TestPWMMotorController motor{0};

  prosdk::sim::RoboRioSim::ResetData();
  encoderSim.ResetData();

  for (int i = 0; i < 140; i++) {
    // RobotPeriodic runs first
    motor.SetThrottle(controller.Calculate(encoder.GetDistance(), 750));

    // Then, SimulationPeriodic runs
    prosdk::sim::RoboRioSim::SetVInVoltage(
        prosdk::sim::BatterySim::Calculate({sim.GetCurrentDraw()}));
    sim.SetInputVoltage(motor.GetThrottle() *
                        prosdk::RobotController::GetBatteryVoltage());
    sim.Update(20_ms);
    encoderSim.SetDistance(sim.GetAngularPosition().value());
    encoderSim.SetRate(sim.GetAngularVelocity().value());
  }

  CHECK_THAT(encoder.GetDistance(), Catch::Matchers::WithinAbs(750, 1.0));
  CHECK_THAT(encoder.GetRate(), Catch::Matchers::WithinAbs(0, 0.1));
}
