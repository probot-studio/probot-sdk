// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "motorcontrol/TestPWMMotorController.hpp"
#include "prosdk/hardware/rotation/Encoder.hpp"
#include "prosdk/math/controller/PIDController.hpp"
#include "prosdk/math/controller/SimpleMotorFeedforward.hpp"
#include "prosdk/math/system/Models.hpp"
#include "prosdk/simulation/BatterySim.hpp"
#include "prosdk/simulation/EncoderSim.hpp"
#include "prosdk/simulation/FlywheelSim.hpp"
#include "prosdk/simulation/RoboRioSim.hpp"
#include "prosdk/system/RobotController.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/units/angular_velocity.hpp"

TEST_CASE("StateSpaceSimTest FlywheelSim", "[wpilibc][simulation]") {
  const prosdk::math::LinearSystem<1, 1, 1> plant =
      prosdk::math::Models::FlywheelFromSysId(0.02_V / 1_rad_per_s,
                                           0.01_V / 1_rad_per_s_sq);
  prosdk::sim::FlywheelSim sim{plant, prosdk::math::DCMotor::NEO(2)};
  prosdk::math::PIDController controller{0.2, 0.0, 0.0};
  prosdk::math::SimpleMotorFeedforward<prosdk::units::radian> feedforward{
      0_V, 0.02_V / 1_rad_per_s, 0.01_V / 1_rad_per_s_sq};
  prosdk::Encoder encoder{0, 1};
  prosdk::sim::EncoderSim encoderSim{encoder};
  prosdk::TestPWMMotorController motor{0};

  prosdk::sim::RoboRioSim::ResetData();
  encoderSim.ResetData();

  for (int i = 0; i < 100; i++) {
    // RobotPeriodic runs first
    auto voltageOut = controller.Calculate(encoder.GetRate(), 200.0);
    motor.SetVoltage(prosdk::units::volt_t{voltageOut} +
                     feedforward.Calculate(200_rad_per_s));

    // Then, SimulationPeriodic runs
    prosdk::sim::RoboRioSim::SetVInVoltage(
        prosdk::sim::BatterySim::Calculate({sim.GetCurrentDraw()}));
    sim.SetInput(prosdk::math::Vectord<1>{
        motor.GetThrottle() * prosdk::RobotController::GetInputVoltage()});
    sim.Update(20_ms);
    encoderSim.SetRate(sim.GetAngularVelocity().value());
  }

  REQUIRE(std::abs(200 - encoder.GetRate()) < 0.1);
}
