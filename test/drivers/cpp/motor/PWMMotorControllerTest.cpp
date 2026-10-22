// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>

#include "prosdk/drivers/motor/Spark.hpp"
#include "prosdk/simulation/PWMMotorControllerSim.hpp"

TEST_CASE("PWMMotorControllerTest ReportsThrottleToSimulation",
          "[drivers][pwm-motor-controller]") {
  prosdk::Spark motor{0};
  prosdk::sim::PWMMotorControllerSim sim{motor};

  motor.SetThrottle(0.354);

  CHECK(0.354 == sim.GetThrottle());
}
