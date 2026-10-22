// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/imu/OnboardIMU.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/simulation/OnboardIMUSim.hpp"

using namespace prosdk;

TEST_CASE("OnboardIMUTest SimDevices", "[wpilibc]") {
  OnboardIMU imu{OnboardIMU::FLAT};

  CHECK(0.0 == imu.GetAngleX().value());
  CHECK(0.0 == imu.GetAngleY().value());
  CHECK(0.0 == imu.GetAngleZ().value());

  CHECK(0.0 == imu.GetGyroRateX().value());
  CHECK(0.0 == imu.GetGyroRateY().value());
  CHECK(0.0 == imu.GetGyroRateZ().value());

  CHECK(0.0 == imu.GetAccelX().value());
  CHECK(0.0 == imu.GetAccelY().value());
  CHECK(0.0 == imu.GetAccelZ().value());

  CHECK(0.0 == imu.GetYaw().value());

  sim::OnboardIMUSim sim{};

  sim.SetAngleX(prosdk::units::radian_t{1});
  sim.SetAngleY(prosdk::units::radian_t{2});
  sim.SetAngleZ(prosdk::units::radian_t{3});

  sim.SetGyroRateX(prosdk::units::radians_per_second_t{3.504});
  sim.SetGyroRateY(prosdk::units::radians_per_second_t{1.91});
  sim.SetGyroRateZ(prosdk::units::radians_per_second_t{22.9});

  sim.SetAccelX(prosdk::units::meters_per_second_squared_t{-1});
  sim.SetAccelY(prosdk::units::meters_per_second_squared_t{-2});
  sim.SetAccelZ(prosdk::units::meters_per_second_squared_t{-3});

  sim.SetYaw(prosdk::units::radian_t{1.234});

  CHECK(1.0 == imu.GetAngleX().value());
  CHECK(2.0 == imu.GetAngleY().value());
  CHECK(3.0 == imu.GetAngleZ().value());

  CHECK(3.504 == imu.GetGyroRateX().value());
  CHECK(1.91 == imu.GetGyroRateY().value());
  CHECK(22.9 == imu.GetGyroRateZ().value());

  CHECK(-1.0 == imu.GetAccelX().value());
  CHECK(-2.0 == imu.GetAccelY().value());
  CHECK(-3.0 == imu.GetAccelZ().value());

  CHECK(1.234 == imu.GetYaw().value());
}
