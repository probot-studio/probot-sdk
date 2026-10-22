// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/DifferentialDriveFeedforward.hpp"

#include <prosdk/thirdparty/eigen/Eigen/Core>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/system/LinearSystem.hpp"
#include "prosdk/math/system/Models.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"

TEST_CASE("DifferentialDriveFeedforwardTest CalculateWithTrackwidth",
          "[wpimath]") {
  constexpr auto kvLinear = 1_V / 1_mps;
  constexpr auto kaLinear = 1_V / 1_mps_sq;
  constexpr auto kvAngular = 1_V / 1_rad_per_s;
  constexpr auto kaAngular = 1_V / 1_rad_per_s_sq;
  constexpr auto trackwidth = 1_m;
  constexpr prosdk::units::second_t dt = 20_ms;

  prosdk::math::DifferentialDriveFeedforward differentialDriveFeedforward{
      kvLinear, kaLinear, kvAngular, kaAngular, trackwidth};
  prosdk::math::LinearSystem<2, 2, 2> plant =
      prosdk::math::Models::DifferentialDriveFromSysId(
          kvLinear, kaLinear, kvAngular, kaAngular, trackwidth);
  for (auto currentLeftVelocity = -4_mps; currentLeftVelocity <= 4_mps;
       currentLeftVelocity += 2_mps) {
    for (auto currentRightVelocity = -4_mps; currentRightVelocity <= 4_mps;
         currentRightVelocity += 2_mps) {
      for (auto nextLeftVelocity = -4_mps; nextLeftVelocity <= 4_mps;
           nextLeftVelocity += 2_mps) {
        for (auto nextRightVelocity = -4_mps; nextRightVelocity <= 4_mps;
             nextRightVelocity += 2_mps) {
          auto [left, right] = differentialDriveFeedforward.Calculate(
              currentLeftVelocity, nextLeftVelocity, currentRightVelocity,
              nextRightVelocity, dt);
          Eigen::Vector2d nextX = plant.CalculateX(
              Eigen::Vector2d{currentLeftVelocity, currentRightVelocity},
              Eigen::Vector2d{left, right}, dt);
          CHECK_NEAR(nextX(0), nextLeftVelocity.value(), 1e-6);
          CHECK_NEAR(nextX(1), nextRightVelocity.value(), 1e-6);
        }
      }
    }
  }
}

TEST_CASE("DifferentialDriveFeedforwardTest CalculateWithoutTrackwidth",
          "[wpimath]") {
  constexpr auto kvLinear = 1_V / 1_mps;
  constexpr auto kaLinear = 1_V / 1_mps_sq;
  constexpr auto kvAngular = 1_V / 1_mps;
  constexpr auto kaAngular = 1_V / 1_mps_sq;
  constexpr prosdk::units::second_t dt = 20_ms;

  prosdk::math::DifferentialDriveFeedforward differentialDriveFeedforward{
      kvLinear, kaLinear, kvAngular, kaAngular};
  prosdk::math::LinearSystem<2, 2, 2> plant =
      prosdk::math::Models::DifferentialDriveFromSysId(kvLinear, kaLinear,
                                                    kvAngular, kaAngular);
  for (auto currentLeftVelocity = -4_mps; currentLeftVelocity <= 4_mps;
       currentLeftVelocity += 2_mps) {
    for (auto currentRightVelocity = -4_mps; currentRightVelocity <= 4_mps;
         currentRightVelocity += 2_mps) {
      for (auto nextLeftVelocity = -4_mps; nextLeftVelocity <= 4_mps;
           nextLeftVelocity += 2_mps) {
        for (auto nextRightVelocity = -4_mps; nextRightVelocity <= 4_mps;
             nextRightVelocity += 2_mps) {
          auto [left, right] = differentialDriveFeedforward.Calculate(
              currentLeftVelocity, nextLeftVelocity, currentRightVelocity,
              nextRightVelocity, dt);
          Eigen::Vector2d nextX = plant.CalculateX(
              Eigen::Vector2d{currentLeftVelocity, currentRightVelocity},
              Eigen::Vector2d{left, right}, dt);
          CHECK_NEAR(nextX(0), nextLeftVelocity.value(), 1e-6);
          CHECK_NEAR(nextX(1), nextRightVelocity.value(), 1e-6);
        }
      }
    }
  }
}
