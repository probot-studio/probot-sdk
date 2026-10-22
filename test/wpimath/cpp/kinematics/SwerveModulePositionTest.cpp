// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/SwerveModulePosition.hpp"

#include <catch2/catch_test_macros.hpp>

#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"

TEST_CASE("SwerveModulePositionTest Equality", "[wpimath]") {
  prosdk::math::SwerveModulePosition position1{2_m, 90_deg};
  prosdk::math::SwerveModulePosition position2{2_m, 90_deg};

  CHECK(position1 == position2);
}

TEST_CASE("SwerveModulePositionTest Inequality", "[wpimath]") {
  prosdk::math::SwerveModulePosition position1{1_m, 90_deg};
  prosdk::math::SwerveModulePosition position2{2_m, 90_deg};
  prosdk::math::SwerveModulePosition position3{1_m, 89_deg};

  CHECK(position1 != position2);
  CHECK(position1 != position3);
}
