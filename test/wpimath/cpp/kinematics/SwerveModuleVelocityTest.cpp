// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/SwerveModuleVelocity.hpp"

#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/velocity.hpp"

static constexpr double EPSILON = 1E-9;

TEST_CASE("SwerveModuleVelocityTest Optimize", "[wpimath]") {
  prosdk::math::Rotation2d angleA{45_deg};
  prosdk::math::SwerveModuleVelocity refA{-2_mps, 180_deg};
  auto optimizedA = refA.Optimize(angleA);

  CHECK_NEAR(optimizedA.velocity.value(), 2.0, EPSILON);
  CHECK_NEAR(optimizedA.angle.Degrees().value(), 0.0, EPSILON);

  prosdk::math::Rotation2d angleB{-50_deg};
  prosdk::math::SwerveModuleVelocity refB{4.7_mps, 41_deg};
  auto optimizedB = refB.Optimize(angleB);

  CHECK_NEAR(optimizedB.velocity.value(), -4.7, EPSILON);
  CHECK_NEAR(optimizedB.angle.Degrees().value(), -139.0, EPSILON);
}

TEST_CASE("SwerveModuleVelocityTest NoOptimize", "[wpimath]") {
  prosdk::math::Rotation2d angleA{0_deg};
  prosdk::math::SwerveModuleVelocity refA{2_mps, 89_deg};
  auto optimizedA = refA.Optimize(angleA);

  CHECK_NEAR(optimizedA.velocity.value(), 2.0, EPSILON);
  CHECK_NEAR(optimizedA.angle.Degrees().value(), 89.0, EPSILON);

  prosdk::math::Rotation2d angleB{0_deg};
  prosdk::math::SwerveModuleVelocity refB{-2_mps, -2_deg};
  auto optimizedB = refB.Optimize(angleB);

  CHECK_NEAR(optimizedB.velocity.value(), -2.0, EPSILON);
  CHECK_NEAR(optimizedB.angle.Degrees().value(), -2.0, EPSILON);
}

TEST_CASE("SwerveModuleVelocityTest CosineScaling", "[wpimath]") {
  prosdk::math::Rotation2d angleA{0_deg};
  prosdk::math::SwerveModuleVelocity refA{2_mps, 45_deg};
  auto optimizedA = refA.CosineScale(angleA);

  CHECK_NEAR(optimizedA.velocity.value(), std::sqrt(2.0), EPSILON);
  CHECK_NEAR(optimizedA.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleB{45_deg};
  prosdk::math::SwerveModuleVelocity refB{2_mps, 45_deg};
  auto optimizedB = refB.CosineScale(angleB);

  CHECK_NEAR(optimizedB.velocity.value(), 2.0, EPSILON);
  CHECK_NEAR(optimizedB.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleC{-45_deg};
  prosdk::math::SwerveModuleVelocity refC{2_mps, 45_deg};
  auto optimizedC = refC.CosineScale(angleC);

  CHECK_NEAR(optimizedC.velocity.value(), 0.0, EPSILON);
  CHECK_NEAR(optimizedC.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleD{135_deg};
  prosdk::math::SwerveModuleVelocity refD{2_mps, 45_deg};
  auto optimizedD = refD.CosineScale(angleD);

  CHECK_NEAR(optimizedD.velocity.value(), 0.0, EPSILON);
  CHECK_NEAR(optimizedD.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleE{-135_deg};
  prosdk::math::SwerveModuleVelocity refE{2_mps, 45_deg};
  auto optimizedE = refE.CosineScale(angleE);

  CHECK_NEAR(optimizedE.velocity.value(), -2.0, EPSILON);
  CHECK_NEAR(optimizedE.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleF{180_deg};
  prosdk::math::SwerveModuleVelocity refF{2_mps, 45_deg};
  auto optimizedF = refF.CosineScale(angleF);

  CHECK_NEAR(optimizedF.velocity.value(), -std::sqrt(2.0), EPSILON);
  CHECK_NEAR(optimizedF.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleG{0_deg};
  prosdk::math::SwerveModuleVelocity refG{-2_mps, 45_deg};
  auto optimizedG = refG.CosineScale(angleG);

  CHECK_NEAR(optimizedG.velocity.value(), -std::sqrt(2.0), EPSILON);
  CHECK_NEAR(optimizedG.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleH{45_deg};
  prosdk::math::SwerveModuleVelocity refH{-2_mps, 45_deg};
  auto optimizedH = refH.CosineScale(angleH);

  CHECK_NEAR(optimizedH.velocity.value(), -2.0, EPSILON);
  CHECK_NEAR(optimizedH.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleI{-45_deg};
  prosdk::math::SwerveModuleVelocity refI{-2_mps, 45_deg};
  auto optimizedI = refI.CosineScale(angleI);

  CHECK_NEAR(optimizedI.velocity.value(), 0.0, EPSILON);
  CHECK_NEAR(optimizedI.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleJ{135_deg};
  prosdk::math::SwerveModuleVelocity refJ{-2_mps, 45_deg};
  auto optimizedJ = refJ.CosineScale(angleJ);

  CHECK_NEAR(optimizedJ.velocity.value(), 0.0, EPSILON);
  CHECK_NEAR(optimizedJ.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleK{-135_deg};
  prosdk::math::SwerveModuleVelocity refK{-2_mps, 45_deg};
  auto optimizedK = refK.CosineScale(angleK);

  CHECK_NEAR(optimizedK.velocity.value(), 2.0, EPSILON);
  CHECK_NEAR(optimizedK.angle.Degrees().value(), 45.0, EPSILON);

  prosdk::math::Rotation2d angleL{180_deg};
  prosdk::math::SwerveModuleVelocity refL{-2_mps, 45_deg};
  auto optimizedL = refL.CosineScale(angleL);

  CHECK_NEAR(optimizedL.velocity.value(), std::sqrt(2.0), EPSILON);
  CHECK_NEAR(optimizedL.angle.Degrees().value(), 45.0, EPSILON);
}

TEST_CASE("SwerveModuleVelocityTest Equality", "[wpimath]") {
  prosdk::math::SwerveModuleVelocity state1{2_mps, 90_deg};
  prosdk::math::SwerveModuleVelocity state2{2_mps, 90_deg};

  CHECK(state1 == state2);
}

TEST_CASE("SwerveModuleVelocityTest Inequality", "[wpimath]") {
  prosdk::math::SwerveModuleVelocity state1{1_mps, 90_deg};
  prosdk::math::SwerveModuleVelocity state2{2_mps, 90_deg};
  prosdk::math::SwerveModuleVelocity state3{1_mps, 89_deg};

  CHECK(state1 != state2);
  CHECK(state1 != state3);
}
