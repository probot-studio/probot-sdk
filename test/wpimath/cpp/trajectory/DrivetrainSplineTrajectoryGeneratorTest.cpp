// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp"

#include <cstddef>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/trajectory/TestDrivetrainSplineTrajectory.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"

using namespace prosdk::math;

TEST_CASE("DrivetrainSplineTrajectoryGeneratorTest ObeysConstraints",
          "[wpimath]") {
  TrajectoryConfig config{12_fps, 12_fps_sq};
  auto trajectory = TestDrivetrainSplineTrajectory::GetTrajectory(config);

  constexpr prosdk::units::second_t dt = 20_ms;

  for (auto t = 0_s; t < trajectory.Duration(); t += dt) {
    auto point = trajectory.SampleAt(t);

    CHECK(prosdk::units::math::abs(point.ForwardVelocity()) <= 12_fps + 0.01_fps);
    CHECK(prosdk::units::math::abs(point.ForwardAcceleration()) <=
          12_fps_sq + 0.01_fps_sq);
  }
}

TEST_CASE("DrivetrainSplineTrajectoryGeneratorTest ReturnsEmptyOnMalformed",
          "[wpimath]") {
  const auto t = DrivetrainSplineTrajectoryGenerator::Generate(
      std::vector<Pose2d>{Pose2d{0_m, 0_m, 0_deg}, Pose2d{1_m, 0_m, 180_deg}},
      TrajectoryConfig(12_fps, 12_fps_sq));

  REQUIRE(t.Samples().size() == 1u);
  REQUIRE(t.Duration() == 0_s);
}

TEST_CASE("DrivetrainSplineTrajectoryGeneratorTest CurvatureOptimization",
          "[wpimath]") {
  auto t = DrivetrainSplineTrajectoryGenerator::Generate(
      {{1_m, 0_m, 90_deg},
       {0_m, 1_m, 180_deg},
       {-1_m, 0_m, 270_deg},
       {0_m, -1_m, 0_deg},
       {1_m, 0_m, 90_deg}},
      TrajectoryConfig{12_fps, 12_fps_sq});

  for (size_t i = 1; i < t.Samples().size() - 1; ++i) {
    CHECK(0 != t.Samples()[i].curvature.value());
  }
}
