// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/trajectory/DifferentialSample.hpp"
#include "prosdk/math/trajectory/TestDrivetrainSplineTrajectory.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/math/trajectory/constraint/DifferentialDriveKinematicsConstraint.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"

using namespace prosdk::math;

TEST_CASE("DifferentialDriveKinematicsConstraintTest Constraint", "[wpimath]") {
  const auto maxVelocity = 12_fps;
  const DifferentialDriveKinematics kinematics{27_in};

  auto config = TrajectoryConfig(12_fps, 12_fps_sq);
  config.AddConstraint(
      DifferentialDriveKinematicsConstraint(kinematics, maxVelocity));

  auto trajectory = TestDrivetrainSplineTrajectory::GetTrajectory(config);

  for (auto t = 0_s; t < trajectory.Duration(); t += 20_ms) {
    auto point = trajectory.SampleAt(t);

    const DifferentialSample differentialSample{point, kinematics};

    CHECK(differentialSample.leftVelocity < maxVelocity + 0.05_mps);
    CHECK(differentialSample.rightVelocity < maxVelocity + 0.05_mps);
  }
}
