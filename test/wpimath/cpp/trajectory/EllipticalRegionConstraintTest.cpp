// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/constraint/EllipticalRegionConstraint.hpp"

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/shape/Ellipse2d.hpp"
#include "prosdk/math/trajectory/TestDrivetrainSplineTrajectory.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/math/trajectory/constraint/MaxVelocityConstraint.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/units/velocity.hpp"

using namespace prosdk::math;

TEST_CASE("EllipticalRegionConstraintTest Constraint", "[wpimath]") {
  constexpr auto maxVelocity = 2_fps;
  constexpr prosdk::math::Ellipse2d ellipse{{5_ft, 2.5_ft, 180_deg}, 5_ft, 2.5_ft};

  auto config = TrajectoryConfig(13_fps, 13_fps_sq);
  config.AddConstraint(
      EllipticalRegionConstraint{ellipse, MaxVelocityConstraint{maxVelocity}});
  auto trajectory = TestDrivetrainSplineTrajectory::GetTrajectory(config);

  bool exceededConstraintOutsideRegion = false;
  for (auto& point : trajectory.Samples()) {
    if (ellipse.Contains(point.pose.Translation())) {
      CHECK(prosdk::units::math::abs(point.ForwardVelocity()) <
            maxVelocity + 0.05_mps);
    } else if (prosdk::units::math::abs(point.ForwardVelocity()) >=
               maxVelocity + 0.05_mps) {
      exceededConstraintOutsideRegion = true;
    }
  }
  CHECK(exceededConstraintOutsideRegion);
}
