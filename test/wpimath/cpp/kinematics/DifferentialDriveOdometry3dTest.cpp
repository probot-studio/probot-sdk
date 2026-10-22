// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/DifferentialDriveOdometry3d.hpp"

#include <numbers>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/math/geometry/Translation3d.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"

using namespace prosdk::math;

static constexpr double EPSILON = 1E-9;

TEST_CASE("DifferentialDriveOdometry3dTest Initialize", "[wpimath]") {
  DifferentialDriveOdometry3d odometry{
      prosdk::math::Rotation3d{0_deg, 0_deg, 90_deg}, 0_m, 0_m,
      prosdk::math::Pose3d{1_m, 2_m, 0_m,
                        prosdk::math::Rotation3d{0_deg, 0_deg, 45_deg}}};

  const prosdk::math::Pose3d& pose = odometry.GetPose();

  CHECK_NEAR(pose.X().value(), 1, EPSILON);
  CHECK_NEAR(pose.Y().value(), 2, EPSILON);
  CHECK_NEAR(pose.Z().value(), 0, EPSILON);
  CHECK_NEAR(pose.Rotation().ToRotation2d().Degrees().value(), 45, EPSILON);
}

TEST_CASE("DifferentialDriveOdometry3dTest EncoderDistances", "[wpimath]") {
  DifferentialDriveOdometry3d odometry{
      prosdk::math::Rotation3d{0_deg, 0_deg, 45_deg}, 0_m, 0_m};

  const auto& pose =
      odometry.Update(prosdk::math::Rotation3d{0_deg, 0_deg, 135_deg}, 0_m,
                      prosdk::units::meter_t{5 * std::numbers::pi});

  CHECK_NEAR(pose.X().value(), 5.0, EPSILON);
  CHECK_NEAR(pose.Y().value(), 5.0, EPSILON);
  CHECK_NEAR(pose.Z().value(), 0.0, EPSILON);
  CHECK_NEAR(pose.Rotation().ToRotation2d().Degrees().value(), 90.0, EPSILON);
}

TEST_CASE("DifferentialDriveOdometry3dTest GyroOffset", "[wpimath]") {
  DifferentialDriveOdometry3d odometry{
      prosdk::math::Rotation3d{0_deg, 5_deg, 0_deg}, 0_m, 0_m,
      prosdk::math::Pose3d{prosdk::math::Translation3d{},
                        prosdk::math::Rotation3d{0_deg, 0_deg, 90_deg}}};
  const auto& pose =
      odometry.Update(prosdk::math::Rotation3d{0_deg, 10_deg, 0_deg}, 0_m, 0_m);

  CHECK_NEAR(pose.X().value(), 0.0, EPSILON);
  CHECK_NEAR(pose.Y().value(), 0.0, EPSILON);
  CHECK_NEAR(pose.Z().value(), 0.0, EPSILON);
  CHECK_NEAR(prosdk::units::degree_t{pose.Rotation().X()}.value(), 0.0, EPSILON);
  CHECK_NEAR(prosdk::units::degree_t{pose.Rotation().Y()}.value(), 5.0, EPSILON);
  CHECK_NEAR(prosdk::units::degree_t{pose.Rotation().Z()}.value(), 90.0, EPSILON);
}
