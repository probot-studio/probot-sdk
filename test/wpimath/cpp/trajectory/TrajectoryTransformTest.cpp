// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/kinematics/ChassisAccelerations.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/math/trajectory/DifferentialSample.hpp"
#include "prosdk/math/trajectory/DifferentialTrajectory.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineSample.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp"
#include "prosdk/math/trajectory/HolonomicSample.hpp"
#include "prosdk/math/trajectory/HolonomicTrajectory.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"

void TestSameShapedTrajectory(
    const std::vector<prosdk::math::DrivetrainSplineSample>& statesA,
    const std::vector<prosdk::math::DrivetrainSplineSample>& statesB) {
  for (unsigned int i = 0; i < statesA.size() - 1; i++) {
    auto a1 = statesA[i].pose;
    auto a2 = statesA[i + 1].pose;

    auto b1 = statesB[i].pose;
    auto b2 = statesB[i + 1].pose;

    auto a = a2.RelativeTo(a1);
    auto b = b2.RelativeTo(b1);

    CHECK_NEAR(a.X().value(), b.X().value(), 1E-9);
    CHECK_NEAR(a.Y().value(), b.Y().value(), 1E-9);
    CHECK_NEAR(a.Rotation().Radians().value(), b.Rotation().Radians().value(),
               1E-9);
  }
}

// A rigid transform rotates both the heading and the field-relative
// velocity/acceleration by the same amount, so the heading-relative forward
// scalars (and curvature) are invariant. This would fail if
// TransformBy/RelativeTo rotated the pose but not the velocity/acceleration.
void TestSameForwardScalars(
    const std::vector<prosdk::math::DrivetrainSplineSample>& statesA,
    const std::vector<prosdk::math::DrivetrainSplineSample>& statesB) {
  REQUIRE(statesA.size() == statesB.size());
  for (unsigned int i = 0; i < statesA.size(); i++) {
    CHECK_NEAR(statesA[i].ForwardVelocity().value(),
               statesB[i].ForwardVelocity().value(), 1E-9);
    CHECK_NEAR(statesA[i].ForwardAcceleration().value(),
               statesB[i].ForwardAcceleration().value(), 1E-9);
    CHECK_NEAR(statesA[i].curvature.value(), statesB[i].curvature.value(),
               1E-9);
  }
}

template <typename SampleType>
void TestSameShapedTrajectory(const std::vector<SampleType>& statesA,
                              const std::vector<SampleType>& statesB) {
  REQUIRE(statesA.size() == statesB.size());
  for (unsigned int i = 0; i < statesA.size() - 1; i++) {
    auto a = statesA[i + 1].pose.RelativeTo(statesA[i].pose);
    auto b = statesB[i + 1].pose.RelativeTo(statesB[i].pose);

    CHECK_NEAR(a.X().value(), b.X().value(), 1E-9);
    CHECK_NEAR(a.Y().value(), b.Y().value(), 1E-9);
    CHECK_NEAR(a.Rotation().Radians().value(), b.Rotation().Radians().value(),
               1E-9);
  }
}

// A rigid transform rotates both the heading and the field-relative
// velocity/acceleration by the same amount, so the velocity and acceleration
// in each sample's heading frame are invariant. This would fail if
// TransformBy/RelativeTo rotated the pose but not the velocity/acceleration.
template <typename SampleType>
void TestSameRobotRelativeMotion(const std::vector<SampleType>& statesA,
                                 const std::vector<SampleType>& statesB) {
  REQUIRE(statesA.size() == statesB.size());
  for (unsigned int i = 0; i < statesA.size(); i++) {
    auto velocityA =
        statesA[i].velocity.ToRobotRelative(statesA[i].pose.Rotation());
    auto velocityB =
        statesB[i].velocity.ToRobotRelative(statesB[i].pose.Rotation());
    CHECK_NEAR(velocityA.vx.value(), velocityB.vx.value(), 1E-9);
    CHECK_NEAR(velocityA.vy.value(), velocityB.vy.value(), 1E-9);
    CHECK_NEAR(velocityA.omega.value(), velocityB.omega.value(), 1E-9);

    auto accelerationA =
        statesA[i].acceleration.ToRobotRelative(statesA[i].pose.Rotation());
    auto accelerationB =
        statesB[i].acceleration.ToRobotRelative(statesB[i].pose.Rotation());
    CHECK_NEAR(accelerationA.ax.value(), accelerationB.ax.value(), 1E-9);
    CHECK_NEAR(accelerationA.ay.value(), accelerationB.ay.value(), 1E-9);
    CHECK_NEAR(accelerationA.alpha.value(), accelerationB.alpha.value(), 1E-9);
  }
}

prosdk::math::HolonomicTrajectory MakeHolonomicTrajectory() {
  return prosdk::math::HolonomicTrajectory{std::vector<prosdk::math::HolonomicSample>{
      {0_s,
       {1_m, 2_m, 30_deg},
       {1_mps, 0.5_mps, 0.2_rad_per_s},
       {0.3_mps_sq, -0.1_mps_sq, 0.05_rad_per_s_sq}},
      {1_s,
       {2_m, 2.5_m, 40_deg},
       {1.5_mps, -0.25_mps, 0.1_rad_per_s},
       {-0.2_mps_sq, 0.4_mps_sq, -0.1_rad_per_s_sq}},
      {2_s,
       {2.5_m, 3.5_m, 60_deg},
       {0.5_mps, 1_mps, -0.3_rad_per_s},
       {0.1_mps_sq, 0.2_mps_sq, 0.15_rad_per_s_sq}}}};
}

// Wheel velocities are robot-relative, so they're invariant under rigid
// transforms
void TestSameWheelVelocities(
    const std::vector<prosdk::math::DifferentialSample>& statesA,
    const std::vector<prosdk::math::DifferentialSample>& statesB) {
  REQUIRE(statesA.size() == statesB.size());
  for (unsigned int i = 0; i < statesA.size(); i++) {
    CHECK(statesA[i].leftVelocity == statesB[i].leftVelocity);
    CHECK(statesA[i].rightVelocity == statesB[i].rightVelocity);
  }
}

prosdk::math::DifferentialTrajectory MakeDifferentialTrajectory() {
  // Differential drives can't move sideways, so the robot-relative velocity
  // and acceleration have no lateral component
  auto makeSample = [](prosdk::units::second_t time, const prosdk::math::Pose2d& pose,
                       prosdk::units::meters_per_second_t v,
                       prosdk::units::radians_per_second_t ω,
                       prosdk::units::meters_per_second_squared_t a,
                       prosdk::units::radians_per_second_squared_t α,
                       prosdk::units::meters_per_second_t leftVelocity,
                       prosdk::units::meters_per_second_t rightVelocity) {
    return prosdk::math::DifferentialSample{
        time,
        pose,
        prosdk::math::ChassisVelocities{v, 0_mps, ω}.ToFieldRelative(
            pose.Rotation()),
        prosdk::math::ChassisAccelerations{a, 0_mps_sq, α}.ToFieldRelative(
            pose.Rotation()),
        leftVelocity,
        rightVelocity};
  };

  return prosdk::math::DifferentialTrajectory{
      std::vector<prosdk::math::DifferentialSample>{
          makeSample(0_s, {1_m, 2_m, 30_deg}, 1_mps, 0.2_rad_per_s, 0.3_mps_sq,
                     0.05_rad_per_s_sq, 0.95_mps, 1.05_mps),
          makeSample(1_s, {2_m, 2.5_m, 40_deg}, 1.5_mps, 0.1_rad_per_s,
                     -0.2_mps_sq, -0.1_rad_per_s_sq, 1.475_mps, 1.525_mps),
          makeSample(2_s, {2.5_m, 3.5_m, 60_deg}, 0.5_mps, -0.3_rad_per_s,
                     0.1_mps_sq, 0.15_rad_per_s_sq, 0.575_mps, 0.425_mps)}};
}

TEST_CASE("TrajectoryTransformTest HolonomicTransformBy", "[wpimath]") {
  auto trajectory = MakeHolonomicTrajectory();

  auto transformedTrajectory = trajectory.TransformBy({{1_m, 2_m}, 30_deg});

  auto firstPose = transformedTrajectory.SampleAt(0_s).pose;
  auto expectedFirstPose =
      trajectory.Start().pose.TransformBy({{1_m, 2_m}, 30_deg});

  CHECK_NEAR(firstPose.X().value(), expectedFirstPose.X().value(), 1E-9);
  CHECK_NEAR(firstPose.Y().value(), expectedFirstPose.Y().value(), 1E-9);
  CHECK_NEAR(firstPose.Rotation().Degrees().value(), 60.0, 1E-9);

  // Field-relative velocity and acceleration rotate by the transform's rotation
  for (unsigned int i = 0; i < trajectory.Samples().size(); i++) {
    const auto& sample = trajectory.Samples()[i];
    const auto& transformedSample = transformedTrajectory.Samples()[i];

    auto expectedVelocity = sample.velocity.ToFieldRelative(30_deg);
    CHECK_NEAR(transformedSample.velocity.vx.value(),
               expectedVelocity.vx.value(), 1E-9);
    CHECK_NEAR(transformedSample.velocity.vy.value(),
               expectedVelocity.vy.value(), 1E-9);
    CHECK_NEAR(transformedSample.velocity.omega.value(),
               expectedVelocity.omega.value(), 1E-9);

    auto expectedAcceleration = sample.acceleration.ToFieldRelative(30_deg);
    CHECK_NEAR(transformedSample.acceleration.ax.value(),
               expectedAcceleration.ax.value(), 1E-9);
    CHECK_NEAR(transformedSample.acceleration.ay.value(),
               expectedAcceleration.ay.value(), 1E-9);
    CHECK_NEAR(transformedSample.acceleration.alpha.value(),
               expectedAcceleration.alpha.value(), 1E-9);
  }

  TestSameShapedTrajectory(trajectory.Samples(),
                           transformedTrajectory.Samples());
  TestSameRobotRelativeMotion(trajectory.Samples(),
                              transformedTrajectory.Samples());
}

TEST_CASE("TrajectoryTransformTest HolonomicRelativeTo", "[wpimath]") {
  auto trajectory = MakeHolonomicTrajectory();

  auto transformedTrajectory = trajectory.RelativeTo({1_m, 2_m, 30_deg});

  auto firstPose = transformedTrajectory.SampleAt(0_s).pose;

  CHECK_NEAR(firstPose.X().value(), 0, 1E-9);
  CHECK_NEAR(firstPose.Y().value(), 0, 1E-9);
  CHECK_NEAR(firstPose.Rotation().Degrees().value(), 0, 1E-9);

  TestSameShapedTrajectory(trajectory.Samples(),
                           transformedTrajectory.Samples());
  TestSameRobotRelativeMotion(trajectory.Samples(),
                              transformedTrajectory.Samples());
}

TEST_CASE("TrajectoryTransformTest TransformBy", "[wpimath]") {
  prosdk::math::TrajectoryConfig config{3_mps, 3_mps_sq};
  auto trajectory = prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
      prosdk::math::Pose2d{}, {}, prosdk::math::Pose2d{1_m, 1_m, 90_deg}, config);

  auto transformedTrajectory = trajectory.TransformBy({{1_m, 2_m}, 30_deg});

  auto firstPose = transformedTrajectory.SampleAt(0_s).pose;

  CHECK_NEAR(firstPose.X().value(), 1.0, 1E-9);
  CHECK_NEAR(firstPose.Y().value(), 2.0, 1E-9);
  CHECK_NEAR(firstPose.Rotation().Degrees().value(), 30.0, 1E-9);

  TestSameShapedTrajectory(trajectory.Samples(),
                           transformedTrajectory.Samples());
  TestSameForwardScalars(trajectory.Samples(), transformedTrajectory.Samples());
}

TEST_CASE("TrajectoryTransformTest RelativeTo", "[wpimath]") {
  prosdk::math::TrajectoryConfig config{3_mps, 3_mps_sq};
  auto trajectory = prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
      prosdk::math::Pose2d{1_m, 2_m, 30_deg}, {},
      prosdk::math::Pose2d{5_m, 7_m, 90_deg}, config);

  auto transformedTrajectory = trajectory.RelativeTo({1_m, 2_m, 30_deg});

  auto firstPose = transformedTrajectory.SampleAt(0_s).pose;

  CHECK_NEAR(firstPose.X().value(), 0, 1E-9);
  CHECK_NEAR(firstPose.Y().value(), 0, 1E-9);
  CHECK_NEAR(firstPose.Rotation().Degrees().value(), 0, 1E-9);

  TestSameShapedTrajectory(trajectory.Samples(),
                           transformedTrajectory.Samples());
  TestSameForwardScalars(trajectory.Samples(), transformedTrajectory.Samples());
}

TEST_CASE("TrajectoryTransformTest DifferentialTransformBy", "[wpimath]") {
  auto trajectory = MakeDifferentialTrajectory();

  auto transformedTrajectory = trajectory.TransformBy({{1_m, 2_m}, 30_deg});

  auto firstPose = transformedTrajectory.SampleAt(0_s).pose;
  auto expectedFirstPose =
      trajectory.Start().pose.TransformBy({{1_m, 2_m}, 30_deg});

  CHECK_NEAR(firstPose.X().value(), expectedFirstPose.X().value(), 1E-9);
  CHECK_NEAR(firstPose.Y().value(), expectedFirstPose.Y().value(), 1E-9);
  CHECK_NEAR(firstPose.Rotation().Degrees().value(), 60.0, 1E-9);

  // Field-relative velocity and acceleration rotate by the transform's rotation
  for (unsigned int i = 0; i < trajectory.Samples().size(); i++) {
    const auto& sample = trajectory.Samples()[i];
    const auto& transformedSample = transformedTrajectory.Samples()[i];

    auto expectedVelocity = sample.velocity.ToFieldRelative(30_deg);
    CHECK_NEAR(transformedSample.velocity.vx.value(),
               expectedVelocity.vx.value(), 1E-9);
    CHECK_NEAR(transformedSample.velocity.vy.value(),
               expectedVelocity.vy.value(), 1E-9);
    CHECK_NEAR(transformedSample.velocity.omega.value(),
               expectedVelocity.omega.value(), 1E-9);

    auto expectedAcceleration = sample.acceleration.ToFieldRelative(30_deg);
    CHECK_NEAR(transformedSample.acceleration.ax.value(),
               expectedAcceleration.ax.value(), 1E-9);
    CHECK_NEAR(transformedSample.acceleration.ay.value(),
               expectedAcceleration.ay.value(), 1E-9);
    CHECK_NEAR(transformedSample.acceleration.alpha.value(),
               expectedAcceleration.alpha.value(), 1E-9);
  }

  TestSameShapedTrajectory(trajectory.Samples(),
                           transformedTrajectory.Samples());
  TestSameRobotRelativeMotion(trajectory.Samples(),
                              transformedTrajectory.Samples());
  TestSameWheelVelocities(trajectory.Samples(),
                          transformedTrajectory.Samples());
}

TEST_CASE("TrajectoryTransformTest DifferentialRelativeTo", "[wpimath]") {
  auto trajectory = MakeDifferentialTrajectory();

  auto transformedTrajectory = trajectory.RelativeTo({1_m, 2_m, 30_deg});

  auto firstPose = transformedTrajectory.SampleAt(0_s).pose;

  CHECK_NEAR(firstPose.X().value(), 0, 1E-9);
  CHECK_NEAR(firstPose.Y().value(), 0, 1E-9);
  CHECK_NEAR(firstPose.Rotation().Degrees().value(), 0, 1E-9);

  TestSameShapedTrajectory(trajectory.Samples(),
                           transformedTrajectory.Samples());
  TestSameRobotRelativeMotion(trajectory.Samples(),
                              transformedTrajectory.Samples());
  TestSameWheelVelocities(trajectory.Samples(),
                          transformedTrajectory.Samples());
}
