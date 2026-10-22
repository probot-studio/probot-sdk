// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/TwoDeadWheelOdometry.hpp"

#include <limits>
#include <random>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineSample.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectory.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"

using namespace prosdk::math;

class TwoDeadWheelOdometryTest {
 protected:
  prosdk::units::meter_t m_xWheelYPos = 1_m;
  prosdk::units::meter_t m_yWheelXPos = 1_m;

  TwoDeadWheelOdometry odometry{m_xWheelYPos, m_yWheelXPos, 0_m,
                                0_m,          0_rad,        Pose2d{}};

  Matrixd<2, 3> m_inverseKinematicsMatrix = Matrixd<2, 3>{
      {1, 0, -m_xWheelYPos.value()}, {0, 1, m_yWheelXPos.value()}};
};

TEST_CASE_METHOD(TwoDeadWheelOdometryTest,
                 "TwoDeadWheelOdometryTest MultipleConsecutiveUpdates",
                 "[wpimath]") {
  odometry.ResetPosition(1_m, 1_m, 0_rad, Pose2d{});

  odometry.Update(1_m, 1_m, 0_rad);
  auto secondPose = odometry.Update(1_m, 1_m, 0_rad);

  CHECK_NEAR(secondPose.X().value(), 0.0, 0.01);
  CHECK_NEAR(secondPose.Y().value(), 0.0, 0.01);
  CHECK_NEAR(secondPose.Rotation().Radians().value(), 0.0, 0.01);
}

TEST_CASE_METHOD(TwoDeadWheelOdometryTest,
                 "TwoDeadWheelOdometryTest TwoIterations", "[wpimath]") {
  odometry.ResetPosition(0_m, 0_m, 0_rad, Pose2d{});

  odometry.Update(0_m, 0_m, 0_rad);
  auto pose = odometry.Update(0.1_m, 0_m, 0_rad);

  CHECK_NEAR(pose.X().value(), 0.1, 0.01);
  CHECK_NEAR(pose.Y().value(), 0.0, 0.01);
  CHECK_NEAR(pose.Rotation().Radians().value(), 0.0, 0.01);
}

TEST_CASE_METHOD(TwoDeadWheelOdometryTest,
                 "TwoDeadWheelOdometryTest GyroAngleReset", "[wpimath]") {
  odometry.ResetPosition(0_m, 0_m, 90_deg, Pose2d{});

  odometry.Update(1_m, 0_m, 90_deg);
  auto pose = odometry.Update(1_m, 0_m, 90_deg);

  CHECK_NEAR(pose.X().value(), 1, 0.01);
  CHECK_NEAR(pose.Y().value(), 0.0, 0.01);
  CHECK_NEAR(pose.Rotation().Radians().value(), 0.0, 0.01);
}

TEST_CASE_METHOD(TwoDeadWheelOdometryTest,
                 "TwoDeadWheelOdometryTest StraightForwardsForwardKinematics",
                 "[wpimath]") {
  const auto chassisVelocities =
      odometry.ToChassisVelocities(5_mps, 0_mps, 0_rad_per_s);

  CHECK_NEAR(chassisVelocities.vx.value(), 5, 0.1);
  CHECK_NEAR(chassisVelocities.vy.value(), 0, 0.1);
  CHECK_NEAR(chassisVelocities.omega.value(), 0, 0.1);
}

TEST_CASE_METHOD(TwoDeadWheelOdometryTest,
                 "TwoDeadWheelOdometryTest StraightLeftForwardKinematics",
                 "[wpimath]") {
  const auto chassisVelocities =
      odometry.ToChassisVelocities(0_mps, 5_mps, 0_rad_per_s);

  CHECK_NEAR(chassisVelocities.vx.value(), 0, 0.1);
  CHECK_NEAR(chassisVelocities.vy.value(), 5, 0.1);
  CHECK_NEAR(chassisVelocities.omega.value(), 0, 0.1);
}

TEST_CASE_METHOD(TwoDeadWheelOdometryTest,
                 "TwoDeadWheelOdometryTest SpinInPlaceForwardKinematics",
                 "[wpimath]") {
  const auto chassisVelocities =
      odometry.ToChassisVelocities(-5_mps, 5_mps, 5_rad_per_s);

  CHECK_NEAR(chassisVelocities.vx.value(), 0, 0.1);
  CHECK_NEAR(chassisVelocities.vy.value(), 0, 0.1);
  CHECK_NEAR(chassisVelocities.omega.value(), 5, 0.1);
}

TEST_CASE_METHOD(TwoDeadWheelOdometryTest,
                 "TwoDeadWheelOdometryTest MixedMotionForwardKinematics",
                 "[wpimath]") {
  const auto chassisVelocities =
      odometry.ToChassisVelocities(1_mps, -1_mps, 5_rad_per_s);

  CHECK_NEAR(chassisVelocities.vx.value(), 6, 0.1);
  CHECK_NEAR(chassisVelocities.vy.value(), -6, 0.1);
  CHECK_NEAR(chassisVelocities.omega.value(), 5, 0.1);
}

TEST_CASE_METHOD(TwoDeadWheelOdometryTest,
                 "TwoDeadWheelOdometryTest AccuracyFacingTrajectory",
                 "[wpimath]") {
  auto xWheelPos = 0_m;
  auto yWheelPos = 0_m;

  prosdk::math::DrivetrainSplineTrajectory trajectory =
      prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
          std::vector{prosdk::math::Pose2d{0_m, 0_m, 45_deg},
                      prosdk::math::Pose2d{20_m, 20_m, -90_deg},
                      prosdk::math::Pose2d{10_m, 10_m, 135_deg},
                      prosdk::math::Pose2d{30_m, 30_m, -90_deg},
                      prosdk::math::Pose2d{20_m, 20_m, 45_deg}},
          prosdk::math::TrajectoryConfig(0.5_mps, 2.0_mps_sq));

  odometry.ResetPosition(xWheelPos, yWheelPos,
                         trajectory.Start().pose.Rotation(),
                         trajectory.Start().pose);

  std::mt19937 generator{5190};
  std::normal_distribution<double> distribution(0.0, 1.0);

  prosdk::units::second_t dt = 20_ms;
  prosdk::units::second_t t = 0_s;

  double maxError = -std::numeric_limits<double>::max();
  double errorSum = 0;

  prosdk::units::meter_t odometryDistanceTravelled = 0_m;
  prosdk::units::meter_t trajectoryDistanceTravelled = 0_m;

  while (t <= trajectory.Duration()) {
    prosdk::math::DrivetrainSplineSample groundTruthState = trajectory.SampleAt(t);

    trajectoryDistanceTravelled +=
        groundTruthState.ForwardVelocity() * dt +
        0.5 * groundTruthState.ForwardAcceleration() * dt * dt;

    Eigen::Vector2d wheelVelocities =
        m_inverseKinematicsMatrix *
        Eigen::Vector3d{groundTruthState.ForwardVelocity().value(), 0,
                        groundTruthState.ForwardVelocity().value() *
                            groundTruthState.curvature.value()};

    auto xWheelVel = prosdk::units::meters_per_second_t{wheelVelocities(0, 0)} +
                     distribution(generator) * 0.05_mps;
    auto yWheelVel = prosdk::units::meters_per_second_t{wheelVelocities(1, 0)} +
                     distribution(generator) * 0.05_mps;

    xWheelPos += xWheelVel * dt;
    yWheelPos += yWheelVel * dt;

    auto lastPose = odometry.GetPose();

    auto xhat = odometry.Update(
        xWheelPos, yWheelPos,
        groundTruthState.pose.Rotation() +
            prosdk::math::Rotation2d{distribution(generator) * 0.001_rad});

    odometryDistanceTravelled +=
        lastPose.Translation().Distance(xhat.Translation());
    double error = groundTruthState.pose.Translation()
                       .Distance(xhat.Translation())
                       .value();

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  CHECK(errorSum / (trajectory.Duration().value() / dt.value()) < 0.35);
  CHECK(maxError < 0.35);
  CHECK_NEAR(trajectoryDistanceTravelled.value(),
             odometryDistanceTravelled.value(),
             trajectoryDistanceTravelled.value() * 0.05);
}

TEST_CASE_METHOD(TwoDeadWheelOdometryTest,
                 "TwoDeadWheelOdometryTest AccuracyFacingXAxis", "[wpimath]") {
  auto xWheelPos = 0_m;
  auto yWheelPos = 0_m;

  prosdk::math::DrivetrainSplineTrajectory trajectory =
      prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
          std::vector{prosdk::math::Pose2d{0_m, 0_m, 45_deg},
                      prosdk::math::Pose2d{20_m, 20_m, -90_deg},
                      prosdk::math::Pose2d{10_m, 10_m, 135_deg},
                      prosdk::math::Pose2d{30_m, 30_m, -90_deg},
                      prosdk::math::Pose2d{20_m, 20_m, 45_deg}},
          prosdk::math::TrajectoryConfig(0.5_mps, 2.0_mps_sq));

  odometry.ResetPosition(xWheelPos, yWheelPos, Rotation2d{}, Pose2d{});

  std::mt19937 generator{5190};
  std::normal_distribution<double> distribution(0.0, 1.0);

  prosdk::units::second_t dt = 20_ms;
  prosdk::units::second_t t = 0_s;

  double maxError = -std::numeric_limits<double>::max();
  double errorSum = 0;

  prosdk::units::meter_t odometryDistanceTravelled = 0_m;
  prosdk::units::meter_t trajectoryDistanceTravelled = 0_m;

  while (t <= trajectory.Duration()) {
    prosdk::math::DrivetrainSplineSample groundTruthState = trajectory.SampleAt(t);

    trajectoryDistanceTravelled +=
        groundTruthState.ForwardVelocity() * dt +
        0.5 * groundTruthState.ForwardAcceleration() * dt * dt;

    Eigen::Vector2d wheelVelocities =
        m_inverseKinematicsMatrix *
        Eigen::Vector3d{groundTruthState.ForwardVelocity().value() *
                            groundTruthState.pose.Rotation().Cos(),
                        groundTruthState.ForwardVelocity().value() *
                            groundTruthState.pose.Rotation().Sin(),
                        0};

    auto xWheelVel = prosdk::units::meters_per_second_t{wheelVelocities(0, 0)} +
                     distribution(generator) * 0.05_mps;
    auto yWheelVel = prosdk::units::meters_per_second_t{wheelVelocities(1, 0)} +
                     distribution(generator) * 0.05_mps;

    xWheelPos += xWheelVel * dt;
    yWheelPos += yWheelVel * dt;

    auto lastPose = odometry.GetPose();

    auto xhat = odometry.Update(
        xWheelPos, yWheelPos,
        prosdk::math::Rotation2d{distribution(generator) * 0.001_rad});

    odometryDistanceTravelled +=
        lastPose.Translation().Distance(xhat.Translation());
    double error = groundTruthState.pose.Translation()
                       .Distance(xhat.Translation())
                       .value();

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  CHECK(errorSum / (trajectory.Duration().value() / dt.value()) < 0.35);
  CHECK(maxError < 0.35);
  CHECK_NEAR(trajectoryDistanceTravelled.value(),
             odometryDistanceTravelled.value(),
             trajectoryDistanceTravelled.value() * 0.05);
}
