// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/MecanumDriveOdometry.hpp"

#include <limits>
#include <random>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/kinematics/MecanumDriveKinematics.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelPositions.hpp"
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

class MecanumDriveOdometryTest {
 protected:
  Translation2d m_fl{12_m, 12_m};
  Translation2d m_fr{12_m, -12_m};
  Translation2d m_bl{-12_m, 12_m};
  Translation2d m_br{-12_m, -12_m};

  MecanumDriveWheelPositions zero;

  MecanumDriveKinematics kinematics{m_fl, m_fr, m_bl, m_br};
  MecanumDriveOdometry odometry{kinematics, 0_rad, zero};
};

TEST_CASE_METHOD(MecanumDriveOdometryTest,
                 "MecanumDriveOdometryTest MultipleConsecutiveUpdates",
                 "[wpimath]") {
  MecanumDriveWheelPositions wheelDeltas{3.536_m, 3.536_m, 3.536_m, 3.536_m};

  odometry.ResetPosition(0_rad, wheelDeltas, Pose2d{});

  odometry.Update(0_deg, wheelDeltas);
  auto secondPose = odometry.Update(0_deg, wheelDeltas);

  CHECK_NEAR(secondPose.X().value(), 0.0, 0.01);
  CHECK_NEAR(secondPose.Y().value(), 0.0, 0.01);
  CHECK_NEAR(secondPose.Rotation().Radians().value(), 0.0, 0.01);
}

TEST_CASE_METHOD(MecanumDriveOdometryTest,
                 "MecanumDriveOdometryTest TwoIterations", "[wpimath]") {
  odometry.ResetPosition(0_rad, zero, Pose2d{});
  MecanumDriveWheelPositions wheelDeltas{0.3536_m, 0.3536_m, 0.3536_m,
                                         0.3536_m};

  odometry.Update(0_deg, MecanumDriveWheelPositions{});
  auto pose = odometry.Update(0_deg, wheelDeltas);

  CHECK_NEAR(pose.X().value(), 0.3536, 0.01);
  CHECK_NEAR(pose.Y().value(), 0.0, 0.01);
  CHECK_NEAR(pose.Rotation().Radians().value(), 0.0, 0.01);
}

TEST_CASE_METHOD(MecanumDriveOdometryTest,
                 "MecanumDriveOdometryTest 90DegreeTurn", "[wpimath]") {
  odometry.ResetPosition(0_rad, zero, Pose2d{});
  MecanumDriveWheelPositions wheelDeltas{-13.328_m, 39.986_m, -13.329_m,
                                         39.986_m};
  odometry.Update(0_deg, MecanumDriveWheelPositions{});
  auto pose = odometry.Update(90_deg, wheelDeltas);

  CHECK_NEAR(pose.X().value(), 8.4855, 0.01);
  CHECK_NEAR(pose.Y().value(), 8.4855, 0.01);
  CHECK_NEAR(pose.Rotation().Degrees().value(), 90.0, 0.01);
}

TEST_CASE_METHOD(MecanumDriveOdometryTest,
                 "MecanumDriveOdometryTest GyroAngleReset", "[wpimath]") {
  odometry.ResetPosition(90_deg, zero, Pose2d{});

  MecanumDriveWheelPositions wheelDeltas{0.3536_m, 0.3536_m, 0.3536_m,
                                         0.3536_m};

  odometry.Update(90_deg, MecanumDriveWheelPositions{});
  auto pose = odometry.Update(90_deg, wheelDeltas);

  CHECK_NEAR(pose.X().value(), 0.3536, 0.01);
  CHECK_NEAR(pose.Y().value(), 0.0, 0.01);
  CHECK_NEAR(pose.Rotation().Radians().value(), 0.0, 0.01);
}

TEST_CASE_METHOD(MecanumDriveOdometryTest,
                 "MecanumDriveOdometryTest AccuracyFacingTrajectory",
                 "[wpimath]") {
  prosdk::math::MecanumDriveKinematics kinematics{
      prosdk::math::Translation2d{1_m, 1_m}, prosdk::math::Translation2d{1_m, -1_m},
      prosdk::math::Translation2d{-1_m, -1_m},
      prosdk::math::Translation2d{-1_m, 1_m}};

  prosdk::math::MecanumDriveWheelPositions wheelPositions;

  prosdk::math::MecanumDriveOdometry odometry{kinematics, prosdk::math::Rotation2d{},
                                           wheelPositions};

  prosdk::math::DrivetrainSplineTrajectory trajectory =
      prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
          std::vector{prosdk::math::Pose2d{0_m, 0_m, 45_deg},
                      prosdk::math::Pose2d{3_m, 0_m, -90_deg},
                      prosdk::math::Pose2d{0_m, 0_m, 135_deg},
                      prosdk::math::Pose2d{-3_m, 0_m, -90_deg},
                      prosdk::math::Pose2d{0_m, 0_m, 45_deg}},
          prosdk::math::TrajectoryConfig(5.0_mps, 2.0_mps_sq));

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);

  prosdk::units::second_t dt = 20_ms;
  prosdk::units::second_t t = 0_s;

  double maxError = -std::numeric_limits<double>::max();
  double errorSum = 0;

  while (t < trajectory.Duration()) {
    prosdk::math::DrivetrainSplineSample groundTruthState = trajectory.SampleAt(t);

    auto wheelVelocities =
        kinematics.ToWheelVelocities(groundTruthState.velocity.ToRobotRelative(
            groundTruthState.pose.Rotation()));

    wheelVelocities.frontLeft += distribution(generator) * 0.1_mps;
    wheelVelocities.frontRight += distribution(generator) * 0.1_mps;
    wheelVelocities.rearLeft += distribution(generator) * 0.1_mps;
    wheelVelocities.rearRight += distribution(generator) * 0.1_mps;

    wheelPositions.frontLeft += wheelVelocities.frontLeft * dt;
    wheelPositions.frontRight += wheelVelocities.frontRight * dt;
    wheelPositions.rearLeft += wheelVelocities.rearLeft * dt;
    wheelPositions.rearRight += wheelVelocities.rearRight * dt;

    auto xhat = odometry.Update(
        groundTruthState.pose.Rotation() +
            prosdk::math::Rotation2d{distribution(generator) * 0.05_rad},
        wheelPositions);
    double error = groundTruthState.pose.Translation()
                       .Distance(xhat.Translation())
                       .value();

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  CHECK(errorSum / (trajectory.Duration().value() / dt.value()) < 0.06);
  CHECK(maxError < 0.125);
}

TEST_CASE_METHOD(MecanumDriveOdometryTest,
                 "MecanumDriveOdometryTest AccuracyFacingXAxis", "[wpimath]") {
  prosdk::math::MecanumDriveKinematics kinematics{
      prosdk::math::Translation2d{1_m, 1_m}, prosdk::math::Translation2d{1_m, -1_m},
      prosdk::math::Translation2d{-1_m, -1_m},
      prosdk::math::Translation2d{-1_m, 1_m}};

  prosdk::math::MecanumDriveWheelPositions wheelPositions;

  prosdk::math::MecanumDriveOdometry odometry{kinematics, prosdk::math::Rotation2d{},
                                           wheelPositions};

  prosdk::math::DrivetrainSplineTrajectory trajectory =
      prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
          std::vector{prosdk::math::Pose2d{0_m, 0_m, 45_deg},
                      prosdk::math::Pose2d{3_m, 0_m, -90_deg},
                      prosdk::math::Pose2d{0_m, 0_m, 135_deg},
                      prosdk::math::Pose2d{-3_m, 0_m, -90_deg},
                      prosdk::math::Pose2d{0_m, 0_m, 45_deg}},
          prosdk::math::TrajectoryConfig(5.0_mps, 2.0_mps_sq));

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);

  prosdk::units::second_t dt = 20_ms;
  prosdk::units::second_t t = 0_s;

  double maxError = -std::numeric_limits<double>::max();
  double errorSum = 0;

  while (t < trajectory.Duration()) {
    prosdk::math::DrivetrainSplineSample groundTruthState = trajectory.SampleAt(t);

    auto wheelVelocities = kinematics.ToWheelVelocities(
        {groundTruthState.ForwardVelocity() *
             groundTruthState.pose.Rotation().Cos(),
         groundTruthState.ForwardVelocity() *
             groundTruthState.pose.Rotation().Sin(),
         0_rad_per_s});

    wheelVelocities.frontLeft += distribution(generator) * 0.1_mps;
    wheelVelocities.frontRight += distribution(generator) * 0.1_mps;
    wheelVelocities.rearLeft += distribution(generator) * 0.1_mps;
    wheelVelocities.rearRight += distribution(generator) * 0.1_mps;

    wheelPositions.frontLeft += wheelVelocities.frontLeft * dt;
    wheelPositions.frontRight += wheelVelocities.frontRight * dt;
    wheelPositions.rearLeft += wheelVelocities.rearLeft * dt;
    wheelPositions.rearRight += wheelVelocities.rearRight * dt;

    auto xhat = odometry.Update(
        prosdk::math::Rotation2d{distribution(generator) * 0.05_rad},
        wheelPositions);
    double error = groundTruthState.pose.Translation()
                       .Distance(xhat.Translation())
                       .value();

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  CHECK(errorSum / (trajectory.Duration().value() / dt.value()) < 0.06);
  CHECK(maxError < 0.125);
}
