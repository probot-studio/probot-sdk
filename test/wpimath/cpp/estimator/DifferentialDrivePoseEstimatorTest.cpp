// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/estimator/DifferentialDrivePoseEstimator.hpp"

#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineSample.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectory.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/print.hpp"

void testFollowTrajectory(
    const prosdk::math::DifferentialDriveKinematics& kinematics,
    prosdk::math::DifferentialDrivePoseEstimator& estimator,
    const prosdk::math::DrivetrainSplineTrajectory& trajectory,
    std::function<
        prosdk::math::ChassisVelocities(prosdk::math::DrivetrainSplineSample&)>
        chassisVelocitiesGenerator,
    std::function<prosdk::math::Pose2d(prosdk::math::DrivetrainSplineSample&)>
        visionMeasurementGenerator,
    const prosdk::math::Pose2d& startingPose, const prosdk::math::Pose2d& endingPose,
    const prosdk::units::second_t dt,
    const prosdk::units::second_t VISION_UPDATE_RATE,
    const prosdk::units::second_t VISION_UPDATE_DELAY, const bool checkError,
    const bool debug) {
  prosdk::units::meter_t leftDistance = 0_m;
  prosdk::units::meter_t rightDistance = 0_m;

  estimator.ResetPosition(prosdk::math::Rotation2d{}, leftDistance, rightDistance,
                          startingPose);

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);

  prosdk::units::second_t t = 0_s;

  std::vector<std::pair<prosdk::units::second_t, prosdk::math::Pose2d>> visionPoses;
  std::vector<
      std::tuple<prosdk::units::second_t, prosdk::units::second_t, prosdk::math::Pose2d>>
      visionLog;

  double maxError = -std::numeric_limits<double>::max();
  double errorSum = 0;

  if (debug) {
    prosdk::util::print(
        "time, est_x, est_y, est_theta, true_x, true_y, true_theta, left, "
        "right\n");
  }

  while (t < trajectory.Duration()) {
    prosdk::math::DrivetrainSplineSample groundTruthState = trajectory.SampleAt(t);

    // We are due for a new vision measurement if it's been `visionUpdateRate`
    // seconds since the last vision measurement
    if (visionPoses.empty() ||
        visionPoses.back().first + VISION_UPDATE_RATE < t) {
      auto visionPose =
          visionMeasurementGenerator(groundTruthState) +
          prosdk::math::Transform2d{
              prosdk::math::Translation2d{distribution(generator) * 0.1_m,
                                       distribution(generator) * 0.1_m},
              prosdk::math::Rotation2d{distribution(generator) * 0.05_rad}};
      visionPoses.push_back({t, visionPose});
    }

    // We should apply the oldest vision measurement if it has been
    // `visionUpdateDelay` seconds since it was measured
    if (!visionPoses.empty() &&
        visionPoses.front().first + VISION_UPDATE_DELAY < t) {
      auto visionEntry = visionPoses.front();
      estimator.AddVisionMeasurement(visionEntry.second, visionEntry.first);
      visionPoses.erase(visionPoses.begin());
      visionLog.push_back({t, visionEntry.first, visionEntry.second});
    }

    auto chassisVelocities = chassisVelocitiesGenerator(groundTruthState);

    auto wheelVelocities = kinematics.ToWheelVelocities(chassisVelocities);

    leftDistance += wheelVelocities.left * dt;
    rightDistance += wheelVelocities.right * dt;

    auto xhat = estimator.UpdateWithTime(
        t,
        groundTruthState.pose.Rotation() +
            prosdk::math::Rotation2d{distribution(generator) * 0.05_rad} -
            trajectory.Start().pose.Rotation(),
        leftDistance, rightDistance);

    if (debug) {
      prosdk::util::print(
          "{}, {}, {}, {}, {}, {}, {}, {}, {}\n", t.value(), xhat.X().value(),
          xhat.Y().value(), xhat.Rotation().Radians().value(),
          groundTruthState.pose.X().value(), groundTruthState.pose.Y().value(),
          groundTruthState.pose.Rotation().Radians().value(),
          leftDistance.value(), rightDistance.value());
    }

    double error = groundTruthState.pose.Translation()
                       .Distance(xhat.Translation())
                       .value();

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  if (debug) {
    prosdk::util::print(
        "apply_time, measured_time, vision_x, vision_y, vision_theta\n");

    prosdk::units::second_t apply_time;
    prosdk::units::second_t measure_time;
    prosdk::math::Pose2d vision_pose;
    for (auto record : visionLog) {
      std::tie(apply_time, measure_time, vision_pose) = record;
      prosdk::util::print("{}, {}, {}, {}, {}\n", apply_time.value(),
                       measure_time.value(), vision_pose.X().value(),
                       vision_pose.Y().value(),
                       vision_pose.Rotation().Radians().value());
    }
  }

  CHECK_NEAR(endingPose.X().value(),
             estimator.GetEstimatedPosition().X().value(), 0.08);
  CHECK_NEAR(endingPose.Y().value(),
             estimator.GetEstimatedPosition().Y().value(), 0.08);
  CHECK_NEAR(endingPose.Rotation().Radians().value(),
             estimator.GetEstimatedPosition().Rotation().Radians().value(),
             0.15);

  if (checkError) {
    // NOLINTNEXTLINE(bugprone-integer-division)
    CHECK(errorSum / (trajectory.Duration() / dt) < 0.05);
    CHECK(maxError < 0.2);
  }
}

TEST_CASE("DifferentialDrivePoseEstimatorTest Accuracy", "[wpimath]") {
  prosdk::math::DifferentialDriveKinematics kinematics{1.0_m};

  prosdk::math::DifferentialDrivePoseEstimator estimator{prosdk::math::Rotation2d{},
                                                      0_m,
                                                      0_m,
                                                      prosdk::math::Pose2d{},
                                                      {0.02, 0.02, 0.01},
                                                      {0.1, 0.1, 0.1}};

  prosdk::math::DrivetrainSplineTrajectory trajectory =
      prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
          std::vector{prosdk::math::Pose2d{0_m, 0_m, 45_deg},
                      prosdk::math::Pose2d{3_m, 0_m, -90_deg},
                      prosdk::math::Pose2d{0_m, 0_m, 135_deg},
                      prosdk::math::Pose2d{-3_m, 0_m, -90_deg},
                      prosdk::math::Pose2d{0_m, 0_m, 45_deg}},
          prosdk::math::TrajectoryConfig(2_mps, 2_mps_sq));

  testFollowTrajectory(
      kinematics, estimator, trajectory,
      [&](prosdk::math::DrivetrainSplineSample& state) {
        return prosdk::math::ChassisVelocities{
            state.ForwardVelocity(), 0_mps,
            state.ForwardVelocity() * state.curvature};
      },
      [&](prosdk::math::DrivetrainSplineSample& state) { return state.pose; },
      trajectory.Start().pose, {0_m, 0_m, prosdk::math::Rotation2d{45_deg}}, 20_ms,
      100_ms, 250_ms, true, false);
}

TEST_CASE("DifferentialDrivePoseEstimatorTest BadInitialPose", "[wpimath]") {
  prosdk::math::DifferentialDriveKinematics kinematics{1.0_m};

  prosdk::math::DifferentialDrivePoseEstimator estimator{prosdk::math::Rotation2d{},
                                                      0_m,
                                                      0_m,
                                                      prosdk::math::Pose2d{},
                                                      {0.02, 0.02, 0.01},
                                                      {0.1, 0.1, 0.1}};

  prosdk::math::DrivetrainSplineTrajectory trajectory =
      prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
          std::vector{prosdk::math::Pose2d{0_m, 0_m, 45_deg},
                      prosdk::math::Pose2d{3_m, 0_m, -90_deg},
                      prosdk::math::Pose2d{0_m, 0_m, 135_deg},
                      prosdk::math::Pose2d{-3_m, 0_m, -90_deg},
                      prosdk::math::Pose2d{0_m, 0_m, 45_deg}},
          prosdk::math::TrajectoryConfig(2_mps, 2_mps_sq));

  for (prosdk::units::degree_t offset_direction_degs = 0_deg;
       offset_direction_degs < 360_deg; offset_direction_degs += 45_deg) {
    for (prosdk::units::degree_t offset_heading_degs = 0_deg;
         offset_heading_degs < 360_deg; offset_heading_degs += 45_deg) {
      auto pose_offset = prosdk::math::Rotation2d{offset_direction_degs};
      auto heading_offset = prosdk::math::Rotation2d{offset_heading_degs};

      auto initial_pose = trajectory.Start().pose +
                          prosdk::math::Transform2d{
                              prosdk::math::Translation2d{pose_offset.Cos() * 1_m,
                                                       pose_offset.Sin() * 1_m},
                              heading_offset};

      testFollowTrajectory(
          kinematics, estimator, trajectory,
          [&](prosdk::math::DrivetrainSplineSample& state) {
            return state.velocity.ToRobotRelative(state.pose.Rotation());
          },
          [&](prosdk::math::DrivetrainSplineSample& state) { return state.pose; },
          initial_pose, {0_m, 0_m, prosdk::math::Rotation2d{45_deg}}, 20_ms,
          100_ms, 250_ms, false, false);
    }
  }
}

TEST_CASE("DifferentialDrivePoseEstimatorTest SimultaneousVisionMeasurements",
          "[wpimath]") {
  // This tests for multiple vision measurements applied at the same time.
  // The expected behavior is that all measurements affect the estimated pose.
  // The alternative result is that only one vision measurement affects the
  // outcome. If that were the case, after 1000 measurements, the estimated
  // pose would converge to that measurement.
  prosdk::math::DifferentialDrivePoseEstimator estimator{
      prosdk::math::Rotation2d{},
      0_m,
      0_m,
      prosdk::math::Pose2d{1_m, 2_m, prosdk::math::Rotation2d{270_deg}},
      {0.02, 0.02, 0.01},
      {0.1, 0.1, 0.1}};

  estimator.UpdateWithTime(0_s, prosdk::math::Rotation2d{}, 0_m, 0_m);

  for (int i = 0; i < 1000; i++) {
    estimator.AddVisionMeasurement(
        prosdk::math::Pose2d{0_m, 0_m, prosdk::math::Rotation2d{0_deg}}, 0_s);
    estimator.AddVisionMeasurement(
        prosdk::math::Pose2d{3_m, 1_m, prosdk::math::Rotation2d{90_deg}}, 0_s);
    estimator.AddVisionMeasurement(
        prosdk::math::Pose2d{2_m, 4_m, prosdk::math::Rotation2d{180_deg}}, 0_s);
  }

  {
    auto dx = prosdk::units::math::abs(estimator.GetEstimatedPosition().X() - 0_m);
    auto dy = prosdk::units::math::abs(estimator.GetEstimatedPosition().Y() - 0_m);
    auto dtheta = prosdk::units::math::abs(
        estimator.GetEstimatedPosition().Rotation().Radians() - 0_deg);

    CHECK((dx > 0.08_m || dy > 0.08_m || dtheta > 0.08_rad));
  }

  {
    auto dx = prosdk::units::math::abs(estimator.GetEstimatedPosition().X() - 3_m);
    auto dy = prosdk::units::math::abs(estimator.GetEstimatedPosition().Y() - 1_m);
    auto dtheta = prosdk::units::math::abs(
        estimator.GetEstimatedPosition().Rotation().Radians() - 90_deg);

    CHECK((dx > 0.08_m || dy > 0.08_m || dtheta > 0.08_rad));
  }

  {
    auto dx = prosdk::units::math::abs(estimator.GetEstimatedPosition().X() - 2_m);
    auto dy = prosdk::units::math::abs(estimator.GetEstimatedPosition().Y() - 4_m);
    auto dtheta = prosdk::units::math::abs(
        estimator.GetEstimatedPosition().Rotation().Radians() - 180_deg);

    CHECK((dx > 0.08_m || dy > 0.08_m || dtheta > 0.08_rad));
  }
}

TEST_CASE(
    "DifferentialDrivePoseEstimatorTest TestDiscardStaleVisionMeasurements",
    "[wpimath]") {
  prosdk::math::DifferentialDrivePoseEstimator estimator{
      prosdk::math::Rotation2d{}, 0_m, 0_m, prosdk::math::Pose2d{}, {0.1, 0.1, 0.1},
      {0.45, 0.45, 0.45}};

  // Add enough measurements to fill up the buffer
  for (auto time = 0_s; time < 4_s; time += 20_ms) {
    estimator.UpdateWithTime(time, prosdk::math::Rotation2d{}, 0_m, 0_m);
  }

  auto odometryPose = estimator.GetEstimatedPosition();

  // Apply a vision measurement from 3 seconds ago
  estimator.AddVisionMeasurement(
      prosdk::math::Pose2d{prosdk::math::Translation2d{10_m, 10_m},
                        prosdk::math::Rotation2d{0.1_rad}},
      1_s, {0.1, 0.1, 0.1});

  CHECK_NEAR(odometryPose.X().value(),
             estimator.GetEstimatedPosition().X().value(), 1e-6);
  CHECK_NEAR(odometryPose.Y().value(),
             estimator.GetEstimatedPosition().Y().value(), 1e-6);
  CHECK_NEAR(odometryPose.Rotation().Radians().value(),
             estimator.GetEstimatedPosition().Rotation().Radians().value(),
             1e-6);
}

TEST_CASE("DifferentialDrivePoseEstimatorTest TestSampleAt", "[wpimath]") {
  prosdk::math::DifferentialDrivePoseEstimator estimator{
      prosdk::math::Rotation2d{}, 0_m, 0_m, prosdk::math::Pose2d{}, {1.0, 1.0, 1.0},
      {1.0, 1.0, 1.0}};

  // Returns empty when null
  CHECK(std::nullopt == estimator.SampleAt(1_s));

  // Add odometry measurements, but don't fill up the buffer
  // Add a tiny tolerance for the upper bound because of floating point rounding
  // error
  for (double time = 1; time <= 2 + 1e-9; time += 0.02) {
    estimator.UpdateWithTime(prosdk::units::second_t{time},
                             prosdk::math::Rotation2d{}, prosdk::units::meter_t{time},
                             prosdk::units::meter_t{time});
  }

  // Sample at an added time
  CHECK(
      std::optional(prosdk::math::Pose2d{1.02_m, 0_m, prosdk::math::Rotation2d{}}) ==
      estimator.SampleAt(1.02_s));
  // Sample between updates (test interpolation)
  CHECK(
      std::optional(prosdk::math::Pose2d{1.01_m, 0_m, prosdk::math::Rotation2d{}}) ==
      estimator.SampleAt(1.01_s));
  // Sampling before the oldest value returns the oldest value
  CHECK(std::optional(prosdk::math::Pose2d{1_m, 0_m, prosdk::math::Rotation2d{}}) ==
        estimator.SampleAt(0.5_s));
  // Sampling after the newest value returns the newest value
  CHECK(std::optional(prosdk::math::Pose2d{2_m, 0_m, prosdk::math::Rotation2d{}}) ==
        estimator.SampleAt(2.5_s));

  // Add a vision measurement after the odometry measurements (while keeping all
  // of the old odometry measurements)
  estimator.AddVisionMeasurement(
      prosdk::math::Pose2d{2_m, 0_m, prosdk::math::Rotation2d{1_rad}}, 2.2_s);

  // Make sure nothing changed (except the newest value)
  CHECK(
      std::optional(prosdk::math::Pose2d{1.02_m, 0_m, prosdk::math::Rotation2d{}}) ==
      estimator.SampleAt(1.02_s));
  CHECK(
      std::optional(prosdk::math::Pose2d{1.01_m, 0_m, prosdk::math::Rotation2d{}}) ==
      estimator.SampleAt(1.01_s));
  CHECK(std::optional(prosdk::math::Pose2d{1_m, 0_m, prosdk::math::Rotation2d{}}) ==
        estimator.SampleAt(0.5_s));

  // Add a vision measurement before the odometry measurements that's still in
  // the buffer
  estimator.AddVisionMeasurement(
      prosdk::math::Pose2d{1_m, 0.2_m, prosdk::math::Rotation2d{}}, 0.9_s);

  // Everything should be the same except Y is 0.1 (halfway between 0 and 0.2)
  CHECK(std::optional(
            prosdk::math::Pose2d{1.02_m, 0.1_m, prosdk::math::Rotation2d{}}) ==
        estimator.SampleAt(1.02_s));
  CHECK(std::optional(
            prosdk::math::Pose2d{1.01_m, 0.1_m, prosdk::math::Rotation2d{}}) ==
        estimator.SampleAt(1.01_s));
  CHECK(std::optional(prosdk::math::Pose2d{1_m, 0.1_m, prosdk::math::Rotation2d{}}) ==
        estimator.SampleAt(0.5_s));
  CHECK(std::optional(prosdk::math::Pose2d{2_m, 0.1_m, prosdk::math::Rotation2d{}}) ==
        estimator.SampleAt(2.5_s));
}

TEST_CASE("DifferentialDrivePoseEstimatorTest TestReset", "[wpimath]") {
  prosdk::math::DifferentialDrivePoseEstimator estimator{
      prosdk::math::Rotation2d{},
      0_m,
      0_m,
      prosdk::math::Pose2d{-1_m, -1_m, prosdk::math::Rotation2d{1_rad}},
      {1.0, 1.0, 1.0},
      {1.0, 1.0, 1.0}};

  // Test initial pose
  CHECK_DOUBLE_EQ(-1, estimator.GetEstimatedPosition().X().value());
  CHECK_DOUBLE_EQ(-1, estimator.GetEstimatedPosition().Y().value());
  CHECK_DOUBLE_EQ(
      1, estimator.GetEstimatedPosition().Rotation().Radians().value());

  // Test reset position
  estimator.ResetPosition(prosdk::math::Rotation2d{}, 1_m, 1_m,
                          prosdk::math::Pose2d{1_m, 0_m, prosdk::math::Rotation2d{}});

  CHECK_DOUBLE_EQ(1, estimator.GetEstimatedPosition().X().value());
  CHECK_DOUBLE_EQ(0, estimator.GetEstimatedPosition().Y().value());
  CHECK_DOUBLE_EQ(
      0, estimator.GetEstimatedPosition().Rotation().Radians().value());

  // Test orientation and wheel positions
  estimator.Update(prosdk::math::Rotation2d{}, 2_m, 2_m);

  CHECK_DOUBLE_EQ(2, estimator.GetEstimatedPosition().X().value());
  CHECK_DOUBLE_EQ(0, estimator.GetEstimatedPosition().Y().value());
  CHECK_DOUBLE_EQ(
      0, estimator.GetEstimatedPosition().Rotation().Radians().value());

  // Test reset rotation
  estimator.ResetRotation(prosdk::math::Rotation2d{90_deg});

  CHECK_DOUBLE_EQ(2, estimator.GetEstimatedPosition().X().value());
  CHECK_DOUBLE_EQ(0, estimator.GetEstimatedPosition().Y().value());
  CHECK_DOUBLE_EQ(
      std::numbers::pi / 2,
      estimator.GetEstimatedPosition().Rotation().Radians().value());

  // Test orientation
  estimator.Update(prosdk::math::Rotation2d{}, 3_m, 3_m);

  CHECK_DOUBLE_EQ(2, estimator.GetEstimatedPosition().X().value());
  CHECK_DOUBLE_EQ(1, estimator.GetEstimatedPosition().Y().value());
  CHECK_DOUBLE_EQ(
      std::numbers::pi / 2,
      estimator.GetEstimatedPosition().Rotation().Radians().value());

  // Test reset translation
  estimator.ResetTranslation(prosdk::math::Translation2d{-1_m, -1_m});

  CHECK_DOUBLE_EQ(-1, estimator.GetEstimatedPosition().X().value());
  CHECK_DOUBLE_EQ(-1, estimator.GetEstimatedPosition().Y().value());
  CHECK_DOUBLE_EQ(
      std::numbers::pi / 2,
      estimator.GetEstimatedPosition().Rotation().Radians().value());

  // Test reset pose
  estimator.ResetPose(prosdk::math::Pose2d{});

  CHECK_DOUBLE_EQ(0, estimator.GetEstimatedPosition().X().value());
  CHECK_DOUBLE_EQ(0, estimator.GetEstimatedPosition().Y().value());
  CHECK_DOUBLE_EQ(
      0, estimator.GetEstimatedPosition().Rotation().Radians().value());
}
