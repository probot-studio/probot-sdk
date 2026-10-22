// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/estimator/KalmanFilter.hpp"

#include <prosdk/thirdparty/eigen/Eigen/Core>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/random/Normal.hpp"
#include "prosdk/math/system/LinearSystem.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/units/time.hpp"

TEST_CASE("KalmanFilterTest SwerveStationary", "[wpimath]") {
  constexpr prosdk::units::second_t dt = 20_ms;

  // Swerve drive with x = [x, y, θ, v_x, v_y, ω]ᵀ, u = [a_x, a_y, α]ᵀ,
  // y = [x, y, θ]ᵀ
  prosdk::math::LinearSystem<6, 3, 3> plant{
      Eigen::Matrix<double, 6, 6>{{0, 0, 0, 1, 0, 0},
                                  {0, 0, 0, 0, 1, 0},
                                  {0, 0, 0, 0, 0, 1},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0}},
      Eigen::Matrix<double, 6, 3>{
          {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      Eigen::Matrix<double, 3, 6>{
          {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}},
      Eigen::Matrix<double, 3, 3>::Zero()};

  prosdk::math::KalmanFilter<6, 3, 3> filter{
      plant, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, {1.0, 1.0, 1.0}, dt};

  constexpr Eigen::Vector<double, 3> u{{0.0}, {0.0}, {0.0}};

  for (int i = 0; i < 100; ++i) {
    auto y = prosdk::math::Normal(1.0, 1.0, 1.0);

    filter.Correct(u, y);
    filter.Predict(u, dt);
  }

  CHECK_NEAR(0.0, filter.Xhat(0), 0.3);
  CHECK_NEAR(0.0, filter.Xhat(1), 0.3);
}

TEST_CASE("KalmanFilterTest SwerveBadInitialPose", "[wpimath]") {
  constexpr prosdk::units::second_t dt = 20_ms;

  // Swerve drive with x = [x, y, θ, v_x, v_y, ω]ᵀ, u = [a_x, a_y, α]ᵀ,
  // y = [x, y, θ]ᵀ
  prosdk::math::LinearSystem<6, 3, 3> plant{
      Eigen::Matrix<double, 6, 6>{{0, 0, 0, 1, 0, 0},
                                  {0, 0, 0, 0, 1, 0},
                                  {0, 0, 0, 0, 0, 1},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0}},
      Eigen::Matrix<double, 6, 3>{
          {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      Eigen::Matrix<double, 3, 6>{
          {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}},
      Eigen::Matrix<double, 3, 3>::Zero()};

  prosdk::math::KalmanFilter<6, 3, 3> filter{
      plant, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, {0.1, 0.1, 0.25}, dt};

  // Set nonzero position
  filter.SetXhat(0, 0.5);
  filter.SetXhat(1, 0.5);

  constexpr Eigen::Vector<double, 3> u{{0.0}, {0.0}, {0.0}};

  // Let filter converge to zero position
  for (int i = 0; i < 300; ++i) {
    auto y = prosdk::math::Normal(0.1, 0.1, 0.25);

    filter.Correct(u, y);
    filter.Predict(u, dt);
  }

  CHECK_NEAR(0.0, filter.Xhat(0), 0.2);
  CHECK_NEAR(0.0, filter.Xhat(1), 0.2);
}

TEST_CASE("KalmanFilterTest SwerveMovingOverTrajectory", "[wpimath]") {
  constexpr prosdk::units::second_t dt = 20_ms;

  // Swerve drive with x = [x, y, θ, v_x, v_y, ω]ᵀ, u = [a_x, a_y, α]ᵀ,
  // y = [x, y, θ]ᵀ
  prosdk::math::LinearSystem<6, 3, 3> plant{
      Eigen::Matrix<double, 6, 6>{{0, 0, 0, 1, 0, 0},
                                  {0, 0, 0, 0, 1, 0},
                                  {0, 0, 0, 0, 0, 1},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0}},
      Eigen::Matrix<double, 6, 3>{
          {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      Eigen::Matrix<double, 3, 6>{
          {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}},
      Eigen::Matrix<double, 3, 3>::Zero()};

  prosdk::math::KalmanFilter<6, 3, 3> filter{
      plant, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, {0.2, 0.2, 1.0 / 3.0}, dt};

  auto trajectory = prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
      {prosdk::math::Pose2d{0_m, 0_m, 0_rad}, prosdk::math::Pose2d{5_m, 5_m, 0_rad}},
      prosdk::math::TrajectoryConfig{2_mps, 2_mps_sq});

  Eigen::Vector3d lastVelocity{0.0, 0.0, 0.0};

  for (prosdk::units::second_t t = 0_s; t < trajectory.Duration(); t += dt) {
    auto sample = trajectory.SampleAt(t);

    Eigen::Vector3d y{sample.pose.Translation().X().value(),
                      sample.pose.Translation().Y().value(),
                      sample.pose.Rotation().Radians().value()};
    y += prosdk::math::Normal(0.2, 0.2, 1.0 / 3.0);

    // The velocity is field-relative; the input is its time derivative.
    Eigen::Vector3d velocity{sample.velocity.vx.value(),
                             sample.velocity.vy.value(),
                             sample.velocity.omega.value()};
    Eigen::Vector3d u = (velocity - lastVelocity) / dt.value();

    filter.Correct(u, y);
    filter.Predict(u, dt);

    lastVelocity = velocity;
  }

  CHECK_NEAR(
      trajectory.SampleAt(trajectory.Duration()).pose.Translation().X().value(),
      filter.Xhat(0), 0.2);
  CHECK_NEAR(
      trajectory.SampleAt(trajectory.Duration()).pose.Translation().Y().value(),
      filter.Xhat(1), 0.2);
}
