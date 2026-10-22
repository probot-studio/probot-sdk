// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/estimator/MerweUKF.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

#include <prosdk/thirdparty/eigen/Eigen/QR>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/estimator/AngleStatistics.hpp"
#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/math/random/Normal.hpp"
#include "prosdk/math/system/DCMotor.hpp"
#include "prosdk/math/system/Discretization.hpp"
#include "prosdk/math/system/Models.hpp"
#include "prosdk/math/system/NumericalIntegration.hpp"
#include "prosdk/math/system/NumericalJacobian.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineSample.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectory.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/math/util/StateSpaceUtil.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/base.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/mass.hpp"
#include "prosdk/units/moment_of_inertia.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"
#include "prosdk/util/array.hpp"

namespace {

// First test system, differential drive
prosdk::math::Vectord<5> DriveDynamics(const prosdk::math::Vectord<5>& x,
                                    const prosdk::math::Vectord<2>& u) {
  auto motors = prosdk::math::DCMotor::CIM(2);

  // constexpr double Glow = 15.32;    // Low gear ratio
  constexpr double Ghigh = 7.08;       // High gear ratio
  constexpr auto rb = 0.8382_m / 2.0;  // Robot radius
  constexpr auto r = 0.0746125_m;      // Wheel radius
  constexpr auto m = 63.503_kg;        // Robot mass
  constexpr auto J = 5.6_kg_sq_m;      // Robot moment of inertia

  auto C1 = -std::pow(Ghigh, 2) * motors.Kt /
            (motors.Kv * motors.R * prosdk::units::math::pow<2>(r));
  auto C2 = Ghigh * motors.Kt / (motors.R * r);
  auto k1 = (1 / m + prosdk::units::math::pow<2>(rb) / J);
  auto k2 = (1 / m - prosdk::units::math::pow<2>(rb) / J);

  prosdk::units::meters_per_second_t vl{x(3)};
  prosdk::units::meters_per_second_t vr{x(4)};
  prosdk::units::volt_t Vl{u(0)};
  prosdk::units::volt_t Vr{u(1)};

  auto v = 0.5 * (vl + vr);
  return prosdk::math::Vectord<5>{
      v.value() * std::cos(x(2)), v.value() * std::sin(x(2)),
      ((vr - vl) / (2.0 * rb)).value(),
      k1.value() * ((C1 * vl).value() + (C2 * Vl).value()) +
          k2.value() * ((C1 * vr).value() + (C2 * Vr).value()),
      k2.value() * ((C1 * vl).value() + (C2 * Vl).value()) +
          k1.value() * ((C1 * vr).value() + (C2 * Vr).value())};
}

prosdk::math::Vectord<3> DriveLocalMeasurementModel(
    const prosdk::math::Vectord<5>& x,
    [[maybe_unused]] const prosdk::math::Vectord<2>& u) {
  return prosdk::math::Vectord<3>{x(2), x(3), x(4)};
}

prosdk::math::Vectord<5> DriveGlobalMeasurementModel(
    const prosdk::math::Vectord<5>& x,
    [[maybe_unused]] const prosdk::math::Vectord<2>& u) {
  return prosdk::math::Vectord<5>{x(0), x(1), x(2), x(3), x(4)};
}

TEST_CASE("MerweUKFTest DriveInit", "[wpimath]") {
  constexpr prosdk::units::second_t dt = 5_ms;

  prosdk::math::MerweUKF<5, 2, 3> observer{DriveDynamics,
                                        DriveLocalMeasurementModel,
                                        {0.5, 0.5, 10.0, 1.0, 1.0},
                                        {0.0001, 0.01, 0.01},
                                        prosdk::math::AngleMean<5, 2 * 5 + 1>(2),
                                        prosdk::math::AngleMean<3, 2 * 5 + 1>(0),
                                        prosdk::math::AngleResidual<5>(2),
                                        prosdk::math::AngleResidual<3>(0),
                                        prosdk::math::AngleAdd<5>(2),
                                        dt};
  prosdk::math::Vectord<2> u{12.0, 12.0};
  observer.Predict(u, dt);

  auto localY = DriveLocalMeasurementModel(observer.Xhat(), u);
  observer.Correct(u, localY);

  auto globalY = DriveGlobalMeasurementModel(observer.Xhat(), u);
  auto R = prosdk::math::CovarianceMatrix(0.01, 0.01, 0.0001, 0.01, 0.01);
  observer.Correct<5>(
      u, globalY, DriveGlobalMeasurementModel, R,
      prosdk::math::AngleMean<5, 2 * 5 + 1>(2), prosdk::math::AngleResidual<5>(2),
      prosdk::math::AngleResidual<5>(2), prosdk::math::AngleAdd<5>(2));
}

TEST_CASE("MerweUKFTest DriveConvergence", "[wpimath]") {
  constexpr prosdk::units::second_t dt = 5_ms;
  constexpr auto rb = 0.8382_m / 2.0;  // Robot radius

  prosdk::math::MerweUKF<5, 2, 3> observer{DriveDynamics,
                                        DriveLocalMeasurementModel,
                                        {0.5, 0.5, 10.0, 1.0, 1.0},
                                        {0.0001, 0.5, 0.5},
                                        prosdk::math::AngleMean<5, 2 * 5 + 1>(2),
                                        prosdk::math::AngleMean<3, 2 * 5 + 1>(0),
                                        prosdk::math::AngleResidual<5>(2),
                                        prosdk::math::AngleResidual<3>(0),
                                        prosdk::math::AngleAdd<5>(2),
                                        dt};

  auto waypoints = std::vector<prosdk::math::Pose2d>{
      prosdk::math::Pose2d{2.75_m, 22.521_m, 0_rad},
      prosdk::math::Pose2d{24.73_m, 19.68_m, 5.846_rad}};
  auto trajectory = prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
      waypoints, {8.8_mps, 0.1_mps_sq});

  prosdk::math::Vectord<5> r = prosdk::math::Vectord<5>::Zero();
  prosdk::math::Vectord<2> u = prosdk::math::Vectord<2>::Zero();

  auto B = prosdk::math::NumericalJacobianU<5, 5, 2>(
      DriveDynamics, prosdk::math::Vectord<5>::Zero(),
      prosdk::math::Vectord<2>::Zero());

  observer.SetXhat(prosdk::math::Vectord<5>{
      trajectory.Start().pose.Translation().X().value(),
      trajectory.Start().pose.Translation().Y().value(),
      trajectory.Start().pose.Rotation().Radians().value(), 0.0, 0.0});

  auto trueXhat = observer.Xhat();

  auto duration = trajectory.Duration();
  for (size_t i = 0; i < (duration / dt).value(); ++i) {
    auto ref = trajectory.SampleAt(dt * i);
    prosdk::units::meters_per_second_t vl =
        ref.ForwardVelocity() * (1 - (ref.curvature * rb).value());
    prosdk::units::meters_per_second_t vr =
        ref.ForwardVelocity() * (1 + (ref.curvature * rb).value());

    prosdk::math::Vectord<5> nextR{
        ref.pose.Translation().X().value(), ref.pose.Translation().Y().value(),
        ref.pose.Rotation().Radians().value(), vl.value(), vr.value()};

    auto localY =
        DriveLocalMeasurementModel(trueXhat, prosdk::math::Vectord<2>::Zero());
    observer.Correct(u, localY + prosdk::math::Normal(0.0001, 0.5, 0.5));

    prosdk::math::Vectord<5> rdot = (nextR - r) / dt.value();
    u = B.householderQr().solve(
        rdot - DriveDynamics(r, prosdk::math::Vectord<2>::Zero()));

    observer.Predict(u, dt);

    r = nextR;
    trueXhat = prosdk::math::RK4(DriveDynamics, trueXhat, u, dt);
  }

  auto localY = DriveLocalMeasurementModel(trueXhat, u);
  observer.Correct(u, localY);

  auto globalY = DriveGlobalMeasurementModel(trueXhat, u);
  auto R = prosdk::math::CovarianceMatrix(0.01, 0.01, 0.0001, 0.5, 0.5);
  observer.Correct<5>(u, globalY, DriveGlobalMeasurementModel, R,
                      prosdk::math::AngleMean<5, 2 * 5 + 1>(2),
                      prosdk::math::AngleResidual<5>(2),
                      prosdk::math::AngleResidual<5>(2), prosdk::math::AngleAdd<5>(2)

  );

  auto finalPosition = trajectory.SampleAt(trajectory.Duration());
  CHECK_NEAR(finalPosition.pose.Translation().X().value(), observer.Xhat(0),
             0.055);
  CHECK_NEAR(finalPosition.pose.Translation().Y().value(), observer.Xhat(1),
             0.15);
  CHECK_NEAR(finalPosition.pose.Rotation().Radians().value(), observer.Xhat(2),
             0.00005);
  CHECK_NEAR(0.0, observer.Xhat(3), 0.1);
  CHECK_NEAR(0.0, observer.Xhat(4), 0.1);
}

TEST_CASE("MerweUKFTest LinearUKF", "[wpimath]") {
  constexpr prosdk::units::second_t dt = 20_ms;
  auto plant = prosdk::math::Models::FlywheelFromSysId(0.02_V / 1_rad_per_s,
                                                    0.006_V / 1_rad_per_s_sq);
  prosdk::math::MerweUKF<1, 1, 1> observer{
      [&](const prosdk::math::Vectord<1>& x, const prosdk::math::Vectord<1>& u) {
        return plant.A() * x + plant.B() * u;
      },
      [&](const prosdk::math::Vectord<1>& x, const prosdk::math::Vectord<1>& u) {
        return plant.CalculateY(x, u);
      },
      {0.05},
      {1.0},
      dt};

  prosdk::math::Matrixd<1, 1> discA;
  prosdk::math::Matrixd<1, 1> discB;
  prosdk::math::DiscretizeAB<1, 1>(plant.A(), plant.B(), dt, &discA, &discB);

  prosdk::math::Vectord<1> ref{100.0};
  prosdk::math::Vectord<1> u{0.0};

  for (int i = 0; i < 2.0 / dt.value(); ++i) {
    observer.Predict(u, dt);

    u = discB.householderQr().solve(ref - discA * ref);
  }

  CHECK_NEAR(ref(0, 0), observer.Xhat(0), 5);
}

TEST_CASE("MerweUKFTest RoundTripP", "[wpimath]") {
  constexpr prosdk::units::second_t dt = 5_ms;

  prosdk::math::MerweUKF<2, 2, 2> observer{
      [](const prosdk::math::Vectord<2>& x, const prosdk::math::Vectord<2>& u) {
        return x;
      },
      [](const prosdk::math::Vectord<2>& x, const prosdk::math::Vectord<2>& u) {
        return x;
      },
      {0.0, 0.0},
      {0.0, 0.0},
      dt};

  prosdk::math::Matrixd<2, 2> P({{2, 1}, {1, 2}});
  observer.SetP(P);

  REQUIRE(observer.P().isApprox(P));
}

// Second system, single motor feedforward estimator
prosdk::math::Vectord<4> MotorDynamics(const prosdk::math::Vectord<4>& x,
                                    const prosdk::math::Vectord<1>& u) {
  double v = x(1);
  double kV = x(2);
  double kA = x(3);

  double V = u(0);

  double a = -kV / kA * v + 1.0 / kA * V;
  return prosdk::math::Vectord<4>{v, a, 0.0, 0.0};
}

prosdk::math::Vectord<3> MotorMeasurementModel(const prosdk::math::Vectord<4>& x,
                                            const prosdk::math::Vectord<1>& u) {
  double p = x(0);
  double v = x(1);
  double kV = x(2);
  double kA = x(3);

  double V = u(0);

  double a = -kV / kA * v + 1.0 / kA * V;
  return prosdk::math::Vectord<3>{p, v, a};
}

prosdk::math::Vectord<1> MotorControlInput(double t) {
  return prosdk::math::Vectord<1>{
      std::clamp(8 * std::sin(std::numbers::pi * std::sqrt(2.0) * t) +
                     6 * std::sin(std::numbers::pi * std::sqrt(3.0) * t) +
                     4 * std::sin(std::numbers::pi * std::sqrt(5.0) * t),
                 -12.0, 12.0)};
}

TEST_CASE("MerweUKFTest MotorConvergence", "[wpimath]") {
  constexpr prosdk::units::second_t dt = 10_ms;
  constexpr int steps = 500;
  constexpr double true_kV = 3;
  constexpr double true_kA = 0.2;

  constexpr double pos_stddev = 0.02;
  constexpr double vel_stddev = 0.1;
  constexpr double accel_stddev = 0.1;

  std::vector<prosdk::math::Vectord<4>> states(steps + 1);
  std::vector<prosdk::math::Vectord<1>> inputs(steps);
  std::vector<prosdk::math::Vectord<3>> measurements(steps);
  states[0] = prosdk::math::Vectord<4>{{0.0}, {0.0}, {true_kV}, {true_kA}};

  constexpr prosdk::math::Matrixd<4, 4> A{{0.0, 1.0, 0.0, 0.0},
                                       {0.0, -true_kV / true_kA, 0.0, 0.0},
                                       {0.0, 0.0, 0.0, 0.0},
                                       {0.0, 0.0, 0.0, 0.0}};
  constexpr prosdk::math::Matrixd<4, 1> B{{0.0}, {1.0 / true_kA}, {0.0}, {0.0}};

  prosdk::math::Matrixd<4, 4> discA;
  prosdk::math::Matrixd<4, 1> discB;
  prosdk::math::DiscretizeAB(A, B, dt, &discA, &discB);

  for (int i = 0; i < steps; ++i) {
    inputs[i] = MotorControlInput(i * dt.value());
    states[i + 1] = discA * states[i] + discB * inputs[i];
    measurements[i] = MotorMeasurementModel(states[i + 1], inputs[i]) +
                      prosdk::math::Normal(pos_stddev, vel_stddev, accel_stddev);
  }

  prosdk::math::Vectord<4> P0{0.001, 0.001, 10, 10};

  prosdk::math::MerweUKF<4, 1, 3> observer{
      MotorDynamics, MotorMeasurementModel,
      prosdk::util::array{0.1, 1.0, 1e-10, 1e-10},
      prosdk::util::array{pos_stddev, vel_stddev, accel_stddev}, dt};

  observer.SetXhat(prosdk::math::Vectord<4>{0.0, 0.0, 2.0, 2.0});
  observer.SetP(P0.asDiagonal());

  for (int i = 0; i < steps; ++i) {
    observer.Predict(inputs[i], dt);
    observer.Correct(inputs[i], measurements[i]);
  }

  CHECK_NEAR(true_kV, observer.Xhat(2), true_kV * 0.5);
  CHECK_NEAR(true_kA, observer.Xhat(3), true_kA * 0.5);
}

}  // namespace
