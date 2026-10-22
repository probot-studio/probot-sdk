// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/LTVDifferentialDriveController.hpp"

#include <cmath>

#include <prosdk/thirdparty/eigen/Eigen/Core>

#include "prosdk/math/controller/DifferentialDriveWheelVoltages.hpp"
#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/linalg/DARE.hpp"
#include "prosdk/math/system/Discretization.hpp"
#include "prosdk/math/util/MathUtil.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"

using namespace prosdk::math;

DifferentialDriveWheelVoltages LTVDifferentialDriveController::Calculate(
    const Pose2d& currentPose, prosdk::units::meters_per_second_t leftVelocity,
    prosdk::units::meters_per_second_t rightVelocity, const Pose2d& poseRef,
    prosdk::units::meters_per_second_t leftVelocityRef,
    prosdk::units::meters_per_second_t rightVelocityRef) {
  // This implements the linear time-varying differential drive controller in
  // theorem 8.7.4 of https://controls-in-frc.link/
  //
  //     [x ]
  //     [y ]       [Vₗ]
  // x = [θ ]   u = [Vᵣ]
  //     [vₗ]
  //     [vᵣ]

  prosdk::units::meters_per_second_t velocity{(leftVelocity + rightVelocity) /
                                           2.0};

  // The DARE is ill-conditioned if the velocity is close to zero, so don't
  // let the system stop.
  if (prosdk::units::math::abs(velocity) < 1e-4_mps) {
    velocity = 1e-4_mps;
  }

  Eigen::Vector<double, 5> r{poseRef.X().value(), poseRef.Y().value(),
                             poseRef.Rotation().Radians().value(),
                             leftVelocityRef.value(), rightVelocityRef.value()};
  Eigen::Vector<double, 5> x{currentPose.X().value(), currentPose.Y().value(),
                             currentPose.Rotation().Radians().value(),
                             leftVelocity.value(), rightVelocity.value()};

  m_error = r - x;
  m_error(2) =
      prosdk::math::AngleModulus(prosdk::units::radian_t{m_error(2)}).value();

  Eigen::Matrix<double, 5, 5> A{
      {0.0, 0.0, 0.0, 0.5, 0.5},
      {0.0, 0.0, velocity.value(), 0.0, 0.0},
      {0.0, 0.0, 0.0, -1.0 / m_trackwidth.value(), 1.0 / m_trackwidth.value()},
      {0.0, 0.0, 0.0, m_A(0, 0), m_A(0, 1)},
      {0.0, 0.0, 0.0, m_A(1, 0), m_A(1, 1)}};
  Eigen::Matrix<double, 5, 2> B{{0.0, 0.0},
                                {0.0, 0.0},
                                {0.0, 0.0},
                                {m_B(0, 0), m_B(0, 1)},
                                {m_B(1, 0), m_B(1, 1)}};

  Eigen::Matrix<double, 5, 5> discA;
  Eigen::Matrix<double, 5, 2> discB;
  DiscretizeAB(A, B, m_dt, &discA, &discB);

  auto S = DARE<5, 2>(discA, discB, m_Q, m_R, false).value();

  // K = (BᵀSB + R)⁻¹BᵀSA
  Eigen::Matrix<double, 2, 5> K = (discB.transpose() * S * discB + m_R)
                                      .llt()
                                      .solve(discB.transpose() * S * discA);

  Eigen::Matrix<double, 5, 5> inRobotFrame{
      {std::cos(x(2)), std::sin(x(2)), 0.0, 0.0, 0.0},
      {-std::sin(x(2)), std::cos(x(2)), 0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 1.0}};

  Eigen::Vector2d u = K * inRobotFrame * m_error;

  return DifferentialDriveWheelVoltages{prosdk::units::volt_t{u(0)},
                                        prosdk::units::volt_t{u(1)}};
}
