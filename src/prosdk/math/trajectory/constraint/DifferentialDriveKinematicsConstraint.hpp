// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/math/trajectory/constraint/TrajectoryConstraint.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {
/**
 * A class that enforces constraints on the differential drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities for both sides of the drivetrain stay below a certain
 * limit.
 */
class WPILIB_DLLEXPORT DifferentialDriveKinematicsConstraint
    : public TrajectoryConstraint {
 public:
  constexpr DifferentialDriveKinematicsConstraint(
      DifferentialDriveKinematics kinematics,
      prosdk::units::meters_per_second_t maxVelocity)
      : m_kinematics(std::move(kinematics)), m_maxVelocity(maxVelocity) {}

  constexpr prosdk::units::meters_per_second_t MaxVelocity(
      const Pose2d& pose, prosdk::units::curvature_t curvature,
      prosdk::units::meters_per_second_t velocity) const override {
    auto wheelVelocities =
        m_kinematics.ToWheelVelocities({velocity, 0_mps, velocity * curvature})
            .Desaturate(m_maxVelocity);

    return m_kinematics.ToChassisVelocities(wheelVelocities).vx;
  }

  constexpr MinMax MinMaxAcceleration(
      const Pose2d& pose, prosdk::units::curvature_t curvature,
      prosdk::units::meters_per_second_t velocity) const override {
    return {};
  }

 private:
  DifferentialDriveKinematics m_kinematics;
  prosdk::units::meters_per_second_t m_maxVelocity;
};
}  // namespace prosdk::math
