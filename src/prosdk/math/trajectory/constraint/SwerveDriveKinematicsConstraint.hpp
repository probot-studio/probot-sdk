// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/SwerveDriveKinematics.hpp"
#include "prosdk/math/trajectory/constraint/TrajectoryConstraint.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/units/velocity.hpp"

namespace prosdk::math {

/**
 * A class that enforces constraints on the swerve drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities of the wheels stay below a certain limit.
 */
template <size_t NumModules>
class SwerveDriveKinematicsConstraint : public TrajectoryConstraint {
 public:
  SwerveDriveKinematicsConstraint(
      const prosdk::math::SwerveDriveKinematics<NumModules>& kinematics,
      prosdk::units::meters_per_second_t maxVelocity)
      : m_kinematics(kinematics), m_maxVelocity(maxVelocity) {}

  prosdk::units::meters_per_second_t MaxVelocity(
      const Pose2d& pose, prosdk::units::curvature_t curvature,
      prosdk::units::meters_per_second_t velocity) const override {
    auto xVelocity = velocity * pose.Rotation().Cos();
    auto yVelocity = velocity * pose.Rotation().Sin();
    auto wheelVelocities = m_kinematics.ToSwerveModuleVelocities(
        {xVelocity, yVelocity, velocity * curvature});

    auto normVelocities = m_kinematics.ToChassisVelocities(
        m_kinematics.DesaturateWheelVelocities(wheelVelocities, m_maxVelocity));

    return prosdk::units::math::hypot(normVelocities.vx, normVelocities.vy);
  }

  MinMax MinMaxAcceleration(
      const Pose2d& pose, prosdk::units::curvature_t curvature,
      prosdk::units::meters_per_second_t velocity) const override {
    return {};
  }

 private:
  prosdk::math::SwerveDriveKinematics<NumModules> m_kinematics;
  prosdk::units::meters_per_second_t m_maxVelocity;
};

}  // namespace prosdk::math
