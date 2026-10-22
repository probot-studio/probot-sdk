// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/trajectory/constraint/TrajectoryConstraint.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/curvature.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

/**
 * A constraint on the maximum absolute centripetal acceleration allowed when
 * traversing a trajectory. The centripetal acceleration of a robot is defined
 * as the velocity squared divided by the radius of curvature.
 *
 * Effectively, limiting the maximum centripetal acceleration will cause the
 * robot to slow down around tight turns, making it easier to track trajectories
 * with sharp turns.
 */
class WPILIB_DLLEXPORT CentripetalAccelerationConstraint
    : public TrajectoryConstraint {
 public:
  constexpr explicit CentripetalAccelerationConstraint(
      prosdk::units::meters_per_second_squared_t maxCentripetalAcceleration)
      : m_maxCentripetalAcceleration(maxCentripetalAcceleration) {}

  constexpr prosdk::units::meters_per_second_t MaxVelocity(
      const Pose2d& pose, prosdk::units::curvature_t curvature,
      prosdk::units::meters_per_second_t velocity) const override {
    // ac = v²/r
    // k (curvature) = 1/r

    // therefore, ac = v²k
    // ac/k = v²
    // v = √(ac/k)

    // We have to multiply by 1_rad here to get the units to cancel out nicely.
    // The units library defines a unit for radians although it is technically
    // unitless.
    return prosdk::units::math::sqrt(m_maxCentripetalAcceleration /
                                  prosdk::units::math::abs(curvature) * 1_rad);
  }

  constexpr MinMax MinMaxAcceleration(
      const Pose2d& pose, prosdk::units::curvature_t curvature,
      prosdk::units::meters_per_second_t velocity) const override {
    // The acceleration of the robot has no impact on the centripetal
    // acceleration of the robot.
    return {};
  }

 private:
  prosdk::units::meters_per_second_squared_t m_maxCentripetalAcceleration;
};
}  // namespace prosdk::math
