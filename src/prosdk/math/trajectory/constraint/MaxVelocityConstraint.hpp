// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/trajectory/constraint/TrajectoryConstraint.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

/**
 * Represents a constraint that enforces a max velocity. This can be composed
 * with the EllipticalRegionConstraint or RectangularRegionConstraint to enforce
 * a max velocity within a region.
 */
class WPILIB_DLLEXPORT MaxVelocityConstraint : public TrajectoryConstraint {
 public:
  /**
   * Constructs a new MaxVelocityConstraint.
   *
   * @param maxVelocity The max velocity.
   */
  constexpr explicit MaxVelocityConstraint(
      prosdk::units::meters_per_second_t maxVelocity)
      : m_maxVelocity(prosdk::units::math::abs(maxVelocity)) {}

  constexpr prosdk::units::meters_per_second_t MaxVelocity(
      const Pose2d& pose, prosdk::units::curvature_t curvature,
      prosdk::units::meters_per_second_t velocity) const override {
    return m_maxVelocity;
  }

  constexpr MinMax MinMaxAcceleration(
      const Pose2d& pose, prosdk::units::curvature_t curvature,
      prosdk::units::meters_per_second_t velocity) const override {
    return {};
  }

 private:
  prosdk::units::meters_per_second_t m_maxVelocity;
};

}  // namespace prosdk::math
