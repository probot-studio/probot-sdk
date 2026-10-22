// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/util/MathExtras.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {
/**
 * Represents the position of one swerve module.
 */
struct WPILIB_DLLEXPORT SwerveModulePosition {
  /**
   * Distance the wheel of a module has traveled
   */
  prosdk::units::meter_t distance = 0_m;

  /**
   * Angle of the module.
   */
  Rotation2d angle;

  /**
   * Checks equality between this SwerveModulePosition and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  constexpr bool operator==(const SwerveModulePosition& other) const {
    return prosdk::units::math::abs(distance - other.distance) < 1E-9_m &&
           angle == other.angle;
  }

  constexpr SwerveModulePosition Interpolate(
      const SwerveModulePosition& endValue, double t) const {
    return {prosdk::util::Lerp(distance, endValue.distance, t),
            prosdk::util::Lerp(angle, endValue.angle, t)};
  }
};
}  // namespace prosdk::math

#include "prosdk/math/kinematics/proto/SwerveModulePositionProto.hpp"
#include "prosdk/math/kinematics/struct/SwerveModulePositionStruct.hpp"
