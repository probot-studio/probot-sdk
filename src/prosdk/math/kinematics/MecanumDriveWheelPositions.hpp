// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/units/length.hpp"
#include "prosdk/util/MathExtras.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {
/**
 * Represents the wheel positions for a mecanum drive drivetrain.
 */
struct WPILIB_DLLEXPORT MecanumDriveWheelPositions {
  /**
   * Distance driven by the front-left wheel.
   */
  prosdk::units::meter_t frontLeft = 0_m;

  /**
   * Distance driven by the front-right wheel.
   */
  prosdk::units::meter_t frontRight = 0_m;

  /**
   * Distance driven by the rear-left wheel.
   */
  prosdk::units::meter_t rearLeft = 0_m;

  /**
   * Distance driven by the rear-right wheel.
   */
  prosdk::units::meter_t rearRight = 0_m;

  /**
   * Checks equality between this MecanumDriveWheelPositions and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  constexpr bool operator==(const MecanumDriveWheelPositions& other) const =
      default;

  constexpr MecanumDriveWheelPositions Interpolate(
      const MecanumDriveWheelPositions& endValue, double t) const {
    return {prosdk::util::Lerp(frontLeft, endValue.frontLeft, t),
            prosdk::util::Lerp(frontRight, endValue.frontRight, t),
            prosdk::util::Lerp(rearLeft, endValue.rearLeft, t),
            prosdk::util::Lerp(rearRight, endValue.rearRight, t)};
  }
};
}  // namespace prosdk::math

#include "prosdk/math/kinematics/proto/MecanumDriveWheelPositionsProto.hpp"
#include "prosdk/math/kinematics/struct/MecanumDriveWheelPositionsStruct.hpp"
