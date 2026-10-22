// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdexcept>
#include <utility>

#include <prosdk/thirdparty/eigen/Eigen/Core>

#include "prosdk/math/controller/DifferentialDriveWheelVoltages.hpp"
#include "prosdk/math/system/LinearSystem.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

/**
 * Filters the provided voltages to limit a differential drive's linear and
 * angular acceleration.
 *
 * The differential drive model can be created via the functions in Models.
 */
class WPILIB_DLLEXPORT DifferentialDriveAccelerationLimiter {
 public:
  /**
   * Constructs a DifferentialDriveAccelerationLimiter.
   *
   * @param system The differential drive dynamics.
   * @param trackwidth The distance between the differential drive's left and
   *                   right wheels.
   * @param maxLinearAccel The maximum linear acceleration.
   * @param maxAngularAccel The maximum angular acceleration.
   */
  DifferentialDriveAccelerationLimiter(
      LinearSystem<2, 2, 2> system, prosdk::units::meter_t trackwidth,
      prosdk::units::meters_per_second_squared_t maxLinearAccel,
      prosdk::units::radians_per_second_squared_t maxAngularAccel)
      : DifferentialDriveAccelerationLimiter(system, trackwidth,
                                             -maxLinearAccel, maxLinearAccel,
                                             maxAngularAccel) {}

  /**
   * Constructs a DifferentialDriveAccelerationLimiter.
   *
   * @param system The differential drive dynamics.
   * @param trackwidth The distance between the differential drive's left and
   *                   right wheels.
   * @param minLinearAccel The minimum (most negative) linear acceleration.
   * @param maxLinearAccel The maximum (most positive) linear acceleration.
   * @param maxAngularAccel The maximum angular acceleration.
   * @throws std::invalid_argument if minimum linear acceleration is greater
   * than maximum linear acceleration
   */
  DifferentialDriveAccelerationLimiter(
      LinearSystem<2, 2, 2> system, prosdk::units::meter_t trackwidth,
      prosdk::units::meters_per_second_squared_t minLinearAccel,
      prosdk::units::meters_per_second_squared_t maxLinearAccel,
      prosdk::units::radians_per_second_squared_t maxAngularAccel)
      : m_system{std::move(system)},
        m_trackwidth{trackwidth},
        m_minLinearAccel{minLinearAccel},
        m_maxLinearAccel{maxLinearAccel},
        m_maxAngularAccel{maxAngularAccel} {
    if (minLinearAccel > maxLinearAccel) {
      throw std::invalid_argument(
          "maxLinearAccel must be greater than minLinearAccel");
    }
  }

  /**
   * Returns the next voltage pair subject to acceleration constraints.
   *
   * @param leftVelocity The left wheel velocity.
   * @param rightVelocity The right wheel velocity.
   * @param leftVoltage The unconstrained left motor voltage.
   * @param rightVoltage The unconstrained right motor voltage.
   * @return The constrained wheel voltages.
   */
  DifferentialDriveWheelVoltages Calculate(
      prosdk::units::meters_per_second_t leftVelocity,
      prosdk::units::meters_per_second_t rightVelocity,
      prosdk::units::volt_t leftVoltage, prosdk::units::volt_t rightVoltage);

 private:
  LinearSystem<2, 2, 2> m_system;
  prosdk::units::meter_t m_trackwidth;
  prosdk::units::meters_per_second_squared_t m_minLinearAccel;
  prosdk::units::meters_per_second_squared_t m_maxLinearAccel;
  prosdk::units::radians_per_second_squared_t m_maxAngularAccel;
};

}  // namespace prosdk::math
