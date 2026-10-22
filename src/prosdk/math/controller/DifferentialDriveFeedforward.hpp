// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/controller/DifferentialDriveWheelVoltages.hpp"
#include "prosdk/math/system/LinearSystem.hpp"
#include "prosdk/math/system/Models.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {
/**
 * A helper class which computes the feedforward outputs for a differential
 * drive drivetrain.
 */
class WPILIB_DLLEXPORT DifferentialDriveFeedforward {
  prosdk::math::LinearSystem<2, 2, 2> m_plant;

 public:
  /**
   * Creates a new DifferentialDriveFeedforward with the specified parameters.
   *
   * @param kvLinear The linear velocity gain in volts per (meters per second).
   * @param kaLinear The linear acceleration gain in volts per (meters per
   * second squared).
   * @param kvAngular The angular velocity gain in volts per (radians per
   * second).
   * @param kaAngular The angular acceleration gain in volts per (radians per
   * second squared).
   * @param trackwidth The distance between the differential drive's left and
   * right wheels, in meters.
   */
  constexpr DifferentialDriveFeedforward(
      decltype(1_V / 1_mps) kvLinear, decltype(1_V / 1_mps_sq) kaLinear,
      decltype(1_V / 1_rad_per_s) kvAngular,
      decltype(1_V / 1_rad_per_s_sq) kaAngular, prosdk::units::meter_t trackwidth)
      // See Models::DifferentialDriveFromSysId(decltype(1_V / 1_mps),
      // decltype(1_V / 1_mps_sq), decltype(1_V / 1_rad_per_s), decltype(1_V /
      // 1_rad_per_s_sq))
      : DifferentialDriveFeedforward{kvLinear, kaLinear,
                                     kvAngular * 2.0 / trackwidth * 1_rad,
                                     kaAngular * 2.0 / trackwidth * 1_rad} {}

  /**
   * Creates a new DifferentialDriveFeedforward with the specified parameters.
   *
   * @param kvLinear The linear velocity gain in volts per (meters per second).
   * @param kaLinear The linear acceleration gain in volts per (meters per
   * second squared).
   * @param kvAngular The angular velocity gain in volts per (meters per
   * second).
   * @param kaAngular The angular acceleration gain in volts per (meters per
   * second squared).
   */
  constexpr DifferentialDriveFeedforward(decltype(1_V / 1_mps) kvLinear,
                                         decltype(1_V / 1_mps_sq) kaLinear,
                                         decltype(1_V / 1_mps) kvAngular,
                                         decltype(1_V / 1_mps_sq) kaAngular)
      : m_plant{prosdk::math::Models::DifferentialDriveFromSysId(
            kvLinear, kaLinear, kvAngular, kaAngular)},
        kvLinear{kvLinear},
        kaLinear{kaLinear},
        kvAngular{kvAngular},
        kaAngular{kaAngular} {}

  /**
   * Calculates the differential drive feedforward inputs given velocity
   * references.
   *
   * @param currentLeftVelocity The current left velocity of the differential
   * drive in meters/second.
   * @param nextLeftVelocity The next left velocity of the differential drive in
   * meters/second.
   * @param currentRightVelocity The current right velocity of the differential
   * drive in meters/second.
   * @param nextRightVelocity The next right velocity of the differential drive
   * in meters/second.
   * @param dt Discretization timestep.
   */
  DifferentialDriveWheelVoltages Calculate(
      prosdk::units::meters_per_second_t currentLeftVelocity,
      prosdk::units::meters_per_second_t nextLeftVelocity,
      prosdk::units::meters_per_second_t currentRightVelocity,
      prosdk::units::meters_per_second_t nextRightVelocity,
      prosdk::units::second_t dt);

  decltype(1_V / 1_mps) kvLinear;
  decltype(1_V / 1_mps_sq) kaLinear;
  decltype(1_V / 1_mps) kvAngular;
  decltype(1_V / 1_mps_sq) kaAngular;
};
}  // namespace prosdk::math

#include "prosdk/math/controller/proto/DifferentialDriveFeedforwardProto.hpp"
#include "prosdk/math/controller/struct/DifferentialDriveFeedforwardStruct.hpp"
