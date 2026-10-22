// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include "prosdk/math/system/DCMotor.hpp"
#include "prosdk/simulation/LinearSystemSim.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/mass.hpp"
#include "prosdk/units/moment_of_inertia.hpp"

namespace prosdk::sim {
/**
 * Represents a simulated arm mechanism.
 */
class SingleJointedArmSim : public LinearSystemSim<2, 1, 2> {
 public:
  /**
   * Creates a simulated arm mechanism.
   *
   * @param system The system representing this arm. This system can be created
   *     with prosdk::math::Models::SingleJointedArmFromPhysicalConstants().
   * @param gearbox The type and number of motors on the arm gearbox.
   * @param gearing The gear ratio of the arm (numbers greater than 1 represent
   *     reductions).
   * @param armLength The length of the arm.
   * @param minAngle The minimum angle that the arm is capable of, with 0 being
   *     horizontal.
   * @param maxAngle The maximum angle that the arm is capable of, with 0 being
   *     horizontal.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingAngle The initial position of the arm.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  SingleJointedArmSim(const prosdk::math::LinearSystem<2, 1, 2>& system,
                      const prosdk::math::DCMotor& gearbox, double gearing,
                      prosdk::units::meter_t armLength,
                      prosdk::units::radian_t minAngle,
                      prosdk::units::radian_t maxAngle, bool simulateGravity,
                      prosdk::units::radian_t startingAngle,
                      const std::array<double, 2>& measurementStdDevs = {0.0,
                                                                         0.0});
  /**
   * Creates a simulated arm mechanism.
   *
   * @param gearbox The type and number of motors on the arm gearbox.
   * @param gearing The gear ratio of the arm (numbers greater than 1 represent
   *     reductions).
   * @param moi The moment of inertia of the arm. This can be calculated from
   *     CAD software.
   * @param armLength The length of the arm.
   * @param minAngle The minimum angle that the arm is capable of, with 0 being
   *     horizontal.
   * @param maxAngle The maximum angle that the arm is capable of, with 0 being
   *     horizontal.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingAngle The initial position of the arm.
   * @param measurementStdDevs The standard deviation of the measurement noise.
   */
  SingleJointedArmSim(
      const prosdk::math::DCMotor& gearbox, double gearing,
      prosdk::units::kilogram_square_meter_t moi, prosdk::units::meter_t armLength,
      prosdk::units::radian_t minAngle, prosdk::units::radian_t maxAngle,
      bool simulateGravity, prosdk::units::radian_t startingAngle,
      const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

  using LinearSystemSim::SetState;

  /**
   * Sets the arm's state. The new angle will be limited between the minimum and
   * maximum allowed limits.
   *
   * @param angle The new angle.
   * @param velocity The new angular velocity.
   */
  void SetState(prosdk::units::radian_t angle,
                prosdk::units::radians_per_second_t velocity);

  /**
   * Returns whether the arm would hit the lower limit.
   *
   * @param armAngle The arm height.
   * @return Whether the arm would hit the lower limit.
   */
  bool WouldHitLowerLimit(prosdk::units::radian_t armAngle) const;

  /**
   * Returns whether the arm would hit the upper limit.
   *
   * @param armAngle The arm height.
   * @return Whether the arm would hit the upper limit.
   */
  bool WouldHitUpperLimit(prosdk::units::radian_t armAngle) const;

  /**
   * Returns whether the arm has hit the lower limit.
   *
   * @return Whether the arm has hit the lower limit.
   */
  bool HasHitLowerLimit() const;

  /**
   * Returns whether the arm has hit the upper limit.
   *
   * @return Whether the arm has hit the upper limit.
   */
  bool HasHitUpperLimit() const;

  /**
   * Returns the current arm angle.
   *
   * @return The current arm angle.
   */
  prosdk::units::radian_t GetAngle() const;

  /**
   * Returns the current arm velocity.
   *
   * @return The current arm velocity.
   */
  prosdk::units::radians_per_second_t GetVelocity() const;

  /**
   * Returns the arm current draw.
   *
   * @return The arm current draw.
   */
  prosdk::units::ampere_t GetCurrentDraw() const;

  /**
   * Sets the input voltage for the arm.
   *
   * @param voltage The input voltage.
   */
  void SetInputVoltage(prosdk::units::volt_t voltage);

  /**
   * Calculates a rough estimate of the moment of inertia of an arm given its
   * length and mass.
   *
   * @param length The length of the arm.
   * @param mass   The mass of the arm.
   *
   * @return The calculated moment of inertia.
   */
  static constexpr prosdk::units::kilogram_square_meter_t EstimateMOI(
      prosdk::units::meter_t length, prosdk::units::kilogram_t mass) {
    return 1.0 / 3.0 * mass * length * length;
  }

 protected:
  /**
   * Updates the state estimate of the arm.
   *
   * @param currentXhat The current state estimate.
   * @param u           The system inputs (voltage).
   * @param dt          The time difference between controller updates.
   */
  prosdk::math::Vectord<2> UpdateX(const prosdk::math::Vectord<2>& currentXhat,
                                const prosdk::math::Vectord<1>& u,
                                prosdk::units::second_t dt) override;

 private:
  prosdk::units::meter_t m_armLen;
  prosdk::units::radian_t m_minAngle;
  prosdk::units::radian_t m_maxAngle;
  const prosdk::math::DCMotor m_gearbox;
  double m_gearing;
  bool m_simulateGravity;
};
}  // namespace prosdk::sim
