// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include "prosdk/math/system/DCMotor.hpp"
#include "prosdk/simulation/LinearSystemSim.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/mass.hpp"
#include "prosdk/units/velocity.hpp"

namespace prosdk::sim {
/**
 * Represents a simulated elevator mechanism.
 */
class ElevatorSim : public LinearSystemSim<2, 1, 2> {
 public:
  template <typename Distance>
  using Velocity_t = prosdk::units::unit_t<prosdk::units::compound_unit<
      Distance, prosdk::units::inverse<prosdk::units::seconds>>>;

  template <typename Distance>
  using Acceleration_t = prosdk::units::unit_t<prosdk::units::compound_unit<
      prosdk::units::compound_unit<Distance,
                                prosdk::units::inverse<prosdk::units::seconds>>,
      prosdk::units::inverse<prosdk::units::seconds>>>;

  /**
   * Constructs a simulated elevator mechanism.
   *
   * @param plant The linear system that represents the elevator. This system
   *     can be created with prosdk::math::Models::ElevatorFromPhysicalConstants().
   * @param gearbox The type of and number of motors in your elevator gearbox.
   * @param minHeight The minimum allowed height of the elevator.
   * @param maxHeight The maximum allowed height of the elevator.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingHeight The starting height of the elevator.
   * @param measurementStdDevs The standard deviation of the measurements.
   */
  ElevatorSim(const prosdk::math::LinearSystem<2, 1, 2>& plant,
              const prosdk::math::DCMotor& gearbox, prosdk::units::meter_t minHeight,
              prosdk::units::meter_t maxHeight, bool simulateGravity,
              prosdk::units::meter_t startingHeight,
              const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

  /**
   * Constructs a simulated elevator mechanism.
   *
   * @param gearbox The type of and number of motors in your elevator gearbox.
   * @param gearing The gearing of the elevator (numbers greater than 1
   *     represent reductions).
   * @param carriageMass The mass of the elevator carriage.
   * @param drumRadius The radius of the drum that your cable is wrapped around.
   * @param minHeight The minimum allowed height of the elevator.
   * @param maxHeight The maximum allowed height of the elevator.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingHeight The starting height of the elevator.
   * @param measurementStdDevs The standard deviation of the measurements.
   */
  ElevatorSim(const prosdk::math::DCMotor& gearbox, double gearing,
              prosdk::units::kilogram_t carriageMass,
              prosdk::units::meter_t drumRadius, prosdk::units::meter_t minHeight,
              prosdk::units::meter_t maxHeight, bool simulateGravity,
              prosdk::units::meter_t startingHeight,
              const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

  /**
   * Constructs a simulated elevator mechanism.
   *
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param gearbox The type of and number of motors in your elevator gearbox.
   * @param minHeight The minimum allowed height of the elevator.
   * @param maxHeight The maximum allowed height of the elevator.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingHeight The starting height of the elevator.
   * @param measurementStdDevs The standard deviation of the measurements.
   */
  template <typename Distance>
    requires std::same_as<prosdk::units::meter, Distance> ||
             std::same_as<prosdk::units::radian, Distance>
  ElevatorSim(decltype(1_V / Velocity_t<Distance>(1)) kV,
              decltype(1_V / Acceleration_t<Distance>(1)) kA,
              const prosdk::math::DCMotor& gearbox, prosdk::units::meter_t minHeight,
              prosdk::units::meter_t maxHeight, bool simulateGravity,
              prosdk::units::meter_t startingHeight,
              const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});
  using LinearSystemSim::SetState;

  /**
   * Sets the elevator's state. The new position will be limited between the
   * minimum and maximum allowed heights.
   *
   * @param position The new position
   * @param velocity The new velocity
   */
  void SetState(prosdk::units::meter_t position,
                prosdk::units::meters_per_second_t velocity);

  /**
   * Returns whether the elevator would hit the lower limit.
   *
   * @param elevatorHeight The elevator height.
   * @return Whether the elevator would hit the lower limit.
   */
  bool WouldHitLowerLimit(prosdk::units::meter_t elevatorHeight) const;

  /**
   * Returns whether the elevator would hit the upper limit.
   *
   * @param elevatorHeight The elevator height.
   * @return Whether the elevator would hit the upper limit.
   */
  bool WouldHitUpperLimit(prosdk::units::meter_t elevatorHeight) const;

  /**
   * Returns whether the elevator has hit the lower limit.
   *
   * @return Whether the elevator has hit the lower limit.
   */
  bool HasHitLowerLimit() const;

  /**
   * Returns whether the elevator has hit the upper limit.
   *
   * @return Whether the elevator has hit the upper limit.
   */
  bool HasHitUpperLimit() const;

  /**
   * Returns the position of the elevator.
   *
   * @return The position of the elevator.
   */
  prosdk::units::meter_t GetPosition() const;

  /**
   * Returns the velocity of the elevator.
   *
   * @return The velocity of the elevator.
   */
  prosdk::units::meters_per_second_t GetVelocity() const;

  /**
   * Returns the elevator current draw.
   *
   * @return The elevator current draw.
   */
  prosdk::units::ampere_t GetCurrentDraw() const;

  /**
   * Sets the input voltage for the elevator.
   *
   * @param voltage The input voltage.
   */
  void SetInputVoltage(prosdk::units::volt_t voltage);

 protected:
  /**
   * Updates the state estimate of the elevator.
   *
   * @param currentXhat The current state estimate.
   * @param u The system inputs (voltage).
   * @param dt The time difference between controller updates.
   */
  prosdk::math::Vectord<2> UpdateX(const prosdk::math::Vectord<2>& currentXhat,
                                const prosdk::math::Vectord<1>& u,
                                prosdk::units::second_t dt) override;

 private:
  prosdk::math::DCMotor m_gearbox;
  prosdk::units::meter_t m_minHeight;
  prosdk::units::meter_t m_maxHeight;
  bool m_simulateGravity;
};
}  // namespace prosdk::sim
