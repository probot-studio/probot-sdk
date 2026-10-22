// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/system/DCMotor.hpp"
#include "prosdk/math/system/LinearSystem.hpp"
#include "prosdk/simulation/LinearSystemSim.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/moment_of_inertia.hpp"
#include "prosdk/units/torque.hpp"

namespace prosdk::sim {
/**
 * Represents a simulated DC motor mechanism.
 */
class DCMotorSim : public LinearSystemSim<2, 1, 2> {
 public:
  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant The linear system representing the DC motor. This system can
   *     be created with
   *     prosdk::math::Models::SingleJointedArmFromPhysicalConstants() or
   *     prosdk::math::Models::SingleJointedArmFromSysId().
   * @param gearbox The type of and number of motors in the DC motor gearbox.
   * @param measurementStdDevs The standard deviation of the measurement noise.
   */
  DCMotorSim(const prosdk::math::LinearSystem<2, 1, 2>& plant,
             const prosdk::math::DCMotor& gearbox,
             const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

  using LinearSystemSim::SetState;

  /**
   * Sets the state of the DC motor.
   *
   * @param angularPosition The new position
   * @param angularVelocity The new velocity
   */
  void SetState(prosdk::units::radian_t angularPosition,
                prosdk::units::radians_per_second_t angularVelocity);

  /**
   * Sets the DC motor's angular position.
   *
   * @param angularPosition The new position in radians.
   */
  void SetAngle(prosdk::units::radian_t angularPosition);

  /**
   * Sets the DC motor's angular velocity.
   *
   * @param angularVelocity The new velocity in radians per second.
   */
  void SetAngularVelocity(prosdk::units::radians_per_second_t angularVelocity);

  /**
   * Returns the DC motor position.
   *
   * @return The DC motor position.
   */
  prosdk::units::radian_t GetAngularPosition() const;

  /**
   * Returns the DC motor velocity.
   *
   * @return The DC motor velocity.
   */
  prosdk::units::radians_per_second_t GetAngularVelocity() const;

  /**
   * Returns the DC motor acceleration.
   *
   * @return The DC motor acceleration
   */
  prosdk::units::radians_per_second_squared_t GetAngularAcceleration() const;

  /**
   * Returns the DC motor torque.
   *
   * @return The DC motor torque
   */
  prosdk::units::newton_meter_t GetTorque() const;

  /**
   * Returns the DC motor current draw.
   *
   * @return The DC motor current draw.
   */
  prosdk::units::ampere_t GetCurrentDraw() const;

  /**
   * Gets the input voltage for the DC motor.
   *
   * @return The DC motor input voltage.
   */
  prosdk::units::volt_t GetInputVoltage() const;

  /**
   * Sets the input voltage for the DC motor.
   *
   * @param voltage The input voltage.
   */
  void SetInputVoltage(prosdk::units::volt_t voltage);

  /**
   * Returns the gearbox.
   */
  const prosdk::math::DCMotor& GetGearbox() const;

  /**
   * Returns the gearing;
   */
  double GetGearing() const;

  /**
   * Returns the moment of inertia
   */
  prosdk::units::kilogram_square_meter_t GetJ() const;

 private:
  prosdk::math::DCMotor m_gearbox;
  double m_gearing;
  prosdk::units::kilogram_square_meter_t m_j;
};
}  // namespace prosdk::sim
