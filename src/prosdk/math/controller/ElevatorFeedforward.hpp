// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <prosdk/thirdparty/gcem/gcem.hpp>

#include "prosdk/math/util/MathShared.hpp"
#include "prosdk/units/base.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/voltage.hpp"
#include "prosdk/util/MathExtras.hpp"

namespace prosdk::math {
/**
 * A helper class that computes feedforward outputs for a simple elevator
 * (modeled as a motor acting against the force of gravity).
 */
class ElevatorFeedforward {
 public:
  using Distance = prosdk::units::meters;
  using Velocity =
      prosdk::units::compound_unit<Distance,
                                prosdk::units::inverse<prosdk::units::seconds>>;
  using Acceleration =
      prosdk::units::compound_unit<Velocity,
                                prosdk::units::inverse<prosdk::units::seconds>>;
  using kv_unit = prosdk::units::compound_unit<prosdk::units::volts,
                                            prosdk::units::inverse<Velocity>>;
  using ka_unit = prosdk::units::compound_unit<prosdk::units::volts,
                                            prosdk::units::inverse<Acceleration>>;

  /**
   * Creates a new ElevatorFeedforward with the specified gains.
   *
   * @param kS The static gain, in volts.
   * @param kG The gravity gain, in volts.
   * @param kV The velocity gain, in volt seconds per distance.
   * @param kA The acceleration gain, in volt seconds² per distance.
   * @param dt The period in seconds.
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   * @throws IllegalArgumentException for period &le; zero.
   */
  constexpr ElevatorFeedforward(
      prosdk::units::volt_t kS, prosdk::units::volt_t kG,
      prosdk::units::unit_t<kv_unit> kV,
      prosdk::units::unit_t<ka_unit> kA = prosdk::units::unit_t<ka_unit>(0),
      prosdk::units::second_t dt = 20_ms)
      : kS(kS), kG(kG), kV(kV), kA(kA), m_dt(dt) {
    if (kV.value() < 0) {
      prosdk::math::MathSharedStore::ReportError(
          "kV must be a non-negative number, got {}!", kV.value());
      this->kV = prosdk::units::unit_t<kv_unit>{0};
      prosdk::math::MathSharedStore::ReportWarning("kV defaulted to 0.");
    }
    if (kA.value() < 0) {
      prosdk::math::MathSharedStore::ReportError(
          "kA must be a non-negative number, got {}!", kA.value());
      this->kA = prosdk::units::unit_t<ka_unit>{0};
      prosdk::math::MathSharedStore::ReportWarning("kA defaulted to 0;");
    }
    if (dt <= 0_ms) {
      prosdk::math::MathSharedStore::ReportError(
          "period must be a positive number, got {}!", dt.value());
      this->m_dt = 20_ms;
      prosdk::math::MathSharedStore::ReportWarning("period defaulted to 20 ms.");
    }
  }

  /**
   * Calculates the feedforward from the gains and reference assuming discrete
   * control. Use this method when the reference does not change.
   *
   * @param currentVelocity The velocity reference.
   * @return The computed feedforward, in volts.
   */
  constexpr prosdk::units::volt_t Calculate(
      prosdk::units::unit_t<Velocity> currentVelocity) const {
    return Calculate(currentVelocity, currentVelocity);
  }

  /**
   * Calculates the feedforward from the gains and references assuming discrete
   * control.
   *
   * Note this method is inaccurate when the velocity crosses 0.
   *
   * @param currentVelocity The current velocity reference.
   * @param nextVelocity    The next velocity reference.
   * @return The computed feedforward, in volts.
   */
  constexpr prosdk::units::volt_t Calculate(
      prosdk::units::unit_t<Velocity> currentVelocity,
      prosdk::units::unit_t<Velocity> nextVelocity) const {
    // See wpimath/docs/ElevatorFeedforward.md for derivation
    if (kA < decltype(kA)(1e-9)) {
      return kS * prosdk::util::sgn(nextVelocity) + kG + kV * nextVelocity;
    } else {
      double A = -kV.value() / kA.value();
      double B = 1.0 / kA.value();
      double A_d = gcem::exp(A * m_dt.value());
      double B_d = A > -1e-9 ? B * m_dt.value() : 1.0 / A * (A_d - 1.0) * B;
      return kG + kS * prosdk::util::sgn(currentVelocity) +
             prosdk::units::volt_t{
                 1.0 / B_d *
                 (nextVelocity.value() - A_d * currentVelocity.value())};
    }
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply
   * and an acceleration.  Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the acceleration constraint, and this will give you
   * a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param acceleration The acceleration of the elevator.
   * @return The maximum possible velocity at the given acceleration.
   */
  constexpr prosdk::units::unit_t<Velocity> MaxAchievableVelocity(
      prosdk::units::volt_t maxVoltage,
      prosdk::units::unit_t<Acceleration> acceleration) {
    // Assume max velocity is positive
    return (maxVoltage - kS - kG - kA * acceleration) / kV;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply
   * and an acceleration.  Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the acceleration constraint, and this will give you
   * a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param acceleration The acceleration of the elevator.
   * @return The minimum possible velocity at the given acceleration.
   */
  constexpr prosdk::units::unit_t<Velocity> MinAchievableVelocity(
      prosdk::units::volt_t maxVoltage,
      prosdk::units::unit_t<Acceleration> acceleration) {
    // Assume min velocity is negative, ks flips sign
    return (-maxVoltage + kS - kG - kA * acceleration) / kV;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage
   * supply and a velocity. Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the velocity constraint, and this will give you
   * a simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param velocity The velocity of the elevator.
   * @return The maximum possible acceleration at the given velocity.
   */
  constexpr prosdk::units::unit_t<Acceleration> MaxAchievableAcceleration(
      prosdk::units::volt_t maxVoltage, prosdk::units::unit_t<Velocity> velocity) {
    return (maxVoltage - kS * prosdk::util::sgn(velocity) - kG - kV * velocity) /
           kA;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage
   * supply and a velocity. Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the velocity constraint, and this will give you
   * a simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param velocity The velocity of the elevator.
   * @return The minimum possible acceleration at the given velocity.
   */
  constexpr prosdk::units::unit_t<Acceleration> MinAchievableAcceleration(
      prosdk::units::volt_t maxVoltage, prosdk::units::unit_t<Velocity> velocity) {
    return MaxAchievableAcceleration(-maxVoltage, velocity);
  }

  /**
   * Sets the static gain.
   *
   * This setter is intended for online tuning only. Feedforward gains are
   * assumed constant, so gain scheduling means the system was not correctly
   * modeled.
   *
   * @param kS The static gain.
   */
  constexpr void SetKs(prosdk::units::volt_t kS) { this->kS = kS; }

  /**
   * Sets the gravity gain.
   *
   * This setter is intended for online tuning only. Feedforward gains are
   * assumed constant, so gain scheduling means the system was not correctly
   * modeled.
   *
   * @param kG The gravity gain.
   */
  constexpr void SetKg(prosdk::units::volt_t kG) { this->kG = kG; }

  /**
   * Sets the velocity gain.
   *
   * This setter is intended for online tuning only. Feedforward gains are
   * assumed constant, so gain scheduling means the system was not correctly
   * modeled.
   *
   * @param kV The velocity gain.
   */
  constexpr void SetKv(prosdk::units::unit_t<kv_unit> kV) { this->kV = kV; }

  /**
   * Sets the acceleration gain.
   *
   * This setter is intended for online tuning only. Feedforward gains are
   * assumed constant, so gain scheduling means the system was not correctly
   * modeled.
   *
   * @param kA The acceleration gain.
   */
  constexpr void SetKa(prosdk::units::unit_t<ka_unit> kA) { this->kA = kA; }

  /**
   * Returns the static gain.
   *
   * @return The static gain.
   */
  constexpr prosdk::units::volt_t GetKs() const { return kS; }

  /**
   * Returns the gravity gain.
   *
   * @return The gravity gain.
   */
  constexpr prosdk::units::volt_t GetKg() const { return kG; }

  /**
   * Returns the velocity gain.
   *
   * @return The velocity gain.
   */
  constexpr prosdk::units::unit_t<kv_unit> GetKv() const { return kV; }

  /**
   * Returns the acceleration gain.
   *
   * @return The acceleration gain.
   */
  constexpr prosdk::units::unit_t<ka_unit> GetKa() const { return kA; }

 private:
  /// The static gain.
  prosdk::units::volt_t kS;

  /// The gravity gain.
  prosdk::units::volt_t kG;

  /// The velocity gain.
  prosdk::units::unit_t<kv_unit> kV;

  /// The acceleration gain.
  prosdk::units::unit_t<ka_unit> kA;

  /** The period. */
  prosdk::units::second_t m_dt;
};
}  // namespace prosdk::math

#include "prosdk/math/controller/proto/ElevatorFeedforwardProto.hpp"
#include "prosdk/math/controller/struct/ElevatorFeedforwardStruct.hpp"
