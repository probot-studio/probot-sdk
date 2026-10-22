// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <concepts>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "prosdk/hal/SimDevice.hpp"
#include "prosdk/hardware/discrete/PWM.hpp"
#include "prosdk/hardware/motor/MotorController.hpp"
#include "prosdk/hardware/motor/MotorSafety.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/voltage.hpp"

namespace prosdk {

/**
 * Common base class for all PWM Motor Controllers.
 */
class PWMMotorController : public MotorController,
                           public MotorSafety,
                           public prosdk::telemetry::TelemetryLoggable {
 public:
  PWMMotorController(PWMMotorController&&) = default;
  PWMMotorController& operator=(PWMMotorController&&) = default;

  void SetThrottle(double throttle) override;

  double GetThrottle() const override;

  /**
   * Gets the voltage output of the motor controller, nominally between -12 V
   * and 12 V.
   *
   * @return The voltage of the motor controller, nominally between -12 V and 12
   *     V.
   */
  virtual prosdk::units::volt_t GetVoltage() const;

  void SetInverted(bool isInverted) override;

  bool GetInverted() const override;

  void Disable() override;

  // MotorSafety interface
  void StopMotor() override;
  std::string GetDescription() const override;

  int GetChannel() const;

  /**
   * Optionally eliminate the deadband from a motor controller.
   *
   * @param eliminateDeadband If true, set the motor curve on the motor
   *                          controller to eliminate the deadband in the middle
   *                          of the range. Otherwise, keep the full range
   *                          without modifying any values.
   */
  void EnableDeadbandElimination(bool eliminateDeadband);

  /**
   * Make the given PWM motor controller follow the output of this one.
   *
   * @param follower The motor controller follower.
   */
  void AddFollower(PWMMotorController& follower);

  /**
   * Make the given PWM motor controller follow the output of this one.
   *
   * @param follower The motor controller follower.
   */
  template <std::derived_from<PWMMotorController> T>
  void AddFollower(T&& follower) {
    m_owningFollowers.emplace_back(
        std::make_unique<std::decay_t<T>>(std::forward<T>(follower)));
  }

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;

  std::string_view GetTelemetryType() const override;

 protected:
  /**
   * Constructor for a PWM Motor Controller connected via PWM.
   *
   * @param channel The SmartIO channel that the controller is attached to.
   */
  explicit PWMMotorController(int channel);

  /// PWM instances for motor controller.
  PWM m_pwm;

  void SetDutyCycleInternal(double dutyCycle);
  double GetDutyCycleInternal() const;

  void SetBounds(prosdk::units::microsecond_t maxPwm,
                 prosdk::units::microsecond_t deadbandMaxPwm,
                 prosdk::units::microsecond_t centerPwm,
                 prosdk::units::microsecond_t deadbandMinPwm,
                 prosdk::units::microsecond_t minPwm);

 private:
  bool m_isInverted = false;
  std::vector<PWMMotorController*> m_nonowningFollowers;
  std::vector<std::unique_ptr<PWMMotorController>> m_owningFollowers;

  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimDouble m_simThrottle;

  bool m_eliminateDeadband{0};
  prosdk::units::microsecond_t m_minPwm{0};
  prosdk::units::microsecond_t m_deadbandMinPwm{0};
  prosdk::units::microsecond_t m_centerPwm{0};
  prosdk::units::microsecond_t m_deadbandMaxPwm{0};
  prosdk::units::microsecond_t m_maxPwm{0};

  prosdk::units::microsecond_t GetMinPositivePwm() const;
  prosdk::units::microsecond_t GetMaxNegativePwm() const;
  prosdk::units::microsecond_t GetPositiveScaleFactor() const;
  prosdk::units::microsecond_t GetNegativeScaleFactor() const;

  PWM* GetPwm() { return &m_pwm; }
};

}  // namespace wpi
