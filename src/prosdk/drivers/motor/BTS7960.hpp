// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// BTS7960 (IBT-2 module) H-bridge motor driver on ESP32 GPIOs (design/drivers.md, "BTS7960").
// The PWM comes from the HAL digital PWM generators (DigitalOutput::EnablePWM, LEDC on the
// ESP32): it is forced to 0 while the output gate is closed (robot disabled), and the emergency
// stop drives every pin low.

#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "prosdk/hal/SimDevice.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/hardware/motor/MotorController.hpp"
#include "prosdk/hardware/motor/MotorSafety.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * BTS7960 / IBT-2 dual half-bridge motor driver.
 *
 * Two wirings are supported (R_EN and L_EN are always bridged to one enable signal):
 *
 * - Wiring::PWM_ON_INPUTS (default): RPWM and LPWM carry the PWM (forward drives RPWM,
 *   reverse drives LPWM), the enable GPIO is held high while the motor is enabled. Uses two
 *   PWM generators. A throttle of 0 brakes (both low sides on); Disable() coasts (enable low).
 * - Wiring::PWM_ON_ENABLE (old Probot Lib wiring): the enable GPIO carries the PWM, RPWM and
 *   LPWM select the direction. Uses one PWM generator. Off-time and throttle 0 coast.
 *
 * The HAL has 6 PWM generators with one shared frequency (DigitalOutput::SetPWMRate); the
 * constructor sets it to kDefaultPwmRate. The BTS7960 accepts up to 25 kHz.
 */
class BTS7960 : public MotorController,
                public MotorSafety,
                public prosdk::telemetry::TelemetryLoggable {
 public:
  /// How the module's inputs are wired, see the class description.
  enum class Wiring {
    /// PWM on RPWM/LPWM, enable high.
    PWM_ON_INPUTS,
    /// PWM on the bridged R_EN/L_EN, direction on RPWM/LPWM.
    PWM_ON_ENABLE
  };

  /// PWM frequency the constructor sets for all digital PWM generators (20 kHz, inaudible).
  static constexpr double kDefaultPwmRate = 20000.0;

  /**
   * Constructs a BTS7960 driver.
   *
   * @param rpwm   GPIO wired to RPWM.
   * @param lpwm   GPIO wired to LPWM.
   * @param enable GPIO wired to R_EN and L_EN (bridged).
   * @param wiring Where the PWM goes.
   */
  BTS7960(int rpwm, int lpwm, int enable, Wiring wiring = Wiring::PWM_ON_INPUTS);

  BTS7960(BTS7960&&) = default;
  BTS7960& operator=(BTS7960&&) = default;

  /** Sets the throttle, -1 to 1 (clamped; NaN is 0). Feeds the motor safety watchdog. */
  void SetThrottle(double throttle) override;
  /** The last throttle set (0 after Disable). */
  double GetThrottle() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  /** Turns the bridge off (coast) until the next SetThrottle. */
  void Disable() override;

  // MotorSafety
  void StopMotor() override;
  std::string GetDescription() const override;

  /** The wiring given to the constructor. */
  Wiring GetWiring() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Output(double throttle);

  Wiring m_wiring;
  DigitalOutput m_rpwm;
  DigitalOutput m_lpwm;
  DigitalOutput m_enable;
  double m_throttle = 0.0;
  bool m_inverted = false;
  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimDouble m_simThrottle;
};

}  // namespace prosdk
