// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// TI DRV8307 sensored BLDC motor controller (e.g. Boardoza DRV8307 48W board) on ESP32 GPIOs
// (design/boardoza/P8-motors.md). The chip commutates from the motor's Hall sensors itself; the
// host gives a PWM duty (a HAL digital PWM generator, LEDC on the ESP32: forced to 0 while the
// robot is disabled), the direction, BRAKE and the active-low ENABLEn, and can read the HALLOUT
// tach, LOCKn and FAULTn outputs. The emergency stop drives every output pin low: ENABLEn low
// keeps the chip enabled, at 0 % duty (no drive).

#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "prosdk/hal/SimDevice.hpp"
#include "prosdk/hardware/counter/Tachometer.hpp"
#include "prosdk/hardware/discrete/DigitalInput.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/hardware/motor/MotorController.hpp"
#include "prosdk/hardware/motor/MotorSafety.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/angular_velocity.hpp"

namespace prosdk {

/**
 * TI DRV8307 three-phase BLDC controller in clock PWM mode (datasheet SLVSCK2A).
 *
 * A non-zero throttle sets DIR (high for positive), BRAKE low, PWM duty = |throttle| and
 * ENABLEn low. A throttle of 0 does not set 0 % duty while enabled (section 7.4.1): it brings
 * ENABLEn high, with BRAKE high (SetBrakeMode(true), default: all low sides on, section 8.1.3)
 * or low (coast), and the PWM at 0. Disable() coasts (ENABLEn high, BRAKE low).
 *
 * After ENABLEn goes high the chip runs its stop-and-power-down sequence to the end: the motor
 * can be driven again only after it stopped, and the chip enters standby 1 s after the last
 * HALLOUT edge (sections 7.3.3, 8.1.2). Reversing a spinning motor makes the chip coast until
 * the Hall U period exceeds 160 ms, then it drives the new direction (section 7.3.4).
 *
 * dir = -1 or brake = -1 when the board's DIR switch or BRAKE jumper sets that input: without
 * DIR negative throttles stop the motor, without BRAKE a throttle of 0 coasts.
 *
 * The HAL has 6 PWM generators with one shared frequency (DigitalOutput::SetPWMRate); the
 * constructor sets it to kDefaultPwmRate, inside the DRV8307's 16-50 kHz input range
 * (section 6.3).
 */
class DRV8307 : public MotorController,
                public MotorSafety,
                public prosdk::telemetry::TelemetryLoggable {
 public:
  /// PWM frequency the constructor sets for all digital PWM generators.
  static constexpr double kDefaultPwmRate = 20000.0;

  /**
   * Constructs a DRV8307 driver.
   *
   * @param pwm     GPIO wired to PWM.
   * @param enableN GPIO wired to ENABLEn (active low).
   * @param dir     GPIO wired to DIR, or -1.
   * @param brake   GPIO wired to BRAKE, or -1.
   * @param lockN   GPIO wired to LOCKn (open drain; input pull-up), or -1.
   * @param hallOut GPIO wired to HALLOUT (open drain; counted on a PCNT unit), or -1.
   * @param faultN  GPIO wired to FAULTn (open drain; input pull-up), or -1.
   */
  DRV8307(int pwm, int enableN, int dir, int brake, int lockN = -1, int hallOut = -1,
          int faultN = -1);

  DRV8307(DRV8307&&) = default;
  DRV8307& operator=(DRV8307&&) = default;

  /** Sets the throttle, -1 to 1 (clamped; NaN is 0). Feeds the motor safety watchdog. */
  void SetThrottle(double throttle) override;
  /** The last throttle set (0 after Disable). */
  double GetThrottle() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  /** Coasts (ENABLEn high, BRAKE low, PWM 0) until the next SetThrottle. */
  void Disable() override;

  /** Brake (true, default) or coast (false) at a throttle of 0. Needs the BRAKE pin. */
  void SetBrakeMode(bool brake);
  bool GetBrakeMode() const;

  /** True while LOCKn is low: the speed is steady and the chip commutates on Hall U only. */
  bool IsLocked() const;

  /**
   * True while FAULTn is low (section 7.3.11): undervoltage, overvoltage, overcurrent (retry
   * after 5 ms), charge pump failure or short, locked rotor or overtemperature (retry after 5 s).
   * False without the pin.
   */
  bool IsFault() const;

  /** Motor pole pairs (default 1): HALLOUT has one period per electrical revolution. */
  void SetPolePairs(int polePairs);
  int GetPolePairs() const;

  /** The motor speed from HALLOUT (unsigned: HALLOUT has no direction). 0 without the pin. */
  prosdk::units::revolutions_per_minute_t GetRpm() const;

  // MotorSafety
  void StopMotor() override;
  std::string GetDescription() const override;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Output(double throttle);

  DigitalOutput m_pwm;
  DigitalOutput m_enableN;
  std::optional<DigitalOutput> m_dir;
  std::optional<DigitalOutput> m_brakePin;
  std::optional<DigitalInput> m_lockN;
  std::optional<Tachometer> m_hallOut;
  std::optional<DigitalInput> m_faultN;
  double m_throttle = 0.0;
  bool m_inverted = false;
  bool m_brake = true;
  int m_polePairs = 1;
  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimDouble m_simThrottle;
};

}  // namespace prosdk
