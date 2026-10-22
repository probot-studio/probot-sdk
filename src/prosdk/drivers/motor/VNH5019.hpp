// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ST VNH5019 H-bridge motor driver (e.g. Boardoza / Pololu VNH5019 boards) on ESP32 GPIOs
// (design/drivers.md, "VNH5019"). INA/INB select the direction, PWM sets the speed (a HAL
// digital PWM generator, LEDC on the ESP32: forced to 0 while the robot is disabled), and the
// optional ENA/ENB (EN/DIAG) pins are driven high while the motor is enabled. The emergency stop
// drives every pin low (bridge off).

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
 * ST VNH5019 full-bridge motor driver.
 *
 * Forward: INA high, INB low, PWM = throttle. Reverse: INA low, INB high. A throttle of 0 brakes
 * (INA = INB = low, PWM = brake strength: the low sides short the motor) or coasts (PWM 0) as
 * set by SetBrakeMode(). Disable() coasts and pulls ENA/ENB low.
 *
 * The HAL has 6 PWM generators with one shared frequency (DigitalOutput::SetPWMRate); the
 * constructor sets it to kDefaultPwmRate (the VNH5019 accepts up to 20 kHz).
 */
class VNH5019 : public MotorController,
                public MotorSafety,
                public prosdk::telemetry::TelemetryLoggable {
 public:
  /// PWM frequency the constructor sets for all digital PWM generators.
  static constexpr double kDefaultPwmRate = 20000.0;

  /**
   * Constructs a VNH5019 driver whose EN/DIAG pins are pulled up on the board.
   *
   * @param ina GPIO wired to INA.
   * @param inb GPIO wired to INB.
   * @param pwm GPIO wired to PWM.
   */
  VNH5019(int ina, int inb, int pwm);

  /**
   * Constructs a VNH5019 driver that also drives EN/DIAG A and B (high while enabled). Use a
   * series resistor (1 kOhm or more): the chip pulls EN/DIAG low on a fault.
   *
   * @param ina GPIO wired to INA.
   * @param inb GPIO wired to INB.
   * @param pwm GPIO wired to PWM.
   * @param ena GPIO wired to EN/DIAG A.
   * @param enb GPIO wired to EN/DIAG B (may equal ena when both are bridged; -1 for none).
   */
  VNH5019(int ina, int inb, int pwm, int ena, int enb);

  VNH5019(VNH5019&&) = default;
  VNH5019& operator=(VNH5019&&) = default;

  /** Sets the throttle, -1 to 1 (clamped; NaN is 0). Feeds the motor safety watchdog. */
  void SetThrottle(double throttle) override;
  /** The last throttle set (0 after Disable). */
  double GetThrottle() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  /** Turns the bridge off (coast, ENA/ENB low) until the next SetThrottle. */
  void Disable() override;

  /** Brake (true, default) or coast (false) at a throttle of 0. */
  void SetBrakeMode(bool brake);
  bool GetBrakeMode() const;

  /** PWM duty cycle used for braking, 0 to 1 (default 1: full brake). */
  void SetBrakeStrength(double strength);
  double GetBrakeStrength() const;

  // MotorSafety
  void StopMotor() override;
  std::string GetDescription() const override;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Init(int ina);
  void Output(double throttle);
  void SetEnables(bool on);

  DigitalOutput m_ina;
  DigitalOutput m_inb;
  DigitalOutput m_pwm;
  std::optional<DigitalOutput> m_ena;
  std::optional<DigitalOutput> m_enb;
  double m_throttle = 0.0;
  bool m_inverted = false;
  bool m_brake = true;
  double m_brakeStrength = 1.0;
  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimDouble m_simThrottle;
};

}  // namespace prosdk
