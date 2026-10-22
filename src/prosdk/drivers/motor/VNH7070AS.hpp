// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ST VNH7070AS H-bridge motor driver (e.g. Boardoza VNH7070AS breakout board) on ESP32 GPIOs
// (design/boardoza/P8-motors.md). INA/INB select the direction, PWM sets the speed (a HAL
// digital PWM generator, LEDC on the ESP32: forced to 0 while the robot is disabled), SEL0
// follows INA so that the CS pin reports the current of the driving leg, and CS is read on an
// optional ADC1 pin. The emergency stop drives every pin low (INA = INB = SEL0 = PWM = 0: the
// chip enters standby).

#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "prosdk/hal/SimDevice.hpp"
#include "prosdk/hardware/discrete/AnalogInput.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/hardware/motor/MotorController.hpp"
#include "prosdk/hardware/motor/MotorSafety.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/current.hpp"
#include "prosdk/units/impedance.hpp"

namespace prosdk {

/**
 * ST VNH7070AS full-bridge motor driver (datasheet DocID028088 Rev 5).
 *
 * Same control as prosdk::VNH5019: forward is INA high, INB low, PWM = throttle; reverse the
 * opposite. A throttle of 0 brakes (INA = INB = low, PWM = brake strength: both low sides on) or
 * coasts (PWM 0) as set by SetBrakeMode(). Disable() coasts. When INA/INB change, the PWM is 0
 * while they switch and rises 20 us after them (the datasheet's order for leaving standby,
 * Table 6 note 1).
 *
 * SEL0 is driven to the INA level: CS then reports the high-side current HSDA when forward and
 * HSDB when reverse (Table 11). With sel0 = -1 SEL0 is wired by the user, and GetCurrent() is
 * valid in one direction only (SEL0 high: forward, SEL0 low: reverse; CS is Hi-Z in the other).
 * INA = INB = SEL0 = PWM = 0 (coast at 0, Disable, emergency stop) puts the chip in standby
 * after 0.2-1.8 ms (tD_sdby).
 *
 * The HAL has 6 PWM generators with one shared frequency (DigitalOutput::SetPWMRate); the
 * constructor sets it to kDefaultPwmRate, which is the VNH7070AS maximum (fPWM 0-20 kHz,
 * Table 8): never raise the shared rate while this driver is used.
 */
class VNH7070AS : public MotorController,
                  public MotorSafety,
                  public prosdk::telemetry::TelemetryLoggable {
 public:
  /// PWM frequency the constructor sets for all digital PWM generators (the chip's maximum).
  static constexpr double kDefaultPwmRate = 20000.0;
  /// Current sense ratio K = IOUT / ISENSE, typical at 3.5 A and 5.5 A (K2/K3, Table 10).
  static constexpr double kSenseRatio = 1540.0;
  /// CS resistor to ground on the Boardoza board (R5 1.5 kOhm; R4 10 kOhm in series to J1 CS).
  static constexpr prosdk::units::ohm_t kBoardSenseResistance{1500.0};
  /// Minimum CS voltage in a fault (VSENSEH 5-7 V, Table 10).
  static constexpr double kFaultSenseVoltage = 5.0;

  /**
   * Constructs a VNH7070AS driver.
   *
   * @param ina      GPIO wired to INA.
   * @param inb      GPIO wired to INB.
   * @param pwm      GPIO wired to PWM.
   * @param sel0     GPIO wired to SEL0, or -1 when SEL0 is wired by the user.
   * @param csAnalog ADC1 GPIO (1 to 10) wired to CS, or -1 when the current is not read. CS can
   *                 rise to 5-7 V: see design/boardoza/P8-motors.md for the divider.
   */
  VNH7070AS(int ina, int inb, int pwm, int sel0 = -1, int csAnalog = -1);

  VNH7070AS(VNH7070AS&&) = default;
  VNH7070AS& operator=(VNH7070AS&&) = default;

  /** Sets the throttle, -1 to 1 (clamped; NaN is 0). Feeds the motor safety watchdog. */
  void SetThrottle(double throttle) override;
  /** The last throttle set (0 after Disable). */
  double GetThrottle() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  /** Turns the bridge off (coast; INA = INB = SEL0 = PWM = 0, standby) until the next SetThrottle. */
  void Disable() override;

  /** Brake (true, default) or coast (false) at a throttle of 0. */
  void SetBrakeMode(bool brake);
  bool GetBrakeMode() const;

  /** PWM duty cycle used for braking, 0 to 1 (default 1: full brake). */
  void SetBrakeStrength(double strength);
  double GetBrakeStrength() const;

  /**
   * Sets the CS scaling: the resistor from CS to ground and the divider between CS and the ADC
   * pin (ADC voltage / CS voltage, 0 < ratio <= 1; 1 without a divider). Defaults:
   * kBoardSenseResistance, 1.
   */
  void SetCurrentSense(prosdk::units::ohm_t senseResistance, double dividerRatio = 1.0);

  /**
   * The motor current from CS: CS voltage / sense resistance * kSenseRatio. 0 without a CS pin.
   * Reads about 0 while braking or coasting (CS Hi-Z) and in the direction that SEL0 does not
   * select. The ADC reads up to about 3.1 V, so without a divider the reading saturates near
   * 3.2 A on the Boardoza board. Below about 0.2 A K is not linear (K0/K1, Table 10).
   */
  prosdk::units::ampere_t GetCurrent() const;

  /**
   * True if CS is at the fault level (kFaultSenseVoltage or more at the chip, Table 12) while the
   * bridge drives or brakes. False without a CS pin, while coasting or disabled (CS then shows
   * the off-state diagnostic, Table 13), and always without a divider (the ADC cannot read 5 V).
   * With SEL0 = INA, Table 12 reports only the driving high side (forward HSA, reverse HSB) and
   * LSB while braking; a latched low side while driving is not visible. A current high enough
   * to put 5 V on CS (VSENSESAT, Table 10) reads as a fault too. A latched fault is cleared by
   * Disable() and a SetThrottle at least 20 us later, e.g. in the next loop (input toggle,
   * tLATCH_RST 3-20 us, Table 9).
   */
  bool IsFault() const;

  // MotorSafety
  void StopMotor() override;
  std::string GetDescription() const override;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Output(double throttle);
  void SetDirection(bool ina, bool inb);
  double GetSenseVoltage() const;

  DigitalOutput m_ina;
  DigitalOutput m_inb;
  DigitalOutput m_pwm;
  std::optional<DigitalOutput> m_sel0;
  std::optional<AnalogInput> m_cs;
  double m_throttle = 0.0;
  bool m_inverted = false;
  bool m_brake = true;
  double m_brakeStrength = 1.0;
  bool m_bridgeOn = false;
  prosdk::units::ohm_t m_senseResistance = kBoardSenseResistance;
  double m_dividerRatio = 1.0;
  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimDouble m_simThrottle;
};

}  // namespace prosdk
