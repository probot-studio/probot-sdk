// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// API of the WPILib 2026 frc::Servo (WPILib BSD license), re-ported on the 2027 prosdk::PWM.
//
// Hobby servo (design/drivers.md, "Servo"). WPILib 2027 removed Servo; this keeps the 2026 API.
// Two outputs: an ESP32 GPIO through prosdk::PWM (MCPWM, 20 ms period, gated by the output gate
// and the emergency stop like every PWM), or one channel of a PCA9685 (ProHub servo channels).

#pragma once

#include <optional>
#include <string_view>
#include <utility>

#include "prosdk/hal/SimDevice.hpp"
#include "prosdk/hardware/discrete/PWM.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

class PCA9685;

/**
 * Standard hobby servo.
 *
 * The range parameters default to the appropriate values for the Hitec HS-322HD servo provided
 * in the FIRST Kit of Parts in 2008: a 600 to 2400 microsecond pulse every 20 ms maps to 0 to
 * 180 degrees. Use SetBounds() for other servos.
 */
class Servo : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Angle of a position of 1.0, in degrees.
  static constexpr double kMaxServoAngle = 180.0;
  /// Angle of a position of 0.0, in degrees.
  static constexpr double kMinServoAngle = 0.0;
  /// Default pulse width of a position of 1.0.
  static constexpr prosdk::units::microsecond_t kDefaultMaxServoPWM{2400};
  /// Default pulse width of a position of 0.0.
  static constexpr prosdk::units::microsecond_t kDefaultMinServoPWM{600};

  /**
   * Constructs a servo on an ESP32 GPIO (prosdk::PWM with a 20 ms period).
   *
   * @param channel The GPIO the servo signal is attached to.
   */
  explicit Servo(int channel);

  /**
   * Constructs a servo on a PCA9685 output. The PCA9685 period must be longer than the maximum
   * pulse (a frequency of at most about 400 Hz; analog servos expect 50 Hz).
   *
   * @param pca     The PCA9685 (must outlive the servo).
   * @param channel The PCA9685 output, 0 to 15.
   * @throws prosdk::RuntimeError (ParameterOutOfRange) for an invalid channel or a PCA9685
   *         period shorter than the maximum pulse.
   */
  Servo(PCA9685& pca, int channel);

  Servo(Servo&&) = default;
  Servo& operator=(Servo&&) = default;
  ~Servo() override;

  /**
   * Sets the servo position. 0.0 is full left (minimum pulse), 1.0 is full right (maximum
   * pulse). Values outside 0 to 1 are clamped; NaN turns the output off (SetOffline).
   *
   * @param value Position from 0.0 to 1.0.
   */
  void Set(double value);

  /** Stops sending pulses: the servo goes limp (most servos hold no position). */
  void SetOffline();

  /**
   * Gets the last position set, from 0.0 to 1.0 (0.0 while offline).
   *
   * @return Position from 0.0 to 1.0.
   */
  double Get() const;

  /**
   * Sets the servo angle. Angles are clamped to GetMinAngle() to GetMaxAngle() and mapped
   * linearly onto the pulse range.
   *
   * @param angle The angle in degrees.
   */
  void SetAngle(double angle);

  /**
   * Gets the servo angle of the last position set (GetMinAngle() while offline).
   *
   * @return The angle in degrees.
   */
  double GetAngle() const;

  /** The angle of position 1.0, in degrees (180). */
  double GetMaxAngle() const;

  /** The angle of position 0.0, in degrees (0). */
  double GetMinAngle() const;

  /** The GPIO, or the PCA9685 output number. */
  int GetChannel() const;

  /**
   * Sets the pulse widths of position 1.0 (max) and 0.0 (min), as in the 2026 PWM::SetBounds.
   * A servo has no deadband: deadbandMax, center and deadbandMin are ignored. The current
   * position is re-applied with the new bounds.
   */
  void SetBounds(prosdk::units::microsecond_t max, prosdk::units::microsecond_t deadbandMax,
                 prosdk::units::microsecond_t center, prosdk::units::microsecond_t deadbandMin,
                 prosdk::units::microsecond_t min);

  /** Pulse width of position 1.0. */
  prosdk::units::microsecond_t GetMaxPulse() const;

  /** Pulse width of position 0.0. */
  prosdk::units::microsecond_t GetMinPulse() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  // PCA9685 pointer that a move leaves empty (so only one Servo turns the output off).
  struct PcaRef {
    PCA9685* pca = nullptr;
    PcaRef() = default;
    explicit PcaRef(PCA9685* p) : pca{p} {}
    PcaRef(PcaRef&& other) noexcept : pca{std::exchange(other.pca, nullptr)} {}
    PcaRef& operator=(PcaRef&& other) noexcept {
      pca = std::exchange(other.pca, nullptr);
      return *this;
    }
  };

  void Apply();

  std::optional<PWM> m_pwm;
  PcaRef m_pca;
  int m_channel;
  double m_position = 0.0;
  bool m_offline = true;
  prosdk::units::microsecond_t m_maxPwm = kDefaultMaxServoPWM;
  prosdk::units::microsecond_t m_minPwm = kDefaultMinServoPWM;
  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimDouble m_simPosition;
};

}  // namespace prosdk
