// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Brushed DC motor driver with two logic inputs and no enable pin (e.g. Boardoza L9110S dual
// and BA6208 breakout boards) on ESP32 GPIOs (design/boardoza/P8-motors.md). Both inputs carry
// a HAL digital PWM generator (DigitalOutput::EnablePWM, LEDC on the ESP32): forced to 0 while
// the output gate is closed (robot disabled), and the emergency stop drives both pins low.

#pragma once

#include <string>
#include <string_view>

#include "prosdk/hal/SimDevice.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/hardware/motor/MotorController.hpp"
#include "prosdk/hardware/motor/MotorSafety.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * H-bridge with two inputs, A and B (IA/IB, AIN/BIN), and no enable or PWM pin.
 *
 * Forward: PWM = throttle on A, B low. Reverse: PWM on B, A low. During the PWM off-time both
 * inputs are low. A throttle of 0 sets both inputs high (ZeroBehavior::kBrake, default) or both
 * low (ZeroBehavior::kCoast); Disable() sets both low.
 *
 * What the input states do depends on the chip (datasheet truth tables):
 *
 * | A | B | BA6208 (UTC QW-R109-004.G p.3) | L9110 (datasheet in the Boardoza repo) |
 * |---|---|---|---|
 * | H | L | forward                        | forward                                |
 * | L | H | reverse                        | reverse                                |
 * | H | H | brake (both outputs low)       | both outputs low (brake)               |
 * | L | L | both outputs open (coast)      | both outputs low (brake)               |
 *
 * So on a BA6208 the PWM off-time and kCoast coast, on an L9110 they brake: the L9110 sheet has
 * no coast state (unverified on the parts fitted to the Boardoza board: some L9110 sheets show
 * L/L as open).
 *
 * Uses two of the 6 HAL PWM generators, which share one frequency (DigitalOutput::SetPWMRate);
 * the constructor sets it to kDefaultPwmRate. Neither datasheet gives a maximum PWM frequency
 * (unverified for both bipolar parts at 20 kHz).
 */
class TwoPinHBridge : public MotorController,
                      public MotorSafety,
                      public prosdk::telemetry::TelemetryLoggable {
 public:
  /// What a throttle of 0 does, see the class description.
  enum class ZeroBehavior {
    /// Both inputs low.
    kCoast,
    /// Both inputs high.
    kBrake
  };

  /// PWM frequency the constructor sets for all digital PWM generators (20 kHz, inaudible).
  static constexpr double kDefaultPwmRate = 20000.0;

  /**
   * Constructs a two-input H-bridge driver.
   *
   * @param inA          GPIO wired to input A (IA, AIN): high drives forward.
   * @param inB          GPIO wired to input B (IB, BIN): high drives reverse.
   * @param zeroBehavior What a throttle of 0 does.
   */
  TwoPinHBridge(int inA, int inB, ZeroBehavior zeroBehavior = ZeroBehavior::kBrake);

  TwoPinHBridge(TwoPinHBridge&&) = default;
  TwoPinHBridge& operator=(TwoPinHBridge&&) = default;

  /** Sets the throttle, -1 to 1 (clamped; NaN is 0). Feeds the motor safety watchdog. */
  void SetThrottle(double throttle) override;
  /** The last throttle set (0 after Disable). */
  double GetThrottle() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  /** Sets both inputs low until the next SetThrottle. */
  void Disable() override;

  /** What a throttle of 0 does from the next SetThrottle on. */
  void SetZeroBehavior(ZeroBehavior zeroBehavior);
  ZeroBehavior GetZeroBehavior() const;

  // MotorSafety
  void StopMotor() override;
  std::string GetDescription() const override;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Output(double throttle);

  DigitalOutput m_inA;
  DigitalOutput m_inB;
  ZeroBehavior m_zeroBehavior;
  double m_throttle = 0.0;
  bool m_inverted = false;
  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimDouble m_simThrottle;
};

/// Boardoza L9110S dual motor driver board: one TwoPinHBridge per channel (M1_A/M1_B, M2_A/M2_B).
using L9110S = TwoPinHBridge;
/// Boardoza BA6208 motor driver board (A IN, B IN).
using BA6208 = TwoPinHBridge;

}  // namespace prosdk
