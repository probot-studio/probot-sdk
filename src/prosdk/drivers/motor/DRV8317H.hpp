// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// TI DRV8317H three-phase BLDC driver (e.g. Boardoza DRV8317H breakout) with sensored six-step
// commutation on ESP32 GPIOs (design/boardoza/P15-drv8317h.md). The chip has no commutation of
// its own: the HAL's six-step extension (prosdk/platform/hal/SixStep.hpp) switches its INHx/INLx
// inputs from a Hall-pin interrupt, and this class sets the duty and direction. The output gate
// (robot disabled) coasts the motor; the emergency stop drives all six inputs and nSLEEP low for
// good. Hardware-unverified: written from the datasheet, not run on a board.

#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "prosdk/hal/SimDevice.hpp"
#include "prosdk/hardware/discrete/DigitalInput.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/hardware/motor/MotorController.hpp"
#include "prosdk/hardware/motor/MotorSafety.hpp"
#include "prosdk/platform/hal/SixStep.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/util/Handle.hpp"

namespace prosdk {

/**
 * TI DRV8317H in 3x PWM mode (datasheet SLVSGT3, Table 8-4: INLx = 0 is Hi-Z, INLx = 1 makes
 * OUTx follow INHx with internal dead time). The MODE pin (Boardoza jumper J10) must select 3x
 * PWM: MODE floating or tied to AVDD (Table 8-5); it is latched at power-up (section 8.3.2).
 *
 * A non-zero throttle drives the Hall state's phase pair from the commutation table: one phase
 * switches with PWM duty = |throttle|, one is held low, one floats; a negative throttle swaps
 * the two. A throttle of 0 brakes (SetBrakeMode(true), default: all three low sides on) or
 * coasts (all Hi-Z). Disable() coasts. A Hall state the table does not drive (a disconnected
 * sensor reads 7) coasts.
 *
 * The speed and position come from the Hall steps: 6 per electrical revolution, so 6 x pole
 * pairs per motor revolution. Positive is the direction a positive throttle drives (after
 * SetInverted, the inverted direction), if the commutation table matches the motor.
 *
 * Resources: 9 GPIOs (+ nSLEEP, nFAULT), one MCPWM timer and operator (2 of the 12 servo PWM
 * outputs), the GPIO interrupt service; at most 2 motors.
 */
class DRV8317H : public MotorController,
                 public MotorSafety,
                 public prosdk::telemetry::TelemetryLoggable {
 public:
  /// PWM frequency (the DRV8317 accepts up to 200 kHz, datasheet section 1).
  static constexpr double kDefaultPwmRate = 20000.0;

  /// Commutation table: the forward phase pair per Hall state (prosdk/platform/hal/SixStep.hpp).
  using CommutationTable = prosdk::platform::hal::SixStepTable;

  /// GPIOs of one board. INHx, INLx and the Halls are required; nSleep and nFault may be -1.
  struct Pins {
    int inhA = -1;
    int inhB = -1;
    int inhC = -1;
    int inlA = -1;
    int inlB = -1;
    int inlC = -1;
    int hallA = -1;
    int hallB = -1;
    int hallC = -1;
    /// nSLEEP (active-low sleep): driven high by the constructor. -1 when tied high.
    int nSleep = -1;
    /// nFAULT (open drain, pulled up to AVDD on the board). -1 for none.
    int nFault = -1;
  };

  /**
   * Constructs the driver with the commutation table platform::hal::kDefaultSixStepTable (the
   * Boardoza library's; unverified on a motor).
   *
   * @throws prosdk::RuntimeError when a pin is invalid, given twice or in use, or no MCPWM
   *         timer or motor slot is left.
   */
  explicit DRV8317H(const Pins& pins);

  DRV8317H(DRV8317H&&) = default;
  DRV8317H& operator=(DRV8317H&&) = default;

  /** Sets the throttle, -1 to 1 (clamped; NaN is 0). Feeds the motor safety watchdog. */
  void SetThrottle(double throttle) override;
  /** The last throttle set (0 after Disable). */
  double GetThrottle() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  /** Coasts (all phases Hi-Z) until the next SetThrottle. */
  void Disable() override;

  /** Brake (true, default) or coast (false) at a throttle of 0. */
  void SetBrakeMode(bool brake);
  bool GetBrakeMode() const;

  /**
   * Replaces the commutation table. The right one depends on the motor's Hall and winding
   * order. Returns false (and reports the reason to the console) when the table is not six
   * different phase pairs on six Hall states.
   */
  bool SetCommutationTable(const CommutationTable& table);

  /**
   * True while nFAULT is low: overcurrent, VM under- or overvoltage, charge pump undervoltage or
   * overtemperature. The DRV8317H retries on its own after 5 ms (datasheet section 8.3.11).
   * nFAULT is also held low while the chip wakes up, at most tWAKE (3 ms) after the constructor
   * (section 8.4.1.1). False without the pin.
   */
  bool IsFault() const;

  /** Motor pole pairs (default 1). */
  void SetPolePairs(int polePairs);
  int GetPolePairs() const;

  /** The last Hall state read: A | B << 1 | C << 2. */
  int GetHallState() const;

  /** Motor position from the Hall steps since construction (signed, see the class comment). */
  prosdk::units::turn_t GetPosition() const;

  /**
   * Motor speed from the time per Hall step (signed). It decays as 1 / (time since the last step)
   * when steps stop, and is 0 once no step came for 0.5 s.
   */
  prosdk::units::revolutions_per_minute_t GetRpm() const;

  // MotorSafety
  void StopMotor() override;
  std::string GetDescription() const override;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Output(double throttle);
  prosdk::platform::hal::SixStepFeedback Feedback() const;

  Pins m_pins;
  prosdk::util::Handle<int32_t, prosdk::platform::hal::CloseSixStep,
                       prosdk::platform::hal::kInvalidSixStepHandle>
      m_handle;
  std::optional<DigitalOutput> m_nSleep;
  std::optional<DigitalInput> m_nFault;
  double m_throttle = 0.0;
  bool m_inverted = false;
  bool m_brake = true;
  int m_polePairs = 1;
  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimDouble m_simThrottle;
};

}  // namespace prosdk
