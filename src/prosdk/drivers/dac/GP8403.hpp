// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Guestgood (Keyi) GP8403 dual 12-bit DAC with 0-5 V / 0-10 V outputs over prosdk::I2C, for
// example the Boardoza GP8403 breakout (design/boardoza/P5-i2c-misc.md). Write-only chip: the
// driver keeps the last commanded values.
//
// The outputs are not gated: the HAL does not know about them, so they keep their voltage while
// the robot is disabled or e-stopped. Do not use the DAC as the only command of an actuator that
// must stop on disable or e-stop. The destructor sets both outputs to 0 V.

#pragma once

#include <stdint.h>

#include <array>
#include <atomic>
#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/voltage.hpp"

namespace prosdk {

/**
 * GP8403 dual DAC (datasheet translated by StanStrong, sections 3.3.3 to 3.3.6).
 *
 * Code = round(V / full scale x 4095), sent left-justified (code << 4), low byte first. One
 * range for both channels. No thread: a setter writes the chip at once, and only when the code
 * changed (or the last write failed): about 75 us per channel at 400 kHz. Until the range write
 * has been acknowledged (for example the chip did not answer at construction), every setter
 * first repeats the safe sequence of SetRange(): both outputs 0 V, the range, then the stored
 * values. So no code is ever applied with an unknown range.
 */
class GP8403 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Output range (register 0x01; applies to both channels).
  enum class Range { k0to5V, k0to10V };

  /// Default 7-bit address: A2 = A1 = A0 = GND. 0x58 + (A2 << 2 | A1 << 1 | A0) up to 0x5F.
  static constexpr int kDefaultAddress = 0x58;
  /// Number of outputs (VOUT0, VOUT1).
  static constexpr int kNumChannels = 2;
  /// Largest code (12 bits).
  static constexpr int kMaxCode = 4095;

  /**
   * Constructs the driver, sets the range and both outputs to 0 V (the chip's power-on range and
   * value are not documented). A missing chip prints one console line; the constructor does not
   * throw for it.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address (0x58 to 0x5F).
   * @param range   The output range.
   */
  explicit GP8403(prosdk::I2C::Port port, int address = kDefaultAddress,
                  Range range = Range::k0to5V);

  /** Sets both outputs to 0 V. */
  ~GP8403() override;

  GP8403(const GP8403&) = delete;
  GP8403& operator=(const GP8403&) = delete;

  /**
   * Changes the range of both outputs. Both outputs go to 0 V first, then the range is written,
   * then the stored voltages (clamped to the new range) are written again, so no output
   * overshoots during the change.
   */
  void SetRange(Range range);

  /** The range last set. */
  Range GetRange() const;

  /**
   * Sets one output (channel 0 or 1). The voltage is clamped to 0 .. the range (NaN gives 0)
   * and rounded to the 12-bit code.
   */
  void SetVoltage(int channel, prosdk::units::volt_t voltage);

  /** Sets both outputs to the same voltage in one transfer (datasheet 3.3.5). */
  void SetBoth(prosdk::units::volt_t voltage);

  /** The last commanded voltage of a channel, after clamping and rounding to the code. */
  prosdk::units::volt_t GetVoltage(int channel) const;

  /** True when the last I2C transfer succeeded (the chip acknowledged). */
  bool IsConnected() const;

  /** The 7-bit I2C address. */
  int GetAddress() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void CheckChannel(int channel) const;
  uint16_t CodeFor(prosdk::units::volt_t voltage) const;
  double FullScale() const;
  bool WriteRange();
  bool WriteCodes(int first, int count, const uint16_t* codes);
  bool WriteAll();

  prosdk::I2C m_i2c;
  int m_address;

  mutable std::mutex m_mutex;  // the chip and the fields below
  Range m_range;
  std::array<uint16_t, kNumChannels> m_codes{};    // commanded
  std::array<bool, kNumChannels> m_written{};      // the chip holds m_codes[i]
  bool m_rangeWritten = false;                     // the chip acknowledged m_range
  std::atomic<bool> m_connected{false};
};

}  // namespace prosdk
