// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Clocked serial input and short waveforms on DIO pins with interrupt-safe timing
// (design/boardoza/F3-timed-io.md): a thin wrapper over prosdk/platform/hal/BitBang.hpp for
// drivers that own their clock DigitalOutput and data DigitalInput, such as an HX711 load-cell
// ADC. On the robot interrupts are masked only during each clock-high phase (at most 20 us); on
// the host the pins are the DIO sim channels, so a test emulates the chip from DIOSim value
// callbacks.

#pragma once

#include <stdint.h>

#include <optional>
#include <span>

#include "prosdk/hardware/discrete/DigitalInput.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/platform/hal/BitBang.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/**
 * MSB-first clocked input on two DIO pins: per bit the clock goes high for the high time, the
 * data pin is sampled at the end of it, then the clock goes low for the low time. The clock
 * idles low. The pins stay owned by the caller and must stay allocated while this object is used
 * (it keeps their channel numbers only). A clock pin with a digital PWM generator
 * (DigitalOutput::EnablePWM) is not supported.
 */
class DigitalShift {
 public:
  /// Default clock-high time (the HX711's typical T3).
  static constexpr prosdk::units::nanosecond_t kDefaultHighTime{1000};
  /// Default clock-low time (the HX711's typical T4).
  static constexpr prosdk::units::nanosecond_t kDefaultLowTime{1000};

  /**
   * @param clock    The clock output (owned by the caller).
   * @param data     The data input (owned by the caller).
   * @param highTime Clock-high time, 1 ns to 20 us (platform::hal::kMaxMaskedNs).
   * @param lowTime  Clock-low time after each bit, 0 to 1 ms.
   * @throws prosdk::RuntimeError (ParameterOutOfRange) for times outside these ranges.
   */
  DigitalShift(DigitalOutput& clock, DigitalInput& data,
               prosdk::units::nanosecond_t highTime = kDefaultHighTime,
               prosdk::units::nanosecond_t lowTime = kDefaultLowTime)
      : m_clock{clock.GetChannel()},
        m_data{data.GetChannel()},
        m_highNs{ToNs(highTime)},
        m_lowNs{ToNs(lowTime)} {
    if (const char* error = platform::hal::detail::CheckShiftIn(1, m_highNs, m_lowNs)) {
      throw WPILIB_MakeError(err::ParameterOutOfRange, "{} (clock {}, data {})", error, m_clock,
                             m_data);
    }
  }

  /**
   * Clocks `bits` bits in (1 to 64), MSB first.
   *
   * @return The bits (the first one in the most significant position), or nothing after an
   *         error (reported to the console: bad bit count, a pin that is no longer an allocated
   *         DIO, or an emergency stop).
   */
  std::optional<uint64_t> ShiftIn(int bits) {
    uint64_t value = 0;
    int32_t status = 0;
    if (!platform::hal::ShiftIn(m_clock, m_data, bits, m_highNs, m_lowNs, &value, &status)) {
      WPILIB_ReportError(status, "ShiftIn of {} bits (clock {}, data {})", bits, m_clock, m_data);
      return std::nullopt;
    }
    return value;
  }

  /** The clock GPIO. */
  int GetClockChannel() const { return m_clock; }

  /** The data GPIO. */
  int GetDataChannel() const { return m_data; }

 private:
  static uint32_t ToNs(prosdk::units::nanosecond_t t) {
    double ns = t.value();
    // out-of-range and NaN times map to a value the range check refuses
    return ns >= 0.0 && ns <= 4e9 ? static_cast<uint32_t>(ns + 0.5) : UINT32_MAX;
  }

  int m_clock;
  int m_data;
  uint32_t m_highNs;
  uint32_t m_lowNs;
};

/**
 * Drives `pin` through a fixed waveform with interrupts masked for its whole length: at most
 * 64 spans and 20 us in total (longer waveforms: PulseTransmitter). The pin keeps the last
 * span's level. Errors (too long, not an output, emergency stop) are reported to the console.
 *
 * @return True when the waveform was written.
 */
inline bool WriteWaveform(DigitalOutput& pin, std::span<const platform::hal::LevelSpan> spans) {
  int32_t status = 0;
  if (!platform::hal::WriteWaveform(pin.GetChannel(), spans, &status)) {
    WPILIB_ReportError(status, "WriteWaveform on GPIO {}", pin.GetChannel());
    return false;
  }
  return true;
}

}  // namespace prosdk
