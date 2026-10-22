// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Avia HX711 24-bit load-cell ADC on two DIO pins (Boardoza HX711 breakout;
// design/boardoza/P6-loadcell.md). No registers: DOUT goes low when a conversion is ready, then
// 25 to 27 PD_SCK pulses shift the 24-bit result out and select the input and gain of the next
// conversion. The pulses are clocked with prosdk::DigitalShift (interrupts masked per clock-high
// phase on the robot), because PD_SCK high for more than 60 us powers the chip down.
// For load cells, feed Update() into a prosdk::LoadCell.

#pragma once

#include <stdint.h>

#include <mutex>
#include <optional>
#include <string_view>

#include "prosdk/drivers/bus/DigitalShift.hpp"
#include "prosdk/hardware/discrete/DigitalInput.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * HX711 load-cell ADC.
 *
 * No thread: call Update() periodically (for example every robot loop). It never waits for the
 * chip: it reads a conversion only when DOUT is already low (about 60 us of clocking). The chip
 * converts at 10 or 80 samples per second (its RATE pin); a slower loop gets the latest
 * conversion.
 *
 * The first conversions after construction, PowerUp() or a gain change are dropped while the
 * input settles: 4 conversion periods (data sheet: output settling time 400 ms at 10 SPS, 50 ms
 * at 80 SPS), plus the one read that switches the gain.
 */
class HX711 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /** Input and gain of the conversions (data sheet Table 3). */
  enum class Gain {
    k128A,  ///< channel A, gain 128 (25 pulses; the chip's reset default)
    k64A,   ///< channel A, gain 64 (27 pulses)
    k32B,   ///< channel B, gain 32 (26 pulses)
  };

  /// IsConnected() turns false when no conversion was ready for this long (5 periods at 10 SPS).
  static constexpr double kReadyTimeoutSeconds = 0.5;

  /**
   * Allocates the pins and drives PD_SCK low (normal operation).
   *
   * @param dataGpio  GPIO wired to DOUT (the board's DAT pin), a DigitalInput.
   * @param clockGpio GPIO wired to PD_SCK (the board's CLK pin), a DigitalOutput.
   * @param gain      Input and gain.
   */
  HX711(int dataGpio, int clockGpio, Gain gain = Gain::k128A);

  HX711(const HX711&) = delete;
  HX711& operator=(const HX711&) = delete;

  /** Selects the input and gain; it takes effect from the next read (see the class notes). */
  void SetGain(Gain gain);

  /** The selected input and gain. */
  Gain GetGain() const;

  /**
   * Reads a conversion if DOUT is low (non-blocking).
   *
   * @return The new signed 24-bit result (-8388608 to 8388607; the chip saturates at both
   *         ends), or nothing when no conversion is ready, while settling, while powered down, or
   *         when the read failed. After a clocking error (an emergency stop) Update() returns
   *         nothing until reboot.
   */
  std::optional<int32_t> Update();

  /** The last result Update() returned (0 before the first). */
  int32_t GetRaw() const;

  /**
   * True after a valid read, until a read fails or Update() finds no conversion ready for
   * kReadyTimeoutSeconds (for example a missing board). False before the first read.
   */
  bool IsConnected() const;

  /**
   * Powers the chip down (below 1 uA): PD_SCK stays high, which takes effect after 60 us. Update()
   * does nothing until PowerUp().
   */
  void PowerDown();

  /** Powers the chip up: PD_SCK low resets it to channel A, gain 128; settling follows. */
  void PowerUp();

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  prosdk::DigitalOutput m_clock;
  prosdk::DigitalInput m_data;
  prosdk::DigitalShift m_shift;

  mutable std::mutex m_mutex;
  Gain m_gain;
  int m_chipPulses = 0;  // pulse count that selected the conversion in progress; 0: unknown
  int m_discard = 0;     // conversions still to drop
  bool m_poweredDown = false;
  bool m_shiftFailed = false;  // DigitalShift failed (emergency stop): no more clocking
  bool m_connected = false;
  double m_lastReadyTime = 0.0;  // s, monotonic
  int32_t m_raw = 0;
};

}  // namespace prosdk
