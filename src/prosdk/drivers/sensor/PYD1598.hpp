// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Excelitas PYD 1598 low-power DigiPyro (dual-element PIR motion sensor with an on-chip motion
// detection unit) in its wake-up mode (design/boardoza/P12-pulse.md). The host writes a 25-bit
// configuration word on SERIN (here through a prosdk::PulseTransmitter without a carrier: the
// word takes about 2.6 ms, too long for interrupt-masked bit-banging) and then watches the
// bidirectional DIRECT LINK (DL) line: the sensor drives DL high when its motion criteria are met
// and keeps it high until the host pulls it low. The forced and interrupt readouts of the raw
// ADC values are not supported.
//
// The chip is 1.8-3.6 V (3.6 V absolute maximum on every pin): SERIN and DL connect directly to
// ESP32-S3 GPIOs only when the sensor runs from 3.3 V.

#pragma once

#include <stdint.h>

#include <mutex>
#include <optional>
#include <string_view>

#include "prosdk/drivers/bus/PulseTransmitter.hpp"
#include "prosdk/hardware/discrete/DigitalInput.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/**
 * PYD 1598 motion sensor in wake-up mode. Written from the PYD 1588 specification, which
 * Excelitas' brochure gives the same electrical data and functions (unverified on a PYD 1598);
 * the PYD 1588 works the same way.
 *
 * No thread: Poll() reads DL. After a motion event DL stays high until Clear(), which also starts
 * the blind time. A GPIO cannot be a DigitalInput and a DigitalOutput at once, so Configure() and
 * Clear() free the DL input, drive DL low through a DigitalOutput and then take the input back.
 *
 * Not thread-safe except the telemetry: use it from one thread.
 *
 * Resources: one of the 3 RMT TX channels (SERIN) and one DIO input (DL).
 */
class PYD1598 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /**
   * Motion detection settings (data sheet section 2.5). The defaults are a starting point for
   * tests, not values from the data sheet: tune the threshold on the robot.
   */
  struct Config {
    /// Detection threshold on the band-pass filtered signal, 0 to 255 (its scale against the
    /// 14-bit ADC counts is not given in the data sheet).
    int threshold = 32;
    /// Time after Clear() in which no motion is reported: 0.5 to 8 s in 0.5 s steps (rounded
    /// to the nearest step).
    prosdk::units::second_t blindTime{2.0};
    /// Threshold crossings needed within the window time: 1 to 4.
    int pulseCount = 2;
    /// Moving window for the pulse counter: 2, 4, 6 or 8 s (rounded to the nearest).
    prosdk::units::second_t windowTime{4.0};
    /// High-pass cut-off of the band-pass filter: false 0.4 Hz, true 0.2 Hz (long distances).
    bool lowHighPassCutoff = false;
    /// False: a crossing only counts after the filtered signal changed sign; true: every one.
    bool countWithoutSignChange = false;
  };

  /// How long DL is driven low to clear an event (data sheet: at least 160 us).
  static constexpr prosdk::units::microsecond_t kClearTime{200};

  /**
   * Opens the pins and configures the sensor with the default Config.
   *
   * @param serinChannel GPIO of SERIN.
   * @param dlChannel    GPIO of DIRECT LINK.
   * @throws prosdk::RuntimeError when a pin is in use or no RMT TX channel is free.
   */
  PYD1598(int serinChannel, int dlChannel);

  /**
   * Opens the pins and configures the sensor.
   *
   * @throws prosdk::RuntimeError (ParameterOutOfRange) for a setting outside its range, or when
   *         a pin is in use or no RMT TX channel is free.
   */
  PYD1598(int serinChannel, int dlChannel, const Config& config);

  PYD1598(const PYD1598&) = delete;
  PYD1598& operator=(const PYD1598&) = delete;

  /**
   * Writes the configuration word again (the sensor's configuration is volatile: after a power
   * cycle of the sensor it is undefined). Holds DL low while doing so, which also clears a
   * pending event. Blocks for about 5 ms.
   *
   * @return False when the word could not be sent (reported to the console). There is no read
   *         back, so true does not prove that a sensor is connected.
   */
  bool Configure();

  /**
   * Replaces the settings and writes them (see Configure()).
   *
   * @throws prosdk::RuntimeError (ParameterOutOfRange) for a setting outside its range; the
   *         settings then stay unchanged.
   */
  bool Configure(const Config& config);

  /** The settings last given. */
  const Config& GetConfig() const;

  /** True while a motion event is pending (DL high). */
  bool Poll();

  /** Clears a pending event (DL low for kClearTime) and starts the blind time. */
  void Clear();

  /**
   * The 25-bit configuration word for wake-up mode (data sheet Table 5), bit 24 first on SERIN.
   *
   * @throws prosdk::RuntimeError (ParameterOutOfRange) for a setting outside its range.
   */
  static uint32_t EncodeConfig(const Config& config);

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool HoldDlLow(uint32_t word, bool send);

  int m_dlChannel;
  Config m_config;
  uint32_t m_word;
  PulseTransmitter m_serin;
  std::optional<DigitalInput> m_dl;
  bool m_warnedFloating = false;

  mutable std::mutex m_mutex;  // telemetry copies
  bool m_motion = false;
  bool m_configured = false;
};

}  // namespace prosdk
