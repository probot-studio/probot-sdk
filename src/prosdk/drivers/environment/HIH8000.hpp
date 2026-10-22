// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Honeywell HumidIcon HIH8000 series (HIH8120/8121/8130/8131, I2C versions) humidity and
// temperature sensor over prosdk::I2C (Boardoza HIH8131 board; design/boardoza/P1-climate.md).
// The chip measures only on request: a Measurement Request (an empty write) starts one humidity and
// temperature cycle, a Data Fetch (a 4-byte read) returns the result with two status bits.

#pragma once

#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/temperature.hpp"

namespace prosdk {

/**
 * HIH8000 humidity and temperature sensor.
 *
 * No thread: call Update() periodically (for example every robot loop). It never waits: it sends a
 * measurement request, and a later call (at least 40 ms on) fetches the result and sends the next
 * request. One sample takes two calls, so a 20 ms loop gives a new sample about every 40 ms.
 */
class HIH8000 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// The chip's factory default 7-bit I2C address (Honeywell technical note 009061-2-EN, section
  /// 2.1); the HIH8131 board has no documentation, so its address is unverified.
  static constexpr int kDefaultAddress = 0x27;

  /**
   * Constructs the driver. It does not talk to the chip; the first Update() does.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address.
   */
  explicit HIH8000(prosdk::I2C::Port port, int address = kDefaultAddress);

  HIH8000(const HIH8000&) = delete;
  HIH8000& operator=(const HIH8000&) = delete;

  /** Requests a measurement or fetches a finished one (non-blocking). */
  void Update();

  /** Relative humidity in percent (0 to 100) of the last valid sample. */
  double GetHumidity() const;

  /** Temperature of the last valid sample. */
  prosdk::units::celsius_t GetTemperature() const;

  /**
   * True before the first valid sample, and when the last valid sample is more than 1 s old (the
   * chip stopped answering or keeps returning stale data).
   */
  bool IsStale() const;

  /** True when the last transfer succeeded. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool Request(double now);

  prosdk::I2C m_i2c;
  mutable std::mutex m_mutex;
  bool m_connected = false;
  bool m_requested = false;
  double m_requestTime = 0.0;    // s, monotonic
  double m_sampleTime = -1.0;    // s, monotonic; -1: no valid sample yet
  double m_humidity = 0.0;       // %RH
  double m_temperature = 0.0;    // C
};

}  // namespace prosdk
