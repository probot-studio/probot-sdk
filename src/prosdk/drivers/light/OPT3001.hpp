// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// TI OPT3001 ambient light sensor over prosdk::I2C (Boardoza OPT3001 breakout;
// design/boardoza/P2-light-thermal.md). Continuous conversions, 800 ms each, with automatic
// full-scale range (0.01 lux to 83865.6 lux). The INT pin is not used.

#pragma once

#include <stdint.h>

#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/illuminance.hpp"

namespace prosdk {

/**
 * OPT3001 ambient light sensor.
 *
 * No thread: call Update() from a periodic method (two 2-byte reads); GetLux() returns the value
 * it read. The chip makes a new conversion every 800 ms, so updating faster repeats the value.
 */
class OPT3001 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default 7-bit I2C address (ADDR to GND; 0x45 VDD, 0x46 SDA, 0x47 SCL).
  static constexpr int kDefaultAddress = 0x44;

  /**
   * Constructs the driver, checks the manufacturer and device IDs and starts continuous
   * conversions. A missing chip prints one console line; Update() then retries the setup.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address.
   */
  explicit OPT3001(prosdk::I2C::Port port, int address = kDefaultAddress);

  OPT3001(const OPT3001&) = delete;
  OPT3001& operator=(const OPT3001&) = delete;

  /** Reads the latest conversion. Restarts conversions if the chip was reset (power loss). */
  void Update();

  /** Illuminance from the last good Update() (0 before one). */
  prosdk::units::lux_t GetLux() const;

  /** True when the IDs matched and the last transfer succeeded. */
  bool IsConnected() const;

  /** Lux value of a result register word (datasheet equation 3): 0.01 * 2^E[15:12] * R[11:0]. */
  static double ResultToLux(uint16_t result);

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool Configure();

  prosdk::I2C m_i2c;
  mutable std::mutex m_mutex;
  bool m_configured = false;
  bool m_connected = false;
  double m_lux = 0.0;
};

}  // namespace prosdk
