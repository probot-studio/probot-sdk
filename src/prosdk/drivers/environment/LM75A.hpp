// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// NXP LM75A digital temperature sensor over prosdk::I2C (Boardoza LM75ADP breakout;
// design/boardoza/P1-climate.md). 11-bit two's complement, 0.125 C per LSB, a new conversion every
// 100 ms. The over-temperature output OS keeps its power-on setting (comparator, active low,
// Tos 80 C, Thyst 75 C); read it with a prosdk::DigitalInput if needed.

#pragma once

#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/temperature.hpp"

namespace prosdk {

/**
 * LM75A temperature sensor.
 *
 * No thread: GetTemperature() reads the chip (one 2-byte transfer). The chip converts every
 * 100 ms, so reading faster returns the same value.
 */
class LM75A : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default 7-bit I2C address (A2 = A1 = A0 = 0; 0x48 to 0x4F with the address pads).
  static constexpr int kDefaultAddress = 0x48;

  /**
   * Constructs the driver and puts the chip in normal mode with its power-on configuration.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address.
   */
  explicit LM75A(prosdk::I2C::Port port, int address = kDefaultAddress);

  LM75A(const LM75A&) = delete;
  LM75A& operator=(const LM75A&) = delete;

  /** Reads the temperature. On a failed read it returns the last good value (0 before one). */
  prosdk::units::celsius_t GetTemperature();

  /**
   * Shutdown mode (about 3.5 uA): conversions stop and the temperature register keeps its last
   * value. The first new value is ready about 100 ms after leaving shutdown.
   */
  void SetShutdown(bool shutdown);

  /** True when the last transfer succeeded. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  prosdk::I2C m_i2c;
  mutable std::mutex m_mutex;
  bool m_connected = false;
  double m_temperature = 0.0;  // C, last good reading
};

}  // namespace prosdk
