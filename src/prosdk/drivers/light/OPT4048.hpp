// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// TI OPT4048 tristimulus color sensor over prosdk::I2C (Boardoza OPT4048 breakout;
// design/boardoza/P2-light-thermal.md). Continuous conversions with automatic full-scale range,
// 100 ms per channel (400 ms for the 4 channels). Every channel's CRC is checked. The INT pin is
// not used.

#pragma once

#include <stdint.h>

#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/illuminance.hpp"

namespace prosdk {

/**
 * OPT4048 color sensor.
 *
 * No thread: call Update() from a periodic method (one 16-byte read and one 2-byte read); the
 * getters return what it read. A sample whose CRC does not match is dropped and counted.
 */
class OPT4048 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default 7-bit I2C address (ADDR to GND; 0x45 VDD, 0x46 SDA, 0x47 SCL).
  static constexpr int kDefaultAddress = 0x44;

  /// CIE 1931 tristimulus values (the datasheet's example matrix; Y is not scaled to lux).
  struct XYZ {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  /// CIE 1931 chromaticity coordinates.
  struct Chromaticity {
    double x = 0.0;
    double y = 0.0;
  };

  /// One channel's output registers, decoded.
  struct Channel {
    uint32_t mantissa = 0;  // 20 bits
    uint8_t exponent = 0;   // 4 bits
    uint8_t counter = 0;    // 4 bits
    uint8_t crc = 0;        // 4 bits
  };

  /**
   * Constructs the driver, checks the device ID and starts continuous conversions. A missing
   * chip prints one console line; Update() then retries the setup.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address.
   */
  explicit OPT4048(prosdk::I2C::Port port, int address = kDefaultAddress);

  OPT4048(const OPT4048&) = delete;
  OPT4048& operator=(const OPT4048&) = delete;

  /** Reads the 4 channels. Restarts conversions if the chip was reset (power loss). */
  void Update();

  /** Tristimulus values of the last good sample. */
  XYZ GetXYZ() const;

  /** Illuminance of the last good sample (channel 1). */
  prosdk::units::lux_t GetLux() const;

  /** Chromaticity of the last good sample ((0, 0) in darkness). */
  Chromaticity GetCIExy() const;

  /** Number of samples dropped for a CRC mismatch. */
  uint32_t GetCrcErrors() const;

  /** True when the device ID matched and the last transfer succeeded. */
  bool IsConnected() const;

  /** Decodes a channel from its two registers (MSB register first). */
  static Channel DecodeChannel(uint16_t msbRegister, uint16_t lsbRegister);

  /** The 4-bit CRC the chip should send for a channel (datasheet Figure 8-12). */
  static uint8_t ChannelCrc(const Channel& channel);

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool Configure();

  prosdk::I2C m_i2c;
  mutable std::mutex m_mutex;
  bool m_configured = false;
  bool m_idLogged = false;  // used by Configure() only
  bool m_connected = false;
  uint32_t m_crcErrors = 0;
  XYZ m_xyz;
  double m_lux = 0.0;
};

}  // namespace prosdk
