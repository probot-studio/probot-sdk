// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Texas Instruments TMAG5170 3-axis linear Hall-effect sensor over prosdk::SPI (Boardoza TMAG5170
// breakout; design/boardoza/P10-tmag-leds.md). It measures the X, Y and Z field, the die
// temperature and, with its on-chip CORDIC, the angle of the field in one plane: with a
// diametrically magnetized magnet on a shaft it is a contactless absolute angle sensor.
//
// Every SPI frame is 32 bits with a 4-bit CRC, which stays on (the power-up default). The driver
// checks the CRC of every answer, counts mismatches (GetCrcErrors()) and keeps the last good
// values. The chip runs in continuous conversion of X, Y, Z and temperature; a reset of the chip
// (brown-out) is seen in the status bits and the configuration is written again.

#pragma once

#include <stdint.h>

#include <mutex>
#include <string_view>

#include "prosdk/drivers/bus/SPI.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/magnetic_field_strength.hpp"
#include "prosdk/units/temperature.hpp"

namespace prosdk {

/**
 * TMAG5170 3D Hall-effect sensor (A1 and A2 versions).
 *
 * No thread: every getter reads the chip (one 32-bit frame per register, a few microseconds at
 * the default clock) and returns the last good value when the read fails. The chip converts
 * continuously, much faster than a robot loop. A chip that was not configured (no answer at
 * construction) or that reset since (CFG_RESET status bit) is configured again before the read.
 */
class TMAG5170 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Measurement range of all three axes (X_RANGE, Y_RANGE, Z_RANGE).
  enum class Range {
    /// +/-25 mT (TMAG5170A1), +/-75 mT (TMAG5170A2).
    kLow,
    /// +/-50 mT (A1), +/-150 mT (A2); the power-up default.
    kMedium,
    /// +/-100 mT (A1), +/-300 mT (A2).
    kHigh
  };

  /// Samples averaged per result (CONV_AVG); more averaging lowers noise and the update rate.
  enum class Averaging {
    k1x,   ///< 10.0 ksps on 3 axes
    k2x,   ///< 5.7 ksps on 3 axes
    k4x,   ///< 3.1 ksps on 3 axes
    k8x,   ///< 1.6 ksps on 3 axes
    k16x,  ///< 0.8 ksps on 3 axes
    k32x   ///< 0.4 ksps on 3 axes
  };

  /// The two axes the on-chip angle is computed from (ANGLE_EN).
  enum class AnglePlane { kXY, kYZ, kXZ };

  /// A field reading.
  struct Field {
    prosdk::units::millitesla_t x{0};
    prosdk::units::millitesla_t y{0};
    prosdk::units::millitesla_t z{0};
  };

  /// Default SPI clock.
  static constexpr int kDefaultClockRate = 1'000'000;
  /// Highest SPI clock: fSPI 10 MHz maximum (datasheet 6.8).
  static constexpr int kMaxClockRate = 10'000'000;

  /**
   * Opens the device (SPI mode 0), reads the version and configures it: Range::kMedium,
   * Averaging::k8x, angle in the XY plane, continuous conversion. Does not throw for a missing
   * chip: it prints one console line and IsConnected() is false.
   *
   * @param bus     The SPI bus (prosdk::platform::SetSPIPins sets its pins).
   * @param csGpio  The chip-select GPIO (the board's nCS).
   * @param clockHz The SPI clock, clamped to SPI::kMinClockRate to kMaxClockRate.
   * @throws prosdk::RuntimeError when the SPI device cannot be opened (see prosdk::SPI).
   */
  TMAG5170(prosdk::SPI::Port bus, int csGpio, int clockHz = kDefaultClockRate);

  TMAG5170(const TMAG5170&) = delete;
  TMAG5170& operator=(const TMAG5170&) = delete;

  /**
   * Writes the configuration and reads it back.
   *
   * @return false when the chip did not answer or the read-back differed.
   */
  bool Configure(Range range, Averaging averaging, AnglePlane plane = AnglePlane::kXY);

  /** Reads X, Y and Z (three frames; they can come from consecutive conversions). */
  Field GetField();

  /** Reads the die temperature (typical calibration: 25 C at 17522, 60 LSB per C). */
  prosdk::units::celsius_t GetTemperature();

  /**
   * Reads the on-chip angle of the field in the configured plane, 0 to 360 degrees in 1/16
   * degree steps (the CORDIC resolves 1/4 degree). Meaningless without a magnet.
   */
  prosdk::units::degree_t GetAngle();

  /** The range in effect, in mT, for the chip version read at the last configuration. */
  prosdk::units::millitesla_t GetRange() const;

  /**
   * True when the last frame had a valid CRC, the version read at the last configuration was A1
   * or A2 and the configuration read back correctly.
   */
  bool IsConnected() const;

  /** Number of answers with a wrong CRC since construction. */
  int GetCrcErrors() const;

  /**
   * CRC of an SPI frame (datasheet 7.5.2.5): polynomial x^4 + x + 1, initial value 1111b, over
   * bits 31..4 followed by four zero bits. The low 4 bits of frame are ignored.
   */
  static uint8_t Crc4(uint32_t frame);

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool TransferLocked(uint32_t frame, uint32_t* response);
  bool ReadRegisterLocked(int reg, uint16_t* value);
  bool WriteRegisterLocked(int reg, uint16_t value);
  bool ApplyConfigLocked();
  void EnsureConfiguredLocked();

  prosdk::SPI m_spi;
  int m_csGpio;

  mutable std::mutex m_mutex;
  bool m_versionOk = false;
  bool m_isA2 = false;
  bool m_configured = false;
  bool m_lastFrameOk = false;
  bool m_resetSeen = false;  // CFG_RESET in a status since the last configuration
  int m_crcErrors = 0;
  Range m_range = Range::kMedium;
  Averaging m_averaging = Averaging::k8x;
  AnglePlane m_plane = AnglePlane::kXY;
  Field m_field;
  double m_temperature = 0.0;  // C, last good reading
  double m_angle = 0.0;        // degrees, last good reading
};

}  // namespace prosdk
