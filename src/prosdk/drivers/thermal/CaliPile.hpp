// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Excelitas CaliPile thermopile sensors over prosdk::I2C: TPiS 1S 1385 (presence and motion) and
// TPiS 1T 1386 L5.5H (non-contact temperature), Boardoza breakouts
// (design/boardoza/P2-light-thermal.md). Object and ambient temperature from the factory
// calibration in the chip's EEPROM, plus the chip's presence, motion and over-temperature flags.
// The INT pin is not used.

#pragma once

#include <stdint.h>

#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/temperature.hpp"

namespace prosdk {

/**
 * CaliPile thermopile sensor (TPiS 1S 1385, TPiS 1T 1386).
 *
 * No thread: call Update() from a periodic method; the getters return what it read. After
 * power-up the chip answers only the I2C general call, so the first Update() sends the general
 * call reload (Reload()), reads the EEPROM calibration and writes the full configuration. A
 * failed setup is retried once per second. Later Updates read registers 1 to 19 (19 bytes); a
 * failed read starts the setup again (the chip may have lost power).
 *
 * The general call goes to address 0x00, so every chip on the bus that answers general calls sees
 * the 0x04 byte (for most chips a no-op; the reset command is 0x06).
 */
class CaliPile : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// The sensor type. It only checks the EEPROM look-up identifier; the EEPROM decides the math.
  enum class Variant {
    k1S1385,  ///< TPiS 1S 1385: LOOKUP# 1, f(x) = x^3.8
    k1T1386,  ///< TPiS 1T 1386 L5.5H: LOOKUP# 2, f(x) = x^4.2
  };

  /// Default 7-bit I2C address (EEPROM 0x8C with A1 = A0 = 0; 0x0C to 0x0F with A1, A0).
  static constexpr int kDefaultAddress = 0x0C;

  /**
   * Constructs the driver. No bus access happens here (see Update()).
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param variant The sensor type.
   * @param address The 7-bit I2C address.
   */
  CaliPile(prosdk::I2C::Port port, Variant variant, int address = kDefaultAddress);

  CaliPile(const CaliPile&) = delete;
  CaliPile& operator=(const CaliPile&) = delete;

  /** Sets the chip up if needed, then reads the measurement and status registers. */
  void Update();

  /**
   * Sends the general call reload (address 0x00, byte 0x04) and waits 300 us, so the chip loads
   * its address from EEPROM and answers it. Update() calls it on the first setup.
   *
   * @return true when the general call was acknowledged.
   */
  bool Reload();

  /** Object temperature of the last good Update() (0 before one). */
  prosdk::units::celsius_t GetObjectTemperature() const;

  /** The sensor's own temperature of the last good Update() (0 before one). */
  prosdk::units::celsius_t GetAmbientTemperature() const;

  /** The chip's presence flag (chip status bit 3). */
  bool IsPresence() const;

  /** The chip's motion flag (chip status bit 2). */
  bool IsMotion() const;

  /**
   * The chip's over-temperature flag (chip status bit 4): the object is above the threshold set
   * with SetOverTemperatureThreshold(). False while no threshold is set.
   */
  bool IsOverTemperature() const;

  /**
   * Object emissivity times field-of-view coverage, 0 < value <= 1 (default 1, the calibration
   * condition).
   */
  void SetEmissivity(double emissivity);

  /**
   * Sets the over-temperature threshold. The chip compares raw thermopile counts, which depend on
   * the ambient temperature too, so Update() rewrites the threshold when the ambient temperature
   * moves it by 8 counts or more.
   */
  void SetOverTemperatureThreshold(prosdk::units::celsius_t threshold);

  /** True when the setup succeeded and the last transfer succeeded. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool Init();
  uint16_t ThresholdRegister(double objectKelvin, double ambientKelvin) const;

  prosdk::I2C m_i2c;
  prosdk::I2C m_generalCall;  // address 0x00
  Variant m_variant;
  mutable std::mutex m_mutex;
  bool m_initialized = false;
  bool m_connected = false;
  bool m_warned = false;
  double m_lastInitAttempt = -1.0;  // s, monotonic
  // calibration (EEPROM)
  double m_exponent = 3.8;
  double m_ptat25 = 0.0;
  double m_slope = 1.0;  // M, counts per K
  double m_u0 = 0.0;
  double m_k = 1.0;  // before the emissivity
  // settings
  double m_emissivity = 1.0;
  double m_threshold = -1.0;  // K; negative: none
  bool m_thresholdDirty = false;
  uint16_t m_thresholdRegister = 0xFFFF;  // as written to registers 28-29
  // last reading
  double m_objectK = 0.0;
  double m_ambientK = 0.0;
  bool m_haveReading = false;
  uint8_t m_chipStatus = 0;
};

}  // namespace prosdk
