// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// TE Connectivity (MEAS) MS5611-01BA barometric pressure sensor over prosdk::I2C (Boardoza MS5611
// board; design/boardoza/P1-climate.md). 10 to 1200 mbar, 24-bit ADC; the driver reads the
// factory calibration PROM (checked with its CRC-4) and applies the data sheet's first and
// second order compensation in integer arithmetic.

#pragma once

#include <stdint.h>

#include <mutex>
#include <span>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/pressure.hpp"
#include "prosdk/units/temperature.hpp"

namespace prosdk {

/**
 * MS5611 barometer.
 *
 * No thread: call Update() periodically (for example every robot loop). It never waits. Each call
 * reads the finished conversion (if its time is up) and starts the next one, alternating
 * temperature (D2) and pressure (D1), so one compensated sample needs two calls. The first calls
 * reset the chip and read its PROM; after a failure the next attempt is at least 1 s later.
 */
class MS5611 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// 7-bit I2C address with CSB low (the board default); 0x76 with CSB high (data sheet, I2C
  /// interface: address 111011C, C = complement of CSB).
  static constexpr int kDefaultAddress = 0x77;

  /// ADC oversampling ratio. Conversion time (max) 0.60, 1.17, 2.28, 4.54 and 9.04 ms; resolution
  /// improves with the ratio.
  enum class Osr { k256, k512, k1024, k2048, k4096 };

  /**
   * Constructs the driver. It does not talk to the chip; the first Update() does.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address.
   * @param osr     The oversampling ratio for both conversions.
   */
  explicit MS5611(prosdk::I2C::Port port, int address = kDefaultAddress, Osr osr = Osr::k4096);

  MS5611(const MS5611&) = delete;
  MS5611& operator=(const MS5611&) = delete;

  /** Advances the reset / PROM / conversion sequence (non-blocking). */
  void Update();

  /** Pressure of the last compensated sample (0 before one). */
  prosdk::units::pascal_t GetPressure() const;

  /** Temperature of the last compensated sample (0 before one). */
  prosdk::units::celsius_t GetTemperature() const;

  /**
   * Altitude from the last pressure with the international barometric formula,
   * h = 44330 m * (1 - (p / p0)^(1 / 5.255)); 0 before the first sample.
   *
   * @param seaLevel The sea-level pressure p0 (101325 Pa for the standard atmosphere).
   */
  prosdk::units::meter_t GetAltitude(
      prosdk::units::pascal_t seaLevel = prosdk::units::pascal_t{101325.0}) const;

  /** True once a compensated sample exists. */
  bool HasData() const;

  /** True when the last transfer succeeded and the PROM CRC matched. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

  /**
   * CRC-4 of the 8 PROM words (Application Note AN520): the low 4 bits of word 7 are the CRC and
   * are treated as 0, as is the rest of that word's low byte.
   */
  static int PromCrc(std::span<const uint16_t, 8> prom);

  /**
   * First and second order compensation (data sheet Figures 2 and 3).
   *
   * @param prom        The 8 PROM words (C1 to C6 in words 1 to 6).
   * @param d1          The raw pressure.
   * @param d2          The raw temperature.
   * @param temperature Set to the temperature in 0.01 C.
   * @param pressure    Set to the pressure in Pa (0.01 mbar).
   */
  static void Compensate(std::span<const uint16_t, 8> prom, uint32_t d1, uint32_t d2,
                         int32_t& temperature, int32_t& pressure);

 private:
  enum class State { kReset, kLoadProm, kReadD2, kReadD1 };

  bool Command(uint8_t command);
  bool ReadAdc(uint32_t& value);
  void Fail(double now);

  prosdk::I2C m_i2c;
  int m_address;
  Osr m_osr;
  mutable std::mutex m_mutex;
  State m_state = State::kReset;
  double m_stateTime = 0.0;  // s, monotonic; when the current state began
  bool m_everTried = false;
  bool m_warnedCrc = false;
  bool m_connected = false;
  bool m_hasData = false;
  uint16_t m_prom[8] = {};
  uint32_t m_d2 = 0;
  double m_pressure = 0.0;     // Pa
  double m_temperature = 0.0;  // C
};

}  // namespace prosdk
