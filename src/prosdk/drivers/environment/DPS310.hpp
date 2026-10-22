// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Infineon DPS310-compatible barometric pressure sensor over prosdk::I2C (Boardoza HP303B board;
// the HP303B data sheet V1.0 describes the DPS310 register map; design/boardoza/P1-climate.md).
// 300 to 1200 hPa. The driver runs the chip in continuous (background) mode, 8 pressure and 8
// temperature measurements per second at 8 times oversampling, and compensates on the host.

#pragma once

#include <stdint.h>

#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/pressure.hpp"
#include "prosdk/units/temperature.hpp"

namespace prosdk {

/**
 * DPS310 (HP303B) barometer.
 *
 * No thread: call Update() periodically (for example every robot loop). It never waits. The first
 * calls check the product ID, wait for the chip's start-up (up to 40 ms), read the calibration
 * coefficients and start continuous measurement; later calls read new results when the chip
 * flags them (one status read, plus one 6-byte read when there is a result). After a failure the
 * next attempt is at least 1 s later.
 */
class DPS310 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// 7-bit I2C address with SDO floating or high (data sheet 4.5 and 6.1); 0x76 with SDO low.
  static constexpr int kDefaultAddress = 0x77;

  /**
   * Constructs the driver. It does not talk to the chip; the first Update() does.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address.
   */
  explicit DPS310(prosdk::I2C::Port port, int address = kDefaultAddress);

  DPS310(const DPS310&) = delete;
  DPS310& operator=(const DPS310&) = delete;

  /** Advances start-up or reads new results (non-blocking). */
  void Update();

  /** Pressure of the last compensated sample (0 before one). */
  prosdk::units::pascal_t GetPressure() const;

  /** Temperature of the last sample (0 before one). */
  prosdk::units::celsius_t GetTemperature() const;

  /**
   * Altitude from the last pressure with the international barometric formula,
   * h = 44330 m * (1 - (p / p0)^(1 / 5.255)); 0 before the first sample.
   *
   * @param seaLevel The sea-level pressure p0 (101325 Pa for the standard atmosphere).
   */
  prosdk::units::meter_t GetAltitude(
      prosdk::units::pascal_t seaLevel = prosdk::units::pascal_t{101325.0}) const;

  /** True once a compensated pressure exists. */
  bool HasData() const;

  /** True when the last transfer succeeded and the product ID matched. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  enum class State { kStart, kWaitReady, kRunning };

  bool Configure();
  void Fail(double now);

  prosdk::I2C m_i2c;
  int m_address;
  mutable std::mutex m_mutex;
  State m_state = State::kStart;
  double m_stateTime = 0.0;  // s, monotonic; when the current state began
  bool m_everTried = false;
  bool m_warnedId = false;
  bool m_connected = false;
  bool m_hasTemperature = false;
  bool m_hasData = false;
  // calibration coefficients (data sheet Table 18)
  int32_t m_c0 = 0, m_c1 = 0, m_c00 = 0, m_c10 = 0, m_c01 = 0, m_c11 = 0, m_c20 = 0, m_c21 = 0,
          m_c30 = 0;
  double m_tempScaled = 0.0;   // Traw / kT of the last temperature
  double m_pressure = 0.0;     // Pa
  double m_temperature = 0.0;  // C
};

}  // namespace prosdk
