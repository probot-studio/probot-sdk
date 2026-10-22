// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ENS210-compatible temperature and relative humidity sensor over prosdk::I2C (Boardoza TH09C
// board, whose TH09C data sheet V1.01 describes the ENS210 register map; design/boardoza/
// P1-climate.md). The driver runs both sensors in continuous mode (one T and RH pair about every
// 225 ms) and checks the CRC-7 and valid flag of every reading.

#pragma once

#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/temperature.hpp"

namespace prosdk {

/**
 * ENS210 (TH09C) temperature and humidity sensor.
 *
 * No thread: call Update() periodically (for example every robot loop). It never waits. The first
 * calls start continuous measurement and check PART_ID, later calls read the latest values (one
 * 6-byte transfer). After a failed transfer the next start attempt is at least 1 s later.
 */
class ENS210 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// The fixed 7-bit I2C address (TH09C data sheet, "The I2C Slave Address").
  static constexpr int kDefaultAddress = 0x43;

  /// PART_ID register value (data sheet Figure 23).
  static constexpr int kPartId = 0x0210;

  /**
   * Constructs the driver. It does not talk to the chip; the first Update() does.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address.
   */
  explicit ENS210(prosdk::I2C::Port port, int address = kDefaultAddress);

  ENS210(const ENS210&) = delete;
  ENS210& operator=(const ENS210&) = delete;

  /** Starts the sensor or reads its latest values (non-blocking). */
  void Update();

  /** Temperature of the last valid reading (0 before one). */
  prosdk::units::celsius_t GetTemperature() const;

  /** Relative humidity in percent of the last valid reading (0 before one). */
  double GetHumidity() const;

  /** True once a valid temperature and humidity pair has been read. */
  bool HasData() const;

  /** Number of readings dropped because their CRC-7 did not match. */
  int GetCrcErrors() const;

  /** True when the last transfer succeeded. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  enum class State { kStart, kWaitActive, kRunning };

  void Fail(double now);

  prosdk::I2C m_i2c;
  int m_address;
  mutable std::mutex m_mutex;
  State m_state = State::kStart;
  double m_stateTime = 0.0;  // s, monotonic; when the current state began
  bool m_everTried = false;
  bool m_warnedPartId = false;
  bool m_connected = false;
  bool m_hasData = false;
  int m_crcErrors = 0;
  double m_temperature = 0.0;  // C
  double m_humidity = 0.0;     // %RH
};

}  // namespace prosdk
