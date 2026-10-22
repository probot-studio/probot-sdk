// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// NXP PCF8563 real-time clock over prosdk::I2C, for example the Boardoza PCF8563T breakout with
// its coin-cell backup (design/boardoza/P5-i2c-misc.md). Date and time only: no alarm, timer or
// clock output (the constructor turns CLKOUT off). Years 2000 to 2099; the century flag is
// written 0 and ignored on read.

#pragma once

#include <atomic>
#include <ctime>
#include <mutex>
#include <optional>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * NXP PCF8563 real-time clock/calendar (datasheet rev. 10, 3 April 2012).
 *
 * No thread: GetTime() reads the chip (one 7-byte transfer, about 0.25 ms at 400 kHz). Call it
 * when the time is needed, not every robot loop.
 */
class PCF8563 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// 7-bit I2C address (fixed; A2h write / A3h read, datasheet section 9.5.1).
  static constexpr int kAddress = 0x51;

  /**
   * Constructs the driver: clears the test and STOP bits (Control_status_1 = 0), disables the
   * alarm and timer interrupts (Control_status_2 = 0) and turns CLKOUT off. The time is kept. A
   * missing chip prints one console line; the constructor does not throw for it.
   *
   * @param port The I2C port (prosdk::platform::SetI2CPins sets its pins).
   */
  explicit PCF8563(prosdk::I2C::Port port);

  PCF8563(const PCF8563&) = delete;
  PCF8563& operator=(const PCF8563&) = delete;

  /**
   * Reads the date and time (one burst of registers 02h to 08h, as section 8.5 requires).
   * tm_wday and tm_yday are computed from the date; tm_isdst is 0. Empty when the chip does not
   * answer, when the VL flag is set (the clock lost power: no battery; the time is invalid until
   * SetTime()), or when a register holds an impossible value.
   */
  std::optional<std::tm> GetTime();

  /**
   * Sets the date and time from tm_year (100 to 199: 2000 to 2099), tm_mon, tm_mday, tm_hour,
   * tm_min and tm_sec (a leap second 60 is written as 59); tm_wday is computed. The clock is
   * stopped while the registers are written and restarts on release; its first tick follows
   * about 0.508 s later (section 8.10). Clears the VL flag. Throws for an invalid field. Returns
   * false when the chip did not answer.
   */
  bool SetTime(const std::tm& time);

  /** True when the last I2C transfer succeeded. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  prosdk::I2C m_i2c;

  std::mutex m_mutex;  // the chip
  std::atomic<bool> m_connected{false};
  std::atomic<bool> m_timeValid{false};  // VL was 0 in the last GetTime()
};

}  // namespace prosdk
