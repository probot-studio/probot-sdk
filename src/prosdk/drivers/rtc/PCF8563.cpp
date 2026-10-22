// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// PCF8563 driver (PCF8563.hpp). Registers: NXP PCF8563 product data sheet rev. 10, table 4
// (overview), tables 5 to 17 (control, time and date registers), table 22 (CLKOUT_control);
// reading and setting the time: sections 8.5 and 8.10.

#include "prosdk/drivers/rtc/PCF8563.hpp"

#include <stdint.h>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kRegControl1 = 0x00;  // TEST1, STOP, TESTC
constexpr uint8_t kRegControl2 = 0x01;  // TI_TP, AF, TF, AIE, TIE
constexpr uint8_t kRegSeconds = 0x02;   // VL_seconds, then minutes .. years (7 registers)
constexpr uint8_t kRegClkout = 0x0D;    // FE, FD[1:0]

constexpr uint8_t kControl1Stop = 0x20;
constexpr uint8_t kVoltageLow = 0x80;   // VL bit of VL_seconds

uint8_t ToBcd(int value) {
  return static_cast<uint8_t>(((value / 10) << 4) | (value % 10));
}

// -1 for a nibble above 9.
int FromBcd(uint8_t bcd) {
  int tens = bcd >> 4;
  int units = bcd & 0x0F;
  return tens > 9 || units > 9 ? -1 : tens * 10 + units;
}

bool IsLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

int DaysInMonth(int year, int month) {  // month 1 to 12
  static constexpr int kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  return month == 2 && IsLeapYear(year) ? 29 : kDays[month - 1];
}

// 0 = Sunday (Sakamoto's method).
int DayOfWeek(int year, int month, int day) {
  static constexpr int kOffsets[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (month < 3) {
    year -= 1;
  }
  return (year + year / 4 - year / 100 + year / 400 + kOffsets[month - 1] + day) % 7;
}

int DayOfYear(int year, int month, int day) {  // 0 = 1 January
  int days = day - 1;
  for (int m = 1; m < month; ++m) {
    days += DaysInMonth(year, m);
  }
  return days;
}

}  // namespace

PCF8563::PCF8563(I2C::Port port) : m_i2c{port, kAddress} {
  std::scoped_lock lock{m_mutex};
  // Control_status_1/2 in one burst (auto-increment); N bits written 0 (table 4)
  uint8_t control[] = {kRegControl1, 0x00, 0x00};
  bool aborted = m_i2c.WriteBulk(control, sizeof(control));
  aborted = aborted || m_i2c.Write(kRegClkout, 0x00);  // FE = 0: CLKOUT high-impedance
  m_connected = !aborted;
  if (aborted) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] PCF8563 0x{:02X}: no answer on I2C port {}\n", kAddress,
                           static_cast<int>(port));
  }
  util::ReportUsage("PCF8563", kAddress, "");
}

std::optional<std::tm> PCF8563::GetTime() {
  uint8_t regs[7];
  {
    std::scoped_lock lock{m_mutex};
    bool ok = !m_i2c.Read(kRegSeconds, sizeof(regs), regs);
    m_connected = ok;
    if (!ok) {
      return std::nullopt;
    }
  }
  bool valid = (regs[0] & kVoltageLow) == 0;
  m_timeValid = valid;
  if (!valid) {
    return std::nullopt;
  }
  int second = FromBcd(regs[0] & 0x7F);
  int minute = FromBcd(regs[1] & 0x7F);
  int hour = FromBcd(regs[2] & 0x3F);
  int day = FromBcd(regs[3] & 0x3F);
  int month = FromBcd(regs[5] & 0x1F);  // bit 7 is the century flag C
  int year = FromBcd(regs[6]);
  if (second < 0 || second > 59 || minute < 0 || minute > 59 || hour < 0 || hour > 23 ||
      month < 1 || month > 12 || year < 0 || day < 1 || day > DaysInMonth(2000 + year, month)) {
    return std::nullopt;
  }
  std::tm time{};
  time.tm_sec = second;
  time.tm_min = minute;
  time.tm_hour = hour;
  time.tm_mday = day;
  time.tm_mon = month - 1;
  time.tm_year = 100 + year;
  time.tm_wday = DayOfWeek(2000 + year, month, day);
  time.tm_yday = DayOfYear(2000 + year, month, day);
  time.tm_isdst = 0;
  return time;
}

bool PCF8563::SetTime(const std::tm& time) {
  int year = 1900 + time.tm_year;
  int month = time.tm_mon + 1;
  if (year < 2000 || year > 2099 || month < 1 || month > 12 || time.tm_mday < 1 ||
      time.tm_mday > DaysInMonth(year, month) || time.tm_hour < 0 || time.tm_hour > 23 ||
      time.tm_min < 0 || time.tm_min > 59 || time.tm_sec < 0 || time.tm_sec > 60) {
    throw WPILIB_MakeError(err::ParameterOutOfRange,
                           "PCF8563 time {:04}-{:02}-{:02} {:02}:{:02}:{:02} (valid: 2000 to 2099)",
                           year, month, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
  }
  int second = time.tm_sec > 59 ? 59 : time.tm_sec;
  uint8_t regs[] = {kRegSeconds,
                    ToBcd(second),  // VL = 0
                    ToBcd(time.tm_min),
                    ToBcd(time.tm_hour),
                    ToBcd(time.tm_mday),
                    static_cast<uint8_t>(DayOfWeek(year, month, time.tm_mday)),
                    ToBcd(month),  // C = 0
                    ToBcd(year - 2000)};
  std::scoped_lock lock{m_mutex};
  // STOP, write the time in one burst, release STOP (section 8.10)
  bool aborted = m_i2c.Write(kRegControl1, kControl1Stop);
  aborted = aborted || m_i2c.WriteBulk(regs, sizeof(regs));
  aborted = m_i2c.Write(kRegControl1, 0x00) || aborted;  // always try to restart the clock
  m_connected = !aborted;
  if (!aborted) {
    m_timeValid = true;
  }
  return !aborted;
}

bool PCF8563::IsConnected() const {
  return m_connected;
}

void PCF8563::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Time Valid", m_timeValid.load());
  table.Log("Connected", m_connected.load());
}

std::string_view PCF8563::GetTelemetryType() const {
  return "Real-Time Clock";
}
