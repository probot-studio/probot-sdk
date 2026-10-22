// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/rtc/PCF8563.hpp"

#include <cstdint>
#include <ctime>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {
std::tm Date(int year, int month, int day, int hour, int minute, int second) {
  std::tm t{};
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  t.tm_wday = 6;  // ignored: the driver computes the weekday
  return t;
}
}  // namespace

TEST_CASE("PCF8563 configures the chip and reports the VL flag", "[pcf8563]") {
  test::FakeI2CBus bus;
  auto& rtc = bus.Add(PCF8563::kAddress);
  rtc.regs[0x00] = 0x08;  // reset values (datasheet table 27): TESTC
  rtc.regs[0x0D] = 0x80;  // CLKOUT on, 32.768 kHz
  rtc.regs[0x02] = 0x80;  // VL: no valid time
  PCF8563 clock{I2C::Port::PORT_0};
  CHECK(clock.IsConnected());
  CHECK(rtc.regs[0x00] == 0x00);
  CHECK(rtc.regs[0x01] == 0x00);
  CHECK(rtc.regs[0x0D] == 0x00);  // CLKOUT off
  CHECK_FALSE(clock.GetTime().has_value());
  CHECK(clock.IsConnected());
}

TEST_CASE("PCF8563 BCD round trip", "[pcf8563]") {
  test::FakeI2CBus bus;
  auto& rtc = bus.Add(PCF8563::kAddress);
  rtc.regs[0x02] = 0x80;
  PCF8563 clock{I2C::Port::PORT_0};

  rtc.writes.clear();
  REQUIRE(clock.SetTime(Date(2026, 10, 22, 14, 5, 59)));
  REQUIRE(rtc.writes.size() == 3);
  CHECK(rtc.writes[0] == std::vector<uint8_t>{0x00, 0x20});  // STOP
  CHECK(rtc.writes[1] ==
        std::vector<uint8_t>{0x02, 0x59, 0x05, 0x14, 0x22, 4, 0x10, 0x26});  // Thursday
  CHECK(rtc.writes[2] == std::vector<uint8_t>{0x00, 0x00});  // released

  auto time = clock.GetTime();
  REQUIRE(time.has_value());
  CHECK(time->tm_year == 126);
  CHECK(time->tm_mon == 9);
  CHECK(time->tm_mday == 22);
  CHECK(time->tm_hour == 14);
  CHECK(time->tm_min == 5);
  CHECK(time->tm_sec == 59);
  CHECK(time->tm_wday == 4);
  CHECK(time->tm_yday == 294);
  CHECK(time->tm_isdst == 0);

  rtc.regs[0x07] |= 0x80;  // century flag: ignored
  rtc.regs[0x03] |= 0x80;  // unused bit of Minutes: masked
  time = clock.GetTime();
  REQUIRE(time.has_value());
  CHECK(time->tm_year == 126);
  CHECK(time->tm_min == 5);

  REQUIRE(clock.SetTime(Date(2028, 2, 29, 23, 59, 60)));  // leap day; leap second as 59
  CHECK(rtc.regs[0x02] == 0x59);
  CHECK(rtc.regs[0x05] == 0x29);
  CHECK(rtc.regs[0x06] == 2);  // Tuesday
  CHECK(rtc.regs[0x07] == 0x02);
  CHECK(rtc.regs[0x08] == 0x28);

  rtc.regs[0x03] = 0x7A;  // not BCD
  CHECK_FALSE(clock.GetTime().has_value());
  rtc.regs[0x03] = 0x00;
  rtc.regs[0x05] = 0x30;  // 30 February
  CHECK_FALSE(clock.GetTime().has_value());
}

TEST_CASE("PCF8563 rejects invalid times and a missing chip", "[pcf8563]") {
  test::FakeI2CBus bus;
  PCF8563 missing{I2C::Port::PORT_0};
  CHECK_FALSE(missing.IsConnected());
  CHECK_FALSE(missing.GetTime().has_value());
  CHECK_FALSE(missing.SetTime(Date(2030, 1, 1, 0, 0, 0)));

  CHECK_THROWS_AS(missing.SetTime(Date(1999, 12, 31, 0, 0, 0)), RuntimeError);
  CHECK_THROWS_AS(missing.SetTime(Date(2100, 1, 1, 0, 0, 0)), RuntimeError);
  CHECK_THROWS_AS(missing.SetTime(Date(2027, 2, 29, 0, 0, 0)), RuntimeError);
  CHECK_THROWS_AS(missing.SetTime(Date(2027, 13, 1, 0, 0, 0)), RuntimeError);
  CHECK_THROWS_AS(missing.SetTime(Date(2027, 1, 1, 24, 0, 0)), RuntimeError);
  CHECK_THROWS_AS(missing.SetTime(Date(2027, 1, 1, 0, 60, 0)), RuntimeError);
}
