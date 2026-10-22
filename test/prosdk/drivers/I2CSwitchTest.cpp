// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/bus/I2CSwitch.hpp"

#include <cstdint>
#include <span>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/drivers/rtc/PCF8563.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {
// A TPT29545 at 0x70 with a PCF8563 (0x51) behind channel 1: the RTC answers only while
// channel 1 is selected. Read bits B7-B4 are undefined on the TPT29545; the fake sets them.
struct SwitchWithRtc {
  explicit SwitchWithRtc(test::FakeI2CBus& bus) : sw{bus.Add(0x70)}, rtc{bus.Add(0x51)} {
    rtc.nack = true;
    sw.onWrite = [this](std::span<const uint8_t> data) {
      if (!data.empty()) {
        control = data.back();  // the last byte of a write is kept
        rtc.nack = (control & 0x02) == 0;
      }
    };
    sw.onRead = [this](std::span<uint8_t> data) {
      for (auto& byte : data) {
        byte = static_cast<uint8_t>(control | 0xF0);
      }
    };
  }
  test::FakeI2CBus::Device& sw;
  test::FakeI2CBus::Device& rtc;
  uint8_t control = 0x0F;
};
}  // namespace

TEST_CASE("I2CSwitch writes the channel mask", "[i2cswitch]") {
  test::FakeI2CBus bus;
  SwitchWithRtc board{bus};
  I2CSwitch mux{I2C::Port::PORT_0};
  CHECK(mux.IsConnected());
  CHECK(mux.GetAddress() == 0x70);
  CHECK(mux.GetChannels() == 4);
  REQUIRE(board.sw.writes.size() == 1);
  CHECK(board.sw.writes[0] == std::vector<uint8_t>{0x00});  // all deselected
  CHECK(board.control == 0x00);

  CHECK(mux.SelectOnly(2));
  CHECK(board.sw.writes.back() == std::vector<uint8_t>{0x04});  // one byte, no register
  CHECK(mux.Select(0x09));
  CHECK(board.control == 0x09);
  auto selected = mux.GetSelected();
  REQUIRE(selected.has_value());
  CHECK(*selected == 0x09);  // upper bits masked off

  CHECK_THROWS_AS(mux.Select(0x10), RuntimeError);
  CHECK_THROWS_AS(mux.SelectOnly(4), RuntimeError);
  CHECK_THROWS_AS(mux.SelectOnly(-1), RuntimeError);
  CHECK(board.control == 0x09);  // nothing written for a bad argument
  CHECK_THROWS_AS((I2CSwitch{I2C::Port::PORT_0, 0x70, 9}), RuntimeError);
  CHECK_THROWS_AS((I2CSwitch{I2C::Port::PORT_0, 0x70, 0}), RuntimeError);

  I2CSwitch eight{I2C::Port::PORT_0, 0x70, 8};
  CHECK(eight.Select(0x80));
  CHECK(board.control == 0x80);
}

TEST_CASE("I2CSwitch reaches a device behind a channel", "[i2cswitch]") {
  test::FakeI2CBus bus;
  SwitchWithRtc board{bus};
  board.rtc.regs[0x02] = 0x30;  // 2026-01-01 00:00:30, VL clear
  board.rtc.regs[0x05] = 0x01;
  board.rtc.regs[0x07] = 0x01;
  board.rtc.regs[0x08] = 0x26;
  I2CSwitch mux{I2C::Port::PORT_0};

  REQUIRE(mux.SelectOnly(1));
  PCF8563 clock{I2C::Port::PORT_0};
  CHECK(clock.IsConnected());
  auto time = clock.GetTime();
  REQUIRE(time.has_value());
  CHECK(time->tm_sec == 30);

  REQUIRE(mux.Select(0x05));  // channel 1 off
  CHECK_FALSE(clock.GetTime().has_value());
  CHECK_FALSE(clock.IsConnected());

  REQUIRE(mux.SelectOnly(1));
  CHECK(clock.GetTime().has_value());
  CHECK(clock.IsConnected());
}

TEST_CASE("I2CSwitch reports a missing switch", "[i2cswitch]") {
  test::FakeI2CBus bus;
  I2CSwitch mux{I2C::Port::PORT_0, 0x73};
  CHECK_FALSE(mux.IsConnected());
  CHECK_FALSE(mux.SelectOnly(0));
  CHECK_FALSE(mux.GetSelected().has_value());
}
