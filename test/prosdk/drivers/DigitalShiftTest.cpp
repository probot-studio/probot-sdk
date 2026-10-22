// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/bus/DigitalShift.hpp"

#include <stdint.h>

#include <array>
#include <memory>
#include <string_view>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/Value.h"
#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {

// HX711 serial interface (Avia datasheet Fig. 2): DOUT changes after each PD_SCK rising edge
// and holds the next bit, MSB first; after the 24 data bits DOUT goes high.
struct FakeHx711 {
  FakeHx711(int clock, int data) : clk{clock}, dout{data} {
    dout.SetValue(false);  // conversion ready
    cb = clk.RegisterValueCallback(
        [this](std::string_view, const HAL_Value* v) {
          if (v->data.v_boolean) {
            ++pulses;
            dout.SetValue(next < 24 ? ((word >> (23 - next)) & 1u) != 0 : true);
            ++next;
          }
        },
        false);
  }

  sim::DIOSim clk;
  sim::DIOSim dout;
  std::unique_ptr<sim::CallbackStore> cb;
  uint32_t word = 0;
  int next = 0;
  int pulses = 0;
};

}  // namespace

TEST_CASE("DigitalShift reads an HX711 word MSB first", "[digitalshift]") {
  DigitalOutput clock{21};
  DigitalInput data{22};
  FakeHx711 chip{21, 22};
  chip.word = 0xA5C3F1;
  DigitalShift shift{clock, data};
  CHECK(shift.GetClockChannel() == 21);
  CHECK(shift.GetDataChannel() == 22);

  auto value = shift.ShiftIn(24);
  REQUIRE(value.has_value());
  CHECK(*value == 0xA5C3F1u);
  // 25th pulse: channel A, gain 128 for the next conversion; DOUT goes high
  auto gain = shift.ShiftIn(1);
  REQUIRE(gain.has_value());
  CHECK(*gain == 1u);
  CHECK(chip.pulses == 25);
  CHECK_FALSE(clock.Get());  // the clock idles low (high > 60 us powers the HX711 down)
}

TEST_CASE("DigitalShift reads up to 64 bits", "[digitalshift]") {
  DigitalOutput clock{23};
  DigitalInput data{24};
  sim::DIOSim clk{23};
  sim::DIOSim din{24};
  int edges = 0;
  auto cb = clk.RegisterValueCallback(
      [&](std::string_view, const HAL_Value* v) {
        if (v->data.v_boolean) {
          din.SetValue(edges++ % 2 == 0);  // 1010...
        }
      },
      false);
  DigitalShift shift{clock, data, units::nanosecond_t{20000}, units::nanosecond_t{0}};
  auto value = shift.ShiftIn(64);
  REQUIRE(value.has_value());
  CHECK(*value == 0xAAAAAAAAAAAAAAAAull);
  CHECK(edges == 64);
}

TEST_CASE("DigitalShift argument and pin errors", "[digitalshift]") {
  DigitalOutput clock{25};
  DigitalInput data{26};
  CHECK_THROWS_AS((DigitalShift{clock, data, units::nanosecond_t{20001}}), RuntimeError);
  CHECK_THROWS_AS((DigitalShift{clock, data, units::nanosecond_t{0}}), RuntimeError);
  CHECK_THROWS_AS((DigitalShift{clock, data, units::nanosecond_t{1000}, units::nanosecond_t{2e6}}),
                  RuntimeError);

  DigitalShift shift{clock, data};
  CHECK_FALSE(shift.ShiftIn(0).has_value());
  CHECK_FALSE(shift.ShiftIn(65).has_value());

  // the clock must be a DIO output; the data pin an allocated DIO
  DigitalInput notOutput{27};
  int32_t status = 0;
  uint64_t out = 0;
  CHECK_FALSE(platform::hal::ShiftIn(27, 26, 8, 1000, 1000, &out, &status));
  CHECK(status != 0);
  status = 0;
  CHECK_FALSE(platform::hal::ShiftIn(25, 30, 8, 1000, 1000, &out, &status));  // 30: not allocated
  CHECK(status != 0);
}

TEST_CASE("WriteWaveform drives the levels and keeps the last one", "[digitalshift]") {
  DigitalOutput pin{28};
  sim::DIOSim dio{28};
  std::vector<bool> levels;
  auto cb = dio.RegisterValueCallback(
      [&](std::string_view, const HAL_Value* v) { levels.push_back(v->data.v_boolean); }, false);

  // a 3 us start pulse then 2 us low
  std::array<platform::hal::LevelSpan, 2> start{{{true, 3000}, {false, 2000}}};
  CHECK(WriteWaveform(pin, start));
  CHECK(levels == std::vector<bool>{true, false});
  CHECK_FALSE(dio.GetValue());

  std::array<platform::hal::LevelSpan, 1> high{{{true, 500}}};
  CHECK(WriteWaveform(pin, high));
  CHECK(dio.GetValue());

  // longer than 20 us in total: refused, the pin is left as it was
  levels.clear();
  std::array<platform::hal::LevelSpan, 2> tooLong{{{false, 15000}, {true, 6000}}};
  CHECK_FALSE(WriteWaveform(pin, tooLong));
  CHECK(levels.empty());
  CHECK(dio.GetValue());
  CHECK_FALSE(WriteWaveform(pin, std::span<const platform::hal::LevelSpan>{}));

  // not an output
  DigitalInput in{29};
  int32_t status = 0;
  CHECK_FALSE(platform::hal::WriteWaveform(29, high, &status));
  CHECK(status != 0);
}
