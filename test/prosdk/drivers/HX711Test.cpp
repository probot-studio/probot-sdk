// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/adc/HX711.hpp"

#include <stdint.h>

#include <memory>
#include <optional>
#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/Value.h"
#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {

constexpr int kClock = 11;
constexpr int kData = 12;

// HX711 on the DIO sim (Avia data sheet, Serial Interface and Fig. 2): DOUT low means a
// conversion is ready; each PD_SCK rising edge puts the next bit on DOUT, MSB first; from the
// 25th pulse on DOUT is high. The pulse count of a readout (25, 26 or 27) selects the input and
// gain of the next conversion.
struct FakeHx711 {
  FakeHx711() : clk{kClock}, dout{kData} {
    dout.SetValue(true);  // no conversion ready
    cb = clk.RegisterValueCallback(
        [this](std::string_view, const HAL_Value* v) {
          if (v->data.v_boolean) {
            ++pulses;
            dout.SetValue(pulses <= 24 ? ((word >> (24 - pulses)) & 1u) != 0 : true);
          }
        },
        false);
  }

  // A new conversion is ready.
  void Ready(uint32_t value) {
    word = value & 0xFFFFFF;
    pulses = 0;
    dout.SetValue(false);
  }

  sim::DIOSim clk;
  sim::DIOSim dout;
  std::unique_ptr<sim::CallbackStore> cb;
  uint32_t word = 0;
  int pulses = 0;  // since the last Ready()
};

// Feeds conversions until one comes through; returns how many were dropped, or -1.
int DroppedUntilValue(FakeHx711& chip, HX711& hx, uint32_t value, int expectedPulses) {
  for (int i = 0; i < 10; ++i) {
    chip.Ready(value);
    auto sample = hx.Update();
    if (chip.pulses != expectedPulses) {
      return -1;
    }
    if (sample) {
      return i;
    }
  }
  return -1;
}

}  // namespace

TEST_CASE("HX711 reads conversions with 25 pulses for channel A gain 128", "[hx711]") {
  HX711 hx{kData, kClock};
  FakeHx711 chip;
  CHECK(hx.GetGain() == HX711::Gain::k128A);
  CHECK_FALSE(hx.IsConnected());

  // the chip's selection is unknown at first: that conversion plus 4 settling ones are dropped
  CHECK(DroppedUntilValue(chip, hx, 0x123456, 25) == 5);
  CHECK(hx.GetRaw() == 0x123456);
  CHECK(hx.IsConnected());
  CHECK_FALSE(chip.clk.GetValue());  // PD_SCK idles low

  chip.Ready(0x7FFFFF);
  CHECK(hx.Update() == std::optional<int32_t>{8388607});
  chip.Ready(0x800000);
  CHECK(hx.Update() == std::optional<int32_t>{-8388608});
  chip.Ready(0xFFFFFF);
  CHECK(hx.Update() == std::optional<int32_t>{-1});
  CHECK(chip.pulses == 25);
  CHECK(hx.GetRaw() == -1);

  // DOUT high: nothing is clocked
  CHECK_FALSE(hx.Update().has_value());
  CHECK(chip.pulses == 25);
}

TEST_CASE("HX711 selects the gain with 26 and 27 pulses", "[hx711]") {
  HX711 hx{kData, kClock, HX711::Gain::k64A};
  FakeHx711 chip;
  CHECK(DroppedUntilValue(chip, hx, 1000, 27) == 5);

  hx.SetGain(HX711::Gain::k32B);
  CHECK(hx.GetGain() == HX711::Gain::k32B);
  // the read that sends 26 pulses still returns a gain-64 conversion, then 4 settle
  CHECK(DroppedUntilValue(chip, hx, 2000, 26) == 5);
  CHECK(hx.GetRaw() == 2000);

  hx.SetGain(HX711::Gain::k128A);
  CHECK(DroppedUntilValue(chip, hx, 3000, 25) == 5);
  hx.SetGain(HX711::Gain::k128A);  // unchanged: nothing dropped
  CHECK(DroppedUntilValue(chip, hx, 3001, 25) == 0);
}

TEST_CASE("HX711 is not connected without conversions or with DOUT stuck low", "[hx711]") {
  sim::PauseTiming();
  {
    HX711 hx{kData, kClock};
    FakeHx711 chip;
    CHECK(DroppedUntilValue(chip, hx, 7, 25) == 5);
    CHECK(hx.IsConnected());
    sim::StepTiming(0.4_s);
    CHECK_FALSE(hx.Update().has_value());
    CHECK(hx.IsConnected());
    sim::StepTiming(0.2_s);  // 0.6 s without a ready conversion
    CHECK_FALSE(hx.Update().has_value());
    CHECK_FALSE(hx.IsConnected());
    chip.Ready(8);
    CHECK(hx.Update() == std::optional<int32_t>{8});
    CHECK(hx.IsConnected());
  }
  {
    // no chip, DOUT low: the bits after the data read 0 instead of 1
    HX711 hx{kData, kClock};
    sim::DIOSim dout{kData};
    dout.SetValue(false);
    for (int i = 0; i < 8; ++i) {
      CHECK_FALSE(hx.Update().has_value());
    }
    CHECK_FALSE(hx.IsConnected());
    CHECK(hx.GetRaw() == 0);
  }
  sim::ResumeTiming();
}

TEST_CASE("HX711 power down and up", "[hx711]") {
  HX711 hx{kData, kClock};
  FakeHx711 chip;
  CHECK(DroppedUntilValue(chip, hx, 10, 25) == 5);

  hx.PowerDown();
  CHECK(chip.clk.GetValue());  // PD_SCK high (power-down after 60 us)
  chip.Ready(11);
  CHECK_FALSE(hx.Update().has_value());
  CHECK(chip.pulses == 0);  // nothing clocked while powered down

  hx.PowerUp();
  CHECK_FALSE(chip.clk.GetValue());
  // the chip reset to channel A gain 128: only the 4 settling conversions are dropped
  CHECK(DroppedUntilValue(chip, hx, 12, 25) == 4);

  hx.SetGain(HX711::Gain::k64A);
  hx.PowerDown();
  hx.PowerUp();
  // reset to gain 128, so the first read switches the gain: 1 + 4 dropped
  CHECK(DroppedUntilValue(chip, hx, 13, 27) == 5);
}
