// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/sensor/PYD1598.hpp"

#include <stdint.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/Value.h"
#include "prosdk/hardware/discrete/DigitalInput.hpp"
#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/simulation/PulseIOSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {

// The SERIN side of the sensor (data sheet section 2.6): every low-to-high transition starts a
// bit, whose value is the level held after it (sampled 50 us later, inside tSHD 80-150 us).
struct FakeSerin {
  explicit FakeSerin(int serin, int dl) : dlSim{dl} {
    id = sim::OnPulsesSent(serin, [this](std::span<const Pulse> p, uint32_t carrier) {
      trains.emplace_back(p.begin(), p.end());
      carriers.push_back(carrier);
      // DL must be held low by the host during the configuration
      dlLowDuringSend.push_back(!dlSim.GetIsInput() && !dlSim.GetValue());
    });
  }
  ~FakeSerin() { sim::CancelPulsesSent(id); }

  // The word in the last train, or nothing if it is not exactly 25 bits.
  std::optional<uint32_t> LastWord() const {
    const auto& train = trains.back();
    std::vector<uint32_t> rises;
    uint32_t t = 0;
    bool level = false;  // the line idles low
    for (const Pulse& p : train) {
      if (p.level && !level) {
        rises.push_back(t);
      }
      level = p.level;
      t += p.us;
    }
    auto levelAt = [&](uint32_t when) {
      uint32_t start = 0;
      for (const Pulse& p : train) {
        if (when < start + p.us) {
          return p.level;
        }
        start += p.us;
      }
      return false;
    };
    if (rises.size() != 25) {
      return std::nullopt;
    }
    uint32_t word = 0;
    for (uint32_t r : rises) {
      word = (word << 1) | (levelAt(r + 50) ? 1u : 0u);
    }
    return word;
  }

  sim::DIOSim dlSim;
  int id = 0;
  std::vector<std::vector<Pulse>> trains;
  std::vector<uint32_t> carriers;
  std::vector<bool> dlLowDuringSend;
};

}  // namespace

TEST_CASE("PYD1598 configuration word", "[pyd1598]") {
  // default: threshold 32, blind 2 s (3), 2 pulses (1), window 4 s (1), wake up, PIR BPF,
  // reserved [4:3] = 2, HPF 0.4 Hz, count with sign change
  CHECK(PYD1598::EncodeConfig({}) == 0x406B10u);

  PYD1598::Config max;
  max.threshold = 255;
  max.blindTime = units::second_t{8.0};
  max.pulseCount = 4;
  max.windowTime = units::second_t{8.0};
  max.lowHighPassCutoff = true;
  max.countWithoutSignChange = true;
  CHECK(PYD1598::EncodeConfig(max) == 0x1FFFF15u);

  PYD1598::Config min;
  min.threshold = 0;
  min.blindTime = units::second_t{0.5};
  min.pulseCount = 1;
  min.windowTime = units::second_t{2.0};
  CHECK(PYD1598::EncodeConfig(min) == 0x110u);  // only the mode and the reserved bits

  PYD1598::Config rounded = min;
  rounded.blindTime = units::second_t{1.2};  // nearest step: 1 s
  CHECK(PYD1598::EncodeConfig(rounded) == (0x110u | (1u << 13)));

  auto bad = [](auto change) {
    PYD1598::Config c;
    change(c);
    return c;
  };
  CHECK_THROWS_AS(PYD1598::EncodeConfig(bad([](auto& c) { c.threshold = 256; })), RuntimeError);
  CHECK_THROWS_AS(PYD1598::EncodeConfig(bad([](auto& c) { c.threshold = -1; })), RuntimeError);
  CHECK_THROWS_AS(PYD1598::EncodeConfig(bad([](auto& c) { c.pulseCount = 0; })), RuntimeError);
  CHECK_THROWS_AS(PYD1598::EncodeConfig(bad([](auto& c) { c.pulseCount = 5; })), RuntimeError);
  CHECK_THROWS_AS(
      PYD1598::EncodeConfig(bad([](auto& c) { c.blindTime = units::second_t{0.4}; })),
      RuntimeError);
  CHECK_THROWS_AS(
      PYD1598::EncodeConfig(bad([](auto& c) { c.blindTime = units::second_t{8.5}; })),
      RuntimeError);
  CHECK_THROWS_AS(
      PYD1598::EncodeConfig(bad([](auto& c) { c.windowTime = units::second_t{1.0}; })),
      RuntimeError);
  CHECK_THROWS_AS(
      PYD1598::EncodeConfig(bad([](auto& c) { c.windowTime = units::second_t{9.0}; })),
      RuntimeError);
}

// DL channels stay below 31: the host sim HAL has DIO channels 0-30 only (PortsInternal.hpp);
// the SERIN pulse channels may use any GPIO 0-48.

TEST_CASE("PYD1598 writes the word on SERIN with DL held low", "[pyd1598]") {
  FakeSerin fake{30, 24};
  PYD1598 pyd{30, 24};
  REQUIRE(fake.trains.size() == 1);
  CHECK(fake.carriers[0] == 0u);  // plain levels, no carrier
  CHECK(fake.dlLowDuringSend[0]);
  CHECK(fake.LastWord() == std::optional<uint32_t>{0x406B10u});
  CHECK(fake.dlSim.GetIsInput());  // DL is an input again

  // SERIN timing: low for at least tSLT (650 us) before and after the word, a rising edge every
  // 102 us (tSL 1 us, tSH 1 us, tSHD 100 us), so no gap in the word exceeds 150 us
  const auto& train = fake.trains[0];
  CHECK_FALSE(train.front().level);
  CHECK(train.front().us >= 650u);
  CHECK_FALSE(train.back().level);
  CHECK(train.back().us >= 650u);
  uint32_t total = 0;
  for (const Pulse& p : train) {
    CHECK(p.us >= 1u);
    total += p.us;
  }
  CHECK(total == 700u + 25u * 102u + 700u);

  PYD1598::Config config;
  config.threshold = 0x80;
  config.pulseCount = 1;
  CHECK(pyd.Configure(config));
  REQUIRE(fake.trains.size() == 2);
  CHECK(fake.dlLowDuringSend[1]);
  CHECK(fake.LastWord() == std::optional<uint32_t>{PYD1598::EncodeConfig(config)});
  CHECK(pyd.GetConfig().threshold == 0x80);

  // a bad setting changes nothing and sends nothing
  PYD1598::Config bad = config;
  bad.pulseCount = 9;
  CHECK_THROWS_AS(pyd.Configure(bad), RuntimeError);
  CHECK(fake.trains.size() == 2);
  CHECK(pyd.GetConfig().pulseCount == 1);

  CHECK(pyd.Configure());  // the same word again
  REQUIRE(fake.trains.size() == 3);
  CHECK(fake.trains[2] == fake.trains[1]);
}

TEST_CASE("PYD1598 reports and clears motion on DL", "[pyd1598]") {
  PYD1598 pyd{32, 25};
  sim::DIOSim dl{25};
  dl.SetValue(false);  // the sensor holds DL low
  CHECK_FALSE(pyd.Poll());
  dl.SetValue(true);   // motion: the sensor drives DL high and leaves it high
  CHECK(pyd.Poll());
  CHECK(pyd.Poll());

  std::vector<bool> drivenLow;
  auto cb = dl.RegisterValueCallback(
      [&](std::string_view, const HAL_Value* v) {
        if (!dl.GetIsInput()) {
          drivenLow.push_back(!v->data.v_boolean);
        }
      },
      false);
  pyd.Clear();
  CHECK(std::find(drivenLow.begin(), drivenLow.end(), true) != drivenLow.end());
  CHECK(dl.GetIsInput());
  dl.SetValue(false);  // the sensor released the event
  CHECK_FALSE(pyd.Poll());
}

TEST_CASE("PYD1598 pin errors", "[pyd1598]") {
  PYD1598 pyd{34, 26};
  CHECK_THROWS_AS((PYD1598{34, 27}), RuntimeError);  // SERIN in use
  DigitalInput taken{28};
  CHECK_THROWS_AS((PYD1598{38, 28}), RuntimeError);  // DL in use
  PYD1598 again{38, 29};                             // the failed one freed SERIN 38
  PYD1598::Config bad;
  bad.threshold = 300;
  CHECK_THROWS_AS((PYD1598{40, 30, bad}), RuntimeError);
}
