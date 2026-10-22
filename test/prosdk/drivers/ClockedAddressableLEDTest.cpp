// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/led/ClockedAddressableLED.hpp"

#include <array>
#include <cstdint>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeSPIDevice.hpp"
#include "prosdk/simulation/SpiSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {
using Bytes = std::vector<uint8_t>;
using LEDData = AddressableLED::LEDData;

// The frame of the T3A33BRG datasheet (v2.0, pages 9-10): 32 zero bits, per LED
// 0xE0 | dimming, B, G, R, then 32 one bits and N/2 more clocks (ceil(N / 16) bytes of ones).
Bytes Frame(const std::vector<std::array<uint8_t, 4>>& words, int extraBytes) {
  Bytes frame{0x00, 0x00, 0x00, 0x00};
  for (const auto& word : words) {
    frame.insert(frame.end(), word.begin(), word.end());
  }
  frame.insert(frame.end(), 4 + extraBytes, 0xFF);
  return frame;
}
}  // namespace

TEST_CASE("ClockedAddressableLED opens a mode 0 device without chip select", "[clockedled]") {
  test::FakeSPIDevice strip{1, -1};
  {
    ClockedAddressableLED leds{SPI::kPort1, 4};
    platform::hal::SpiDeviceConfig config;
    REQUIRE(sim::GetSpiDeviceConfig(1, -1, &config));
    CHECK(config.mode == 0);
    CHECK(config.clockHz == ClockedAddressableLED::kDefaultClockRate);
    CHECK(leds.GetLength() == 4);
    CHECK(leds.GetBrightness() == 31);
    // all off at construction
    REQUIRE(strip.frames.size() == 1);
    CHECK(strip.frames[0] == Frame({{0xFF, 0, 0, 0}, {0xFF, 0, 0, 0}, {0xFF, 0, 0, 0},
                                    {0xFF, 0, 0, 0}},
                                   1));
    // the bus has no room for another device
    CHECK_THROWS_AS((SPI{SPI::kPort1, 7}), RuntimeError);
  }
  {
    ClockedAddressableLED leds{SPI::kPort1, 1, 40'000'000};  // above CIN's 15 MHz
    platform::hal::SpiDeviceConfig config;
    REQUIRE(sim::GetSpiDeviceConfig(1, -1, &config));
    CHECK(config.clockHz == ClockedAddressableLED::kMaxClockRate);
  }
  {
    ClockedAddressableLED leds{SPI::kPort1, 1, 1000};  // below the SPI minimum
    platform::hal::SpiDeviceConfig config;
    REQUIRE(sim::GetSpiDeviceConfig(1, -1, &config));
    CHECK(config.clockHz == SPI::kMinClockRate);
  }
}

TEST_CASE("ClockedAddressableLED frame for 1 LED", "[clockedled]") {
  test::FakeSPIDevice strip{1, -1};
  ClockedAddressableLED leds{SPI::kPort1, 1};
  leds.SetData({LEDData{0x12, 0x34, 0x56}});  // r, g, b
  REQUIRE(strip.frames.size() == 2);
  // 4 + 4 + 4 + 1 bytes; blue, green, red on the wire
  CHECK(strip.frames[1] == Bytes{0x00, 0x00, 0x00, 0x00, 0xFF, 0x56, 0x34, 0x12,
                                 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
}

TEST_CASE("ClockedAddressableLED frame for 4 LEDs and brightness", "[clockedled]") {
  test::FakeSPIDevice strip{1, -1};
  ClockedAddressableLED leds{SPI::kPort1, 4};
  const std::array<LEDData, 4> colors{LEDData{255, 0, 0}, LEDData{0, 255, 0}, LEDData{0, 0, 255},
                                      LEDData{1, 2, 3}};
  leds.SetData(colors);
  CHECK(strip.frames.back() == Frame({{0xFF, 0x00, 0x00, 0xFF},
                                      {0xFF, 0x00, 0xFF, 0x00},
                                      {0xFF, 0xFF, 0x00, 0x00},
                                      {0xFF, 0x03, 0x02, 0x01}},
                                     1));
  CHECK(strip.frames.back().size() == 25);

  // the brightness goes into the low 5 bits of every first byte and resends the colours
  leds.SetBrightness(4);
  CHECK(leds.GetBrightness() == 4);
  CHECK(strip.frames.back() == Frame({{0xE4, 0x00, 0x00, 0xFF},
                                      {0xE4, 0x00, 0xFF, 0x00},
                                      {0xE4, 0xFF, 0x00, 0x00},
                                      {0xE4, 0x03, 0x02, 0x01}},
                                     1));
  leds.SetBrightness(0);
  CHECK(strip.frames.back()[4] == 0xE0);  // never the 0xA0 sleep header
  leds.SetBrightness(99);
  CHECK(leds.GetBrightness() == 31);
  CHECK(strip.frames.back()[4] == 0xFF);
  leds.SetBrightness(-3);
  CHECK(leds.GetBrightness() == 0);

  // a shorter buffer turns the rest off; a longer one is cut at the length
  leds.SetBrightness(31);
  leds.SetData({LEDData{9, 8, 7}});
  CHECK(strip.frames.back() == Frame({{0xFF, 7, 8, 9}, {0xFF, 0, 0, 0}, {0xFF, 0, 0, 0},
                                      {0xFF, 0, 0, 0}},
                                     1));
  const std::array<LEDData, 6> longer{LEDData{1, 1, 1}, LEDData{2, 2, 2}, LEDData{3, 3, 3},
                                      LEDData{4, 4, 4}, LEDData{5, 5, 5}, LEDData{6, 6, 6}};
  leds.SetData(longer);
  CHECK(strip.frames.back() == Frame({{0xFF, 1, 1, 1}, {0xFF, 2, 2, 2}, {0xFF, 3, 3, 3},
                                      {0xFF, 4, 4, 4}},
                                     1));
}

TEST_CASE("ClockedAddressableLED frame for 24 LEDs", "[clockedled]") {
  test::FakeSPIDevice strip{1, -1};
  ClockedAddressableLED leds{SPI::kPort1, 24};
  std::array<LEDData, 24> colors;
  std::vector<std::array<uint8_t, 4>> words;
  for (int i = 0; i < 24; ++i) {
    colors[i] = LEDData{i, 2 * i, 3 * i};
    words.push_back({0xFF, static_cast<uint8_t>(3 * i), static_cast<uint8_t>(2 * i),
                     static_cast<uint8_t>(i)});
  }
  leds.SetData(colors);
  // 4 + 96 + 4 + 2 bytes: 12 extra clocks round up to 2 bytes
  CHECK(strip.frames.back().size() == 106);
  CHECK(strip.frames.back() == Frame(words, 2));
}

TEST_CASE("ClockedAddressableLED frame for the 14-LED rotary board", "[clockedled]") {
  test::FakeSPIDevice strip{1, -1};
  ClockedAddressableLED leds{SPI::kPort1, 14};
  CHECK(strip.frames.back().size() == 65);  // 4 + 56 + 4 + 1
}
