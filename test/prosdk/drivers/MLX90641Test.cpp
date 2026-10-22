// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/thermal/MLX90641.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/hal/simulation/I2CData.h"

using namespace prosdk;

namespace {
// MLX90641 datasheet rev 4, section 11.2 (calculation example): EEPROM words 0x2410-0x243F with
// their Hamming bits, per-pixel words and RAM words of the example frame.
constexpr std::array<uint16_t, 48> kEeprom2410 = {
    0x0000, 0xB7E8, 0xD016, 0x0000, 0x0000, 0xC2FD, 0x1A43, 0xCA9A,  // 0x2410
    0x5164, 0x018C, 0x018C, 0x018C, 0x9CB1, 0x956C, 0xA5CC, 0x7DD1,  // 0x2418
    0x6D7F, 0x3CD4, 0x27B8, 0x19E6, 0xF137, 0x7814, 0x2658, 0xEF9E,  // 0x2420
    0x917F, 0xF018, 0xE156, 0x4817, 0x1C80, 0x233E, 0xC826, 0xCFFC,  // 0x2428
    0xA009, 0xBB53, 0xF194, 0xFC00, 0x7814, 0xED22, 0xED22, 0xED22,  // 0x2430
    0xED22, 0xED22, 0x80C8, 0xED22, 0x4190, 0xED22, 0xDA58, 0xED22,  // 0x2438
};

void LoadExample(test::FakeI2CBus::Device& dev) {
  for (std::size_t i = 0; i < kEeprom2410.size(); ++i) {
    dev.regs[0x2410 + i] = kEeprom2410[i];
  }
  dev.regs[0x240A] = 0x0040;  // bit 6 set: MLX90641 EEPROM layout
  for (int p = 0; p < 192; ++p) {
    dev.regs[0x2440 + p] = 0xF849;  // offset
    dev.regs[0x2500 + p] = 0xFFFF;  // sensitivity
    dev.regs[0x25C0 + p] = 0xB8C0;  // Kta and Kv
    // the datasheet prints 0xF847 for the subpage 1 offset, a word with a single-bit Hamming
    // error (syndrome 20); 0xD847 has the same 11 data bits and valid check bits
    dev.regs[0x2680 + p] = 0xD847;
  }
  for (int a = 0x0400; a < 0x0580; ++a) {
    dev.regs[a] = 0x03CC;  // raw pixel data 972
  }
  dev.regs[0x0580] = 0x4C54;  // VBE
  dev.regs[0x0588] = 0xFF97;  // CP
  dev.regs[0x058A] = 0x2606;  // gain
  dev.regs[0x05A0] = 0x06D8;  // PTAT
  dev.regs[0x05AA] = 0xCB8A;  // VDD
  dev.regs[0x800D] = 0x0901;  // control register 1: 18 bit, 2 Hz
  dev.regs[0x8000] = 0x0000;  // no new data
}

struct ReadSizes {
  unsigned int largest = 0;
  int full = 0;  // reads of exactly 512 bytes
};

void LogRead(const char*, void* param, unsigned char*, unsigned int count) {
  auto* sizes = static_cast<ReadSizes*>(param);
  sizes->largest = std::max(sizes->largest, count);
  if (count == 512) {
    ++sizes->full;
  }
}
}  // namespace

TEST_CASE("MLX90641 matches the datasheet calculation example", "[mlx90641]") {
  test::FakeI2CBus bus;
  auto& dev = bus.Add(MLX90641::kDefaultAddress, 16, 16);  // 16-bit address and words
  LoadExample(dev);
  ReadSizes sizes;
  int32_t uid = HALSIM_RegisterI2CReadCallback(0, &LogRead, &sizes);

  MLX90641 camera{I2C::Port::PORT_0, MLX90641::kDefaultAddress, MLX90641::RefreshRate::k8Hz,
                  0_ms};
  camera.SetEmissivity(0.94921875);  // the EEPROM emissivity of the example
  CHECK(dev.writes.empty());          // no bus access before the first Update

  camera.Update();  // EEPROM, parameters, refresh rate; no data ready yet
  REQUIRE(camera.IsConnected());
  CHECK(dev.regs[0x800D] == 0x0A01);  // refresh rate bits 9:7 = 100b (8 Hz), rest kept
  std::array<float, MLX90641::kPixelCount> frame{};
  CHECK_FALSE(camera.GetFrame(frame));

  dev.regs[0x8000] = 0x0008;  // new data, subpage 0
  camera.Update();
  CHECK(dev.regs[0x8000] == 0x0030);  // new-data bit cleared by the driver
  CHECK(camera.GetFrameCount() == 1);  // each subpage holds all 192 pixels
  REQUIRE(camera.GetFrame(frame));

  // The datasheet prints Ta = 42.02 C and To = 80.12 C for pixel 88 (row 6, column 9 counted
  // from 1) with Tr = Ta - 5; its own intermediate values give Ta = 42.098 C and To = 80.24 C,
  // recomputed independently (Python, double precision) for this test.
  CHECK(camera.GetAmbientTemperature().value() == Catch::Approx(42.098).margin(0.02));
  CHECK(frame[88] == Catch::Approx(80.24).margin(0.1));
  CHECK(std::all_of(frame.begin(), frame.end(), [](float t) { return std::isfinite(t); }));

  dev.regs[0x8000] = 0x0009;  // subpage 1 publishes a frame as well
  camera.Update();
  CHECK(camera.GetFrameCount() == 2);

  // the 1664-byte EEPROM goes in reads of at most 512 bytes
  CHECK(sizes.largest == 512);
  CHECK(sizes.full >= 3);
  HALSIM_CancelI2CReadCallback(0, uid);
}

TEST_CASE("MLX90641 accepts a corrected EEPROM bit and rejects a double error", "[mlx90641]") {
  {
    test::FakeI2CBus bus;
    auto& dev = bus.Add(MLX90641::kDefaultAddress, 16, 16);
    LoadExample(dev);
    dev.regs[0x2680] = 0xF847;  // the datasheet word: syndrome 20, bit 13 corrected (-9)
    MLX90641 camera{I2C::Port::PORT_0, MLX90641::kDefaultAddress, MLX90641::RefreshRate::k8Hz,
                    0_ms};
    camera.Update();
    CHECK(camera.IsConnected());
    CHECK(dev.regs[0x800D] == 0x0A01);
  }
  {
    test::FakeI2CBus bus;
    auto& dev = bus.Add(MLX90641::kDefaultAddress, 16, 16);
    LoadExample(dev);
    dev.regs[0x2680] = 0xD844;  // two data bits flipped: syndrome 6, not correctable (-10)
    MLX90641 camera{I2C::Port::PORT_0, MLX90641::kDefaultAddress, MLX90641::RefreshRate::k8Hz,
                    0_ms};
    camera.Update();
    CHECK_FALSE(camera.IsConnected());
    CHECK(dev.regs[0x800D] == 0x0901);  // refresh rate not written
  }
}

TEST_CASE("MLX90641 reports a missing camera", "[mlx90641]") {
  test::FakeI2CBus bus;
  MLX90641 camera{I2C::Port::PORT_0, MLX90641::kDefaultAddress, MLX90641::RefreshRate::k8Hz,
                  0_ms};
  camera.Update();
  CHECK_FALSE(camera.IsConnected());
  std::array<float, MLX90641::kPixelCount> frame{};
  CHECK_FALSE(camera.GetFrame(frame));
  CHECK(camera.GetFrameCount() == 0);
}
