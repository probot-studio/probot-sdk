// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/thermal/MLX90640.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/hal/simulation/I2CData.h"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {
// MLX90640 datasheet rev 12, section 11.2 (calculation example): the EEPROM words 0x2410-0x243F
// of the example (Table 11 inputs), pixel words 0x08A0 and RAM words of the example frame.
constexpr std::array<uint16_t, 48> kEeprom2410 = {
    0x4210, 0xFFBB, 0x0202, 0xF202, 0xF2F2, 0xE2E2, 0xD1E1, 0xB1D1,  // 0x2410
    0xF10F, 0xF00F, 0xE0EF, 0xE0EF, 0xE1E1, 0xF3F2, 0xF404, 0xE504,  // 0x2418
    0x79A6, 0x2F44, 0xFFDD, 0x2210, 0x3333, 0x2233, 0xEF01, 0x9ACC,  // 0x2420
    0xEEDC, 0x10FF, 0x2221, 0x3333, 0x2333, 0x0112, 0xEEFF, 0xBBDD,  // 0x2428
    0x18EF, 0x2FF1, 0x5952, 0x9D68, 0x5454, 0x0994, 0x6956, 0x5354,  // 0x2430
    0x2363, 0xE446, 0xFBB5, 0x044B, 0xF020, 0x9797, 0x9797, 0x2889,  // 0x2438
};

void LoadExample(test::FakeI2CBus::Device& dev) {
  for (std::size_t i = 0; i < kEeprom2410.size(); ++i) {
    dev.regs[0x2410 + i] = kEeprom2410[i];
  }
  // 0x240A bit 11 set: calibrated in interleaved mode, the mode of the example frame, so the
  // library applies no interleaved/chess correction (as the datasheet example)
  dev.regs[0x240A] = 0x0800;
  for (int p = 0; p < 768; ++p) {
    dev.regs[0x2440 + p] = 0x08A0;  // per-pixel calibration of the example; no bad pixels
    dev.regs[0x0400 + p] = 0x0261;  // raw pixel data 609
  }
  dev.regs[0x0700] = 0x4BF2;  // VBE
  dev.regs[0x0708] = 0xFFCA;  // CP subpage 0
  dev.regs[0x0728] = 0xFFC8;  // CP subpage 1
  dev.regs[0x070A] = 0x1881;  // gain
  dev.regs[0x0720] = 0x06AF;  // PTAT
  dev.regs[0x072A] = 0xCCC5;  // VDD
  dev.regs[0x800D] = 0x0901;  // control register 1: interleaved, 18 bit, 2 Hz
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

TEST_CASE("MLX90640 matches the datasheet calculation example", "[mlx90640]") {
  test::FakeI2CBus bus;
  auto& dev = bus.Add(MLX90640::kDefaultAddress, 16, 16);  // 16-bit address and words
  LoadExample(dev);
  ReadSizes sizes;
  int32_t uid = HALSIM_RegisterI2CReadCallback(0, &LogRead, &sizes);

  MLX90640 camera{I2C::Port::PORT_0, MLX90640::kDefaultAddress, MLX90640::RefreshRate::k8Hz,
                  0_ms};
  camera.SetEmissivity(1.0);  // as the example; the reflected temperature then cancels out
  CHECK(dev.writes.empty());  // no bus access before the first Update
  CHECK_FALSE(camera.IsConnected());

  camera.Update();  // EEPROM, parameters, refresh rate; no data ready yet
  REQUIRE(camera.IsConnected());
  CHECK(dev.regs[0x800D] == 0x0A01);  // refresh rate bits 9:7 = 100b (8 Hz), rest kept
  CHECK(camera.GetFrameCount() == 0);

  std::array<float, MLX90640::kPixelCount> frame{};
  CHECK_FALSE(camera.GetFrame(frame));

  dev.regs[0x8000] = 0x0008;  // new data, subpage 0
  camera.Update();
  CHECK(dev.regs[0x8000] == 0x0030);  // new-data bit cleared by the driver
  CHECK(camera.GetFrameCount() == 0);  // half a frame

  dev.regs[0x8000] = 0x0009;  // new data, subpage 1
  camera.Update();
  CHECK(camera.GetFrameCount() == 1);
  REQUIRE(camera.GetFrame(frame));

  // Example results: Ta = 39.184 C. Pixel (12, 16) = index 367 (subpage 1 in interleaved mode):
  // the datasheet prints To = 80.36 C; the library's math on the same inputs (alpha minus the
  // mean of the two CP alphas times TGC) gives 80.18 C, recomputed independently for this test.
  CHECK(camera.GetAmbientTemperature().value() == Catch::Approx(39.184).margin(0.01));
  CHECK(frame[367] == Catch::Approx(80.18).margin(0.1));
  CHECK(std::all_of(frame.begin(), frame.end(), [](float t) { return std::isfinite(t); }));

  // the 1664-byte EEPROM and the 1536-byte pixel block go in reads of at most 512 bytes
  CHECK(sizes.largest == 512);
  CHECK(sizes.full >= 6);

  dev.regs[0x8000] = 0x0000;
  camera.Update();  // nothing new
  CHECK(camera.GetFrameCount() == 1);
  HALSIM_CancelI2CReadCallback(0, uid);
}

TEST_CASE("MLX90640 reports a missing camera", "[mlx90640]") {
  test::FakeI2CBus bus;
  MLX90640 camera{I2C::Port::PORT_0, MLX90640::kDefaultAddress, MLX90640::RefreshRate::k8Hz,
                  0_ms};
  camera.Update();
  CHECK_FALSE(camera.IsConnected());
  std::array<float, MLX90640::kPixelCount> frame{};
  CHECK_FALSE(camera.GetFrame(frame));
  CHECK(camera.GetFrameCount() == 0);
}

TEST_CASE("MLX90640 sets the camera up again after a lost transfer", "[mlx90640]") {
  test::FakeI2CBus bus;
  auto& dev = bus.Add(MLX90640::kDefaultAddress, 16, 16);
  LoadExample(dev);
  sim::PauseTiming();
  auto controlWrites = [&dev] {
    return std::count_if(dev.writes.begin(), dev.writes.end(), [](const auto& w) {
      return w.size() == 4 && w[0] == 0x80 && w[1] == 0x0D;
    });
  };

  MLX90640 camera{I2C::Port::PORT_0, MLX90640::kDefaultAddress, MLX90640::RefreshRate::k8Hz,
                  0_ms};
  camera.Update();
  REQUIRE(camera.IsConnected());
  CHECK(controlWrites() == 1);

  dev.nack = true;  // unplugged
  camera.Update();
  CHECK_FALSE(camera.IsConnected());

  dev.nack = false;
  dev.regs[0x800D] = 0x0901;  // back at the EEPROM refresh rate after the power loss
  camera.Update();             // within a second of the last setup: no new attempt
  CHECK_FALSE(camera.IsConnected());
  CHECK(controlWrites() == 1);

  sim::StepTiming(1.1_s);
  camera.Update();
  CHECK(camera.IsConnected());
  CHECK(controlWrites() == 2);
  CHECK(dev.regs[0x800D] == 0x0A01);
  sim::ResumeTiming();
}

TEST_CASE("MLX90640 reader thread starts and stops", "[mlx90640]") {
  test::FakeI2CBus bus;
  auto& dev = bus.Add(MLX90640::kDefaultAddress, 16, 16);
  {
    std::scoped_lock lock{bus.mutex};
    LoadExample(dev);
  }
  MLX90640 camera{I2C::Port::PORT_0};  // default: thread polling every 10 ms
  CHECK(camera.GetFrameCount() == 0);    // the status register never reports new data
  // destroyed at the end of the scope: the thread joins
}
