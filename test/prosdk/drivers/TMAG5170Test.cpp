// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/magnetic/TMAG5170.hpp"

#include <array>
#include <cstdint>
#include <initializer_list>
#include <random>
#include <span>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeSPIDevice.hpp"
#include "prosdk/simulation/SpiSim.hpp"

using namespace prosdk;

namespace {
using Bytes = std::vector<uint8_t>;

// CRC from the XOR equations 10 to 13 of the datasheet (SBASAF4 7.5.2.5), initial bits 1111b,
// over the frame with its low 4 bits zero: an independent form of the driver's serial CRC.
uint8_t DatasheetCrc(uint32_t frame) {
  const uint32_t f = frame & 0xFFFFFFF0u;
  auto x = [f](std::initializer_list<int> bits) {
    uint32_t v = 0;
    for (int b : bits) {
      v ^= (f >> b) & 1u;
    }
    return v;
  };
  const uint32_t c0 = x({30, 26, 25, 24, 23, 21, 19, 18, 15, 11, 10, 9, 8, 6, 4, 3, 0}) ^ 1u;
  const uint32_t c1 = x({31, 30, 27, 23, 22, 21, 20, 18, 16, 15, 12, 8, 7, 6, 5, 3, 1, 0});
  const uint32_t c2 = x({31, 28, 24, 23, 22, 21, 19, 17, 16, 13, 9, 8, 7, 6, 4, 2, 1});
  const uint32_t c3 = x({29, 25, 24, 23, 22, 20, 18, 17, 14, 10, 9, 8, 7, 5, 3, 2}) ^ 1u;
  return static_cast<uint8_t>(c0 | (c1 << 1) | (c2 << 2) | (c3 << 3));
}

// A TMAG5170 register file with the SPI frame of 7.5.2: a frame with a wrong CRC does not write
// and its answer's last CRC bit is inverted; an answer carries CFG_RESET in bit 30 (STAT10),
// which reading AFE_STATUS clears.
struct TmagModel {
  std::array<uint16_t, 0x15> regs{};
  bool corruptAnswers = false;
  int badFrames = 0;

  explicit TmagModel(int version = 0) { PowerUp(version); }

  void PowerUp(int version = 0) {
    regs.fill(0);
    regs[0x0D] = 0x8000;                                    // AFE_STATUS CFG_RESET
    regs[0x0F] = static_cast<uint16_t>(0x0040 | (version << 4));  // TEST_CONFIG VER
  }

  void Transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    if (tx.size() != 4) {
      ++badFrames;
      return;
    }
    const uint32_t f = (uint32_t{tx[0]} << 24) | (uint32_t{tx[1]} << 16) |
                       (uint32_t{tx[2]} << 8) | tx[3];
    const bool crcOk = DatasheetCrc(f) == (f & 0x0F);
    if (!crcOk) {
      ++badFrames;
    }
    const size_t reg = (f >> 24) & 0x7F;
    const auto value = static_cast<uint16_t>(f >> 8);
    uint32_t answer = (regs[0x0D] & 0x8000) ? 0x40000000u : 0u;
    if (f & 0x80000000u) {
      if (reg < regs.size()) {
        answer |= uint32_t{regs[reg]} << 8;
      }
      if (reg == 0x0D) {
        regs[0x0D] &= 0x7FFF;  // read to clear
      }
    } else if (crcOk && reg < regs.size()) {
      if (reg == 0x0F) {
        regs[0x0F] = static_cast<uint16_t>((regs[0x0F] & ~0x7) | (value & 0x7));
      } else {
        regs[reg] = value;
      }
    }
    uint8_t crc = DatasheetCrc(answer);
    if (!crcOk || corruptAnswers) {
      crc ^= 0x01;
    }
    answer |= crc;
    rx[0] = static_cast<uint8_t>(answer >> 24);
    rx[1] = static_cast<uint8_t>(answer >> 16);
    rx[2] = static_cast<uint8_t>(answer >> 8);
    rx[3] = static_cast<uint8_t>(answer);
  }
};

void Attach(test::FakeSPIDevice& dev, TmagModel& model) {
  dev.handler = [&model](std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    model.Transfer(tx, rx);
  };
}
}  // namespace

TEST_CASE("TMAG5170 CRC matches the datasheet", "[tmag5170]") {
  // 7.5.2.5: the frame that turns the CRC off is 0x0F000407
  CHECK(TMAG5170::Crc4(0x0F000407) == 0x7);
  CHECK(TMAG5170::Crc4(0x0F000400) == 0x7);  // the low 4 bits are not part of the CRC
  std::mt19937 random{5170};
  for (int i = 0; i < 2000; ++i) {
    const uint32_t frame = random();
    REQUIRE(TMAG5170::Crc4(frame) == DatasheetCrc(frame));
  }
}

TEST_CASE("TMAG5170 configures the chip", "[tmag5170]") {
  test::FakeSPIDevice dev{0, 5};
  TmagModel model;
  Attach(dev, model);
  TMAG5170 sensor{SPI::kPort0, 5};
  CHECK(sensor.IsConnected());
  CHECK(sensor.GetCrcErrors() == 0);
  CHECK(model.badFrames == 0);  // every frame had a valid CRC

  platform::hal::SpiDeviceConfig config;
  REQUIRE(sim::GetSpiDeviceConfig(0, 5, &config));
  CHECK(config.mode == 0);
  CHECK(config.clockHz == TMAG5170::kDefaultClockRate);
  {
    TMAG5170 clamped{SPI::kPort0, 9, 40'000'000};  // above fSPI 10 MHz
    REQUIRE(sim::GetSpiDeviceConfig(0, 9, &config));
    CHECK(config.clockHz == TMAG5170::kMaxClockRate);
  }

  // TEST_CONFIG = 0 (CRC on), then AFE_STATUS and TEST_CONFIG reads
  REQUIRE(dev.frames.size() >= 3);
  CHECK(dev.frames[0] == Bytes{0x0F, 0x00, 0x00, 0x0E});
  CHECK(dev.frames[1] == Bytes{0x8D, 0x00, 0x00, 0x01});
  CHECK(dev.frames[2] == Bytes{0x8F, 0x00, 0x00, 0x08});
  CHECK((model.regs[0x0D] & 0x8000) == 0);  // CFG_RESET read and cleared

  // SENSOR_CONFIG: angle X-Y, channels X, Y, Z, all +/-50 mT
  CHECK(model.regs[0x01] == 0x41C0);
  CHECK(model.regs[0x02] == 0x0000);
  // DEVICE_CONFIG: 8x averaging, continuous conversion, temperature channel on
  CHECK(model.regs[0x00] == 0x3028);
  CHECK(sensor.GetRange().value() == Catch::Approx(50.0));

  CHECK(sensor.Configure(TMAG5170::Range::kLow, TMAG5170::Averaging::k32x,
                         TMAG5170::AnglePlane::kXZ));
  CHECK(model.regs[0x01] == 0xC1D5);  // angle X-Z, X/Y/Z, +/-25 mT each
  CHECK(model.regs[0x00] == 0x5028);  // 32x
  CHECK(sensor.GetRange().value() == Catch::Approx(25.0));
  CHECK(model.badFrames == 0);
}

TEST_CASE("TMAG5170 decodes field, temperature and angle", "[tmag5170]") {
  test::FakeSPIDevice dev{0, 6};
  TmagModel model;
  Attach(dev, model);
  TMAG5170 sensor{SPI::kPort0, 6};
  REQUIRE(sensor.IsConnected());

  // equation 1: B = result / 2^16 x 2 x range
  model.regs[0x09] = 0x4000;
  model.regs[0x0A] = 0xC000;
  model.regs[0x0B] = 0x7FFF;
  auto field = sensor.GetField();
  CHECK(field.x.value() == Catch::Approx(25.0));
  CHECK(field.y.value() == Catch::Approx(-25.0));
  CHECK(field.z.value() == Catch::Approx(50.0 * 32767.0 / 32768.0));
  REQUIRE(sensor.Configure(TMAG5170::Range::kHigh, TMAG5170::Averaging::k1x));
  field = sensor.GetField();
  CHECK(field.x.value() == Catch::Approx(50.0));
  CHECK(field.y.value() == Catch::Approx(-50.0));

  // equation 3 with the typical values: 17522 at 25 C, 60 LSB per C
  model.regs[0x0C] = 17522;
  CHECK(sensor.GetTemperature().value() == Catch::Approx(25.0));
  model.regs[0x0C] = 17522 + 600;
  CHECK(sensor.GetTemperature().value() == Catch::Approx(35.0));
  model.regs[0x0C] = 17522 - 1500;
  CHECK(sensor.GetTemperature().value() == Catch::Approx(0.0).margin(1e-9));

  // 7.5.1.4 examples: 0x1628 is 354.50 degrees, 0x0114 is 17.25 degrees
  model.regs[0x13] = 0x1628;
  CHECK(sensor.GetAngle().value() == Catch::Approx(354.5));
  model.regs[0x13] = 0x0114;
  CHECK(sensor.GetAngle().value() == Catch::Approx(17.25));
  model.regs[0x13] = 0xE114;  // bits 15..13 are not part of the angle
  CHECK(sensor.GetAngle().value() == Catch::Approx(17.25));
  CHECK(sensor.GetCrcErrors() == 0);
}

TEST_CASE("TMAG5170 A2 version scales the ranges by three", "[tmag5170]") {
  test::FakeSPIDevice dev{0, 7};
  TmagModel model{1};
  Attach(dev, model);
  TMAG5170 sensor{SPI::kPort0, 7};
  REQUIRE(sensor.IsConnected());
  CHECK(sensor.GetRange().value() == Catch::Approx(150.0));
  model.regs[0x09] = 0x4000;
  CHECK(sensor.GetField().x.value() == Catch::Approx(75.0));
}

TEST_CASE("TMAG5170 counts CRC errors and keeps the last values", "[tmag5170]") {
  test::FakeSPIDevice dev{0, 5};
  TmagModel model;
  Attach(dev, model);
  TMAG5170 sensor{SPI::kPort0, 5};
  model.regs[0x13] = 0x0114;
  CHECK(sensor.GetAngle().value() == Catch::Approx(17.25));

  model.corruptAnswers = true;
  model.regs[0x13] = 0x1628;
  CHECK(sensor.GetAngle().value() == Catch::Approx(17.25));
  CHECK(sensor.GetCrcErrors() == 1);
  CHECK_FALSE(sensor.IsConnected());
  model.regs[0x09] = 0x4000;
  CHECK(sensor.GetField().x.value() == Catch::Approx(0.0).margin(1e-12));
  CHECK(sensor.GetCrcErrors() == 2);  // the first of the three reads failed

  model.corruptAnswers = false;
  CHECK(sensor.GetAngle().value() == Catch::Approx(354.5));
  CHECK(sensor.IsConnected());
  CHECK(sensor.GetCrcErrors() == 2);
}

TEST_CASE("TMAG5170 configures a chip again after a reset", "[tmag5170]") {
  test::FakeSPIDevice dev{0, 5};
  TmagModel model;
  Attach(dev, model);
  TMAG5170 sensor{SPI::kPort0, 5};
  model.regs[0x13] = 0x0114;
  CHECK(sensor.GetAngle().value() == Catch::Approx(17.25));

  model.PowerUp();  // brown-out: registers at their reset values, CFG_RESET set
  model.regs[0x13] = 0x1628;
  // the answer shows CFG_RESET: the value is not taken
  CHECK(sensor.GetAngle().value() == Catch::Approx(17.25));
  // the next read writes the configuration first
  CHECK(sensor.GetAngle().value() == Catch::Approx(354.5));
  CHECK(model.regs[0x01] == 0x41C0);
  CHECK(model.regs[0x00] == 0x3028);
  CHECK((model.regs[0x0D] & 0x8000) == 0);
  CHECK(sensor.IsConnected());
}

TEST_CASE("TMAG5170 without a chip is not connected", "[tmag5170]") {
  test::FakeSPIDevice dev{0, 5};
  // no chip: every answer is zeros, whose CRC (1001b) is wrong
  dev.handler = [](std::span<const uint8_t>, std::span<uint8_t>) {};
  TMAG5170 sensor{SPI::kPort0, 5};
  CHECK_FALSE(sensor.IsConnected());
  CHECK(sensor.GetAngle().value() == Catch::Approx(0.0).margin(1e-12));
  CHECK(sensor.GetCrcErrors() > 0);

  // a chip connected later is configured on the next read
  TmagModel model;
  Attach(dev, model);
  model.regs[0x13] = 0x0114;
  CHECK(sensor.GetAngle().value() == Catch::Approx(17.25));
  CHECK(sensor.IsConnected());
  CHECK(model.regs[0x00] == 0x3028);
}

TEST_CASE("TMAG5170 with a reserved version is not connected", "[tmag5170]") {
  test::FakeSPIDevice dev{0, 5};
  TmagModel model{2};
  Attach(dev, model);
  TMAG5170 sensor{SPI::kPort0, 5};
  CHECK_FALSE(sensor.IsConnected());
  CHECK(model.regs[0x00] == 0);  // nothing configured
}
