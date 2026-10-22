// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/environment/MS5611.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {
using Bytes = std::vector<uint8_t>;

// The data sheet example (Figure 2): C1..C6, D1 = 9085466, D2 = 8569150 give 20.07 C and
// 1000.09 mbar. Words 0 and 7 are 0, which makes the CRC-4 0.
constexpr std::array<uint16_t, 8> kProm = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};

// A command-style chip: one command byte per write, the answer in a separate read.
struct Chip {
  std::array<uint16_t, 8> prom = kProm;
  uint32_t d1 = 9085466;
  uint32_t d2 = 8569150;
  uint32_t adc = 0;  // finished conversion; ADC read returns it once, then 0
  uint8_t command = 0;
};

test::FakeI2CBus::Device& AddChip(test::FakeI2CBus& bus, Chip& chip, int address = 0x77) {
  auto& dev = bus.Add(address);
  dev.onWrite = [&chip](std::span<const uint8_t> data) {
    if (data.size() != 1) {
      return;
    }
    chip.command = data[0];
    if ((data[0] & 0xF0) == 0x40) {
      chip.adc = chip.d1;
    } else if ((data[0] & 0xF0) == 0x50) {
      chip.adc = chip.d2;
    }
  };
  dev.onRead = [&chip](std::span<uint8_t> buf) {
    if (chip.command == 0x00 && buf.size() == 3) {
      buf[0] = static_cast<uint8_t>(chip.adc >> 16);
      buf[1] = static_cast<uint8_t>(chip.adc >> 8);
      buf[2] = static_cast<uint8_t>(chip.adc);
      chip.adc = 0;
    } else if ((chip.command & 0xF1) == 0xA0 && buf.size() == 2) {
      uint16_t word = chip.prom[(chip.command >> 1) & 0x07];
      buf[0] = static_cast<uint8_t>(word >> 8);
      buf[1] = static_cast<uint8_t>(word & 0xFF);
    }
  };
  return dev;
}
}  // namespace

TEST_CASE("MS5611 compensation matches the data sheet example", "[ms5611]") {
  int32_t temperature = 0;
  int32_t pressure = 0;
  MS5611::Compensate(kProm, 9085466, 8569150, temperature, pressure);
  CHECK(temperature == 2007);
  CHECK(pressure == 100009);
  // second order, below 20 C
  MS5611::Compensate(kProm, 9085466, 8000000, temperature, pressure);
  CHECK(temperature == -61);
  CHECK(pressure == 95989);
  // second order, below -15 C
  MS5611::Compensate(kProm, 9085466, 7500000, temperature, pressure);
  CHECK(temperature == -2129);
  CHECK(pressure == 91910);
}

TEST_CASE("MS5611 PROM CRC-4", "[ms5611]") {
  CHECK(MS5611::PromCrc(kProm) == 0);
  // computed with the AN520 crc4() algorithm (AN520 itself prints no example vector)
  constexpr std::array<uint16_t, 8> an520 = {0x3132, 0x3334, 0x3536, 0x3738,
                                             0x3940, 0x4142, 0x4344, 0x4500};
  CHECK(MS5611::PromCrc(an520) == 0xB);
  std::array<uint16_t, 8> prom = kProm;
  prom[0] = 0x1234;
  prom[7] = 0x560E;  // the low nibble (the CRC itself) does not count
  CHECK(MS5611::PromCrc(prom) == 0xE);
  prom[7] = 0x5600;
  CHECK(MS5611::PromCrc(prom) == 0xE);
}

TEST_CASE("MS5611 resets, reads the PROM and alternates conversions", "[ms5611]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  Chip chip;
  auto& dev = AddChip(bus, chip);
  MS5611 baro{I2C::Port::PORT_0};
  baro.Update();
  CHECK(baro.IsConnected());
  REQUIRE(dev.writes.size() == 1);
  CHECK(dev.writes[0] == Bytes{0x1E});
  baro.Update();  // the PROM reload takes 2.8 ms
  CHECK(dev.writes.size() == 1);

  sim::StepTiming(5_ms);
  baro.Update();  // 8 PROM reads, then a D2 conversion at OSR 4096
  REQUIRE(dev.writes.size() == 10);
  CHECK(dev.writes[1] == Bytes{0xA0});
  CHECK(dev.writes[8] == Bytes{0xAE});
  CHECK(dev.writes[9] == Bytes{0x58});
  baro.Update();  // 9.04 ms conversion: too early
  CHECK(dev.writes.size() == 10);

  sim::StepTiming(10_ms);
  baro.Update();  // read D2, start D1
  REQUIRE(dev.writes.size() == 12);
  CHECK(dev.writes[10] == Bytes{0x00});
  CHECK(dev.writes[11] == Bytes{0x48});
  CHECK_FALSE(baro.HasData());
  CHECK(baro.GetAltitude().value() == Catch::Approx(0.0).margin(1e-12));  // no pressure yet

  sim::StepTiming(10_ms);
  baro.Update();  // read D1, compensate, start D2
  REQUIRE(dev.writes.size() == 14);
  CHECK(dev.writes[13] == Bytes{0x58});
  CHECK(baro.HasData());
  CHECK(baro.GetTemperature().value() == Catch::Approx(20.07));
  CHECK(baro.GetPressure().value() == Catch::Approx(100009.0));
  CHECK(baro.GetAltitude().value() == Catch::Approx(110.14375).margin(1e-3));
  CHECK(baro.GetAltitude(units::pascal_t{100009.0}).value() ==
        Catch::Approx(0.0).margin(1e-9));

  // a lost conversion reads 0: the same conversion is started again, the values stay
  chip.adc = 0;
  sim::StepTiming(10_ms);
  baro.Update();
  REQUIRE(dev.writes.size() == 16);
  CHECK(dev.writes[15] == Bytes{0x58});
  CHECK(baro.IsConnected());
  CHECK(baro.GetPressure().value() == Catch::Approx(100009.0));
  sim::StepTiming(10_ms);
  baro.Update();
  CHECK(dev.writes.back() == Bytes{0x48});
  sim::ResumeTiming();
}

TEST_CASE("MS5611 uses the OSR and address it is given", "[ms5611]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  Chip chip;
  auto& dev = AddChip(bus, chip, 0x76);
  MS5611 baro{I2C::Port::PORT_0, 0x76, MS5611::Osr::k256};
  baro.Update();
  sim::StepTiming(5_ms);
  baro.Update();
  CHECK(dev.writes.back() == Bytes{0x50});
  sim::StepTiming(1_ms);  // 0.60 ms max at OSR 256
  baro.Update();
  CHECK(dev.writes.back() == Bytes{0x40});
  sim::StepTiming(1_ms);
  baro.Update();
  CHECK(baro.HasData());
  sim::ResumeTiming();
}

TEST_CASE("MS5611 rejects a bad PROM and retries a missing chip", "[ms5611]") {
  sim::PauseTiming();
  test::FakeI2CBus bus;
  MS5611 baro{I2C::Port::PORT_0};
  baro.Update();
  CHECK_FALSE(baro.IsConnected());

  Chip chip;
  chip.prom[7] = 0x0005;  // wrong CRC
  auto& dev = AddChip(bus, chip);
  sim::StepTiming(1100_ms);
  baro.Update();
  CHECK(baro.IsConnected());
  sim::StepTiming(5_ms);
  baro.Update();
  CHECK_FALSE(baro.IsConnected());
  CHECK(dev.writes.back() == Bytes{0xAE});  // no conversion started

  chip.prom[7] = 0x0000;
  sim::StepTiming(1100_ms);
  baro.Update();
  CHECK(dev.writes.back() == Bytes{0x1E});
  sim::StepTiming(5_ms);
  baro.Update();
  CHECK(baro.IsConnected());
  CHECK(dev.writes.back() == Bytes{0x58});

  dev.nack = true;
  sim::StepTiming(10_ms);
  baro.Update();
  CHECK_FALSE(baro.IsConnected());
  CHECK_FALSE(baro.HasData());
  sim::ResumeTiming();
}
