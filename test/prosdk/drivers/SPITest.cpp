// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/bus/SPI.hpp"

#include <array>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeSPIDevice.hpp"
#include "prosdk/platform/HalConfig.hpp"
#include "prosdk/simulation/SpiSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {
using Bytes = std::vector<uint8_t>;

platform::hal::SpiDeviceConfig ConfigOf(int bus, int cs) {
  platform::hal::SpiDeviceConfig config;
  REQUIRE(sim::GetSpiDeviceConfig(bus, cs, &config));
  return config;
}
}  // namespace

TEST_CASE("SetSPIPins validates the pins", "[spi]") {
  CHECK(platform::SetSPIPins(0, 12, 11, 13));
  auto config = platform::GetSPIConfig(0);
  CHECK(config.sck == 12);
  CHECK(config.mosi == 11);
  CHECK(config.miso == 13);
  CHECK(platform::SetSPIPins(1, 40, 41, -1));  // transmit only
  CHECK(platform::GetSPIConfig(1).miso == -1);
  CHECK_FALSE(platform::SetSPIPins(2, 1, 2, 3));
  CHECK_FALSE(platform::SetSPIPins(0, 12, 12, 13));
  CHECK_FALSE(platform::SetSPIPins(0, 12, -1, 13));
  CHECK_FALSE(platform::SetSPIPins(0, 12, 11, 49));
  CHECK(platform::GetSPIConfig(0).sck == 12);  // unchanged by the failed calls
  CHECK(platform::GetSPIConfig(-1).sck == -1);
}

TEST_CASE("SPI stores mode, clock and chip-select polarity", "[spi]") {
  SPI spi{SPI::kPort0, 5};
  CHECK(spi.GetPort() == SPI::kPort0);
  CHECK(spi.GetChipSelect() == 5);
  auto config = ConfigOf(0, 5);
  CHECK(config.mode == 0);
  CHECK(config.clockHz == 1'000'000);
  CHECK_FALSE(config.csActiveHigh);

  spi.SetMode(SPI::Mode::kMode3);
  spi.SetClockRate(8'000'000);
  spi.SetChipSelectActiveHigh();
  config = ConfigOf(0, 5);
  CHECK(config.mode == 3);
  CHECK(config.clockHz == 8'000'000);
  CHECK(config.csActiveHigh);
  spi.SetChipSelectActiveLow();
  CHECK_FALSE(ConfigOf(0, 5).csActiveHigh);

  CHECK_THROWS_AS(spi.SetClockRate(0), RuntimeError);
  CHECK_THROWS_AS(spi.SetClockRate(SPI::kMaxClockRate + 1), RuntimeError);
  CHECK(ConfigOf(0, 5).clockHz == 8'000'000);
  CHECK_THROWS_AS((SPI{static_cast<SPI::Port>(2), 5}), RuntimeError);
}

TEST_CASE("SPI routes transfers by chip select", "[spi]") {
  test::FakeSPIDevice a{0, 5};
  test::FakeSPIDevice b{0, 6};
  a.regs[0x0F] = 0xA1;
  b.regs[0x0F] = 0xB2;
  SPI spiA{SPI::kPort0, 5};
  SPI spiB{SPI::kPort0, 6};

  std::array<uint8_t, 2> rx{};
  const std::array<uint8_t, 2> whoAmI{0x8F, 0x00};
  CHECK(spiA.Transaction(whoAmI.data(), rx.data(), 2) == 2);
  CHECK(rx[1] == 0xA1);
  CHECK(spiB.Transaction(whoAmI.data(), rx.data(), 2) == 2);
  CHECK(rx[1] == 0xB2);
  CHECK(a.frames == std::vector<Bytes>{{0x8F, 0x00}});
  CHECK(b.frames == std::vector<Bytes>{{0x8F, 0x00}});

  const std::array<uint8_t, 3> write{0x10, 0x55, 0x66};
  CHECK(spiB.Write(write.data(), 3) == 3);
  CHECK(b.regs[0x10] == 0x55);
  CHECK(b.regs[0x11] == 0x66);
  CHECK(a.regs[0x10] == 0);
  CHECK(a.frames.size() == 1);

  // a device on the other bus
  test::FakeSPIDevice other{1, 7};
  SPI spiC{SPI::kPort1, 7};
  other.handler = [](std::span<const uint8_t>, std::span<uint8_t> rx) { rx[0] = 0x42; };
  rx = {};
  CHECK(spiC.Read(true, rx.data(), 2) == 2);
  CHECK(rx == std::array<uint8_t, 2>{0x42, 0});
  CHECK(other.frames == std::vector<Bytes>{{0, 0}});  // Read sends zeros
  CHECK(a.frames.size() == 1);
}

TEST_CASE("SPI segmented transactions hold the chip select", "[spi]") {
  test::FakeSPIDevice dev{0, 5};
  dev.handler = [](std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    // answer every byte after the 4-byte header with its index
    for (size_t i = 4; i < rx.size(); ++i) {
      rx[i] = static_cast<uint8_t>(i);
    }
    (void)tx;
  };
  SPI spi{SPI::kPort0, 5};
  const std::array<uint8_t, 1> command{0x03};
  const std::array<uint8_t, 3> address{0x01, 0x02, 0x03};
  std::array<uint8_t, 4> data{};
  const std::array<SpiSegment, 4> segments{{{command.data(), nullptr, 1},
                                            {address.data(), nullptr, 3},
                                            {nullptr, data.data(), 4},
                                            {nullptr, nullptr, 0}}};
  CHECK(spi.TransactionSegments(segments) == 8);
  REQUIRE(dev.frames.size() == 1);  // one chip-select assertion
  CHECK(dev.frames[0] == Bytes{0x03, 0x01, 0x02, 0x03, 0, 0, 0, 0});
  CHECK(data == std::array<uint8_t, 4>{4, 5, 6, 7});

  const std::array<SpiSegment, 1> negative{{{nullptr, nullptr, -1}}};
  CHECK(spi.TransactionSegments(negative) == -1);
  CHECK(dev.frames.size() == 1);
  CHECK(spi.TransactionSegments({}) == 0);
  const std::array<SpiSegment, 1> empty{{{nullptr, nullptr, 0}}};
  CHECK(spi.TransactionSegments(empty) == 0);
  CHECK(dev.frames.size() == 1);  // 0 bytes: no chip-select assertion
}

TEST_CASE("SPI device without chip select is alone on its bus", "[spi]") {
  {
    SPI leds{SPI::kPort1, -1};
    CHECK(leds.GetChipSelect() == -1);
    CHECK_THROWS_AS((SPI{SPI::kPort1, 7}), RuntimeError);
    CHECK_THROWS_AS((SPI{SPI::kPort1, -1}), RuntimeError);
    SPI flash{SPI::kPort0, 7};  // the other bus is free

    test::FakeSPIDevice strip{1, -1};
    const std::array<uint8_t, 4> start{0, 0, 0, 0};
    CHECK(leds.Write(start.data(), 4) == 4);
    CHECK(strip.frames == std::vector<Bytes>{{0, 0, 0, 0}});
  }
  {
    SPI flash{SPI::kPort1, 7};  // the bus is free again
    CHECK_THROWS_AS((SPI{SPI::kPort1, -1}), RuntimeError);
  }
}

TEST_CASE("SPI chip selects are owned once", "[spi]") {
  SPI first{SPI::kPort0, 5};
  CHECK_THROWS_AS((SPI{SPI::kPort0, 5}), RuntimeError);
  CHECK_THROWS_AS((SPI{SPI::kPort1, 5}), RuntimeError);  // one GPIO, one owner
  CHECK_THROWS_AS((SPI{SPI::kPort0, -2}), RuntimeError);

  // bus 1 (SPI3) has 3 hardware chip selects
  SPI c1{SPI::kPort1, 1};
  SPI c2{SPI::kPort1, 2};
  SPI c3{SPI::kPort1, 3};
  CHECK_THROWS_AS((SPI{SPI::kPort1, 4}), RuntimeError);

  // a moved-from object no longer owns the device
  SPI moved{std::move(first)};
  CHECK(moved.GetChipSelect() == 5);
  CHECK_THROWS_AS((SPI{SPI::kPort0, 5}), RuntimeError);
  SPI target{SPI::kPort0, 6};
  target = std::move(moved);
  CHECK(target.GetChipSelect() == 5);
  SPI reopened{SPI::kPort0, 6};  // freed by the assignment
  CHECK_THROWS_AS((SPI{SPI::kPort0, 5}), RuntimeError);
}
