// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/memory/W25QFlash.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeSPIDevice.hpp"
#include "prosdk/simulation/SpiSim.hpp"

using namespace prosdk;

namespace {
using Bytes = std::vector<uint8_t>;

// A W25Q40CL: 512 KiB array, status register 1 (BUSY S0, WEL S1), the instructions the driver
// uses. Program and erase change the array at once and then report BUSY for busyAfterOp reads
// of the status register.
struct NorModel {
  std::vector<uint8_t> mem = std::vector<uint8_t>(512 * 1024, 0xFF);
  uint8_t status = 0;
  int busyPolls = 0;
  int busyAfterOp = 2;

  void Transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    if (tx.empty()) {
      return;
    }
    const bool busy = (status & 0x01) != 0;
    const bool writable = status == 0x02;  // WEL set, not busy
    const uint32_t address =
        tx.size() >= 4 ? (uint32_t{tx[1]} << 16) | (uint32_t{tx[2]} << 8) | tx[3] : 0;
    switch (tx[0]) {
      case 0x9F:  // JEDEC ID
        rx[1] = 0xEF;
        rx[2] = 0x40;
        rx[3] = 0x13;
        break;
      case 0x05:  // read status register 1
        for (size_t i = 1; i < rx.size(); ++i) {
          rx[i] = status;
        }
        if (busy && --busyPolls <= 0) {
          status &= static_cast<uint8_t>(~0x01);
        }
        break;
      case 0x06:  // write enable
        if (!busy) {
          status |= 0x02;
        }
        break;
      case 0x02:  // page program: wraps inside the 256-byte page
        if (writable) {
          for (size_t i = 4; i < tx.size(); ++i) {
            mem[(address & ~0xFFu) | ((address + i - 4) & 0xFFu)] &= tx[i];
          }
          Busy();
        }
        break;
      case 0x20:  // sector erase
        Erase(writable, address & ~0xFFFu, 4096);
        break;
      case 0xD8:  // 64 KiB block erase
        Erase(writable, address & ~0xFFFFu, 65536);
        break;
      case 0xC7:  // chip erase
        Erase(writable, 0, mem.size());
        break;
      case 0x0B:  // fast read: one dummy byte
        if (!busy) {
          for (size_t i = 5; i < rx.size(); ++i) {
            rx[i] = mem[(address + i - 5) % mem.size()];
          }
        }
        break;
      default:
        break;
    }
  }

  void Erase(bool writable, uint32_t start, size_t size) {
    if (writable) {
      std::fill_n(mem.begin() + start, size, uint8_t{0xFF});
      Busy();
    }
  }

  void Busy() {
    status = 0x01;  // BUSY; WEL clears when the operation ends
    busyPolls = busyAfterOp;
  }
};

// the frames that start with an instruction
std::vector<Bytes> FramesOf(const test::FakeSPIDevice& dev, uint8_t instruction) {
  std::vector<Bytes> out;
  for (const Bytes& f : dev.frames) {
    if (!f.empty() && f[0] == instruction) {
      out.push_back(f);
    }
  }
  return out;
}
}  // namespace

TEST_CASE("W25QFlash identifies the chip", "[w25qflash]") {
  test::FakeSPIDevice dev{0, 10};
  NorModel nor;
  dev.handler = [&nor](std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    nor.Transfer(tx, rx);
  };
  W25QFlash flash{SPI::kPort0, 10};
  CHECK(flash.IsConnected());
  CHECK(flash.GetSize() == 512u * 1024u);
  REQUIRE(dev.frames.size() >= 2);
  CHECK(dev.frames[0] == Bytes{0xAB});  // release power-down first
  CHECK(dev.frames[1] == Bytes{0x9F, 0, 0, 0});
  CHECK(flash.ReadJedecId() == W25QFlash::kW25Q40JedecId);

  platform::hal::SpiDeviceConfig config;
  REQUIRE(sim::GetSpiDeviceConfig(0, 10, &config));
  CHECK(config.mode == 0);
  CHECK(config.clockHz == W25QFlash::kDefaultClockRate);
  CHECK(flash.GetTelemetryType() == "Flash Memory");
}

TEST_CASE("W25QFlash splits programs on page boundaries", "[w25qflash]") {
  test::FakeSPIDevice dev{0, 10};
  NorModel nor;
  dev.handler = [&nor](std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    nor.Transfer(tx, rx);
  };
  W25QFlash flash{SPI::kPort0, 10, 20'000'000};

  std::array<uint8_t, 32> data{};
  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = static_cast<uint8_t>(i + 1);
  }
  REQUIRE(flash.Program(0x1F0, data));
  auto programs = FramesOf(dev, 0x02);
  REQUIRE(programs.size() == 2);  // 16 bytes to the end of page 1, 16 into page 2
  CHECK(programs[0].size() == 4 + 16);
  CHECK(Bytes(programs[0].begin(), programs[0].begin() + 4) == Bytes{0x02, 0x00, 0x01, 0xF0});
  CHECK(programs[1].size() == 4 + 16);
  CHECK(Bytes(programs[1].begin(), programs[1].begin() + 4) == Bytes{0x02, 0x00, 0x02, 0x00});
  CHECK(FramesOf(dev, 0x06).size() == 2);  // a write enable per page
  CHECK(nor.mem[0x1EF] == 0xFF);
  CHECK(nor.mem[0x1F0] == 1);
  CHECK(nor.mem[0x20F] == 32);
  CHECK(nor.mem[0x210] == 0xFF);

  std::array<uint8_t, 32> back{};
  REQUIRE(flash.Read(0x1F0, back));
  CHECK(back == data);
  const Bytes& read = dev.frames.back();
  REQUIRE(read.size() == 5 + 32);
  CHECK(Bytes(read.begin(), read.begin() + 5) == Bytes{0x0B, 0x00, 0x01, 0xF0, 0x00});

  // the range must lie inside the 512 KiB
  const std::array<uint8_t, 2> two{0, 0};
  CHECK_FALSE(flash.Program(512 * 1024 - 1, two));
  std::array<uint8_t, 1> one{};
  CHECK_FALSE(flash.Read(512 * 1024, one));
  CHECK(flash.Read(512 * 1024 - 1, one));
}

TEST_CASE("W25QFlash erases without waiting", "[w25qflash]") {
  test::FakeSPIDevice dev{0, 10};
  NorModel nor;
  dev.handler = [&nor](std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    nor.Transfer(tx, rx);
  };
  W25QFlash flash{SPI::kPort0, 10};
  nor.mem[0x0FFF] = 0;
  nor.mem[0x1234] = 0;
  nor.mem[0x2000] = 0;
  nor.mem[0x20000] = 0;
  nor.busyAfterOp = 3;

  REQUIRE(flash.EraseSector(0x1234));
  CHECK(dev.frames.back() == Bytes{0x20, 0x00, 0x12, 0x34});
  CHECK(flash.IsBusy());                    // status read 1 of 3
  std::array<uint8_t, 1> one{};
  CHECK_FALSE(flash.Read(0x1234, one));     // 2: busy, nothing read
  CHECK_FALSE(flash.EraseBlock64(0));       // 3: busy, nothing started
  CHECK(FramesOf(dev, 0xD8).empty());
  CHECK(flash.WaitReady(100_ms));
  CHECK_FALSE(flash.IsBusy());
  CHECK(nor.mem[0x1234] == 0xFF);
  CHECK(nor.mem[0x0FFF] == 0);  // other sectors untouched
  CHECK(nor.mem[0x2000] == 0);

  REQUIRE(flash.EraseBlock64(0x12345));
  CHECK(dev.frames.back() == Bytes{0xD8, 0x01, 0x23, 0x45});
  CHECK(flash.WaitReady(100_ms));
  CHECK(nor.mem[0x0FFF] == 0);
  CHECK(nor.mem[0x20000] == 0);

  REQUIRE(flash.EraseChip());
  CHECK(dev.frames.back() == Bytes{0xC7});
  CHECK(flash.WaitReady(100_ms));
  CHECK(std::all_of(nor.mem.begin(), nor.mem.end(), [](uint8_t b) { return b == 0xFF; }));

  CHECK_FALSE(flash.EraseSector(512 * 1024));  // outside the chip

  nor.busyAfterOp = 1'000'000;
  REQUIRE(flash.EraseSector(0));
  CHECK_FALSE(flash.WaitReady(5_ms));
  const std::array<uint8_t, 1> zero{0};
  CHECK_FALSE(flash.Program(0, zero));
}

TEST_CASE("W25QFlash without a chip", "[w25qflash]") {
  // no device on CS 11: every transfer reads zeros
  W25QFlash flash{SPI::kPort0, 11};
  CHECK_FALSE(flash.IsConnected());
  CHECK(flash.GetSize() == 0);
  std::array<uint8_t, 4> data{1, 2, 3, 4};
  CHECK_FALSE(flash.Read(0, data));
  CHECK_FALSE(flash.Program(0, data));
  CHECK_FALSE(flash.EraseSector(0));
  CHECK_FALSE(flash.EraseChip());
}
