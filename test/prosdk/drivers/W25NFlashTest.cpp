// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/memory/W25NFlash.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <span>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeSPIDevice.hpp"

using namespace prosdk;

namespace {
using Bytes = std::vector<uint8_t>;
constexpr int kColumns = W25NFlash::kPageSize + W25NFlash::kSpareSize;

// A W25N01GVxxIG in buffer read mode: pages of 2112 bytes (absent from `pages`: erased), the
// 2112-byte data buffer, SR-1 (protection, BP3-BP0/TB set after reset), SR-2 (ECC-E, BUF) and
// SR-3 (ECC bits, P-FAIL, E-FAIL, WEL, BUSY). Every operation reports BUSY for busyAfterOp reads
// of SR-3. Any protection bit protects the whole array (a simplification).
struct NandModel {
  std::map<int, Bytes> pages;
  Bytes buffer = Bytes(kColumns, 0xFF);
  uint8_t sr1 = 0x7C;
  uint8_t sr2 = 0x18;
  uint8_t sr3 = 0;
  int busyPolls = 0;
  int busyAfterOp = 1;
  std::map<int, int> eccOfPage;    // ECC-1/ECC-0 reported after a Page Data Read
  std::vector<uint8_t> sr2AtRead;  // SR-2 at every Page Data Read

  Bytes Page(int page) const {
    auto it = pages.find(page);
    return it == pages.end() ? Bytes(kColumns, 0xFF) : it->second;
  }

  uint8_t* Register(uint8_t address) {
    switch (address & 0xF0) {
      case 0xA0:
        return &sr1;
      case 0xB0:
        return &sr2;
      default:
        return &sr3;
    }
  }

  void Transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    if (tx.empty()) {
      return;
    }
    const bool busy = (sr3 & 0x01) != 0;
    const bool wel = (sr3 & 0x02) != 0;
    const int page = tx.size() >= 4 ? (tx[2] << 8) | tx[3] : 0;
    const int column = tx.size() >= 3 ? ((tx[1] << 8) | tx[2]) & 0x0FFF : 0;
    switch (tx[0]) {
      case 0xFF:  // reset: SR-1 protects all, BUF = 1 (xxIG), ECC-E kept
        sr1 = 0x7C;
        sr2 = static_cast<uint8_t>((sr2 & 0x10) | 0x08);
        sr3 = 0;
        Busy();
        break;
      case 0x9F:  // JEDEC ID after one dummy byte
        rx[2] = 0xEF;
        rx[3] = 0xAA;
        rx[4] = 0x21;
        break;
      case 0x0F:  // read status register
        rx[2] = *Register(tx[1]);
        if ((tx[1] & 0xF0) == 0xC0 && busy && --busyPolls <= 0) {
          sr3 &= static_cast<uint8_t>(~0x01);
        }
        break;
      case 0x1F:  // write status register
        *Register(tx[1]) = tx[2];
        break;
      case 0x06:  // write enable
        if (!busy) {
          sr3 |= 0x02;
        }
        break;
      case 0x13: {  // page data read
        buffer = Page(page);
        sr2AtRead.push_back(sr2);
        const int ecc = (sr2 & 0x10) != 0 && eccOfPage.count(page) ? eccOfPage[page] : 0;
        sr3 = static_cast<uint8_t>((sr3 & ~0x32) | (ecc << 4));
        Busy();
        break;
      }
      case 0x0B:  // fast read from the buffer: column, one dummy byte
        for (size_t i = 4; i < rx.size(); ++i) {
          const size_t at = static_cast<size_t>(column) + i - 4;
          rx[i] = at < buffer.size() ? buffer[at] : 0xFF;
        }
        break;
      case 0x02:  // load program data: the rest of the buffer to FFh
        if (wel) {
          std::fill(buffer.begin(), buffer.end(), uint8_t{0xFF});
          for (size_t i = 3; i < tx.size() && column + i - 3 < buffer.size(); ++i) {
            buffer[column + i - 3] = tx[i];
          }
        }
        break;
      case 0x10:  // program execute
        if (wel) {
          sr3 &= static_cast<uint8_t>(~0x08);
          if ((sr1 & 0x7C) != 0) {
            sr3 |= 0x08;  // P-FAIL: protected
          } else {
            Bytes p = Page(page);
            for (size_t i = 0; i < p.size(); ++i) {
              p[i] &= buffer[i];
            }
            pages[page] = p;
          }
          sr3 &= static_cast<uint8_t>(~0x02);
          Busy();
        }
        break;
      case 0xD8:  // block erase
        if (wel) {
          sr3 &= static_cast<uint8_t>(~0x04);
          if ((sr1 & 0x7C) != 0) {
            sr3 |= 0x04;  // E-FAIL: protected
          } else {
            const int first = page & ~63;
            for (int p = first; p < first + 64; ++p) {
              pages.erase(p);
            }
          }
          sr3 &= static_cast<uint8_t>(~0x02);
          Busy();
        }
        break;
      default:
        break;
    }
  }

  void Busy() {
    sr3 |= 0x01;
    busyPolls = busyAfterOp;
  }
};

struct Fixture {
  test::FakeSPIDevice dev{0, 12};
  NandModel nand;
  Fixture() {
    dev.handler = [this](std::span<const uint8_t> tx, std::span<uint8_t> rx) {
      nand.Transfer(tx, rx);
    };
  }
  bool Sent(const Bytes& frame) const {
    return std::find(dev.frames.begin(), dev.frames.end(), frame) != dev.frames.end();
  }
};
}  // namespace

TEST_CASE("W25NFlash resets, unprotects and identifies the chip", "[w25nflash]") {
  Fixture f;
  W25NFlash flash{SPI::kPort0, 12};
  CHECK(flash.IsConnected());
  auto reset = std::find(f.dev.frames.begin(), f.dev.frames.end(), Bytes{0xFF});
  auto id = std::find(f.dev.frames.begin(), f.dev.frames.end(), Bytes{0x9F, 0, 0, 0, 0});
  REQUIRE(reset != f.dev.frames.end());
  REQUIRE(id != f.dev.frames.end());
  CHECK(reset < id);
  CHECK(f.Sent({0x1F, 0xA0, 0x00}));  // block protection cleared
  CHECK(f.Sent({0x1F, 0xB0, 0x18}));  // ECC on, buffer read mode
  CHECK(f.nand.sr1 == 0x00);
  CHECK(f.nand.sr2 == 0x18);
  CHECK(flash.ReadId() == W25NFlash::kJedecId);
  CHECK(flash.GetTelemetryType() == "Flash Memory");
}

TEST_CASE("W25NFlash programs, reads and erases pages", "[w25nflash]") {
  Fixture f;
  W25NFlash flash{SPI::kPort0, 12};
  Bytes data(300);
  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = static_cast<uint8_t>(i);
  }
  REQUIRE(flash.ProgramPage(70, data));
  auto load = std::find_if(f.dev.frames.begin(), f.dev.frames.end(),
                           [](const Bytes& fr) { return !fr.empty() && fr[0] == 0x02; });
  REQUIRE(load != f.dev.frames.end());
  CHECK(load->size() == 3 + 300);
  CHECK(Bytes(load->begin(), load->begin() + 3) == Bytes{0x02, 0x00, 0x00});
  CHECK(f.Sent({0x10, 0x00, 0x00, 70}));
  CHECK(f.nand.pages.count(70) == 1);

  Bytes back(300);
  REQUIRE(flash.ReadPage(70, back));
  CHECK(back == data);
  CHECK(flash.GetEccStatus() == W25NFlash::EccStatus::kOk);
  CHECK(f.Sent({0x13, 0x00, 0x00, 70}));
  CHECK(Bytes(f.dev.frames[f.dev.frames.size() - 2].begin(),
              f.dev.frames[f.dev.frames.size() - 2].begin() + 4) == Bytes{0x0B, 0, 0, 0});

  std::array<uint8_t, 2> two{};
  REQUIRE(flash.ReadPage(70, two, 10));
  CHECK(two == std::array<uint8_t, 2>{10, 11});
  CHECK(f.Sent({0x0B, 0x00, 0x0A, 0x00, 0, 0}));
  std::array<uint8_t, 4> tail{};
  REQUIRE(flash.ReadPage(70, tail, W25NFlash::kPageSize - 2));  // across into the spare area
  CHECK(tail == std::array<uint8_t, 4>{0xFF, 0xFF, 0xFF, 0xFF});

  // the spare area is reached by column
  const std::array<uint8_t, 1> mark{0x5A};
  REQUIRE(flash.ProgramPage(71, mark, W25NFlash::kPageSize + 4));
  std::array<uint8_t, 1> one{};
  REQUIRE(flash.ReadPage(71, one, W25NFlash::kPageSize + 4));
  CHECK(one[0] == 0x5A);

  REQUIRE(flash.EraseBlock(1));  // pages 64 to 127
  CHECK(f.Sent({0xD8, 0x00, 0x00, 64}));
  REQUIRE(flash.ReadPage(70, back));
  CHECK(std::all_of(back.begin(), back.end(), [](uint8_t b) { return b == 0xFF; }));

  CHECK_FALSE(flash.ReadPage(W25NFlash::kPageCount, back));
  CHECK_FALSE(flash.ReadPage(-1, back));
  CHECK_FALSE(flash.ReadPage(0, two, kColumns - 1));
  CHECK_FALSE(flash.ProgramPage(0, std::span<const uint8_t>{}));
  CHECK_FALSE(flash.ProgramPage(0, two, kColumns - 1));
  CHECK_FALSE(flash.EraseBlock(W25NFlash::kBlockCount));
}

TEST_CASE("W25NFlash reports ECC results and failures", "[w25nflash]") {
  Fixture f;
  W25NFlash flash{SPI::kPort0, 12};
  std::array<uint8_t, 8> data{};
  f.nand.eccOfPage[5] = 1;
  CHECK(flash.ReadPage(5, data));
  CHECK(flash.GetEccStatus() == W25NFlash::EccStatus::kCorrected);
  f.nand.eccOfPage[6] = 2;
  CHECK_FALSE(flash.ReadPage(6, data));
  CHECK(flash.GetEccStatus() == W25NFlash::EccStatus::kUncorrectable);
  CHECK(flash.ReadPage(7, data));
  CHECK(flash.GetEccStatus() == W25NFlash::EccStatus::kOk);

  // a protected array: the chip reports P-FAIL and E-FAIL
  f.nand.sr1 = 0x7C;
  CHECK_FALSE(flash.ProgramPage(8, data));
  CHECK_FALSE(flash.EraseBlock(2));
  f.nand.sr1 = 0x00;
  CHECK(flash.ProgramPage(8, data));

  // the chip never finishes
  f.nand.busyAfterOp = 1'000'000;
  CHECK_FALSE(flash.EraseBlock(3));
}

TEST_CASE("W25NFlash reads bad-block markers with the ECC off", "[w25nflash]") {
  Fixture f;
  W25NFlash flash{SPI::kPort0, 12};
  Bytes spareMarked(kColumns, 0xFF);
  spareMarked[W25NFlash::kPageSize] = 0x00;
  f.nand.pages[3 * 64] = spareMarked;
  Bytes mainMarked(kColumns, 0xFF);
  mainMarked[0] = 0x00;
  f.nand.pages[4 * 64] = mainMarked;

  CHECK(flash.IsBadBlock(3));
  CHECK(flash.IsBadBlock(4));
  CHECK_FALSE(flash.IsBadBlock(5));
  REQUIRE_FALSE(f.nand.sr2AtRead.empty());
  CHECK(f.nand.sr2AtRead.back() == 0x08);  // ECC-E off, BUF on during the marker read
  CHECK(f.nand.sr2 == 0x18);               // and back on afterwards
  CHECK(f.Sent({0x0B, 0x08, 0x00, 0x00, 0}));  // column 2048
  CHECK(flash.IsBadBlock(-1));
  CHECK(flash.IsBadBlock(W25NFlash::kBlockCount));
}

TEST_CASE("W25NFlash without a chip", "[w25nflash]") {
  // no device on CS 13: every transfer reads zeros
  W25NFlash flash{SPI::kPort0, 13};
  CHECK_FALSE(flash.IsConnected());
  CHECK(flash.ReadId() == 0);
  std::array<uint8_t, 4> data{1, 2, 3, 4};
  CHECK_FALSE(flash.ReadPage(0, data));  // zeros from no chip are not data
  CHECK_FALSE(flash.ProgramPage(0, data));
  CHECK_FALSE(flash.EraseBlock(0));
  CHECK(flash.IsBadBlock(0));
  CHECK(flash.GetEccStatus() == W25NFlash::EccStatus::kOk);
}
