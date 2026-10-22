// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/memory/NT3H2111.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <span>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {
using Block = std::array<uint8_t, NT3H2111::kBlockSize>;

// The I2C side of an NTAG I2C plus (datasheet rev. 3.0, figures 18 and 19): a 1-byte write sets
// the block address, a 17-byte write stores a block, [FEh, REGA] selects a session register,
// [FEh, REGA, MASK, REGDAT] writes the masked bits of one.
struct FakeNtag {
  explicit FakeNtag(test::FakeI2CBus& bus) : dev{bus.Add(NT3H2111::kDefaultAddress)} {
    blocks[0] = Block{0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
    dev.onWrite = [this](std::span<const uint8_t> data) {
      register_ = -1;
      if (data.size() == 1) {
        block = data[0];
      } else if (data.size() == 1 + NT3H2111::kBlockSize) {
        std::copy(data.begin() + 1, data.end(), blocks[data[0]].begin());
      } else if (data.size() == 2 && data[0] == 0xFE) {
        register_ = data[1];
      } else if (data.size() == 4 && data[0] == 0xFE && data[1] == 6) {
        nsReg = static_cast<uint8_t>((nsReg & ~data[2]) | (data[3] & data[2]));
      }
    };
    dev.onRead = [this](std::span<uint8_t> data) {
      if (register_ >= 0) {
        data[0] = register_ == 6 ? nsReg : 0;
        return;
      }
      const Block& b = blocks[block];
      std::copy_n(b.begin(), std::min<std::size_t>(data.size(), b.size()), data.begin());
    };
  }
  test::FakeI2CBus::Device& dev;
  std::map<int, Block> blocks;
  int block = 0;
  int register_ = -1;
  uint8_t nsReg = 0;
};

Block Pattern(uint8_t first) {
  Block b;
  for (std::size_t i = 0; i < b.size(); ++i) {
    b[i] = static_cast<uint8_t>(first + i);
  }
  return b;
}
}  // namespace

TEST_CASE("NT3H2111 identifies the chip from block 0", "[nt3h2111]") {
  test::FakeI2CBus bus;
  FakeNtag tag{bus};
  NT3H2111 nfc{I2C::Port::PORT_0};
  CHECK(nfc.IsConnected());
  CHECK(nfc.GetAddress() == 0x55);
  REQUIRE(tag.dev.writes.size() == 1);
  CHECK(tag.dev.writes[0] == std::vector<uint8_t>{0x00});  // MEMA of block 0
  CHECK(tag.dev.reads == 1);

  tag.blocks[0][0] = 0x00;
  NT3H2111 other{I2C::Port::PORT_0};
  CHECK_FALSE(other.IsConnected());
}

TEST_CASE("NT3H2111 16-byte block framing", "[nt3h2111]") {
  test::FakeI2CBus bus;
  FakeNtag tag{bus};
  NT3H2111 nfc{I2C::Port::PORT_0};

  Block data = Pattern(0xA0);
  tag.dev.writes.clear();
  auto start = std::chrono::steady_clock::now();
  REQUIRE(nfc.WriteBlock(0x01, data));
  REQUIRE(tag.dev.writes.size() == 1);
  std::vector<uint8_t> expected{0x01};
  expected.insert(expected.end(), data.begin(), data.end());
  CHECK(tag.dev.writes[0] == expected);
  CHECK(tag.blocks[0x01] == data);

  Block read{};
  REQUIRE(nfc.ReadBlock(0x01, read));  // waits for the EEPROM write cycle
  CHECK(std::chrono::steady_clock::now() - start >= std::chrono::milliseconds{5});
  CHECK(read == data);
  CHECK(tag.dev.writes.back() == std::vector<uint8_t>{0x01});

  Block sram = Pattern(0x10);
  REQUIRE(nfc.WriteBlock(0xFB, sram));
  REQUIRE(nfc.ReadBlock(0xFB, read));
  CHECK(read == sram);
  REQUIRE(nfc.WriteBlock(NT3H2111::kLastUserBlock, data));
  REQUIRE(nfc.ReadBlock(NT3H2111::kLastEepromBlock, read));

  CHECK_THROWS_AS(nfc.ReadBlock(0x3B, read), RuntimeError);
  CHECK_THROWS_AS(nfc.ReadBlock(0xFE, read), RuntimeError);
  CHECK_THROWS_AS(nfc.ReadBlock(-1, read), RuntimeError);
  CHECK_THROWS_AS(nfc.WriteBlock(0x00, data), RuntimeError);  // I2C address byte
  CHECK_THROWS_AS(nfc.WriteBlock(0x38, data), RuntimeError);  // lock bytes, AUTH0
  CHECK_THROWS_AS(nfc.WriteBlock(0x3A, data), RuntimeError);  // configuration
  CHECK_THROWS_AS(nfc.WriteBlock(0xFC, data), RuntimeError);

  tag.dev.nack = true;  // missing, or the memory is locked to NFC
  Block unchanged = read;
  CHECK_FALSE(nfc.ReadBlock(0x01, read));
  CHECK(read == unchanged);
  CHECK_FALSE(nfc.WriteBlock(0x02, data));
  CHECK_FALSE(nfc.IsConnected());
}

TEST_CASE("NT3H2111 session register: field and release", "[nt3h2111]") {
  test::FakeI2CBus bus;
  FakeNtag tag{bus};
  NT3H2111 nfc{I2C::Port::PORT_0};

  tag.nsReg = 0x41;  // I2C_LOCKED, RF_FIELD_PRESENT
  CHECK(nfc.IsFieldPresent());
  CHECK(tag.dev.writes.back() == std::vector<uint8_t>{0xFE, 0x06});
  REQUIRE(nfc.ReleaseToNfc());
  CHECK(tag.dev.writes.back() == std::vector<uint8_t>{0xFE, 0x06, 0x40, 0x00});
  CHECK(tag.nsReg == 0x01);  // only I2C_LOCKED cleared
  tag.nsReg = 0x00;
  CHECK_FALSE(nfc.IsFieldPresent());
  tag.dev.nack = true;
  CHECK_FALSE(nfc.IsFieldPresent());
}

TEST_CASE("NT3H2111 field detect pin", "[nt3h2111]") {
  test::FakeI2CBus bus;
  FakeNtag tag{bus};
  NT3H2111 nfc{I2C::Port::PORT_0, NT3H2111::kDefaultAddress, 22};
  sim::DIOSim fd{22};
  tag.dev.writes.clear();
  fd.SetValue(true);  // released: pulled up, no field
  CHECK_FALSE(nfc.IsFieldPresent());
  fd.SetValue(false);  // pulled low: field on
  CHECK(nfc.IsFieldPresent());
  CHECK(tag.dev.writes.empty());  // the pin, not the bus
}
