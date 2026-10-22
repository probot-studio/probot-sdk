// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/range/VL53L7CX.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeI2CBus.hpp"
#include "prosdk/drivers/range/st/platform.h"
#include "prosdk/drivers/range/st/VL53L7CXPlatform.hpp"
extern "C" {
#include "prosdk/drivers/range/st/vl53l7cx_api.h"
}
#include "prosdk/simulation/DIOSim.hpp"

using namespace prosdk;
using namespace std::chrono_literals;

namespace {
using Bytes = std::vector<uint8_t>;

// Writes a ULD block header (type:4, size:12, idx:16, low bits first) in host order.
void PutHeader(uint8_t* at, uint32_t type, uint32_t size, uint32_t idx) {
  uint32_t word = type | (size << 4) | (idx << 16);
  std::memcpy(at, &word, sizeof(word));
}
}  // namespace

TEST_CASE("VL53L7CX platform sends 16-bit indexes and splits long blocks", "[vl53l7cx]") {
  test::FakeI2CBus bus;
  auto& sensor = bus.Add(0x29, 16);
  detail::VL53L7CXBus link{I2C::Port::PORT_0, 0x29};
  VL53L7CX_Platform platform{};
  link.Attach(&platform, 0x29);
  CHECK(platform.address == 0x52);  // the ULD's 8-bit form

  // 2500 bytes: chunks of 1024, 1024 and 452, each with the index of its first byte
  Bytes data(2500);
  for (std::size_t i = 0; i < data.size(); ++i) {
    data[i] = static_cast<uint8_t>(i * 7 + 1);
  }
  REQUIRE(VL53L7CX_WrMulti(&platform, 0x1000, data.data(), data.size()) == 0);
  REQUIRE(sensor.writes.size() == 3);
  CHECK(sensor.writes[0].size() == 2 + 1024);
  CHECK(sensor.writes[1].size() == 2 + 1024);
  CHECK(sensor.writes[2].size() == 2 + 452);
  CHECK(Bytes(sensor.writes[0].begin(), sensor.writes[0].begin() + 2) == Bytes{0x10, 0x00});
  CHECK(Bytes(sensor.writes[1].begin(), sensor.writes[1].begin() + 2) == Bytes{0x14, 0x00});
  CHECK(Bytes(sensor.writes[2].begin(), sensor.writes[2].begin() + 2) == Bytes{0x18, 0x00});
  bool stored = true;
  for (std::size_t i = 0; i < data.size(); ++i) {
    stored = stored && sensor.regs[0x1000 + i] == data[i];
  }
  CHECK(stored);

  // reads: one index write plus one read per chunk
  sensor.writes.clear();
  Bytes back(1500);
  REQUIRE(VL53L7CX_RdMulti(&platform, 0x1000, back.data(), back.size()) == 0);
  CHECK(sensor.reads == 2);
  REQUIRE(sensor.writes.size() == 2);
  CHECK(sensor.writes[0] == Bytes{0x10, 0x00});
  CHECK(sensor.writes[1] == Bytes{0x14, 0x00});
  CHECK(Bytes(data.begin(), data.begin() + 1500) == back);

  // single registers
  sensor.writes.clear();
  REQUIRE(VL53L7CX_WrByte(&platform, 0x7FFF, 0x02) == 0);
  CHECK(sensor.writes == std::vector<Bytes>{{0x7F, 0xFF, 0x02}});
  uint8_t value = 0;
  REQUIRE(VL53L7CX_RdByte(&platform, 0x7FFF, &value) == 0);
  CHECK(value == 0x02);

  // the sensor sends big-endian 32-bit words
  std::array<uint8_t, 8> words{1, 2, 3, 4, 5, 6, 7, 8};
  VL53L7CX_SwapBuffer(words.data(), words.size());
  CHECK(words == std::array<uint8_t, 8>{4, 3, 2, 1, 8, 7, 6, 5});

  // a NACK fails the call; after abort nothing reaches the bus
  sensor.nack = true;
  CHECK(VL53L7CX_WrByte(&platform, 0x7FFF, 0x00) != 0);
  CHECK(VL53L7CX_RdByte(&platform, 0x0000, &value) != 0);
  sensor.nack = false;
  link.abort = true;
  std::size_t writes = sensor.writes.size();
  CHECK(VL53L7CX_WrByte(&platform, 0x7FFF, 0x00) != 0);
  CHECK(VL53L7CX_RdMulti(&platform, 0x0000, back.data(), 4) != 0);
  CHECK(VL53L7CX_WaitMs(&platform, 10'000) == 0);  // returns at once
  CHECK(sensor.writes.size() == writes);
}

TEST_CASE("VL53L7CX ULD identity check and address change", "[vl53l7cx]") {
  test::FakeI2CBus bus;
  auto& sensor = bus.Add(0x29, 16);
  auto& moved = bus.Add(0x30, 16);
  sensor.regs[0x0000] = 0xF0;  // device id
  sensor.regs[0x0001] = 0x02;  // revision id
  detail::VL53L7CXBus link{I2C::Port::PORT_0, 0x29};
  auto dev = std::make_unique<VL53L7CX_Configuration>();
  link.Attach(&dev->platform, 0x29);

  uint8_t alive = 0;
  REQUIRE(vl53l7cx_is_alive(dev.get(), &alive) == 0);
  CHECK(alive == 1);
  CHECK(sensor.writes.front() == Bytes{0x7F, 0xFF, 0x00});  // page 0 for the id registers
  CHECK(sensor.writes.back() == Bytes{0x7F, 0xFF, 0x02});

  // the ULD writes the 7-bit address to register 0x0004, then talks at the new address
  sensor.writes.clear();
  REQUIRE(vl53l7cx_set_i2c_address(dev.get(), 0x30 << 1) == 0);
  CHECK(sensor.writes == std::vector<Bytes>{{0x7F, 0xFF, 0x00}, {0x00, 0x04, 0x30}});
  CHECK(moved.writes == std::vector<Bytes>{{0x7F, 0xFF, 0x02}});
  CHECK(link.i2c.GetDeviceAddress() == 0x30);

  sensor.regs[0x0000] = 0x00;  // another chip
  link.Attach(&dev->platform, 0x29);
  REQUIRE(vl53l7cx_is_alive(dev.get(), &alive) == 0);
  CHECK(alive == 0);
}

TEST_CASE("VL53L7CX frame decode: chunked read, word swap, ULD conversion", "[vl53l7cx]") {
  test::FakeI2CBus bus;
  auto& sensor = bus.Add(0x29, 16);
  detail::VL53L7CXBus link{I2C::Port::PORT_0, 0x29};
  auto dev = std::make_unique<VL53L7CX_Configuration>();
  link.Attach(&dev->platform, 0x29);

  // A synthetic 4x4 frame in host order (after VL53L7CX_SwapBuffer), laid out as ULD 2.0.0 parses
  // it; not a recording of a real sensor. 128 bytes is what vl53l7cx_start_ranging() computes for
  // 4x4 with the outputs platform.h leaves on (target count, distance, target status).
  constexpr uint32_t kSize = 128;
  std::array<uint8_t, kSize> frame{};
  frame[0] = 7;  // stream count
  frame[8] = 0x12;  // header id, repeated in the footer
  frame[9] = 0x34;
  PutHeader(&frame[16], 1, 16, VL53L7CX_NB_TARGET_DETECTED_IDX);  // 16 x u8 at 20
  PutHeader(&frame[36], 2, 16, VL53L7CX_DISTANCE_IDX);            // 16 x i16 at 40
  PutHeader(&frame[72], 1, 16, VL53L7CX_TARGET_STATUS_IDX);       // 16 x u8 at 76
  for (int zone = 0; zone < 16; ++zone) {
    frame[20 + zone] = zone == 3 ? 0 : 1;  // zone 3 saw no target
    int16_t raw = static_cast<int16_t>((100 + 10 * zone) * 4);  // firmware unit: mm / 4
    if (zone == 5) {
      raw = -8;  // below 0: reported as 0
    }
    std::memcpy(&frame[40 + 2 * zone], &raw, sizeof(raw));
    frame[76 + zone] = zone % 2 == 0 ? 5 : 9;
  }
  frame[kSize - 4] = 0x12;
  frame[kSize - 3] = 0x34;

  auto load = [&] {
    std::array<uint8_t, kSize> wire = frame;
    VL53L7CX_SwapBuffer(wire.data(), kSize);  // big-endian words on the bus
    for (uint32_t i = 0; i < kSize; ++i) {
      sensor.regs[i] = wire[i];
    }
  };
  load();
  dev->data_read_size = kSize;
  auto results = std::make_unique<VL53L7CX_ResultsData>();
  REQUIRE(vl53l7cx_get_ranging_data(dev.get(), results.get()) == 0);
  CHECK(sensor.writes.back() == Bytes{0x00, 0x00});  // read from index 0
  CHECK(dev->streamcount == 7);
  CHECK(results->distance_mm[0] == 100);
  CHECK(results->distance_mm[1] == 110);
  CHECK(results->distance_mm[5] == 0);
  CHECK(results->distance_mm[15] == 250);
  CHECK(results->target_status[0] == 5);
  CHECK(results->target_status[1] == 9);
  CHECK(results->target_status[3] == 255);  // no target
  CHECK(results->nb_target_detected[3] == 0);

  frame[kSize - 3] = 0x35;  // footer does not match the header
  load();
  CHECK(vl53l7cx_get_ranging_data(dev.get(), results.get()) == VL53L7CX_STATUS_CORRUPTED_FRAME);
}

TEST_CASE("VL53L7CX without a sensor is not ready and stops at once", "[vl53l7cx]") {
  test::FakeI2CBus bus;  // nothing at 0x30; another chip (ID 0x00, not 0xF0) at 0x29
  auto& other = bus.Add(0x29, 16);
  auto tof = std::make_unique<VL53L7CX>(I2C::Port::PORT_0, /*lpnChannel=*/6, /*address=*/0x30);
  sim::DIOSim lpn{6};
  CHECK_FALSE(lpn.GetValue());  // I2C held off until the thread runs
  std::atomic<bool> lpnHigh{true};
  other.onWrite = [&](std::span<const uint8_t>) {  // writes are still logged; reads return 0
    if (!lpn.GetValue()) {
      lpnHigh = false;
    }
  };
  {
    std::scoped_lock lock{bus.mutex};
    CHECK(other.writes.empty());  // the constructor does not use the bus
  }

  CHECK_FALSE(tof->IsReady());  // starts the thread
  std::array<int16_t, VL53L7CX::kMaxZones> distances{};
  std::array<uint8_t, VL53L7CX::kMaxZones> status{};
  CHECK(tof->GetDistances(distances, status) == 0);
  CHECK(tof->GetTargetStatus(status) == 0);

  // the thread raises LPn, finds nothing at 0x30 and probes 0x29 with vl53l7cx_is_alive (page 0,
  // ID registers 0x0000 and 0x0001, page 2); the ID does not match
  const std::vector<Bytes> probe{{0x7F, 0xFF, 0x00}, {0x00, 0x00}, {0x00, 0x01}, {0x7F, 0xFF, 0x02}};
  auto deadline = std::chrono::steady_clock::now() + 2s;
  bool probed = false;
  while (!probed && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
    std::scoped_lock lock{bus.mutex};
    probed = other.writes.size() >= probe.size();
  }
  {
    std::scoped_lock lock{bus.mutex};
    CHECK(other.writes == probe);  // no firmware upload to a chip that is not a VL53L7CX
  }
  CHECK(lpnHigh);  // I2C on during the probe

  // after the failure LPn goes low again (I2C off) until the next attempt, 1 s later
  deadline = std::chrono::steady_clock::now() + 500ms;
  while (lpn.GetValue() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  CHECK_FALSE(lpn.GetValue());
  CHECK_FALSE(tof->IsReady());
  CHECK(tof->GetDistances(distances) == 0);

  // the destructor must not wait out the 1 s retry pause
  auto start = std::chrono::steady_clock::now();
  tof.reset();
  CHECK(std::chrono::steady_clock::now() - start < 500ms);
}
