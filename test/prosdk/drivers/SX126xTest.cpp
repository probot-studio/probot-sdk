// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/radio/SX126x.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FakeSPIDevice.hpp"
#include "prosdk/simulation/DIOSim.hpp"

using namespace prosdk;

namespace {
using Bytes = std::vector<uint8_t>;

constexpr int kCs = 10;
constexpr int kBusy = 20;
constexpr int kDio1 = 21;
constexpr int kReset = 22;
constexpr int kRxEn = 23;
constexpr int kTxEn = 24;

// An SX1262 as the driver sees it: the status byte (chip mode), the IRQ register behind DIO1,
// the device errors, the registers the driver touches and the 256-byte data buffer. The driver's
// thread calls Transfer, the test sets the received packet and raises IRQs; both under mutex.
struct RadioModel {
  std::mutex mutex;
  sim::DIOSim dio1{kDio1};
  bool present = true;               // false: nothing answers (MISO reads zeros)
  uint8_t chipMode = 0x2;            // STDBY_RC
  uint16_t irq = 0;
  uint16_t deviceErrors = 0x0020;    // XOSC_START_ERR from the power-on start (TCXO)
  uint16_t calibrationErrors = 0;    // device errors after Calibrate
  std::vector<uint8_t> regs = std::vector<uint8_t>(0x1000, 0);
  std::array<uint8_t, 256> buffer{};
  uint8_t rxStart = 0;
  uint8_t rxLength = 0;
  uint8_t rssiPkt = 0;
  uint8_t snrPkt = 0;

  RadioModel() {
    regs[0x0736] = 0x0D;  // reset values, table 12-1
    regs[0x0740] = 0x14;
    regs[0x0741] = 0x24;
    regs[0x0889] = 0x01;
    regs[0x08D8] = 0xC8;
  }

  void Raise(uint16_t flags) {
    std::scoped_lock lock{mutex};
    irq |= flags;
    dio1.SetValue(true);
  }

  void Transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    std::scoped_lock lock{mutex};
    if (tx.empty() || !present) {
      return;
    }
    const uint16_t address = tx.size() >= 3 ? static_cast<uint16_t>((tx[1] << 8) | tx[2]) : 0;
    switch (tx[0]) {
      case 0xC0:  // GetStatus
        rx[1] = static_cast<uint8_t>(chipMode << 4);
        break;
      case 0x80:  // SetStandby
        chipMode = 0x2;
        break;
      case 0x82:  // SetRx
        chipMode = 0x5;
        break;
      case 0x83:  // SetTx
        chipMode = 0x6;
        break;
      case 0x07:  // ClearDeviceErrors
        deviceErrors = 0;
        break;
      case 0x89:  // Calibrate
        deviceErrors = calibrationErrors;
        break;
      case 0x17:  // GetDeviceErrors
        rx[2] = static_cast<uint8_t>(deviceErrors >> 8);
        rx[3] = static_cast<uint8_t>(deviceErrors);
        break;
      case 0x12:  // GetIrqStatus
        rx[2] = static_cast<uint8_t>(irq >> 8);
        rx[3] = static_cast<uint8_t>(irq);
        break;
      case 0x02:  // ClearIrqStatus
        irq &= static_cast<uint16_t>(~address);
        if (irq == 0) {
          dio1.SetValue(false);
        }
        break;
      case 0x0D:  // WriteRegister
        for (size_t i = 3; i < tx.size(); ++i) {
          regs[address + i - 3] = tx[i];
        }
        break;
      case 0x1D:  // ReadRegister: one NOP after the address
        for (size_t i = 4; i < rx.size(); ++i) {
          rx[i] = regs[address + i - 4];
        }
        break;
      case 0x0E:  // WriteBuffer
        for (size_t i = 2; i < tx.size(); ++i) {
          buffer[(tx[1] + i - 2) & 0xFF] = tx[i];
        }
        break;
      case 0x1E:  // ReadBuffer: one NOP after the offset
        for (size_t i = 3; i < rx.size(); ++i) {
          rx[i] = buffer[(tx[1] + i - 3) & 0xFF];
        }
        break;
      case 0x13:  // GetRxBufferStatus
        rx[2] = rxLength;
        rx[3] = rxStart;
        break;
      case 0x14:  // GetPacketStatus
        rx[2] = rssiPkt;
        rx[3] = snrPkt;
        break;
      default:
        break;
    }
  }
};

// A radio with its model: BUSY and DIO1 low (the sim starts inputs high).
struct Bench {
  explicit Bench(const SX126x::Config& config = {})
      : radio{SPI::kPort0, kCs, kBusy, kDio1, kReset, kRxEn, kTxEn, config} {
    fake.handler = [this](std::span<const uint8_t> tx, std::span<uint8_t> rx) {
      model.Transfer(tx, rx);
    };
    busy.SetValue(false);
    std::scoped_lock lock{model.mutex};
    model.dio1.SetValue(false);
  }

  test::FakeSPIDevice fake{0, kCs};
  RadioModel model;
  SX126x radio;
  sim::DIOSim busy{kBusy};
  sim::DIOSim reset{kReset};
  sim::DIOSim rxEn{kRxEn};
  sim::DIOSim txEn{kTxEn};
};

template <typename F>
bool WaitFor(F condition) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  return true;
}
}  // namespace

TEST_CASE("SX126x Begin configures the chip in the datasheet order", "[sx126x]") {
  SX126x::Config config;
  config.frequency = units::hertz_t{868'000'000.0};
  config.syncWord = 0x3444;
  config.preambleLength = 12;
  Bench b{config};
  b.model.regs[0x0736] = 0x09;  // IQ bit 2 clear, to see it set
  REQUIRE(b.radio.Begin());
  CHECK(b.radio.IsConnected());
  CHECK(b.reset.GetValue());  // released after the pulse
  CHECK_FALSE(b.rxEn.GetValue());
  CHECK_FALSE(b.txEn.GetValue());

  const std::vector<Bytes> expected = {
      {0x80, 0x00},                          // SetStandby(STDBY_RC)
      {0xC0, 0x00},                          // GetStatus: STDBY_RC
      {0x97, 0x02, 0x00, 0x01, 0x40},        // SetDIO3AsTcxoCtrl(1.8 V, 5 ms)
      {0x07, 0x00, 0x00},                    // ClearDeviceErrors
      {0x89, 0x7F},                          // Calibrate(all)
      {0x98, 0xD7, 0xDB},                    // CalibrateImage(863-870 MHz)
      {0x17, 0x00, 0x00, 0x00},              // GetDeviceErrors
      {0x8A, 0x01},                          // SetPacketType(LoRa)
      {0x86, 0x36, 0x40, 0x00, 0x00},        // SetRfFrequency(868 MHz)
      {0x95, 0x04, 0x07, 0x00, 0x01},        // SetPaConfig(SX1262, +22 dBm PA)
      {0x8E, 0x0E, 0x04},                    // SetTxParams(14 dBm, 200 us)
      {0x1D, 0x08, 0xD8, 0x00, 0x00},        // ReadRegister(TxClampConfig)
      {0x0D, 0x08, 0xD8, 0xDE},              // bits 4-1 set
      {0x8F, 0x00, 0x00},                    // SetBufferBaseAddress
      {0x8B, 0x07, 0x04, 0x01, 0x00},        // SF7, 125 kHz, 4/5, no LDRO
      {0x8C, 0x00, 0x0C, 0x00, 0xFF, 0x01, 0x00},  // preamble 12, explicit, 255, CRC, IQ
      {0x1D, 0x07, 0x36, 0x00, 0x00},        // ReadRegister(IQ polarity)
      {0x0D, 0x07, 0x36, 0x0D},              // bit 2 set (15.4)
      {0x0D, 0x07, 0x40, 0x34, 0x44},        // sync word
      {0x08, 0x02, 0x63, 0x02, 0x63, 0x00, 0x00, 0x00, 0x00},  // IRQs on DIO1
      {0x02, 0x03, 0xFF},                    // ClearIrqStatus(all)
  };
  CHECK(b.fake.frames == expected);
  CHECK(b.model.regs[0x08D8] == 0xDE);
  CHECK(b.model.regs[0x0736] == 0x0D);
}

TEST_CASE("SX126x Send writes the buffer and switches the RF path", "[sx126x]") {
  Bench b;
  REQUIRE(b.radio.Begin());
  CHECK(b.radio.IsTxDone());
  b.fake.frames.clear();

  const uint8_t payload[] = {0x01, 0x02, 0x03};
  REQUIRE(b.radio.Send(payload));
  CHECK_FALSE(b.radio.IsTxDone());
  CHECK(b.txEn.GetValue());
  CHECK_FALSE(b.rxEn.GetValue());
  // timeout: 2 x 30.976 ms + 100 ms = 10365 steps of 15.625 us
  const std::vector<Bytes> expected = {
      {0x80, 0x00},
      {0x8C, 0x00, 0x08, 0x00, 0x03, 0x01, 0x00},
      {0x1D, 0x08, 0x89, 0x00, 0x00},  // ReadRegister(TxModulation)
      {0x0D, 0x08, 0x89, 0x05},        // bit 2 set below 500 kHz (15.1)
      {0x0E, 0x00, 0x01, 0x02, 0x03},
      {0x02, 0x03, 0xFF},
      {0x83, 0x00, 0x28, 0x7D},
  };
  CHECK(b.fake.frames == expected);
  CHECK(b.model.regs[0x0889] == 0x05);
  CHECK_FALSE(b.radio.Send(payload));  // one send at a time
  CHECK_FALSE(b.radio.Send(std::span<const uint8_t>{}));

  b.fake.frames.clear();
  b.model.Raise(0x0001);  // TxDone
  REQUIRE(WaitFor([&] { return b.radio.IsTxDone(); }));
  CHECK_FALSE(b.txEn.GetValue());
  CHECK(b.radio.GetSentCount() == 1);
  CHECK(b.radio.GetTxTimeoutCount() == 0);
  const std::vector<Bytes> irq = {{0x12, 0x00, 0x00, 0x00}, {0x02, 0x00, 0x01}};
  CHECK(b.fake.frames == irq);
  CHECK(b.model.irq == 0);

  REQUIRE(b.radio.Send(payload));
  b.model.Raise(0x0200);  // Timeout
  REQUIRE(WaitFor([&] { return b.radio.IsTxDone(); }));
  CHECK(b.radio.GetSentCount() == 1);
  CHECK(b.radio.GetTxTimeoutCount() == 1);
}

TEST_CASE("SX126x receives packets and drops CRC errors", "[sx126x]") {
  Bench b;
  REQUIRE(b.radio.Begin());
  b.fake.frames.clear();
  REQUIRE(b.radio.StartReceive());
  CHECK(b.rxEn.GetValue());
  CHECK_FALSE(b.txEn.GetValue());
  const std::vector<Bytes> expected = {
      {0x80, 0x00},
      {0x8C, 0x00, 0x08, 0x00, 0xFF, 0x01, 0x00},
      {0x02, 0x03, 0xFF},
      {0x82, 0xFF, 0xFF, 0xFF},  // continuous
  };
  CHECK(b.fake.frames == expected);
  CHECK_FALSE(b.radio.Poll().has_value());

  {
    std::scoped_lock lock{b.model.mutex};
    b.model.buffer[0x10] = 'h';
    b.model.buffer[0x11] = 'i';
    b.model.buffer[0x12] = '!';
    b.model.rxStart = 0x10;
    b.model.rxLength = 3;
    b.model.rssiPkt = 80;    // -40 dBm
    b.model.snrPkt = 0xF8;   // -8 / 4 = -2 dB
  }
  b.model.Raise(0x0002);  // RxDone
  std::optional<SX126x::Packet> packet;
  REQUIRE(WaitFor([&] {
    packet = b.radio.Poll();
    return packet.has_value();
  }));
  CHECK(packet->data == Bytes{'h', 'i', '!'});
  CHECK(packet->rssi == Catch::Approx(-40.0));
  CHECK(packet->snr == Catch::Approx(-2.0));
  CHECK(b.radio.GetReceivedCount() == 1);

  b.model.Raise(0x0042);  // RxDone + CrcErr
  REQUIRE(WaitFor([&] { return b.radio.GetCrcErrorCount() == 1; }));
  CHECK_FALSE(b.radio.Poll().has_value());
  CHECK(b.radio.GetReceivedCount() == 1);

  // a send pauses the reception; it resumes after TxDone
  const uint8_t payload[] = {0xAA};
  REQUIRE(b.radio.Send(payload));
  CHECK(b.txEn.GetValue());
  CHECK_FALSE(b.rxEn.GetValue());
  b.fake.frames.clear();
  b.model.Raise(0x0001);
  REQUIRE(WaitFor([&] { return b.radio.IsTxDone(); }));
  CHECK(b.rxEn.GetValue());
  CHECK_FALSE(b.txEn.GetValue());
  REQUIRE_FALSE(b.fake.frames.empty());
  CHECK(b.fake.frames.back() == Bytes{0x82, 0xFF, 0xFF, 0xFF});

  b.radio.StopReceive();
  CHECK_FALSE(b.rxEn.GetValue());
  CHECK(b.fake.frames.back() == Bytes{0x80, 0x00});
}

TEST_CASE("SX126x waits for BUSY and reports a missing chip", "[sx126x]") {
  {
    Bench b;
    b.busy.SetValue(true);
    std::thread release{[&] {
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
      b.busy.SetValue(false);
    }};
    CHECK(b.radio.Begin());  // the first command waits until BUSY is low
    release.join();
    CHECK(b.radio.IsConnected());
  }
  {
    Bench b;
    b.busy.SetValue(true);  // stuck: no command is sent
    CHECK_FALSE(b.radio.Begin());
    CHECK(b.fake.frames.empty());
    CHECK_FALSE(b.radio.IsConnected());
    const uint8_t payload[] = {0x01};
    CHECK_FALSE(b.radio.Send(payload));
    CHECK_FALSE(b.radio.StartReceive());
  }
  {
    Bench b;
    b.model.present = false;
    CHECK_FALSE(b.radio.Begin());
    CHECK_FALSE(b.radio.IsConnected());
  }
  {
    // unplugged after Begin: BUSY and DIO1 read high (input pull-ups). The thread must not hold
    // the lock while it waits, and the radio reports itself lost.
    Bench b;
    REQUIRE(b.radio.Begin());
    REQUIRE(b.radio.StartReceive());
    b.busy.SetValue(true);
    b.model.Raise(0x0002);
    REQUIRE(WaitFor([&] { return !b.radio.IsConnected(); }));
    const auto start = std::chrono::steady_clock::now();
    CHECK_FALSE(b.radio.Poll().has_value());
    const uint8_t payload[] = {0x01};
    CHECK_FALSE(b.radio.Send(payload));  // fails at once, no new 100 ms wait
    CHECK(std::chrono::steady_clock::now() - start < std::chrono::milliseconds{50});
    // plugged back in: BUSY low again, the pending RxDone is handled
    b.busy.SetValue(false);
    REQUIRE(WaitFor([&] { return b.radio.IsConnected(); }));
    CHECK(b.radio.GetReceivedCount() == 1);
  }
  {
    Bench b;
    b.model.calibrationErrors = 0x0020;  // XOSC_START_ERR: wrong TCXO voltage
    CHECK_FALSE(b.radio.Begin());
    CHECK(b.fake.frames.back() == Bytes{0x17, 0x00, 0x00, 0x00});
  }
}

TEST_CASE("SX126x time on air and low data rate optimization", "[sx126x]") {
  using Catch::Approx;
  SX126x::Config config;
  {
    Bench b{config};
    // SF7, 125 kHz, 4/5, preamble 8: 1.024 ms symbols
    CHECK(b.radio.GetTimeOnAir(10).value() == Approx(41.216));
    CHECK(b.radio.GetTimeOnAir(3).value() == Approx(30.976));
  }
  config.spreadingFactor = 12;  // 32.768 ms symbols: LDRO on
  {
    Bench b{config};
    CHECK(b.radio.GetTimeOnAir(10).value() == Approx(991.232));
    REQUIRE(b.radio.Begin());
    bool found = false;
    for (const Bytes& frame : b.fake.frames) {
      if (frame[0] == 0x8B) {
        CHECK(frame == Bytes{0x8B, 0x0C, 0x04, 0x01, 0x01});
        found = true;
      }
    }
    CHECK(found);
  }
  config.spreadingFactor = 6;
  config.bandwidth = SX126x::Config::Bandwidth::k500kHz;
  {
    Bench b{config};
    CHECK(b.radio.GetTimeOnAir(10).value() == Approx(5.408));
    REQUIRE(b.radio.Begin());
    b.model.regs[0x0889] = 0x05;
    const uint8_t payload[] = {0x01};
    REQUIRE(b.radio.Send(payload));
    CHECK(b.model.regs[0x0889] == 0x01);  // 15.1: bit 2 clear at 500 kHz
  }
  config.powerDbm = 23;
  {
    Bench b{config};
    CHECK_FALSE(b.radio.Begin());
  }
}
