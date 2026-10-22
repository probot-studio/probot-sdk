// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// An SPI device on the host SPI simulation (prosdk/simulation/SpiSim.hpp) at (bus, cs). Every
// transaction (one chip-select assertion) is recorded in `frames`. The reply comes from
// `handler` when it is set; otherwise from a register file: the first byte is the register
// address, a read when `readFlag` is set in it (the flag is masked off), followed by
// `readDummyBytes` ignored bytes and then the data with auto-increment. A write stores the data
// bytes from the address on.

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

#include "prosdk/simulation/SpiSim.hpp"

namespace prosdk::test {

class FakeSPIDevice {
 public:
  FakeSPIDevice(int bus, int cs)
      : m_id{sim::RegisterSpiDevice(
            bus, cs, [this](std::span<const uint8_t> tx, std::span<uint8_t> rx) {
              Transfer(tx, rx);
            })} {}
  ~FakeSPIDevice() { sim::CancelSpiDevice(m_id); }
  FakeSPIDevice(const FakeSPIDevice&) = delete;
  FakeSPIDevice& operator=(const FakeSPIDevice&) = delete;

  /// Every transaction's sent bytes, in order.
  std::vector<std::vector<uint8_t>> frames;
  /// The register file.
  std::array<uint8_t, 256> regs{};
  /// Read flag in the address byte (0x80: bit 7 set = read, as most sensors).
  uint8_t readFlag = 0x80;
  /// Bytes between the address and the data of a read (BMI088 accelerometer: 1).
  int readDummyBytes = 0;
  /// Replaces the register file when set: fill rx (zero-filled, same length as tx).
  std::function<void(std::span<const uint8_t> tx, std::span<uint8_t> rx)> handler;

 private:
  void Transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    frames.emplace_back(tx.begin(), tx.end());
    if (handler) {
      handler(tx, rx);
      return;
    }
    if (tx.empty()) {
      return;
    }
    const bool read = (tx[0] & readFlag) != 0;
    const uint8_t address = static_cast<uint8_t>(tx[0] & ~readFlag);
    if (read) {
      for (size_t i = 1 + static_cast<size_t>(readDummyBytes); i < rx.size(); ++i) {
        rx[i] = regs[(address + i - 1 - readDummyBytes) & 0xFF];
      }
    } else {
      for (size_t i = 1; i < tx.size(); ++i) {
        regs[(address + i - 1) & 0xFF] = tx[i];
      }
    }
  }

  int m_id;
};

}  // namespace prosdk::test
