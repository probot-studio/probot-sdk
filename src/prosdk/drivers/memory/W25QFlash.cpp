// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// W25Q NOR flash driver (W25QFlash.hpp). Instructions, status bits and timing: Winbond W25Q40CL
// datasheet rev. E (2017-07-28): section 7.1.1 (SPI modes 0 and 3), 8.1 (status register 1:
// BUSY S0, WEL S1), 9.1.1 (IDs: EFh, JEDEC 4013h), 9.1.2 to 9.1.4 (instruction tables), 9.1.11
// (Fast Read: one dummy byte), 9.1.19 (Page Program wraps inside the 256-byte page), 9.1.28
// (Release Power-down, tRES1), 10.6 (tRES1 3 us, tPP 0.8 ms, tSE 300 ms, tBE2 1 s, tCE 4 s
// maximum).

#include "prosdk/drivers/memory/W25QFlash.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kWriteEnable = 0x06;
constexpr uint8_t kReadStatus1 = 0x05;
constexpr uint8_t kPageProgram = 0x02;
constexpr uint8_t kSectorErase = 0x20;
constexpr uint8_t kBlockErase64 = 0xD8;
constexpr uint8_t kChipErase = 0xC7;
constexpr uint8_t kFastRead = 0x0B;
constexpr uint8_t kReleasePowerDown = 0xAB;
constexpr uint8_t kJedecId = 0x9F;

constexpr uint8_t kStatusBusy = 0x01;  // S0
constexpr uint8_t kStatusWel = 0x02;   // S1

constexpr uint8_t kWinbond = 0xEF;
constexpr int kMaxCapacityCode = 24;  // 2^24 bytes: the end of 24-bit addresses

// tPP is 0.8 ms at most; the margin covers the polling and a busy bus
constexpr units::millisecond_t kPageProgramTimeout{10};

}  // namespace

W25QFlash::W25QFlash(SPI::Port bus, int csGpio, int clockHz) : m_spi{bus, csGpio} {
  m_spi.SetMode(SPI::kMode0);
  m_spi.SetClockRate(clockHz);
  uint32_t jedecId = 0;
  {
    std::scoped_lock lock{m_mutex};
    // a chip left in power-down ignores every other instruction; tRES1 is 3 us
    Command(kReleasePowerDown);
    std::this_thread::sleep_for(std::chrono::microseconds{3});
    jedecId = ReadJedecIdLocked();
  }
  const bool connected = (jedecId >> 16) == kWinbond;
  const int capacity = static_cast<int>(jedecId & 0xFF);
  if (connected && capacity <= kMaxCapacityCode) {
    m_size = uint32_t{1} << capacity;
  }
  m_connected = connected;
  if (!connected) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] W25QFlash on SPI bus {} CS {}: no Winbond flash (JEDEC ID "
                           "0x{:06X})\n",
                           static_cast<int>(bus), csGpio, jedecId);
  }
  util::ReportUsage("W25QFlash", csGpio, "");
}

uint32_t W25QFlash::ReadJedecId() {
  std::scoped_lock lock{m_mutex};
  return ReadJedecIdLocked();
}

// m_mutex held
uint32_t W25QFlash::ReadJedecIdLocked() {
  uint8_t tx[4] = {kJedecId, 0, 0, 0};
  uint8_t rx[4] = {};
  if (m_spi.Transaction(tx, rx, sizeof(tx)) != static_cast<int>(sizeof(tx))) {
    return 0;
  }
  return (uint32_t{rx[1]} << 16) | (uint32_t{rx[2]} << 8) | rx[3];
}

bool W25QFlash::IsConnected() const {
  return m_connected;
}

uint32_t W25QFlash::GetSize() const {
  return m_size;
}

// m_mutex held
bool W25QFlash::Command(uint8_t instruction) {
  return m_spi.Write(&instruction, 1) == 1;
}

// m_mutex held
bool W25QFlash::CommandWithAddress(uint8_t instruction, uint32_t address) {
  const uint8_t tx[4] = {instruction, static_cast<uint8_t>(address >> 16),
                         static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address)};
  return m_spi.Write(tx, sizeof(tx)) == static_cast<int>(sizeof(tx));
}

// m_mutex held
bool W25QFlash::ReadStatus(uint8_t* status) {
  uint8_t tx[2] = {kReadStatus1, 0};
  uint8_t rx[2] = {};
  if (m_spi.Transaction(tx, rx, sizeof(tx)) != static_cast<int>(sizeof(tx))) {
    return false;
  }
  *status = rx[1];
  m_busy = (rx[1] & kStatusBusy) != 0;
  return true;
}

// m_mutex held. Write Enable, checked: WEL must read back as 1 (and BUSY as 0).
bool W25QFlash::WriteEnable() {
  uint8_t status = 0;
  return Command(kWriteEnable) && ReadStatus(&status) &&
         (status & (kStatusWel | kStatusBusy)) == kStatusWel;
}

// m_mutex held
bool W25QFlash::WaitNotBusy(units::millisecond_t timeout) {
  const auto start = std::chrono::steady_clock::now();
  const auto limit = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{timeout}.value())};
  while (true) {
    uint8_t status = 0;
    if (ReadStatus(&status) && (status & kStatusBusy) == 0) {
      return true;
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed >= limit) {
      return false;
    }
    // a page program takes 0.4 ms (typical): poll at first, then let other tasks run
    if (elapsed < std::chrono::milliseconds{1}) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
  }
}

bool W25QFlash::InRange(uint32_t address, size_t size) const {
  const uint32_t chipSize = m_size;
  return chipSize != 0 && address < chipSize && size <= chipSize - address;
}

bool W25QFlash::Read(uint32_t address, std::span<uint8_t> data) {
  std::scoped_lock lock{m_mutex};
  if (!InRange(address, data.size())) {
    return false;
  }
  uint8_t status = 0;
  if (!ReadStatus(&status) || (status & kStatusBusy) != 0) {
    return false;  // a Read issued while busy is ignored by the chip (9.1.10)
  }
  // 0Bh, A23-A0, one dummy byte, then the data
  const uint8_t header[5] = {kFastRead, static_cast<uint8_t>(address >> 16),
                             static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address),
                             0};
  const SpiSegment segments[] = {{header, nullptr, sizeof(header)},
                                 {nullptr, data.data(), static_cast<int>(data.size())}};
  return m_spi.TransactionSegments(segments) >= 0;
}

bool W25QFlash::Program(uint32_t address, std::span<const uint8_t> data) {
  std::scoped_lock lock{m_mutex};
  if (!InRange(address, data.size())) {
    return false;
  }
  uint8_t status = 0;
  if (!ReadStatus(&status) || (status & kStatusBusy) != 0) {
    return false;
  }
  size_t done = 0;
  while (done < data.size()) {
    const uint32_t at = address + static_cast<uint32_t>(done);
    // up to the end of this page: the chip wraps to the page start instead of crossing it
    const size_t n = std::min<size_t>(kPageSize - (at % kPageSize), data.size() - done);
    if (!WriteEnable()) {
      return false;
    }
    const uint8_t header[4] = {kPageProgram, static_cast<uint8_t>(at >> 16),
                               static_cast<uint8_t>(at >> 8), static_cast<uint8_t>(at)};
    const SpiSegment segments[] = {{header, nullptr, sizeof(header)},
                                   {data.data() + done, nullptr, static_cast<int>(n)}};
    if (m_spi.TransactionSegments(segments) < 0 || !WaitNotBusy(kPageProgramTimeout)) {
      return false;
    }
    done += n;
  }
  return true;
}

// m_mutex held
bool W25QFlash::StartErase(uint8_t instruction, uint32_t address) {
  uint8_t status = 0;
  if (!ReadStatus(&status) || (status & kStatusBusy) != 0 || !WriteEnable()) {
    return false;
  }
  bool ok = instruction == kChipErase ? Command(instruction)
                                      : CommandWithAddress(instruction, address);
  if (ok) {
    m_busy = true;
  }
  return ok;
}

bool W25QFlash::EraseSector(uint32_t address) {
  std::scoped_lock lock{m_mutex};
  return InRange(address, 1) && StartErase(kSectorErase, address);
}

bool W25QFlash::EraseBlock64(uint32_t address) {
  std::scoped_lock lock{m_mutex};
  return InRange(address, 1) && StartErase(kBlockErase64, address);
}

bool W25QFlash::EraseChip() {
  std::scoped_lock lock{m_mutex};
  return m_size != 0 && StartErase(kChipErase, 0);
}

bool W25QFlash::IsBusy() {
  std::scoped_lock lock{m_mutex};
  uint8_t status = 0;
  // a failed transfer reports busy: nothing can be started then either
  return !ReadStatus(&status) || (status & kStatusBusy) != 0;
}

bool W25QFlash::WaitReady(units::millisecond_t timeout) {
  std::scoped_lock lock{m_mutex};
  return WaitNotBusy(timeout);
}

void W25QFlash::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Connected", m_connected.load());
  table.Log("Busy", m_busy.load());
}

std::string_view W25QFlash::GetTelemetryType() const {
  return "Flash Memory";
}
