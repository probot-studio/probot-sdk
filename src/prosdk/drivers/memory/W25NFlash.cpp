// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// W25N01GV SPI NAND driver (W25NFlash.hpp). Instructions, registers and timing: Winbond
// W25N01GVxxIG/IT datasheet rev. L (2018-05-09): figure 2 (page layout, bad-block marker bytes
// without ECC), 6.1.1 (SPI modes 0 and 3), 7.1 (SR-1 at Axh: BP3-BP0 and TB set at power-up),
// 7.2 (SR-2 at Bxh: ECC-E S4, BUF S3), 7.3 (SR-3 at Cxh: ECC-1/ECC-0 S5/S4, P-FAIL S3, E-FAIL S2,
// WEL S1, BUSY S0), 8.1.1 (JEDEC ID EFh AA21h), 8.1.3 (instruction table, buffer read mode),
// 8.2.1 (reset, tRST up to 500 us), 8.2.4 and 8.2.5 (a write enable is required before program,
// erase and BBM; the status register write is not listed, so none is sent), 8.2.10 to 8.2.16
// (erase, load, execute, page read, fast read), 9.6 (tRD2 60 us, tPP 700 us, tBE 10 ms maximum;
// NoP 4), 10.2 (initial invalid blocks: non-FFh at byte 0 of the main array and of the spare area
// on the first page).

#include "prosdk/drivers/memory/W25NFlash.hpp"

#include <chrono>
#include <thread>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kReset = 0xFF;
constexpr uint8_t kJedecIdInstruction = 0x9F;
constexpr uint8_t kReadRegister = 0x0F;
constexpr uint8_t kWriteRegister = 0x1F;
constexpr uint8_t kWriteEnable = 0x06;
constexpr uint8_t kBlockErase = 0xD8;
constexpr uint8_t kLoadProgramData = 0x02;
constexpr uint8_t kProgramExecute = 0x10;
constexpr uint8_t kPageDataRead = 0x13;
constexpr uint8_t kFastRead = 0x0B;

constexpr uint8_t kRegProtection = 0xA0;     // SR-1
constexpr uint8_t kRegConfiguration = 0xB0;  // SR-2
constexpr uint8_t kRegStatus = 0xC0;         // SR-3

constexpr uint8_t kProtectionBits = 0x7C;  // SR-1: BP3-BP0 (S6-S3), TB (S2)
constexpr uint8_t kConfigEcc = 0x10;       // SR-2 ECC-E (S4)
constexpr uint8_t kConfigBuffer = 0x08;    // SR-2 BUF (S3)
constexpr uint8_t kStatusBusy = 0x01;      // SR-3 S0
constexpr uint8_t kStatusWel = 0x02;       // SR-3 S1
constexpr uint8_t kStatusEraseFail = 0x04;  // SR-3 S2
constexpr uint8_t kStatusProgramFail = 0x08;  // SR-3 S3
constexpr int kStatusEccShift = 4;           // SR-3 S5-S4

constexpr int kColumns = prosdk::W25NFlash::kPageSize + prosdk::W25NFlash::kSpareSize;

// datasheet maximums with a margin for the polling and a busy bus
constexpr int kResetTimeoutUs = 2'000;     // tRST 500 us
constexpr int kReadTimeoutUs = 2'000;      // tRD2 60 us
constexpr int kProgramTimeoutUs = 5'000;   // tPP 700 us
constexpr int kEraseTimeoutUs = 30'000;    // tBE 10 ms

}  // namespace

W25NFlash::W25NFlash(SPI::Port bus, int csGpio, int clockHz) : m_spi{bus, csGpio} {
  m_spi.SetMode(SPI::kMode0);
  m_spi.SetClockRate(clockHz);
  uint32_t id = 0;
  bool connected = false;
  {
    std::scoped_lock lock{m_mutex};
    uint8_t status = 0;
    // a reset during a program or erase can corrupt data (8.2.1): let it finish first
    WaitNotBusy(kEraseTimeoutUs, &status);
    Command(kReset);
    std::this_thread::sleep_for(std::chrono::microseconds{500});  // no command during tRST
    WaitNotBusy(kResetTimeoutUs, &status);
    id = ReadIdLocked();
    bool ok = id == kJedecId;
    // the reset restored the power-up defaults: the whole array is write-protected
    ok = ok && WriteRegister(kRegProtection, 0x00);
    ok = ok && WriteRegister(kRegConfiguration, kConfigEcc | kConfigBuffer);
    uint8_t protection = 0xFF;
    uint8_t config = 0;
    ok = ok && ReadRegister(kRegProtection, &protection) &&
         ReadRegister(kRegConfiguration, &config);
    connected = ok && (protection & kProtectionBits) == 0 &&
                (config & (kConfigEcc | kConfigBuffer)) == (kConfigEcc | kConfigBuffer);
  }
  m_connected = connected;
  if (!connected) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] W25NFlash on SPI bus {} CS {}: {} (JEDEC ID 0x{:06X})\n",
                           static_cast<int>(bus), csGpio,
                           id == kJedecId ? "setup not accepted (protection locked?)"
                                          : "no W25N01GV",
                           id);
  }
  util::ReportUsage("W25NFlash", csGpio, "");
}

uint32_t W25NFlash::ReadId() {
  std::scoped_lock lock{m_mutex};
  return ReadIdLocked();
}

// m_mutex held. 9Fh, one dummy byte, then EFh AAh 21h.
uint32_t W25NFlash::ReadIdLocked() {
  uint8_t tx[5] = {kJedecIdInstruction, 0, 0, 0, 0};
  uint8_t rx[5] = {};
  if (m_spi.Transaction(tx, rx, sizeof(tx)) != static_cast<int>(sizeof(tx))) {
    return 0;
  }
  return (uint32_t{rx[2]} << 16) | (uint32_t{rx[3]} << 8) | rx[4];
}

bool W25NFlash::IsConnected() const {
  return m_connected;
}

// m_mutex held
bool W25NFlash::Command(uint8_t instruction) {
  return m_spi.Write(&instruction, 1) == 1;
}

// m_mutex held. Instruction, 8 dummy clocks, PA[15:8], PA[7:0].
bool W25NFlash::PageCommand(uint8_t instruction, int page) {
  const uint8_t tx[4] = {instruction, 0, static_cast<uint8_t>(page >> 8),
                         static_cast<uint8_t>(page)};
  return m_spi.Write(tx, sizeof(tx)) == static_cast<int>(sizeof(tx));
}

// m_mutex held
bool W25NFlash::ReadRegister(uint8_t address, uint8_t* value) {
  uint8_t tx[3] = {kReadRegister, address, 0};
  uint8_t rx[3] = {};
  if (m_spi.Transaction(tx, rx, sizeof(tx)) != static_cast<int>(sizeof(tx))) {
    return false;
  }
  *value = rx[2];
  return true;
}

// m_mutex held
bool W25NFlash::WriteRegister(uint8_t address, uint8_t value) {
  const uint8_t tx[3] = {kWriteRegister, address, value};
  return m_spi.Write(tx, sizeof(tx)) == static_cast<int>(sizeof(tx));
}

// m_mutex held. Polls SR-3 until BUSY clears; *status is the last value read.
bool W25NFlash::WaitNotBusy(int timeoutUs, uint8_t* status) {
  const auto start = std::chrono::steady_clock::now();
  while (true) {
    if (ReadRegister(kRegStatus, status) && (*status & kStatusBusy) == 0) {
      return true;
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed >= std::chrono::microseconds{timeoutUs}) {
      return false;
    }
    // reads and programs take well under 1 ms: poll; an erase (2 ms typical) sleeps
    if (elapsed < std::chrono::milliseconds{1}) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
  }
}

// m_mutex held. Write Enable, checked: WEL must read back as 1.
bool W25NFlash::WriteEnable() {
  uint8_t status = 0;
  return Command(kWriteEnable) && ReadRegister(kRegStatus, &status) &&
         (status & (kStatusWel | kStatusBusy)) == kStatusWel;
}

// m_mutex held. Page Data Read into the chip's buffer; *status is SR-3 afterwards.
bool W25NFlash::LoadPage(int page, uint8_t* status) {
  return PageCommand(kPageDataRead, page) && WaitNotBusy(kReadTimeoutUs, status);
}

// m_mutex held. Fast Read from the buffer: 0Bh, CA[15:8], CA[7:0], one dummy byte, data.
bool W25NFlash::ReadBuffer(int column, std::span<uint8_t> data) {
  const uint8_t header[4] = {kFastRead, static_cast<uint8_t>(column >> 8),
                             static_cast<uint8_t>(column), 0};
  const SpiSegment segments[] = {{header, nullptr, sizeof(header)},
                                 {nullptr, data.data(), static_cast<int>(data.size())}};
  return m_spi.TransactionSegments(segments) >= 0;
}

bool W25NFlash::ReadPage(int page, std::span<uint8_t> data, int column) {
  if (!m_connected || page < 0 || page >= kPageCount || column < 0 || column > kColumns ||
      data.size() > static_cast<size_t>(kColumns - column)) {
    return false;
  }
  std::scoped_lock lock{m_mutex};
  uint8_t status = 0;
  if (!LoadPage(page, &status) || !ReadBuffer(column, data) ||
      !ReadRegister(kRegStatus, &status)) {
    return false;
  }
  const auto ecc = static_cast<EccStatus>((status >> kStatusEccShift) & 0x03);
  m_ecc = ecc;
  return ecc == EccStatus::kOk || ecc == EccStatus::kCorrected;
}

bool W25NFlash::ProgramPage(int page, std::span<const uint8_t> data, int column) {
  if (!m_connected || page < 0 || page >= kPageCount || column < 0 || column >= kColumns || data.empty() ||
      data.size() > static_cast<size_t>(kColumns - column)) {
    return false;
  }
  std::scoped_lock lock{m_mutex};
  if (!WriteEnable()) {
    return false;
  }
  const uint8_t header[3] = {kLoadProgramData, static_cast<uint8_t>(column >> 8),
                             static_cast<uint8_t>(column)};
  const SpiSegment segments[] = {{header, nullptr, sizeof(header)},
                                 {data.data(), nullptr, static_cast<int>(data.size())}};
  uint8_t status = 0;
  return m_spi.TransactionSegments(segments) >= 0 && PageCommand(kProgramExecute, page) &&
         WaitNotBusy(kProgramTimeoutUs, &status) && (status & kStatusProgramFail) == 0;
}

bool W25NFlash::EraseBlock(int block) {
  if (!m_connected || block < 0 || block >= kBlockCount) {
    return false;
  }
  std::scoped_lock lock{m_mutex};
  uint8_t status = 0;
  return WriteEnable() && PageCommand(kBlockErase, block * kPagesPerBlock) &&
         WaitNotBusy(kEraseTimeoutUs, &status) && (status & kStatusEraseFail) == 0;
}

bool W25NFlash::IsBadBlock(int block) {
  if (!m_connected || block < 0 || block >= kBlockCount) {
    return true;
  }
  std::scoped_lock lock{m_mutex};
  // ECC off, so that the engine cannot alter the marker bytes; restored afterwards
  bool ok = WriteRegister(kRegConfiguration, kConfigBuffer);
  uint8_t status = 0;
  uint8_t main = 0;
  uint8_t spare = 0;
  ok = ok && LoadPage(block * kPagesPerBlock, &status) && ReadBuffer(0, {&main, 1}) &&
       ReadBuffer(kPageSize, {&spare, 1});
  ok = WriteRegister(kRegConfiguration, kConfigEcc | kConfigBuffer) && ok;
  return !ok || main != 0xFF || spare != 0xFF;
}

W25NFlash::EccStatus W25NFlash::GetEccStatus() const {
  return m_ecc;
}

bool W25NFlash::IsBusy() {
  std::scoped_lock lock{m_mutex};
  uint8_t status = 0;
  return !ReadRegister(kRegStatus, &status) || (status & kStatusBusy) != 0;
}

void W25NFlash::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Connected", m_connected.load());
  table.Log("ECC Status", static_cast<int32_t>(m_ecc.load()));
}

std::string_view W25NFlash::GetTelemetryType() const {
  return "Flash Memory";
}
