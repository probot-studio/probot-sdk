// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// NT3H2111 driver (NT3H2111.hpp). NXP NT3H2111/NT3H2211 product data sheet rev. 3.0
// (3 February 2016): memory map from I2C (section 8.3.2, table 6), session registers (8.3.12,
// tables 12 and 14), addressing (9.6, table 15), block READ/WRITE (9.7, figure 18), register
// READ/WRITE (9.8, figure 19), EEPROM write time (2.4).
//
// The block read sends a STOP after the block address, as figure 18 shows. With I2C_CLOCK_STR
// = 0 (configuration register; reset value 1, table 13) the chip needs 50 us between that STOP
// and the next START (9.7); the driver does not add that pause, so it relies on clock
// stretching being enabled (the reset value).

#include "prosdk/drivers/memory/NT3H2111.hpp"

#include <algorithm>
#include <array>
#include <thread>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kSessionBlock = 0xFE;
constexpr uint8_t kRegNsReg = 6;            // byte 6 of the session block (table 12)
constexpr uint8_t kNsRfFieldPresent = 0x01;
constexpr uint8_t kNsI2CLocked = 0x40;
constexpr uint8_t kManufacturerId = 0x04;   // block 0 byte 0 always reads 04h (table 6 note 1)
// 4.5 ms per 16-byte EEPROM block including overhead (section 2.4), plus a 0.5 ms margin
constexpr std::chrono::microseconds kEepromWriteTime{5000};

bool IsReadable(int block) {
  return (block >= 0 && block <= NT3H2111::kLastEepromBlock) ||
         (block >= NT3H2111::kFirstSramBlock && block <= NT3H2111::kLastSramBlock);
}

bool IsSram(int block) {
  return block >= NT3H2111::kFirstSramBlock && block <= NT3H2111::kLastSramBlock;
}

bool IsWritable(int block) {
  return (block >= NT3H2111::kFirstUserBlock && block <= NT3H2111::kLastUserBlock) ||
         IsSram(block);
}

}  // namespace

NT3H2111::NT3H2111(I2C::Port port, int address, int fdGpio)
    : m_i2c{port, address}, m_address{address} {
  if (fdGpio >= 0) {
    m_fd.emplace(fdGpio);
  }
  std::array<uint8_t, kBlockSize> block0{};
  bool ok = ReadBlock(0, block0);
  m_idOk = ok && block0[0] == kManufacturerId;
  if (!m_idOk) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] NT3H2111 0x{:02X} on I2C port {}: {} (block 0 byte 0 "
                           "0x{:02X})\n",
                           address, static_cast<int>(port), ok ? "unexpected chip" : "no answer",
                           block0[0]);
  }
  util::ReportUsage("NT3H2111", address, "");
}

// m_mutex held. The chip does not answer during an EEPROM write cycle (9.1).
void NT3H2111::WaitForWriteCycle() {
  auto now = std::chrono::steady_clock::now();
  if (now < m_writeDone) {
    std::this_thread::sleep_for(m_writeDone - now);
  }
}

bool NT3H2111::ReadBlock(int block, std::span<uint8_t, kBlockSize> data) {
  if (!IsReadable(block)) {
    throw WPILIB_MakeError(err::ParameterOutOfRange,
                           "NT3H2111 read block 0x{:02X} (valid: 0x00 to 0x{:02X}, 0x{:02X} to "
                           "0x{:02X})",
                           block, kLastEepromBlock, kFirstSramBlock, kLastSramBlock);
  }
  std::scoped_lock lock{m_mutex};
  WaitForWriteCycle();
  // WPILib I2C calls return true when the transfer was aborted
  uint8_t mema = static_cast<uint8_t>(block);
  uint8_t buf[kBlockSize];
  bool ok = !m_i2c.WriteBulk(&mema, 1) && !m_i2c.ReadOnly(kBlockSize, buf);
  m_connected = ok;
  if (ok) {
    std::copy(std::begin(buf), std::end(buf), data.begin());
  }
  return ok;
}

bool NT3H2111::WriteBlock(int block, std::span<const uint8_t, kBlockSize> data) {
  if (!IsWritable(block)) {
    throw WPILIB_MakeError(err::ParameterOutOfRange,
                           "NT3H2111 write block 0x{:02X} (valid: 0x{:02X} to 0x{:02X}, "
                           "0x{:02X} to 0x{:02X}; blocks 0 and 0x38 to 0x3A are not written)",
                           block, kFirstUserBlock, kLastUserBlock, kFirstSramBlock,
                           kLastSramBlock);
  }
  uint8_t buf[1 + kBlockSize];
  buf[0] = static_cast<uint8_t>(block);
  std::copy(data.begin(), data.end(), buf + 1);
  std::scoped_lock lock{m_mutex};
  WaitForWriteCycle();
  bool ok = !m_i2c.WriteBulk(buf, sizeof(buf));
  m_connected = ok;
  if (ok && !IsSram(block)) {
    // the STOP of the write starts the internal write cycle (9.2)
    m_writeDone = std::chrono::steady_clock::now() + kEepromWriteTime;
  }
  return ok;
}

// m_mutex held. Register READ (9.8): MEMA = FEh and REGA in one write, STOP, one byte read.
bool NT3H2111::ReadSessionRegister(uint8_t reg, uint8_t* value) {
  WaitForWriteCycle();
  uint8_t cmd[] = {kSessionBlock, reg};
  bool ok = !m_i2c.WriteBulk(cmd, sizeof(cmd)) && !m_i2c.ReadOnly(1, value);
  m_connected = ok;
  return ok;
}

bool NT3H2111::IsFieldPresent() {
  bool field;
  if (m_fd) {
    field = !m_fd->Get();  // FD is pulled low while the field is on
  } else {
    std::scoped_lock lock{m_mutex};
    uint8_t ns = 0;
    field = ReadSessionRegister(kRegNsReg, &ns) && (ns & kNsRfFieldPresent) != 0;
  }
  m_field = field;
  return field;
}

bool NT3H2111::ReleaseToNfc() {
  std::scoped_lock lock{m_mutex};
  WaitForWriteCycle();
  // register WRITE (9.8): MEMA, REGA, MASK (bits to change), REGDAT
  uint8_t cmd[] = {kSessionBlock, kRegNsReg, kNsI2CLocked, 0x00};
  bool ok = !m_i2c.WriteBulk(cmd, sizeof(cmd));
  m_connected = ok;
  return ok;
}

bool NT3H2111::IsConnected() const {
  return m_idOk && m_connected;
}

int NT3H2111::GetAddress() const {
  return m_address;
}

void NT3H2111::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Field Present", m_field.load());
  table.Log("Connected", IsConnected());
}

std::string_view NT3H2111::GetTelemetryType() const {
  return "NFC Tag";
}
