// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// NXP NT3H2111 (NTAG I2C plus 1k) connected NFC tag, I2C host side, over prosdk::I2C
// (design/boardoza/P5-i2c-misc.md): 16-byte block reads and writes of the EEPROM and the SRAM,
// and the field-detect output. A phone reads and writes the same EEPROM over NFC. No NDEF
// parsing, no pass-through mode, no password or lock handling.
//
// Block map from the I2C side (datasheet rev. 3.0, table 6): block 0 = I2C address byte, serial
// number, static lock bytes, capability container; blocks 1 to 0x37 and the first 8 bytes of
// 0x38 = user memory (888 bytes); rest of 0x38 = dynamic lock bytes and AUTH0; 0x39 = ACCESS,
// PWD, PACK, PT_I2C; 0x3A = configuration registers; 0xF8 to 0xFB = SRAM (64 bytes, volatile,
// only while VCC is powered); 0xFE = session registers (register access only).

#pragma once

#include <stdint.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <span>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/hardware/discrete/DigitalInput.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * NT3H2111 NTAG I2C plus 1k, I2C side (datasheet NT3H2111/NT3H2211 rev. 3.0, 3 February 2016).
 *
 * A read is the block address (MEMA) in one write, a STOP, then a 16-byte read (figure 18). An
 * EEPROM write takes about 4.5 ms (section 2.4) during which the chip ignores the bus, so after
 * an EEPROM WriteBlock() the next access to the chip waits until 5 ms have passed (the 0.5 ms
 * margin is not from the datasheet). SRAM writes do not wait.
 *
 * Memory access is arbitrated between NFC and I2C, first come first served (section 11): while a
 * phone uses the memory, block reads and writes are not acknowledged (they return false) and
 * only the session registers answer. After an I2C access the memory stays locked to I2C until
 * ReleaseToNfc() or the watchdog (default 20 ms, section 8.5).
 */
class NT3H2111 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default 7-bit I2C address (table 15: 1010101b; it can be changed in block 0 byte 0).
  static constexpr int kDefaultAddress = 0x55;
  /// Bytes per block.
  static constexpr int kBlockSize = 16;
  /// First and last block that hold only user memory (WriteBlock accepts these).
  static constexpr int kFirstUserBlock = 0x01;
  static constexpr int kLastUserBlock = 0x37;
  /// Last EEPROM block readable from I2C (configuration registers).
  static constexpr int kLastEepromBlock = 0x3A;
  /// SRAM blocks.
  static constexpr int kFirstSramBlock = 0xF8;
  static constexpr int kLastSramBlock = 0xFB;

  /**
   * Constructs the driver and reads block 0 to check the chip: its byte 0 always reads 04h (NXP
   * manufacturer id, table 6 note 1). A missing chip prints one console line; the constructor
   * does not throw for it.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address.
   * @param fdGpio  GPIO of the FD (field detect) output, or -1 for none. FD is open drain and
   *                needs a pull-up to the chip's VCC (the datasheet suggests more than 2 kOhm).
   */
  explicit NT3H2111(prosdk::I2C::Port port, int address = kDefaultAddress, int fdGpio = -1);

  NT3H2111(const NT3H2111&) = delete;
  NT3H2111& operator=(const NT3H2111&) = delete;

  /**
   * Reads one block: 0 to 0x3A or 0xF8 to 0xFB (throws otherwise). Returns false when the chip
   * did not acknowledge (missing, or the memory is locked to NFC); data is then unchanged.
   */
  bool ReadBlock(int block, std::span<uint8_t, kBlockSize> data);

  /**
   * Writes one block: user memory 1 to 0x37 or SRAM 0xF8 to 0xFB (throws otherwise). Block 0
   * holds the chip's I2C address and the lock bytes, and 0x38 to 0x3A hold lock, password and
   * configuration bytes whose effects can be permanent; this driver never writes them. Returns
   * false when the chip did not acknowledge.
   */
  bool WriteBlock(int block, std::span<const uint8_t, kBlockSize> data);

  /**
   * True while an NFC field is present. With an FD pin: FD low (the reset FD_ON / FD_OFF
   * setting, table 13: pulled low when the field switches on, released when it switches off).
   * Without: the RF_FIELD_PRESENT bit of the NS_REG session register (table 14); false when
   * the chip does not answer.
   */
  bool IsFieldPresent();

  /**
   * Ends the I2C access: clears I2C_LOCKED in NS_REG (table 14, section 11.1.1) so that a phone
   * can use the memory at once instead of after the watchdog. Returns false on no answer.
   */
  bool ReleaseToNfc();

  /** True when the last I2C transfer succeeded and block 0 identified the chip. */
  bool IsConnected() const;

  /** The 7-bit I2C address. */
  int GetAddress() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void WaitForWriteCycle();
  bool ReadSessionRegister(uint8_t reg, uint8_t* value);

  prosdk::I2C m_i2c;
  int m_address;
  std::optional<prosdk::DigitalInput> m_fd;

  std::mutex m_mutex;  // the chip, m_writeDone
  std::chrono::steady_clock::time_point m_writeDone{};  // end of the last EEPROM write cycle
  bool m_idOk = false;
  std::atomic<bool> m_connected{false};
  std::atomic<bool> m_field{false};  // last IsFieldPresent()
};

}  // namespace prosdk
