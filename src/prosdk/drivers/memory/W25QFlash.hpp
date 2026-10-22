// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Winbond W25Q serial NOR flash over prosdk::SPI (design/boardoza/P9-storage.md). Written for the
// W25Q40CL of the Boardoza W25Q40 board (512 KiB, JEDEC ID EF 40 13); other W25Q parts with 24-bit
// addresses use the same instructions. Standard SPI only (no dual or quad), mode 0.
//
// Program and erase follow the datasheet rules: an erase sets bytes to FFh, programming only
// clears bits, and Program() splits writes on 256-byte page boundaries (a page program wraps
// inside its page). Erases take up to 300 ms (4 KiB), 1 s (64 KiB) or 4 s (chip), so they do not
// wait: poll IsBusy() or call WaitReady(). Read and Program fail while the chip is busy. A chip
// still erasing when the controller restarts ignores the JEDEC ID read (datasheet 8.1.1), so the
// driver then reports no chip; construct it again once the erase has ended.

#pragma once

#include <stdint.h>

#include <atomic>
#include <mutex>
#include <span>
#include <string_view>

#include "prosdk/drivers/bus/SPI.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/** Winbond W25Q serial NOR flash (W25Q40CL and family) on an SPI bus. */
class W25QFlash : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Program page size in bytes.
  static constexpr int kPageSize = 256;
  /// Smallest erase unit (Sector Erase) in bytes.
  static constexpr int kSectorSize = 4096;
  /// Block Erase (64 KiB) unit in bytes.
  static constexpr int kBlock64Size = 65536;
  /// Default SPI clock (a conservative choice for flying leads; the chip allows 104 MHz at
  /// 3.0-3.6 V, 80 MHz at 2.3-3.6 V).
  static constexpr int kDefaultClockRate = 10'000'000;
  /// JEDEC ID of the W25Q40CL (manufacturer EFh, memory type 40h, capacity 13h).
  static constexpr uint32_t kW25Q40JedecId = 0xEF4013;

  /**
   * Opens the device (SPI mode 0), releases it from power-down and reads its JEDEC ID. Does not
   * throw for a missing chip: it prints one console line and IsConnected() is false.
   *
   * @param bus     The SPI bus (prosdk::platform::SetSPIPins sets its pins).
   * @param csGpio  The chip-select GPIO (the board's /CS).
   * @param clockHz The SPI clock, SPI::kMinClockRate to SPI::kMaxClockRate.
   */
  W25QFlash(prosdk::SPI::Port bus, int csGpio, int clockHz = kDefaultClockRate);

  W25QFlash(const W25QFlash&) = delete;
  W25QFlash& operator=(const W25QFlash&) = delete;

  /**
   * Reads the JEDEC ID (instruction 9Fh): manufacturer << 16 | memory type << 8 | capacity.
   * 0xEF4013 for a W25Q40CL.
   */
  uint32_t ReadJedecId();

  /** True when the JEDEC ID read at construction had the Winbond manufacturer ID (EFh). */
  bool IsConnected() const;

  /**
   * Size in bytes, from the capacity byte of the JEDEC ID read at construction (2^capacity:
   * 13h = 512 KiB). 0 when no Winbond chip answered or the capacity is beyond 24-bit addresses.
   */
  uint32_t GetSize() const;

  /**
   * Reads bytes (Fast Read, 0Bh, valid at every clock rate).
   *
   * @return false when the chip is busy, the range is outside the chip, or the transfer failed.
   */
  bool Read(uint32_t address, std::span<uint8_t> data);

  /**
   * Programs bytes into erased memory, one Page Program (02h) per 256-byte page touched, each
   * waited for (0.8 ms maximum per page). Programming only clears bits: erase first.
   *
   * @return false when the chip is busy, the range is outside the chip, the write enable was
   *         not accepted, or a page did not finish in time.
   */
  bool Program(uint32_t address, std::span<const uint8_t> data);

  /**
   * Starts erasing the 4 KiB sector that contains address (20h). Does not wait (up to 300 ms).
   *
   * @return false when the chip is busy, the address is outside the chip or the write enable
   *         was not accepted.
   */
  bool EraseSector(uint32_t address);

  /** Starts erasing the 64 KiB block that contains address (D8h). Does not wait (up to 1 s). */
  bool EraseBlock64(uint32_t address);

  /** Starts erasing the whole chip (C7h). Does not wait (up to 4 s). */
  bool EraseChip();

  /** Reads the BUSY bit of status register 1 (a program or erase is running). */
  bool IsBusy();

  /**
   * Polls the BUSY bit until it clears. Blocks the caller.
   *
   * @return false when the chip was still busy after timeout.
   */
  bool WaitReady(prosdk::units::millisecond_t timeout);

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool Command(uint8_t instruction);
  bool CommandWithAddress(uint8_t instruction, uint32_t address);
  bool ReadStatus(uint8_t* status);
  bool WriteEnable();
  bool WaitNotBusy(prosdk::units::millisecond_t timeout);
  bool InRange(uint32_t address, size_t size) const;
  bool StartErase(uint8_t instruction, uint32_t address);
  uint32_t ReadJedecIdLocked();

  prosdk::SPI m_spi;

  std::mutex m_mutex;  // the chip (held for a whole operation, WaitReady() included)
  // read without m_mutex (getters, telemetry), so that they never wait for an operation
  std::atomic<uint32_t> m_size{0};
  std::atomic<bool> m_connected{false};
  std::atomic<bool> m_busy{false};  // as last read
};

}  // namespace prosdk
