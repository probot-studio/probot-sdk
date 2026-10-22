// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Winbond W25N01GV 1 Gbit serial SLC NAND flash over prosdk::SPI (design/boardoza/P9-storage.md),
// as on the Boardoza W25N01GVZEIG board. A raw page and block API: no flash translation layer
// and no file system. Standard SPI only (no dual or quad), mode 0.
//
// Geometry: 1024 blocks of 64 pages; a page is 2048 data bytes plus a 64-byte spare area
// (columns 0 to 2111). The driver runs the chip in buffer read mode with the on-chip ECC on (up
// to 4 bit errors per page corrected) and clears the block protection that the chip sets at every
// power-up. NAND rules the caller must keep: erase a block before programming its pages again,
// program the pages of a block in ascending order, at most 4 partial programs per page, and skip
// bad blocks (IsBadBlock() before the first erase: an erase destroys the factory marker).
//
// Every chip access blocks until the chip is done: a page read takes at most 60 us, a page program
// 700 us and a block erase 10 ms (datasheet maximums). IsConnected(), GetEccStatus() and the
// telemetry never wait for an access.

#pragma once

#include <stdint.h>

#include <atomic>
#include <mutex>
#include <span>
#include <string_view>

#include "prosdk/drivers/bus/SPI.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/** Winbond W25N01GV SPI NAND flash on an SPI bus. */
class W25NFlash : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Data bytes per page.
  static constexpr int kPageSize = 2048;
  /// Spare bytes per page (columns 2048 to 2111).
  static constexpr int kSpareSize = 64;
  /// Pages per erase block (128 KiB of data).
  static constexpr int kPagesPerBlock = 64;
  /// Erase blocks.
  static constexpr int kBlockCount = 1024;
  /// Pages (page addresses 0 to 65535).
  static constexpr int kPageCount = kBlockCount * kPagesPerBlock;
  /// Default SPI clock (a conservative choice for flying leads and the board's level shifter;
  /// the chip allows 104 MHz).
  static constexpr int kDefaultClockRate = 10'000'000;
  /// JEDEC ID (manufacturer EFh, device AA21h).
  static constexpr uint32_t kJedecId = 0xEFAA21;

  /** ECC result of the last page read (status register 3, ECC-1 and ECC-0). */
  enum class EccStatus {
    /// No errors.
    kOk = 0,
    /// 1 to 4 bit errors corrected.
    kCorrected = 1,
    /// More than 4 bit errors in the page: the data is not usable.
    kUncorrectable = 2,
    /// Uncorrectable errors in several pages (continuous read mode only, not used here).
    kUncorrectableMultiple = 3
  };

  /**
   * Opens the device (SPI mode 0), resets the chip, reads its JEDEC ID, clears the block
   * protection and selects buffer read mode with ECC on. Does not throw for a missing chip: it
   * prints one console line, IsConnected() is false and every page or block access fails
   * (IsBadBlock() reports every block as bad).
   *
   * @param bus     The SPI bus (prosdk::platform::SetSPIPins sets its pins).
   * @param csGpio  The chip-select GPIO (the board's nCS).
   * @param clockHz The SPI clock, SPI::kMinClockRate to SPI::kMaxClockRate.
   */
  W25NFlash(prosdk::SPI::Port bus, int csGpio, int clockHz = kDefaultClockRate);

  W25NFlash(const W25NFlash&) = delete;
  W25NFlash& operator=(const W25NFlash&) = delete;

  /** Reads the JEDEC ID (9Fh): manufacturer << 16 | device ID; 0xEFAA21. */
  uint32_t ReadId();

  /** True when the JEDEC ID read at construction was kJedecId and the setup was accepted. */
  bool IsConnected() const;

  /**
   * Reads a page into the chip's buffer (13h) and data.size() bytes of it from column on
   * (Fast Read, 0Bh). column + data.size() must not exceed kPageSize + kSpareSize.
   *
   * @return false when an argument is out of range, the chip did not finish in time, a transfer
   *         failed, or the page had uncorrectable errors (GetEccStatus()).
   */
  bool ReadPage(int page, std::span<uint8_t> data, int column = 0);

  /**
   * Programs data into a page from column on (Load Program Data, 02h, which sets the rest of
   * the buffer to FFh, then Program Execute, 10h). With ECC on the chip computes the ECC bytes of
   * the spare area itself and overwrites what was loaded there.
   *
   * @return false when an argument is out of range, the write enable was not accepted, the chip
   *         did not finish in time or reported a program failure (P-FAIL, also set for a
   *         protected page).
   */
  bool ProgramPage(int page, std::span<const uint8_t> data, int column = 0);

  /**
   * Erases a block (all 64 pages to FFh, D8h).
   *
   * @return false when the block is out of range, the write enable was not accepted, the chip
   *         did not finish in time or reported an erase failure (E-FAIL).
   */
  bool EraseBlock(int block);

  /**
   * True when the block carries a bad-block marker: a byte other than FFh at column 0 or at
   * column 2048 (the first spare byte) of its first page. The markers are read with the ECC
   * switched off. Also true when the block is out of range, no chip is connected or the read
   * failed, so that an unverified block is never used.
   */
  bool IsBadBlock(int block);

  /** ECC result of the last ReadPage(). */
  EccStatus GetEccStatus() const;

  /** Reads the BUSY bit of status register 3. */
  bool IsBusy();

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool Command(uint8_t instruction);
  bool PageCommand(uint8_t instruction, int page);
  bool ReadRegister(uint8_t address, uint8_t* value);
  bool WriteRegister(uint8_t address, uint8_t value);
  bool WaitNotBusy(int timeoutUs, uint8_t* status);
  bool WriteEnable();
  bool LoadPage(int page, uint8_t* status);
  bool ReadBuffer(int column, std::span<uint8_t> data);
  uint32_t ReadIdLocked();

  prosdk::SPI m_spi;

  std::mutex m_mutex;  // the chip (held for a whole operation)
  // read without m_mutex (getters, telemetry), so that they never wait for an operation
  std::atomic<bool> m_connected{false};
  std::atomic<EccStatus> m_ecc{EccStatus::kOk};
};

}  // namespace prosdk
