// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// SD card on a prosdk SPI bus (design/boardoza/P9-storage.md, Boardoza MicroSD board). ESP32:
// mounts the card's FAT file system with ESP-IDF esp_vfs_fat_sdspi_mount on the bus that
// prosdk::SPI uses (SPI2 or SPI3, pins from prosdk::platform::SetSPIPins), so other prosdk SPI
// devices can share the bus. Files are then reached through the mount point with the C and C++
// standard library, and DataLogManager writes there after SetLogDirectory("/sd/logs")
// (prosdk/platform/Storage.hpp). DataLogManager's old-log cleanup does not run on the card:
// GetStorageSpace() knows LittleFS only. Host: there is no SD card simulation; MountSdCard
// returns false.

#pragma once

#include <string_view>

namespace prosdk::platform {

/** Lowest SD clock: the card identification rate (ESP-IDF SDMMC_FREQ_PROBING, 400 kHz). */
inline constexpr int kSdCardMinClockRate = 400'000;
/** Highest SD clock (ESP-IDF SDMMC_FREQ_HIGHSPEED, 40 MHz). */
inline constexpr int kSdCardMaxClockRate = 40'000'000;
/** Default SD clock (ESP-IDF SDMMC_FREQ_DEFAULT, 20 MHz). */
inline constexpr int kSdCardDefaultClockRate = 20'000'000;

/**
 * Mounts an SD card in SPI mode (FAT file system; at most 5 files open at once). The card is
 * initialized at 400 kHz and then runs at up to hz. Blocks while the card is identified. The
 * mount lasts until the program ends. Prints one console line on success (card size) and on
 * failure.
 *
 * @param spiBus     The SPI bus, 0 (SPI2) or 1 (SPI3); its pins come from SetSPIPins.
 * @param csGpio     The card's chip-select GPIO.
 * @param mountPoint Where the file system appears, for example "/sd".
 * @param hz         The SPI clock after identification, kSdCardMinClockRate to
 *                   kSdCardMaxClockRate.
 * @return true when the card is mounted. false on the host, for invalid arguments, a bus that
 *         cannot start (no pins, a pin in use, a device without chip select on it), a chip select
 *         in use, a mount point already mounted, no card, or no FAT file system on it (the card
 *         is never formatted).
 */
bool MountSdCard(int spiBus, int csGpio, std::string_view mountPoint = "/sd",
                 int hz = kSdCardDefaultClockRate);

}  // namespace prosdk::platform
