// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// SD card mount (SdCard.hpp). ESP32: the bus is held with RetainSpiBus (SpiInternal.hpp, F1), the
// chip select is claimed in the HAL pin table (ClaimPin), and esp_vfs_fat_sdspi_mount adds the
// card as its own device on the bus's ESP-IDF host (esp_vfs_fat.h: the bus must be initialized
// with spi_bus_initialize first, which RetainSpiBus does). Host: a stub that returns false.

#include "prosdk/platform/SdCard.hpp"

#include <format>
#include <mutex>
#include <string>
#include <vector>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/HalConfig.hpp"

#if PROSDK_TARGET_HW
#include <driver/gpio.h>
#include <driver/sdspi_host.h>
#include <esp_err.h>
#include <esp_vfs_fat.h>
#include <sd_protocol_types.h>

#include "prosdk/hal/HAL.h"
#include "prosdk/platform/hal/esp32/Esp32Internal.hpp"
#include "prosdk/platform/hal/esp32/SpiInternal.hpp"
#endif

namespace prosdk::platform {
namespace {

bool ValidArguments(int spiBus, int csGpio, std::string_view mountPoint, int hz) {
  if (spiBus < 0 || spiBus >= kNumSPIBuses || csGpio < 0 || csGpio > 48 ||
      hz < kSdCardMinClockRate || hz > kSdCardMaxClockRate || mountPoint.size() < 2 ||
      mountPoint.front() != '/' || mountPoint.back() == '/') {
    ConsolePrint(ConsoleLevel::kError,
                 "[prosdk] MountSdCard: invalid arguments (SPI bus {}, CS {}, mount point "
                 "\"{}\", {} Hz)\n",
                 spiBus, csGpio, mountPoint, hz);
    return false;
  }
  return true;
}

#if PROSDK_TARGET_HW
std::mutex gMutex;                      // the list below and the mounting
std::vector<std::string> gMountPoints;  // mounted by MountSdCard (kept until the program ends)
#endif

}  // namespace

#if PROSDK_TARGET_HW

bool MountSdCard(int spiBus, int csGpio, std::string_view mountPoint, int hz) {
  if (!ValidArguments(spiBus, csGpio, mountPoint, hz)) {
    return false;
  }
  namespace esp = ::prosdk::hal::esp32;
  namespace detail = ::prosdk::platform::hal::detail;
  const std::string path{mountPoint};
  std::scoped_lock lock{gMutex};
  for (const std::string& mounted : gMountPoints) {
    if (mounted == path) {
      ConsolePrint(ConsoleLevel::kError, "[prosdk] MountSdCard: {} is already mounted\n", path);
      return false;
    }
  }
  int32_t status = 0;
  if (!detail::RetainSpiBus(spiBus, &status)) {
    ConsolePrint(ConsoleLevel::kError, "[prosdk] MountSdCard {}: {}\n", path,
                 HAL_GetLastError(&status));
    return false;
  }
  auto claimed = esp::ClaimPin(csGpio, ::prosdk::hal::HAL_HandleEnum::VENDOR, "SD card",
                               std::format("SD card {} (SPI bus {} CS)", path, spiBus));
  if (!claimed) {
    status = claimed.error();
    ConsolePrint(ConsoleLevel::kError, "[prosdk] MountSdCard {}: {}\n", path,
                 HAL_GetLastError(&status));
    detail::ReleaseSpiBus(spiBus);
    return false;
  }
  // the Boardoza board has no pull-up on MISO (DO): the card's output floats until it is in
  // SPI mode. The internal pull-up is weak; long wires may need an external one.
  const int miso = GetSPIConfig(spiBus).miso;
  if (miso >= 0) {
    gpio_pullup_en(static_cast<gpio_num_t>(miso));
  }

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = detail::SpiHostOf(spiBus);
  host.max_freq_khz = hz / 1000;
  sdspi_device_config_t device = SDSPI_DEVICE_CONFIG_DEFAULT();
  device.host_id = detail::SpiHostOf(spiBus);
  device.gpio_cs = static_cast<gpio_num_t>(csGpio);
  esp_vfs_fat_mount_config_t mount{};
  mount.format_if_mount_failed = false;  // never erase a card that does not mount
  mount.max_files = 5;
  sdmmc_card_t* card = nullptr;
  esp_err_t err = esp_vfs_fat_sdspi_mount(path.c_str(), &host, &device, &mount, &card);
  if (err != ESP_OK) {
    esp::ReleasePin(*claimed, ::prosdk::hal::HAL_HandleEnum::VENDOR);
    detail::ReleaseSpiBus(spiBus);
    ConsolePrint(ConsoleLevel::kError,
                 "[prosdk] MountSdCard {}: no card or no FAT file system (SPI bus {} CS {}: "
                 "{})\n",
                 path, spiBus, csGpio, esp_err_to_name(err));
    return false;
  }
  gMountPoints.push_back(path);
  const uint64_t bytes =
      static_cast<uint64_t>(card->csd.capacity) * static_cast<uint64_t>(card->csd.sector_size);
  ConsolePrint(ConsoleLevel::kInfo, "[prosdk] SD card mounted at {} ({} MB)\n", path,
               bytes / 1'000'000);
  // the claim and the bus stay held: the mount lasts until the program ends
  return true;
}

#else

bool MountSdCard(int spiBus, int csGpio, std::string_view mountPoint, int hz) {
  if (!ValidArguments(spiBus, csGpio, mountPoint, hz)) {
    return false;
  }
  ConsolePrint(ConsoleLevel::kWarning,
               "[prosdk] MountSdCard {}: no SD card in the simulation\n", mountPoint);
  return false;
}

#endif

}  // namespace prosdk::platform
