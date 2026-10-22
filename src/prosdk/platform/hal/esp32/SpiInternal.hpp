// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Internals of the ESP32-S3 SPI backend (SPI.cpp) for other platform code that puts its own
// ESP-IDF device on a prosdk SPI bus, such as the SD card mount (esp_vfs_fat_sdspi_mount). Such a
// user holds the bus with RetainSpiBus while its device exists, claims its chip-select GPIO with
// ClaimPin (Esp32Internal.hpp) and adds its device to SpiHostOf(bus). Target build only.

#pragma once

#include <stdint.h>

#include <driver/spi_common.h>

namespace prosdk::platform::hal::detail {

/** The ESP-IDF host of a prosdk SPI bus: 0 -> SPI2_HOST, 1 -> SPI3_HOST; SPI_HOST_MAX otherwise. */
spi_host_device_t SpiHostOf(int bus);

/**
 * Starts the bus if needed (pins from prosdk::platform::SetSPIPins, claimed in the HAL pin table,
 * DMA, 4092-byte transfers) and counts one user, like a prosdk device with a chip select.
 * Returns false with an error status (HAL error message) when the bus is invalid, has no pins or
 * a pin is not usable, or carries a device without chip select.
 */
bool RetainSpiBus(int bus, int32_t* status);

/** Drops a RetainSpiBus user; the bus stops when it has no users left. */
void ReleaseSpiBus(int bus);

}  // namespace prosdk::platform::hal::detail
