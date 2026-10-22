// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// SPI backend interface (design/boardoza/F1-spi.md). WPILib 2027 has no SPI HAL, so this is a
// small C++ interface (like CoreJoystick), not a HAL_* C API; prosdk::SPI is its only user. Two
// implementations: the ESP32-S3 backend on ESP-IDF spi_master (esp32/SPI.cpp; bus 0 = SPI2,
// bus 1 = SPI3) and the host simulation (SPISim.cpp, prosdk/simulation/SpiSim.hpp).
//
// A device is one chip select on a bus. The bus starts lazily on its first device (pins from
// prosdk::platform::SetSPIPins) and stops when its last device closes. A device without chip
// select (cs -1, clocked LEDs) must be alone on its bus: it latches every clock edge, so traffic
// for any other device would corrupt it. OpenSpiDevice refuses the combination either way.
// SPI is not gated by the output gate or the emergency stop (like I2C and serial).

#pragma once

#include <stdint.h>

#include <span>

namespace prosdk {

/**
 * One piece of an SPI transaction (command, address, data). The segments of a transaction are
 * clocked back to back while the chip select stays active; every segment is full duplex.
 */
struct SpiSegment {
  /// Bytes to send; nullptr sends zeros.
  const uint8_t* tx = nullptr;
  /// Receives `size` bytes; nullptr discards what the device sends.
  uint8_t* rx = nullptr;
  /// Length in bytes (0 is allowed and clocks nothing).
  int size = 0;
};

}  // namespace prosdk

namespace prosdk::platform::hal {

/** Handle value of no device. */
inline constexpr int32_t kInvalidSpiHandle = -1;

/** Clock, mode and chip-select polarity of one SPI device. */
struct SpiDeviceConfig {
  /// SPI mode 0..3: (CPOL, CPHA) = (0, 0), (0, 1), (1, 0), (1, 1).
  int mode = 0;
  /// Clock rate in Hz.
  int clockHz = 1'000'000;
  /// Chip select active high (default active low).
  bool csActiveHigh = false;
};

/**
 * Opens a device on bus 0 or 1 with chip select `cs` (a GPIO, or -1 for none). Starts the bus on
 * its first device. Returns the device handle, or kInvalidSpiHandle with an error status (HAL
 * error message) when the bus is invalid or has no pins, a pin is not usable, the chip select is
 * already open, the bus has no free chip select, or a device without chip select would share
 * the bus.
 */
int32_t OpenSpiDevice(int bus, int cs, const SpiDeviceConfig& config, int32_t* status);

/** Changes a device's clock, mode and chip-select polarity (applies to the next transfer). */
void ConfigureSpiDevice(int32_t handle, const SpiDeviceConfig& config, int32_t* status);

/**
 * Clocks the segments as one transaction (chip select held from the first to the last byte).
 * Blocks until done. Returns false for an invalid handle or a failed transfer.
 */
bool SpiTransfer(int32_t handle, std::span<const prosdk::SpiSegment> segments);

/** Closes a device (no-op for kInvalidSpiHandle); the last device of a bus stops the bus. */
void CloseSpiDevice(int32_t handle);

}  // namespace prosdk::platform::hal
