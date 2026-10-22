// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// The prosdk side of the VL53L7CX ULD platform layer (st/platform.h, st/platform.cpp): the I2C
// device behind VL53L7CX_Platform::bus. Used by prosdk::VL53L7CX and its tests only.

#pragma once

#include <stdint.h>

#include <array>
#include <atomic>

#include "prosdk/drivers/range/st/platform.h"
#include "prosdk/hardware/bus/I2C.hpp"

namespace prosdk::detail {

/// Largest data block of one I2C transfer. Longer ULD blocks (the 32 KiB firmware pages) go out as
/// several transfers, each with its own 16-bit index. 1 KiB takes about 23 ms at 400 kHz, under
/// the ESP32 HAL's 50 ms timeout per transfer (design/hal-esp32.md, "I2C").
inline constexpr uint32_t kVL53L7CXChunkSize = 1024;

/** The I2C device of one sensor, as the ULD platform layer uses it. */
struct VL53L7CXBus {
  /** Opens the port for a sensor at a 7-bit address. */
  VL53L7CXBus(prosdk::I2C::Port port, int address) : i2c{port, address} {}

  VL53L7CXBus(const VL53L7CXBus&) = delete;
  VL53L7CXBus& operator=(const VL53L7CXBus&) = delete;

  /** Points a ULD platform at this bus and at a 7-bit address. */
  void Attach(VL53L7CX_Platform* platform, int address) {
    platform->address = static_cast<uint16_t>(address << 1);
    platform->bus = this;
  }

  /// The sensor's device; platform.cpp re-opens it when the ULD changes platform->address.
  prosdk::I2C i2c;
  /// True: every transfer fails at once and waits return at once (the driver is stopping).
  std::atomic<bool> abort{false};
  /// One write transfer: 16-bit index (MSB first) and up to one chunk of data.
  std::array<uint8_t, 2 + kVL53L7CXChunkSize> buffer{};
};

}  // namespace prosdk::detail
