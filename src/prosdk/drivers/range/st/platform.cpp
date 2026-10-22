// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// VL53L7CX ULD platform layer on prosdk::I2C (st/platform.h). Every transfer starts with a 16-bit
// register index, MSB first, and the sensor auto-increments the index after each data byte
// (DS13865 rev 8, section 4 "I2C control interface", figures 12 to 14). Blocks longer than
// kVL53L7CXChunkSize are split into several transfers, each with the index of its first byte.
// Nothing here throws: the callers are the ULD's C functions.

#include "prosdk/drivers/range/st/platform.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstring>
#include <thread>
#include <utility>

#include "prosdk/drivers/range/st/VL53L7CXPlatform.hpp"

using prosdk::detail::kVL53L7CXChunkSize;
using prosdk::detail::VL53L7CXBus;

static_assert(std::endian::native == std::endian::little,
              "VL53L7CX_SwapBuffer assumes a little-endian host (ESP32-S3, x86, ARM)");

namespace {

constexpr uint8_t kError = 255;  // VL53L7CX_STATUS_ERROR (vl53l7cx_api.h)

VL53L7CXBus* BusOf(VL53L7CX_Platform* platform) {
  auto* bus = static_cast<VL53L7CXBus*>(platform->bus);
  return bus && !bus->abort ? bus : nullptr;
}

// The device at platform->address. vl53l7cx_set_i2c_address() changes that address (8-bit form);
// the I2C device follows on the next transfer.
prosdk::I2C* DeviceOf(VL53L7CXBus* bus, VL53L7CX_Platform* platform) {
  int address = platform->address >> 1;
  if (bus->i2c.GetDeviceAddress() != address) {
    try {
      bus->i2c = prosdk::I2C{bus->i2c.GetPort(), address};
    } catch (...) {
      return nullptr;  // the port is already open, so this is not expected
    }
  }
  return &bus->i2c;
}

}  // namespace

extern "C" {

uint8_t VL53L7CX_RdByte(VL53L7CX_Platform* p_platform, uint16_t RegisterAdress, uint8_t* p_value) {
  return VL53L7CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L7CX_WrByte(VL53L7CX_Platform* p_platform, uint16_t RegisterAdress, uint8_t value) {
  return VL53L7CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform* p_platform, uint16_t RegisterAdress, uint8_t* p_values,
                         uint32_t size) {
  VL53L7CXBus* bus = BusOf(p_platform);
  prosdk::I2C* i2c = bus ? DeviceOf(bus, p_platform) : nullptr;
  if (!i2c) {
    return kError;
  }
  for (uint32_t done = 0; done < size;) {
    uint32_t count = std::min(size - done, kVL53L7CXChunkSize);
    uint16_t index = static_cast<uint16_t>(RegisterAdress + done);
    uint8_t indexBytes[2] = {static_cast<uint8_t>(index >> 8), static_cast<uint8_t>(index & 0xFF)};
    if (i2c->Transaction(indexBytes, sizeof(indexBytes), p_values + done,
                         static_cast<int>(count))) {
      return kError;
    }
    done += count;
  }
  return 0;
}

uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform* p_platform, uint16_t RegisterAdress, uint8_t* p_values,
                         uint32_t size) {
  VL53L7CXBus* bus = BusOf(p_platform);
  prosdk::I2C* i2c = bus ? DeviceOf(bus, p_platform) : nullptr;
  if (!i2c) {
    return kError;
  }
  for (uint32_t done = 0; done < size;) {
    uint32_t count = std::min(size - done, kVL53L7CXChunkSize);
    uint16_t index = static_cast<uint16_t>(RegisterAdress + done);
    bus->buffer[0] = static_cast<uint8_t>(index >> 8);
    bus->buffer[1] = static_cast<uint8_t>(index & 0xFF);
    std::memcpy(&bus->buffer[2], p_values + done, count);
    if (i2c->WriteBulk(bus->buffer.data(), static_cast<int>(count + 2))) {
      return kError;
    }
    done += count;
  }
  return 0;
}

void VL53L7CX_SwapBuffer(uint8_t* buffer, uint16_t size) {
  for (uint16_t i = 0; i + 3 < size; i += 4) {
    std::swap(buffer[i], buffer[i + 3]);
    std::swap(buffer[i + 1], buffer[i + 2]);
  }
}

uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform* p_platform, uint32_t TimeMs) {
  if (BusOf(p_platform)) {  // no wait while stopping: the ULD's polling loops then end quickly
    std::this_thread::sleep_for(std::chrono::milliseconds{TimeMs});
  }
  return 0;
}

}  // extern "C"
