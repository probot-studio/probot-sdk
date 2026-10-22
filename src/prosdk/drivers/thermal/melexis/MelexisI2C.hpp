// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// The I2C layer of the vendored Melexis API (MLX90640_API.cpp, MLX90641_API.cpp, Apache-2.0,
// licenses/melexis/LICENSE). It replaces the upstream MLX9064x_I2C_Driver files with prosdk::I2C.
// The API functions only take a 7-bit address, so a driver binds its prosdk::I2C with ScopedI2C
// around the API calls that touch the bus. ScopedI2C holds one process-wide lock, so two cameras
// never interleave their API calls.
//
// Bus format (MLX90640 datasheet rev 12 and MLX90641 datasheet rev 4, section 8, I2C protocol):
// a 16-bit register address, MSB first, then 16-bit words, MSB first; the address counts words.
// A read is split into transfers of at most kMaxReadBytes (design/boardoza/P2-light-thermal.md:
// one 1664-byte transfer takes about 37 ms at 400 kHz, near the 50 ms ESP32 I2C timeout).

#pragma once

#include <stdint.h>

#include <mutex>

#include "prosdk/hardware/bus/I2C.hpp"

namespace prosdk::melexis {

/// Largest single read transfer, in bytes (256 words).
inline constexpr int kMaxReadBytes = 512;

/**
 * Binds an I2C device to the Melexis API functions while it is alive. The API's address argument
 * must equal the bound device's address; otherwise the calls fail with -1.
 */
class ScopedI2C {
 public:
  explicit ScopedI2C(prosdk::I2C& i2c);
  ~ScopedI2C();
  ScopedI2C(const ScopedI2C&) = delete;
  ScopedI2C& operator=(const ScopedI2C&) = delete;

 private:
  std::unique_lock<std::mutex> m_lock;
};

// The functions the Melexis API calls. They return 0 on success and -1 when the transfer aborts
// (NACK or timeout) or no device is bound. A write is not read back (upstream reads it back and
// returns -2 on a mismatch). The general call reset is refused with -1: it resets every chip on
// the bus that answers the general call (OPT3001 among them). Only the unused
// MLX9064x_TriggerMeasurement calls it.
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead,
                     uint16_t* data);
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data);
int MLX90640_I2CGeneralReset(void);
int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead,
                     uint16_t* data);
int MLX90641_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data);
int MLX90641_I2CGeneralReset(void);

}  // namespace prosdk::melexis
