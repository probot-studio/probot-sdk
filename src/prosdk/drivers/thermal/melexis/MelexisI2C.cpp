// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// prosdk::I2C layer of the vendored Melexis API (MelexisI2C.hpp).

#include "prosdk/drivers/thermal/melexis/MelexisI2C.hpp"

#include <algorithm>

namespace prosdk::melexis {

namespace {

std::mutex& BusMutex() {
  static std::mutex mutex;
  return mutex;
}

prosdk::I2C* g_bound = nullptr;  // guarded by BusMutex()

prosdk::I2C* Bound(uint8_t slaveAddr) {
  if (g_bound == nullptr || g_bound->GetDeviceAddress() != slaveAddr) {
    return nullptr;
  }
  return g_bound;
}

int Read(uint8_t slaveAddr, uint16_t startAddress, uint16_t count, uint16_t* data) {
  prosdk::I2C* i2c = Bound(slaveAddr);
  if (i2c == nullptr) {
    return -1;
  }
  uint8_t bytes[kMaxReadBytes];
  uint16_t done = 0;
  while (done < count) {
    int words = std::min<int>(count - done, kMaxReadBytes / 2);
    auto address = static_cast<uint16_t>(startAddress + done);
    uint8_t pointer[2] = {static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address)};
    if (i2c->Transaction(pointer, 2, bytes, words * 2)) {
      return -1;
    }
    for (int i = 0; i < words; ++i) {
      data[done + i] = static_cast<uint16_t>((bytes[2 * i] << 8) | bytes[2 * i + 1]);
    }
    done = static_cast<uint16_t>(done + words);
  }
  return 0;
}

int Write(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
  prosdk::I2C* i2c = Bound(slaveAddr);
  if (i2c == nullptr) {
    return -1;
  }
  uint8_t frame[4] = {static_cast<uint8_t>(writeAddress >> 8), static_cast<uint8_t>(writeAddress),
                      static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data)};
  return i2c->WriteBulk(frame, 4) ? -1 : 0;
}

}  // namespace

ScopedI2C::ScopedI2C(prosdk::I2C& i2c) : m_lock{BusMutex()} {
  g_bound = &i2c;
}

ScopedI2C::~ScopedI2C() {
  g_bound = nullptr;  // before m_lock releases the mutex
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead,
                     uint16_t* data) {
  return Read(slaveAddr, startAddress, nMemAddressRead, data);
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
  return Write(slaveAddr, writeAddress, data);
}

int MLX90640_I2CGeneralReset(void) {
  return -1;
}

int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead,
                     uint16_t* data) {
  return Read(slaveAddr, startAddress, nMemAddressRead, data);
}

int MLX90641_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
  return Write(slaveAddr, writeAddress, data);
}

int MLX90641_I2CGeneralReset(void) {
  return -1;
}

}  // namespace prosdk::melexis
