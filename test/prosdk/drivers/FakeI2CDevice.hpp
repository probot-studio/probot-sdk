// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// A register-file I2C device on the HAL sim bus (port 0): a write sets the register pointer
// (first byte) and stores the rest with auto-increment; a read returns registers from the
// pointer on.

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "prosdk/hal/simulation/I2CData.h"

namespace prosdk::test {

class FakeI2CDevice {
 public:
  FakeI2CDevice() {
    m_readUid = HALSIM_RegisterI2CReadCallback(0, &ReadCallback, this);
    m_writeUid = HALSIM_RegisterI2CWriteCallback(0, &WriteCallback, this);
  }
  ~FakeI2CDevice() {
    HALSIM_CancelI2CReadCallback(0, m_readUid);
    HALSIM_CancelI2CWriteCallback(0, m_writeUid);
  }
  FakeI2CDevice(const FakeI2CDevice&) = delete;
  FakeI2CDevice& operator=(const FakeI2CDevice&) = delete;

  std::array<uint8_t, 256> regs{};
  std::vector<std::vector<uint8_t>> writes;  // every write transaction

 private:
  static void ReadCallback(const char*, void* param, unsigned char* buffer, unsigned int count) {
    auto* self = static_cast<FakeI2CDevice*>(param);
    for (unsigned int i = 0; i < count; ++i) {
      buffer[i] = self->regs[(self->m_pointer + i) & 0xFF];
    }
  }
  static void WriteCallback(const char*, void* param, const unsigned char* buffer,
                            unsigned int count) {
    auto* self = static_cast<FakeI2CDevice*>(param);
    if (count == 0) {
      return;
    }
    self->writes.emplace_back(buffer, buffer + count);
    self->m_pointer = buffer[0];
    for (unsigned int i = 1; i < count; ++i) {
      self->regs[(self->m_pointer + i - 1) & 0xFF] = buffer[i];
    }
  }

  int32_t m_readUid;
  int32_t m_writeUid;
  uint8_t m_pointer = 0;
};

}  // namespace prosdk::test
