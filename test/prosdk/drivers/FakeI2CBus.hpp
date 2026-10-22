// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Several register-file I2C devices on one HAL sim port, told apart by address
// (HALSIM_GetI2CLastAddress, tools/upstream/patches/hal/0004). A transfer to an address with no
// device, or to a device with nack set, fails like an address NACK (the HAL call returns -1, so
// prosdk::I2C reports an abort). design/boardoza/F2-sim-i2c.md.
//
// A write sets the register pointer (its first pointerBits / 8 bytes, MSB first) and stores the
// rest from the pointer on; a read returns registers from the pointer on. Both auto-increment the
// pointer. With registerBits 16 every register is a 16-bit word, sent big- or little-endian, and
// the pointer counts words. onWrite / onRead replace the register file for command-style chips.
// As on the ESP32, AddressOnly() is one empty write and a transaction with nothing to send is a
// read only (no empty write is logged).
//
// One FakeI2CBus per port at a time (it owns the port's result callback), and not together with
// FakeI2CDevice on the same port. The callbacks lock mutex: a test that touches a device while a
// driver thread runs holds it, and never calls the driver while holding it.

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <span>
#include <vector>

#include "prosdk/hal/simulation/I2CData.h"

namespace prosdk::test {

class FakeI2CBus {
 public:
  struct Device {
    Device(int pointerBits, int registerBits, bool littleEndian)
        : pointerBits{pointerBits == 16 ? 16 : 8},
          registerBits{registerBits == 16 ? 16 : 8},
          littleEndian{littleEndian},
          regs(std::size_t{1} << this->pointerBits) {}

    const int pointerBits;    // 8 or 16 (a 16-bit pointer is sent MSB first)
    const int registerBits;   // 8, or 16 for word registers
    const bool littleEndian;  // byte order of a 16-bit register on the bus
    bool nack = false;        // true: the device stops answering
    std::vector<uint16_t> regs;  // 2^pointerBits registers (8-bit registers use the low byte)
    int pointer = 0;
    std::vector<std::vector<uint8_t>> writes;  // every write, pointer bytes included (may be empty)
    int reads = 0;                             // read transfers served (empty ones not counted)
    std::function<void(std::span<const uint8_t>)> onWrite;  // set: replaces the register file
    std::function<void(std::span<uint8_t>)> onRead;
  };

  explicit FakeI2CBus(int port = 0) : m_port{port} {
    m_readUid = HALSIM_RegisterI2CReadCallback(port, &ReadCallback, this);
    m_writeUid = HALSIM_RegisterI2CWriteCallback(port, &WriteCallback, this);
    HALSIM_SetI2CResultCallback(port, &ResultCallback, this);
  }
  ~FakeI2CBus() {
    HALSIM_SetI2CResultCallback(m_port, nullptr, nullptr);
    HALSIM_CancelI2CReadCallback(m_port, m_readUid);
    HALSIM_CancelI2CWriteCallback(m_port, m_writeUid);
  }
  FakeI2CBus(const FakeI2CBus&) = delete;
  FakeI2CBus& operator=(const FakeI2CBus&) = delete;

  // Adds (or replaces) the device at a 7-bit address. The reference stays valid until Remove.
  Device& Add(int address, int pointerBits = 8, int registerBits = 8, bool littleEndian = false) {
    std::scoped_lock lock(mutex);
    m_devices.erase(address);
    return m_devices.try_emplace(address, pointerBits, registerBits, littleEndian).first->second;
  }
  void Remove(int address) {
    std::scoped_lock lock(mutex);
    m_devices.erase(address);
  }
  Device& operator[](int address) { return m_devices.at(address); }

  std::mutex mutex;

 private:
  Device* Find(int address) {
    auto it = m_devices.find(address);
    return it == m_devices.end() ? nullptr : &it->second;
  }

  static int32_t ResultCallback(void* param, int32_t address) {
    auto* self = static_cast<FakeI2CBus*>(param);
    std::scoped_lock lock(self->mutex);
    Device* dev = self->Find(address);
    return dev && !dev->nack ? 0 : -1;
  }

  static void WriteCallback(const char*, void* param, const unsigned char* buffer,
                            unsigned int count) {
    auto* self = static_cast<FakeI2CBus*>(param);
    std::scoped_lock lock(self->mutex);
    Device* dev = self->Find(HALSIM_GetI2CLastAddress(self->m_port));
    if (!dev) {
      return;
    }
    dev->writes.emplace_back(buffer, buffer + count);
    if (dev->onWrite) {
      dev->onWrite({buffer, count});
      return;
    }
    unsigned int pointerBytes = dev->pointerBits / 8;
    if (count < pointerBytes) {
      return;  // no complete pointer: nothing changes
    }
    dev->pointer = pointerBytes == 2 ? (buffer[0] << 8) | buffer[1] : buffer[0];
    int size = static_cast<int>(dev->regs.size());
    if (dev->registerBits == 8) {
      for (unsigned int i = pointerBytes; i < count; ++i) {
        dev->regs[dev->pointer] = buffer[i];
        dev->pointer = (dev->pointer + 1) % size;
      }
    } else {
      for (unsigned int i = pointerBytes; i + 1 < count; i += 2) {  // a trailing odd byte is dropped
        uint16_t first = buffer[i];
        uint16_t second = buffer[i + 1];
        dev->regs[dev->pointer] = dev->littleEndian ? (second << 8) | first : (first << 8) | second;
        dev->pointer = (dev->pointer + 1) % size;
      }
    }
  }

  static void ReadCallback(const char*, void* param, unsigned char* buffer, unsigned int count) {
    auto* self = static_cast<FakeI2CBus*>(param);
    std::scoped_lock lock(self->mutex);
    Device* dev = self->Find(HALSIM_GetI2CLastAddress(self->m_port));
    if (!dev || count == 0) {
      return;
    }
    ++dev->reads;
    if (dev->onRead) {
      dev->onRead({buffer, count});
      return;
    }
    int size = static_cast<int>(dev->regs.size());
    if (dev->registerBits == 8) {
      for (unsigned int i = 0; i < count; ++i) {
        buffer[i] = static_cast<uint8_t>(dev->regs[dev->pointer]);
        dev->pointer = (dev->pointer + 1) % size;
      }
    } else {
      for (unsigned int i = 0; i < count; ++i) {
        uint16_t word = dev->regs[(dev->pointer + i / 2) % size];
        bool high = (i % 2 == 0) != dev->littleEndian;
        buffer[i] = static_cast<uint8_t>(high ? word >> 8 : word & 0xFF);
      }
      dev->pointer = (dev->pointer + (count + 1) / 2) % size;
    }
  }

  int m_port;
  int32_t m_readUid;
  int32_t m_writeUid;
  std::map<int, Device> m_devices;
};

}  // namespace prosdk::test
