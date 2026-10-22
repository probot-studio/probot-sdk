// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "FakeI2CBus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/I2C.h"
#include "prosdk/hal/simulation/I2CData.h"
#include "prosdk/hardware/bus/I2C.hpp"

using namespace prosdk;

namespace {
struct ResultLog {
  std::vector<int32_t> addresses;    // asked by the result callback
  std::vector<int32_t> seenInWrite;  // HALSIM_GetI2CLastAddress inside the write callback
  int32_t result = 0;
};

int32_t LogResult(void* param, int32_t address) {
  auto* log = static_cast<ResultLog*>(param);
  log->addresses.push_back(address);
  return log->result;
}

void LogWrite(const char*, void* param, const unsigned char*, unsigned int) {
  static_cast<ResultLog*>(param)->seenInWrite.push_back(HALSIM_GetI2CLastAddress(1));
}
}  // namespace

TEST_CASE("Sim I2C reports the address and the per-address result", "[i2c][sim]") {
  ResultLog log;
  int32_t uid = HALSIM_RegisterI2CWriteCallback(1, &LogWrite, &log);
  uint8_t byte = 0x42;
  uint8_t rx = 0;
  CHECK(HAL_WriteI2C(HAL_I2C_PORT_1, 0x19, &byte, 1) == 0);  // no result callback: success
  CHECK(HALSIM_GetI2CLastAddress(1) == 0x19);
  CHECK(log.seenInWrite == std::vector<int32_t>{0x19});

  HALSIM_SetI2CResultCallback(1, &LogResult, &log);
  CHECK(HAL_TransactionI2C(HAL_I2C_PORT_1, 0x1E, &byte, 1, &rx, 1) == 0);
  CHECK(log.seenInWrite == std::vector<int32_t>{0x19, 0x1E});
  // nothing to send: a read only, as on the ESP32 (no empty write); nothing to receive: a write
  CHECK(HAL_TransactionI2C(HAL_I2C_PORT_1, 0x27, nullptr, 0, &rx, 1) == 0);
  CHECK(HAL_TransactionI2C(HAL_I2C_PORT_1, 0x27, nullptr, 0, nullptr, 0) == 0);
  CHECK(log.seenInWrite == std::vector<int32_t>{0x19, 0x1E, 0x27});
  log.result = -1;  // NACK: the result is returned and no data callback runs
  CHECK(HAL_WriteI2C(HAL_I2C_PORT_1, 0x68, &byte, 1) == -1);
  CHECK(HAL_ReadI2C(HAL_I2C_PORT_1, 0x69, &rx, 1) == -1);
  CHECK(HAL_TransactionI2C(HAL_I2C_PORT_1, 0x00, &byte, 1, &rx, 1) == -1);
  CHECK(log.addresses == std::vector<int32_t>{0x1E, 0x27, 0x27, 0x68, 0x69, 0x00});
  CHECK(log.seenInWrite.size() == 3);
  CHECK(HALSIM_GetI2CLastAddress(1) == 0x00);

  HALSIM_SetI2CResultCallback(1, nullptr, nullptr);
  CHECK(HAL_WriteI2C(HAL_I2C_PORT_1, 0x68, &byte, 1) == 0);
  CHECK(log.addresses.size() == 6);
  HALSIM_CancelI2CWriteCallback(1, uid);
  HALSIM_ResetI2CData(1);
  CHECK(HALSIM_GetI2CLastAddress(1) == -1);
}

TEST_CASE("FakeI2CBus tells devices apart and NACKs unknown addresses", "[i2c][sim]") {
  test::FakeI2CBus bus;
  auto& accel = bus.Add(0x19);
  auto& mag = bus.Add(0x1E);
  accel.regs[0x0F] = 0x33;
  mag.regs[0x4F] = 0x40;

  I2C accelI2C{I2C::Port::PORT_0, 0x19};
  I2C magI2C{I2C::Port::PORT_0, 0x1E};
  I2C noneI2C{I2C::Port::PORT_0, 0x50};
  uint8_t id = 0;
  CHECK_FALSE(accelI2C.Read(0x0F, 1, &id));
  CHECK(id == 0x33);
  CHECK_FALSE(magI2C.Read(0x4F, 1, &id));
  CHECK(id == 0x40);
  CHECK_FALSE(accelI2C.Write(0x20, 0x57));
  CHECK(accel.regs[0x20] == 0x57);
  CHECK(mag.regs[0x20] == 0);
  CHECK(accel.writes.size() == 2);  // the pointer write of Read, then Write
  CHECK(accel.writes.back() == std::vector<uint8_t>{0x20, 0x57});
  CHECK(mag.writes.size() == 1);
  CHECK(accel.reads == 1);

  id = 0xAA;
  CHECK(noneI2C.Read(0x00, 1, &id));  // aborted
  CHECK(id == 0xAA);                  // untouched
  CHECK(noneI2C.AddressOnly());
  CHECK_FALSE(magI2C.AddressOnly());

  mag.nack = true;  // the device drops off the bus
  CHECK(magI2C.Write(0x60, 0x01));
  CHECK(mag.writes.size() == 2);  // unchanged: the pointer write and the empty AddressOnly write
  CHECK(mag.writes.back().empty());
  mag.nack = false;
  CHECK_FALSE(magI2C.Write(0x60, 0x01));

  bus.Remove(0x1E);
  CHECK(magI2C.AddressOnly());
}

TEST_CASE("FakeI2CBus auto-increments the register pointer", "[i2c][sim]") {
  test::FakeI2CBus bus;
  auto& dev = bus.Add(0x40);
  I2C i2c{I2C::Port::PORT_0, 0x40};
  std::array<uint8_t, 4> data{0x10, 0x11, 0x12, 0x13};
  std::array<uint8_t, 5> frame{0xFE, 1, 2, 3, 4};  // wraps past 0xFF
  CHECK_FALSE(i2c.WriteBulk(frame.data(), static_cast<int>(frame.size())));
  CHECK(dev.regs[0xFE] == 1);
  CHECK(dev.regs[0xFF] == 2);
  CHECK(dev.regs[0x00] == 3);
  CHECK(dev.regs[0x01] == 4);

  dev.regs[0x05] = 0x55;
  dev.regs[0x06] = 0x66;
  dev.regs[0x07] = 0x77;
  CHECK_FALSE(i2c.Read(0x05, 2, data.data()));
  CHECK(data[0] == 0x55);
  CHECK(data[1] == 0x66);
  CHECK_FALSE(i2c.ReadOnly(1, data.data()));  // continues from the pointer
  CHECK(data[0] == 0x77);
  CHECK(dev.pointer == 0x08);
}

TEST_CASE("FakeI2CBus serves 16-bit pointers and word registers", "[i2c][sim]") {
  test::FakeI2CBus bus{1};
  auto& big = bus.Add(0x33, 16, 16);        // 16-bit address, big-endian words (MLX9064x style)
  auto& little = bus.Add(0x58, 8, 16, true);  // 8-bit pointer, little-endian words
  auto& bytes = bus.Add(0x29, 16);          // 16-bit index, byte registers (VL53L7CX style)
  big.regs[0x2400] = 0x1234;
  big.regs[0x2401] = 0xABCD;

  I2C bigI2C{I2C::Port::PORT_1, 0x33};
  std::array<uint8_t, 2> addr{0x24, 0x00};
  std::array<uint8_t, 4> rx{};
  CHECK_FALSE(bigI2C.Transaction(addr.data(), 2, rx.data(), 4));
  CHECK(rx == std::array<uint8_t, 4>{0x12, 0x34, 0xAB, 0xCD});
  CHECK(big.pointer == 0x2402);
  std::array<uint8_t, 5> wr{0x80, 0x0D, 0x19, 0x01, 0xFF};  // the odd trailing byte is dropped
  CHECK_FALSE(bigI2C.WriteBulk(wr.data(), static_cast<int>(wr.size())));
  CHECK(big.regs[0x800D] == 0x1901);
  CHECK(big.regs[0x800E] == 0);

  I2C littleI2C{I2C::Port::PORT_1, 0x58};
  std::array<uint8_t, 3> code{0x02, 0x00, 0x08};  // 0x0800, low byte first
  CHECK_FALSE(littleI2C.WriteBulk(code.data(), 3));
  CHECK(little.regs[0x02] == 0x0800);
  CHECK_FALSE(littleI2C.Read(0x02, 2, rx.data()));
  CHECK(rx[0] == 0x00);
  CHECK(rx[1] == 0x08);

  I2C bytesI2C{I2C::Port::PORT_1, 0x29};
  std::array<uint8_t, 4> chunk{0x7F, 0xFF, 0xA5, 0x5A};
  CHECK_FALSE(bytesI2C.WriteBulk(chunk.data(), 4));
  CHECK(bytes.regs[0x7FFF] == 0xA5);
  CHECK(bytes.regs[0x8000] == 0x5A);
  CHECK(bytes.writes.front() == std::vector<uint8_t>{0x7F, 0xFF, 0xA5, 0x5A});
}

TEST_CASE("FakeI2CBus hooks serve command-style devices", "[i2c][sim]") {
  test::FakeI2CBus bus;
  auto& adc = bus.Add(0x77);
  std::vector<uint8_t> commands;
  adc.onWrite = [&](std::span<const uint8_t> data) {
    commands.insert(commands.end(), data.begin(), data.end());
  };
  adc.onRead = [](std::span<uint8_t> out) {
    for (std::size_t i = 0; i < out.size(); ++i) {
      out[i] = static_cast<uint8_t>(0xC0 + i);
    }
  };
  auto& general = bus.Add(0x00);  // general call: a device like any other

  I2C adcI2C{I2C::Port::PORT_0, 0x77};
  uint8_t cmd = 0x48;
  CHECK_FALSE(adcI2C.WriteBulk(&cmd, 1));
  std::array<uint8_t, 3> rx{};
  CHECK_FALSE(adcI2C.Read(0x00, 3, rx.data()));
  CHECK(commands == std::vector<uint8_t>{0x48, 0x00});
  CHECK(rx == std::array<uint8_t, 3>{0xC0, 0xC1, 0xC2});
  CHECK(adc.regs[0x48] == 0);  // the register file is bypassed

  I2C generalI2C{I2C::Port::PORT_0, 0x00};
  uint8_t reload = 0x04;
  CHECK_FALSE(generalI2C.WriteBulk(&reload, 1));
  REQUIRE(general.writes.size() == 1);
  CHECK(general.writes[0] == std::vector<uint8_t>{0x04});
}
