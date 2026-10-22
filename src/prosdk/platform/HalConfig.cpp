// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/HalConfig.hpp"

#include <mutex>

namespace prosdk::platform {

namespace {
std::mutex gMutex;
I2CConfig gI2C[kNumI2CPorts] = {{8, 9, 400000}, {-1, -1, 400000}};
SerialConfig gSerial[kNumSerialPorts] = {{15, 16}, {-1, -1}};
CanConfig gCan = {-1, -1, 1000000};
SPIConfig gSPI[kNumSPIBuses] = {{-1, -1, -1}, {-1, -1, -1}};

constexpr int kMaxGpio = 48;

bool ValidPin(int pin) {
  return pin >= 0 && pin <= kMaxGpio;
}
}  // namespace

bool SetI2CPins(int port, int sda, int scl, uint32_t frequencyHz) {
  if (port < 0 || port >= kNumI2CPorts || !ValidPin(sda) || !ValidPin(scl) || sda == scl ||
      frequencyHz == 0 || frequencyHz > 1000000) {
    return false;
  }
  std::scoped_lock lock(gMutex);
  gI2C[port] = I2CConfig{sda, scl, frequencyHz};
  return true;
}

I2CConfig GetI2CConfig(int port) {
  if (port < 0 || port >= kNumI2CPorts) {
    return I2CConfig{-1, -1, 0};
  }
  std::scoped_lock lock(gMutex);
  return gI2C[port];
}

bool SetSerialPins(int port, int rx, int tx) {
  if (port < 0 || port >= kNumSerialPorts || !ValidPin(rx) || !ValidPin(tx) || rx == tx) {
    return false;
  }
  std::scoped_lock lock(gMutex);
  gSerial[port] = SerialConfig{rx, tx};
  return true;
}

SerialConfig GetSerialConfig(int port) {
  if (port < 0 || port >= kNumSerialPorts) {
    return SerialConfig{-1, -1};
  }
  std::scoped_lock lock(gMutex);
  return gSerial[port];
}

bool SetSPIPins(int bus, int sck, int mosi, int miso) {
  if (bus < 0 || bus >= kNumSPIBuses || !ValidPin(sck) || !ValidPin(mosi) ||
      !(miso == -1 || ValidPin(miso)) || sck == mosi || sck == miso || mosi == miso) {
    return false;
  }
  std::scoped_lock lock(gMutex);
  gSPI[bus] = SPIConfig{sck, mosi, miso};
  return true;
}

SPIConfig GetSPIConfig(int bus) {
  if (bus < 0 || bus >= kNumSPIBuses) {
    return SPIConfig{-1, -1, -1};
  }
  std::scoped_lock lock(gMutex);
  return gSPI[bus];
}

bool SetCanConfig(int tx, int rx, uint32_t bitrate) {
  if (!ValidPin(tx) || !ValidPin(rx) || tx == rx ||
      (bitrate != 125000 && bitrate != 250000 && bitrate != 500000 && bitrate != 1000000)) {
    return false;
  }
  std::scoped_lock lock(gMutex);
  gCan = CanConfig{tx, rx, bitrate};
  return true;
}

CanConfig GetCanConfig() {
  std::scoped_lock lock(gMutex);
  return gCan;
}

}  // namespace prosdk::platform
