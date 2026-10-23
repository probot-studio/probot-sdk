// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// HAL_* semantics follow allwpilib hal/src/main/native/systemcore/I2C.cpp (WPILib BSD license).
//
// ESP32-S3 HAL backend: I2C masters (design/hal-esp32.md, "I2C").
// Port 0 (kPort0) is controller 0, the bus Arduino's Wire uses: if Wire already started it, the
// HAL shares it as is. With probot-core's INA battery sensor the HAL starts it through core's
// Wire.begin on core's pins (prosdk::platform::GetCoreI2CStart), whichever of core's battery task
// and the robot comes first, and core then sets 100 kHz. Otherwise the HAL starts it on the pins
// from prosdk::platform::SetI2CPins (default SDA 1, SCL 2, 400 kHz: the Boardoza Pulse J3).
// Port 1 is controller 1 and needs SetI2CPins(1, ...) first. Transfers go through
// Arduino's I2C HAL, which serializes users of a bus. Transfers return 0 on success and -1 on
// failure, like systemcore's ioctl.

#include "prosdk/hal/I2C.h"

#include <esp32-hal-i2c.h>

#include <format>
#include <mutex>

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/HalConfig.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;

namespace {

constexpr uint32_t kTimeoutMs = 50;

struct I2CBus {
  std::mutex mutex;
  int count = 0;
  bool ownsBus = false;  // the HAL started the controller (else it is Wire's)
};

I2CBus gBuses[kNumI2CBuses];

bool ValidPort(HAL_I2CPort port) {
  return port >= 0 && port < kNumI2CBuses;
}

int32_t Result(esp_err_t err) {
  return err == ESP_OK ? 0 : -1;
}

}  // namespace

namespace prosdk::hal::init {
void InitializeI2C() {}
}  // namespace prosdk::hal::init

extern "C" {

void HAL_InitializeI2C(HAL_I2CPort port, int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (!ValidPort(port)) {
    *status = MakeErrorIndexOutOfRange(HAL_RESOURCE_OUT_OF_RANGE, "Invalid Index for I2C", 0,
                                       kNumI2CBuses - 1, port);
    return;
  }
  I2CBus& bus = gBuses[port];
  std::scoped_lock lock(bus.mutex);
  *status = HAL_SUCCESS;
  if (bus.count > 0) {
    ++bus.count;
    return;
  }
  const uint8_t num = static_cast<uint8_t>(port);
  // probot-core's INA battery sensor owns controller 0 (Wire) on its pins, which are reserved,
  // and starts it from its own task: start it the same way here, whichever task comes first.
  prosdk::platform::CoreI2CStart coreStart = num == 0 ? prosdk::platform::GetCoreI2CStart()
                                                      : nullptr;
  if (coreStart != nullptr && !i2cIsInit(num) && !coreStart()) {
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                        "I2C port 0 could not start on probot-core's battery sensor pins "
                        "(PROBOT_BATTERY_INA_SDA / PROBOT_BATTERY_INA_SCL)");
    return;
  }
  if (i2cIsInit(num)) {
    bus.ownsBus = false;  // shared with Wire / probot-core
    bus.count = 1;
    return;
  }
  prosdk::platform::I2CConfig config = prosdk::platform::GetI2CConfig(port);
  if (config.sda < 0 || config.scl < 0) {
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                        std::format("I2C port {} has no pins: call "
                                    "prosdk::platform::SetI2CPins({}, sda, scl) first",
                                    static_cast<int>(port), static_cast<int>(port)));
    return;
  }
  for (int gpio : {config.sda, config.scl}) {
    *status = CheckPinUsable(gpio, "I2C");
    if (*status != HAL_SUCCESS) {
      return;
    }
  }
  esp_err_t err = i2cInit(num, static_cast<int8_t>(config.sda), static_cast<int8_t>(config.scl),
                          config.frequencyHz);
  if (err != ESP_OK) {
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                        std::format("I2C port {} could not start on SDA {} / SCL {} ({})",
                                    static_cast<int>(port), config.sda, config.scl,
                                    EspErrName(err)));
    return;
  }
  bus.ownsBus = true;
  bus.count = 1;
}

int32_t HAL_TransactionI2C(HAL_I2CPort port, int32_t deviceAddress, const uint8_t* dataToSend,
                           int32_t sendSize, uint8_t* dataReceived, int32_t receiveSize) {
  if (!ValidPort(port)) {
    (void)MakeErrorIndexOutOfRange(HAL_PARAMETER_OUT_OF_RANGE, "Invalid Index for I2C", 0,
                                   kNumI2CBuses - 1, port);
    return -1;
  }
  const uint8_t num = static_cast<uint8_t>(port);
  const uint16_t address = static_cast<uint16_t>(deviceAddress);
  if (receiveSize <= 0) {
    return Result(i2cWrite(num, address, dataToSend, sendSize, kTimeoutMs));
  }
  if (sendSize <= 0) {
    size_t read = 0;
    esp_err_t err = i2cRead(num, address, dataReceived, receiveSize, kTimeoutMs, &read);
    return err == ESP_OK && read == static_cast<size_t>(receiveSize) ? 0 : -1;
  }
  size_t read = 0;
  esp_err_t err = i2cWriteReadNonStop(num, address, const_cast<uint8_t*>(dataToSend), sendSize,
                                      dataReceived, receiveSize, kTimeoutMs, &read);
  return err == ESP_OK && read == static_cast<size_t>(receiveSize) ? 0 : -1;
}

int32_t HAL_WriteI2C(HAL_I2CPort port, int32_t deviceAddress, const uint8_t* dataToSend,
                     int32_t sendSize) {
  if (!ValidPort(port)) {
    (void)MakeErrorIndexOutOfRange(HAL_PARAMETER_OUT_OF_RANGE, "Invalid Index for I2C", 0,
                                   kNumI2CBuses - 1, port);
    return -1;
  }
  return Result(i2cWrite(static_cast<uint8_t>(port), static_cast<uint16_t>(deviceAddress),
                         dataToSend, sendSize, kTimeoutMs));
}

int32_t HAL_ReadI2C(HAL_I2CPort port, int32_t deviceAddress, uint8_t* buffer, int32_t count) {
  if (!ValidPort(port)) {
    (void)MakeErrorIndexOutOfRange(HAL_PARAMETER_OUT_OF_RANGE, "Invalid Index for I2C", 0,
                                   kNumI2CBuses - 1, port);
    return -1;
  }
  size_t read = 0;
  esp_err_t err = i2cRead(static_cast<uint8_t>(port), static_cast<uint16_t>(deviceAddress),
                          buffer, count, kTimeoutMs, &read);
  return err == ESP_OK && read == static_cast<size_t>(count) ? 0 : -1;
}

void HAL_CloseI2C(HAL_I2CPort port) {
  if (!ValidPort(port)) {
    (void)MakeErrorIndexOutOfRange(HAL_PARAMETER_OUT_OF_RANGE, "Invalid Index for I2C", 0,
                                   kNumI2CBuses - 1, port);
    return;
  }
  I2CBus& bus = gBuses[port];
  std::scoped_lock lock(bus.mutex);
  if (bus.count == 0) {
    return;
  }
  if (--bus.count == 0 && bus.ownsBus) {
    i2cDeinit(static_cast<uint8_t>(port));
    bus.ownsBus = false;
  }
}

}  // extern "C"
