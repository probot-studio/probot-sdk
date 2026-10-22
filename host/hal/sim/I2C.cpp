// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hal/I2C.h"

#include <mutex>

#include "HALInitializer.hpp"
#include "mockdata/I2CDataInternal.hpp"

using namespace prosdk::hal;

namespace prosdk::hal::init {
void InitializeI2C() {}
}  // namespace prosdk::hal::init

extern "C" {
void HAL_InitializeI2C(HAL_I2CPort port, int32_t* status) {
  prosdk::hal::init::CheckInit();
  SimI2CData[port].initialized = true;
}
int32_t HAL_TransactionI2C(HAL_I2CPort port, int32_t deviceAddress,
                           const uint8_t* dataToSend, int32_t sendSize,
                           uint8_t* dataReceived, int32_t receiveSize) {
  // prosdk: per-address result (HALSIM_SetI2CResultCallback). The phases follow the ESP32
  // backend: nothing to receive is a write only (AddressOnly: one empty write), nothing to send
  // is a read only.
  std::scoped_lock lock(SimI2CData[port].transferMutex);
  int32_t result = SimI2CData[port].BeginTransfer(deviceAddress);
  if (result >= 0) {
    if (sendSize > 0 || receiveSize <= 0) {
      SimI2CData[port].Write(deviceAddress, dataToSend, sendSize);
    }
    if (receiveSize > 0) {
      SimI2CData[port].Read(deviceAddress, dataReceived, receiveSize);
    }
  }
  return result;
}
int32_t HAL_WriteI2C(HAL_I2CPort port, int32_t deviceAddress,
                     const uint8_t* dataToSend, int32_t sendSize) {
  std::scoped_lock lock(SimI2CData[port].transferMutex);
  int32_t result = SimI2CData[port].BeginTransfer(deviceAddress);
  if (result >= 0) {
    SimI2CData[port].Write(deviceAddress, dataToSend, sendSize);
  }
  return result;
}
int32_t HAL_ReadI2C(HAL_I2CPort port, int32_t deviceAddress, uint8_t* buffer,
                    int32_t count) {
  std::scoped_lock lock(SimI2CData[port].transferMutex);
  int32_t result = SimI2CData[port].BeginTransfer(deviceAddress);
  if (result >= 0) {
    SimI2CData[port].Read(deviceAddress, buffer, count);
  }
  return result;
}
void HAL_CloseI2C(HAL_I2CPort port) {
  SimI2CData[port].initialized = false;
}
}  // extern "C"
