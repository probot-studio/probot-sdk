// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../PortsInternal.hpp"
#include "I2CDataInternal.hpp"

using namespace prosdk::hal;

namespace prosdk::hal::init {
void InitializeI2CData() {
  static I2CData sid[I2C_PORTS];
  ::prosdk::hal::SimI2CData = sid;
}
}  // namespace prosdk::hal::init

I2CData* prosdk::hal::SimI2CData;

void I2CData::ResetData() {
  initialized.Reset(false);
  read.Reset();
  write.Reset();
  std::scoped_lock lock(transferMutex);
  m_lastAddress = -1;
  m_resultCallback = nullptr;
  m_resultParam = nullptr;
}

int32_t I2CData::BeginTransfer(int32_t deviceAddress) {
  m_lastAddress = deviceAddress;
  return m_resultCallback ? m_resultCallback(m_resultParam, deviceAddress) : 0;
}

void I2CData::SetResultCallback(HALSIM_I2CResultCallback callback,
                                void* param) {
  std::scoped_lock lock(transferMutex);
  m_resultCallback = callback;
  m_resultParam = param;
}

void I2CData::Write(int32_t deviceAddress, const uint8_t* dataToSend,
                    int32_t sendSize) {
  write(dataToSend, sendSize);
}

void I2CData::Read(int32_t deviceAddress, uint8_t* buffer, int32_t count) {
  read(buffer, count);
}

extern "C" {
void HALSIM_ResetI2CData(int32_t index) {
  SimI2CData[index].ResetData();
}

int32_t HALSIM_GetI2CLastAddress(int32_t index) {
  return SimI2CData[index].GetLastAddress();
}

void HALSIM_SetI2CResultCallback(int32_t index,
                                 HALSIM_I2CResultCallback callback,
                                 void* param) {
  SimI2CData[index].SetResultCallback(callback, param);
}

#define DEFINE_CAPI(TYPE, CAPINAME, LOWERNAME)                          \
  HAL_SIMDATAVALUE_DEFINE_CAPI(TYPE, HALSIM, I2C##CAPINAME, SimI2CData, \
                               LOWERNAME)

DEFINE_CAPI(HAL_Bool, Initialized, initialized)

#undef DEFINE_CAPI
#define DEFINE_CAPI(TYPE, CAPINAME, LOWERNAME)                                 \
  HAL_SIMCALLBACKREGISTRY_DEFINE_CAPI(TYPE, HALSIM, I2C##CAPINAME, SimI2CData, \
                                      LOWERNAME)

DEFINE_CAPI(HAL_BufferCallback, Read, read)
DEFINE_CAPI(HAL_ConstBufferCallback, Write, write)

}  // extern "C"
