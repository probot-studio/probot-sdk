// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <mutex>

#include "prosdk/hal/simulation/I2CData.h"
#include "prosdk/hal/simulation/SimCallbackRegistry.hpp"
#include "prosdk/hal/simulation/SimDataValue.hpp"

namespace prosdk::hal {
class I2CData {
  HAL_SIMDATAVALUE_DEFINE_NAME(Initialized)
  HAL_SIMCALLBACKREGISTRY_DEFINE_NAME(Read)
  HAL_SIMCALLBACKREGISTRY_DEFINE_NAME(Write)

 public:
  void Write(int32_t deviceAddress, const uint8_t* dataToSend,
             int32_t sendSize);
  void Read(int32_t deviceAddress, uint8_t* buffer, int32_t count);

  // prosdk: device address and per-address result. A transfer holds transferMutex, calls
  // BeginTransfer and runs Write/Read only when the result is >= 0.
  int32_t BeginTransfer(int32_t deviceAddress);
  int32_t GetLastAddress() const { return m_lastAddress; }
  void SetResultCallback(HALSIM_I2CResultCallback callback, void* param);
  std::recursive_mutex transferMutex;

  SimDataValue<HAL_Bool, HAL_MakeBoolean, GetInitializedName> initialized{
      false};
  SimCallbackRegistry<HAL_BufferCallback, GetReadName> read;
  SimCallbackRegistry<HAL_ConstBufferCallback, GetWriteName> write;

  void ResetData();

 private:
  std::atomic<int32_t> m_lastAddress{-1};
  HALSIM_I2CResultCallback m_resultCallback = nullptr;  // guarded by transferMutex
  void* m_resultParam = nullptr;
};
extern I2CData* SimI2CData;
}  // namespace prosdk::hal
