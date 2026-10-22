// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/hal/Types.h"
#include "prosdk/hal/simulation/NotifyListener.h"

#ifdef __cplusplus
extern "C" {
#endif

void HALSIM_ResetI2CData(int32_t index);

int32_t HALSIM_RegisterI2CInitializedCallback(int32_t index,
                                              HAL_NotifyCallback callback,
                                              void* param,
                                              HAL_Bool initialNotify);
void HALSIM_CancelI2CInitializedCallback(int32_t index, int32_t uid);
HAL_Bool HALSIM_GetI2CInitialized(int32_t index);
void HALSIM_SetI2CInitialized(int32_t index, HAL_Bool initialized);

int32_t HALSIM_RegisterI2CReadCallback(int32_t index,
                                       HAL_BufferCallback callback,
                                       void* param);
void HALSIM_CancelI2CReadCallback(int32_t index, int32_t uid);

int32_t HALSIM_RegisterI2CWriteCallback(int32_t index,
                                        HAL_ConstBufferCallback callback,
                                        void* param);
void HALSIM_CancelI2CWriteCallback(int32_t index, int32_t uid);

// prosdk additions (not in WPILib): device addresses and failure injection.
// HALSIM_GetI2CLastAddress is the 7-bit address of the transfer in progress on the port, valid
// inside a read, write or result callback; outside one it is the last address used (-1 before
// the first transfer). The result callback is called once at the start of every
// HAL_TransactionI2C, HAL_WriteI2C and HAL_ReadI2C on the port; the transfer returns its result.
// A result < 0 (an address NACK) skips the write and read callbacks. Without a result callback
// (or after setting it to NULL) every transfer returns 0, as in WPILib. Transfers on one port
// are serialized.
typedef int32_t (*HALSIM_I2CResultCallback)(void* param, int32_t address);

int32_t HALSIM_GetI2CLastAddress(int32_t index);
void HALSIM_SetI2CResultCallback(int32_t index,
                                 HALSIM_I2CResultCallback callback,
                                 void* param);

#ifdef __cplusplus
}  // extern "C"
#endif
