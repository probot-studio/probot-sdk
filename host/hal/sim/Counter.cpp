// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hal/Counter.h"

#include "CounterInternal.hpp"
#include "HALInitializer.hpp"
#include "PortsInternal.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/HandlesInternal.hpp"
#include "prosdk/hal/handles/LimitedHandleResource.hpp"

namespace prosdk::hal {

LimitedHandleResource<HAL_CounterHandle, Counter, NUM_COUNTERS,
                      HAL_HandleEnum::COUNTER>* counterHandles;
}  // namespace prosdk::hal

namespace prosdk::hal::init {
void InitializeCounter() {
  static LimitedHandleResource<HAL_CounterHandle, Counter, NUM_COUNTERS,
                               HAL_HandleEnum::COUNTER>
      cH;
  counterHandles = &cH;
}
}  // namespace prosdk::hal::init

extern "C" {
HAL_CounterHandle HAL_InitializeCounter(int channel, HAL_Bool risingEdge,
                                        const char* allocationLocation,
                                        int32_t* status) {
  prosdk::hal::init::CheckInit();
  return 0;
}
void HAL_FreeCounter(HAL_CounterHandle counterHandle) {}
void HAL_SetCounterEdgeConfiguration(HAL_CounterHandle counterHandle,
                                     HAL_Bool risingEdge, int32_t* status) {}
void HAL_SetCounterRateWindow(HAL_CounterHandle counterHandle,
                              int32_t windowMilliseconds, int32_t* status) {
  if (windowMilliseconds < 5 || windowMilliseconds > 255) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }

  *status = 0;
}
void HAL_ResetCounter(HAL_CounterHandle counterHandle, int32_t* status) {}
int32_t HAL_GetCounter(HAL_CounterHandle counterHandle, int32_t* status) {
  return 0;
}
double HAL_GetCounterRate(HAL_CounterHandle counterHandle, int32_t* status) {
  return 0.0;
}
HAL_Bool HAL_GetCounterStopped(HAL_CounterHandle counterHandle,
                               int32_t* status) {
  return HAL_GetCounterRate(counterHandle, status) == 0.0;
}
}  // extern "C"
