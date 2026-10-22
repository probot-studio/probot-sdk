// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// HAL_* semantics follow allwpilib hal/src/main/native/systemcore/Counter.cpp (WPILib BSD license).
//
// ESP32-S3 HAL backend: single-input edge counters on a PCNT unit (PulseCounter.hpp). The rate
// is in counts per second.

#include "prosdk/hal/Counter.h"

#include <memory>

#include "Esp32Internal.hpp"
#include "PulseCounter.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/HandlesInternal.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;

namespace {
std::shared_ptr<SmartIo> GetPort(HAL_CounterHandle handle, int32_t* status) {
  auto port = smartIoHandles->Get(handle, HAL_HandleEnum::COUNTER);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
  }
  return port;
}

PulseCounter& Pcnt(const std::shared_ptr<SmartIo>& port) {
  return *static_cast<PulseCounter*>(port->driver.get());
}
}  // namespace

namespace prosdk::hal::init {
void InitializeCounter() {}
}  // namespace prosdk::hal::init

extern "C" {

HAL_CounterHandle HAL_InitializeCounter(int channel, HAL_Bool risingEdge,
                                        const char* allocationLocation, int32_t* status) {
  prosdk::hal::init::CheckInit();
  auto resource = AllocateSmartIo(channel, HAL_HandleEnum::COUNTER, "Counter");
  if (!resource) {
    *status = resource.error();
    return HAL_INVALID_HANDLE;
  }
  auto [handle, port] = *resource;
  std::unique_ptr<PulseCounter> counter;
  *status = PulseCounter::Create(channel, -1, risingEdge, &counter);
  if (*status != HAL_SUCCESS) {
    smartIoHandles->Free(handle, HAL_HandleEnum::COUNTER);
    return HAL_INVALID_HANDLE;
  }
  port->mode = PinMode::kCounter;
  port->driver = std::move(counter);
  port->previousAllocation = allocationLocation ? allocationLocation : "";
  return handle;
}

void HAL_FreeCounter(HAL_CounterHandle counterHandle) {
  auto port = smartIoHandles->Get(counterHandle, HAL_HandleEnum::COUNTER);
  if (port == nullptr) {
    return;
  }
  FreeSmartIo(counterHandle, HAL_HandleEnum::COUNTER, std::move(port), "Counter");
}

void HAL_SetCounterEdgeConfiguration(HAL_CounterHandle counterHandle, HAL_Bool risingEdge,
                                     int32_t* status) {
  auto port = GetPort(counterHandle, status);
  if (port == nullptr) {
    return;
  }
  *status = Pcnt(port).SetEdge(risingEdge);
}

void HAL_SetCounterRateWindow(HAL_CounterHandle counterHandle, int32_t windowMilliseconds,
                              int32_t* status) {
  auto port = GetPort(counterHandle, status);
  if (port == nullptr) {
    return;
  }
  *status = Pcnt(port).SetRateWindow(windowMilliseconds);
}

void HAL_ResetCounter(HAL_CounterHandle counterHandle, int32_t* status) {
  auto port = GetPort(counterHandle, status);
  if (port == nullptr) {
    return;
  }
  *status = Pcnt(port).Reset();
}

int32_t HAL_GetCounter(HAL_CounterHandle counterHandle, int32_t* status) {
  auto port = GetPort(counterHandle, status);
  if (port == nullptr) {
    return 0;
  }
  int32_t count = 0;
  *status = Pcnt(port).GetCount(&count);
  return count;
}

double HAL_GetCounterRate(HAL_CounterHandle counterHandle, int32_t* status) {
  auto port = GetPort(counterHandle, status);
  if (port == nullptr) {
    return 0;
  }
  double rate = 0;
  *status = Pcnt(port).GetRate(&rate);
  return rate;
}

HAL_Bool HAL_GetCounterStopped(HAL_CounterHandle counterHandle, int32_t* status) {
  auto port = GetPort(counterHandle, status);
  if (port == nullptr) {
    return false;
  }
  double rate = 0;
  *status = Pcnt(port).GetRate(&rate);
  if (*status != HAL_SUCCESS) {
    return false;
  }
  return rate == 0;
}

}  // extern "C"
