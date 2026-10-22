// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "PortsInternal.hpp"
#include "prosdk/hal/handles/HandlesInternal.hpp"
#include "prosdk/hal/handles/LimitedHandleResource.hpp"

namespace prosdk::hal {

struct Counter {
  uint8_t index;
};

extern LimitedHandleResource<HAL_CounterHandle, Counter, NUM_COUNTERS,
                             HAL_HandleEnum::COUNTER>* counterHandles;

}  // namespace prosdk::hal
