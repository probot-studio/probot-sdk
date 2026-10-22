// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include <string>

#include "PortsInternal.hpp"
#include "prosdk/hal/handles/HandlesInternal.hpp"
#include "prosdk/hal/handles/IndexedHandleResource.hpp"

namespace prosdk::hal {

struct AnalogPort {
  uint8_t channel;
  std::string previousAllocation;
};

extern IndexedHandleResource<HAL_AnalogInputHandle, prosdk::hal::AnalogPort,
                             NUM_ANALOG_INPUTS, HAL_HandleEnum::ANALOG_INPUT>*
    analogInputHandles;
}  // namespace prosdk::hal
