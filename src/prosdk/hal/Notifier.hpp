// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string_view>

#include "prosdk/hal/Notifier.h"
#include "prosdk/util/string.hpp"

/**
 * Sets the name of a notifier.
 *
 * @param[in] notifierHandle the notifier handle
 * @param[in] name name
 * @param[out] status Error status variable. 0 on success.
 */
inline void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle,
                                std::string_view name, int32_t* status) {
  WPI_String nameStr = prosdk::util::make_string(name);
  HAL_SetNotifierName(notifierHandle, &nameStr, status);
}
