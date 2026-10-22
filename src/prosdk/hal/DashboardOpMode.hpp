// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <span>

#include "prosdk/hal/DriverStationTypes.h"

namespace prosdk::hal {

void InitializeDashboardOpMode();
void SetDashboardOpModeOptions(std::span<const HAL_OpModeOption> options);
void StartDashboardOpMode();
void EnableDashboardOpMode();
int64_t GetDashboardSelectedOpMode(HAL_RobotMode robotMode);

}  // namespace prosdk::hal
