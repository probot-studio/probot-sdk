// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Replacement for the upstream hal/cpp/DashboardOpMode.cpp, which publishes "Auto/Teleop/Utility
// OpMode" string choosers over ntcore so a dashboard can pick the opmode when the Driver Station
// cannot. ntcore is not ported (design/API-DIFFERENCES.md). The probot-core Driver Station lists
// and selects opmodes itself (design/core-requests.md), so here the dashboard path is inert:
// options are accepted and ignored, and no opmode is ever selected by a dashboard (0).
// Backend-independent: compiled into the archive and the host build.

#include "prosdk/hal/DashboardOpMode.hpp"

namespace prosdk::hal {

void InitializeDashboardOpMode() {}

void SetDashboardOpModeOptions(std::span<const HAL_OpModeOption> options) {
  (void)options;
}

void StartDashboardOpMode() {}

void EnableDashboardOpMode() {}

int64_t GetDashboardSelectedOpMode(HAL_RobotMode robotMode) {
  (void)robotMode;
  return 0;
}

}  // namespace prosdk::hal
