// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: power. Vin is probot-core's battery reading (0 when no battery sensor is
// configured: PROBOT_BATTERY_ADC_PIN or PROBOT_BATTERY_INA). Brownout is computed from it with
// the WPILib defaults (6.75 V, recovery 7.25 V) and is only reported (RobotController::
// IsBrownedOut); it does not disable outputs as the roboRIO/systemcore hardware does
// (design/hal-esp32.md, "Power"). With probot-core R12 the brownout voltage is also core's
// low-voltage threshold (io::battery::setLowVoltage, core's own default is off): set at HAL
// initialization and by every HAL_SetBrownoutVoltages, so core's isLowVoltage() flips at the same
// voltage (without prosdk's recovery hysteresis). Battery current (core's
// io::battery::currentAmps(), NaN without an INA sensor) has no HAL function: WPILib 2027 has no
// robot input current, and PowerDistribution is not supported. There is no switchable 3.3 V
// user rail.

#include "prosdk/hal/Power.h"

#include <esp32-hal.h>

#include <atomic>
#include <cmath>

#include "Esp32Internal.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/HAL.h"
#include "prosdk/platform/CoreOps.hpp"

namespace {
constexpr double kBrownoutMin = 4.0;
constexpr double kBrownoutMax = 13.0;
constexpr double kRecoveryMinDelta = 0.25;
constexpr double kNoBattery = 0.5;  // below this there is no reading at all

std::atomic<double> gBrownout{6.75};
std::atomic<double> gRecovery{7.25};
std::atomic<bool> gBrownedOut{false};
}  // namespace

namespace prosdk::hal::init {
// Weak reference from HAL_Initialize (HAL.cpp): runs only when a sketch links this module.
void InitializePower() {
  prosdk::platform::SetLowVoltage(gBrownout.load());
}
}  // namespace prosdk::hal::init

extern "C" {

double HAL_GetVinVoltage(int32_t* status) {
  *status = HAL_SUCCESS;
  return prosdk::platform::GetCoreOps().batteryVoltage();
}

HAL_Bool HAL_GetBrownedOut(int32_t* status) {
  prosdk::hal::init::CheckInit();
  *status = HAL_SUCCESS;
  double v = prosdk::platform::GetCoreOps().batteryVoltage();
  if (!(v > kNoBattery)) {
    gBrownedOut.store(false);
    return false;
  }
  bool browned = gBrownedOut.load();
  if (browned && v >= gRecovery.load()) {
    browned = false;
  } else if (!browned && v < gBrownout.load()) {
    browned = true;
  }
  gBrownedOut.store(browned);
  return browned;
}

double HAL_GetCPUTemp(int32_t* status) {
  *status = HAL_SUCCESS;
  return temperatureRead();
}

// The module's 3.3 V regulator: always on, not measured.
double HAL_GetUserVoltage3V3(int32_t* status) {
  *status = HAL_SUCCESS;
  return 3.3;
}

double HAL_GetUserCurrent3V3(int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetUserActive3V3(int32_t* status) {
  *status = HAL_SUCCESS;
  return true;
}

int32_t HAL_GetUserCurrentFaults3V3(int32_t* status) {
  *status = HAL_SUCCESS;
  return 0;
}

void HAL_SetUserRailEnabled3V3(HAL_Bool enabled, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
}

void HAL_ResetUserCurrentFaults(int32_t* status) {
  *status = HAL_SUCCESS;
}

void HAL_SetBrownoutVoltages(double brownoutVoltage, double recoveryVoltage, int32_t* status) {
  if (!std::isfinite(brownoutVoltage) || !std::isfinite(recoveryVoltage) ||
      brownoutVoltage < kBrownoutMin || brownoutVoltage > kBrownoutMax ||
      recoveryVoltage < kBrownoutMin || recoveryVoltage > kBrownoutMax + kRecoveryMinDelta ||
      recoveryVoltage < brownoutVoltage + kRecoveryMinDelta) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  gBrownout.store(brownoutVoltage);
  gRecovery.store(recoveryVoltage);
  prosdk::platform::SetLowVoltage(brownoutVoltage);  // core R12; no-op without it
  *status = HAL_SUCCESS;
}

}  // extern "C"
