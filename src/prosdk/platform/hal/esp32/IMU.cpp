// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: onboard IMU. The Probot board has no onboard IMU, so every read returns
// zeros with HAL_HANDLE_ERROR and a message (OnboardIMU's WPILIB_CheckErrorStatus throws
// prosdk::RuntimeError on it); the yaw getters, which have no status, return 0 with the current
// timestamp. External IMUs are used
// through I2C/serial device classes instead.

#include "prosdk/hal/IMU.h"

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/HAL.h"

namespace {
constexpr const char* kNoImu = "The ESP32-S3 Probot board has no onboard IMU";

void NoImu(int32_t* status) {
  prosdk::hal::init::CheckInit();
  *status = prosdk::hal::MakeError(HAL_HANDLE_ERROR, kNoImu);
}
}  // namespace

extern "C" {

void HAL_GetIMUAcceleration(HAL_Acceleration3d* accel, int32_t* status) {
  *accel = HAL_Acceleration3d{};
  accel->timestamp = HAL_GetMonotonicTime();
  NoImu(status);
}

void HAL_GetIMUGyroRates(HAL_GyroRate3d* rates, int32_t* status) {
  *rates = HAL_GyroRate3d{};
  rates->timestamp = HAL_GetMonotonicTime();
  NoImu(status);
}

void HAL_GetIMUEulerAnglesFlat(HAL_EulerAngles3d* angles, int32_t* status) {
  *angles = HAL_EulerAngles3d{};
  angles->timestamp = HAL_GetMonotonicTime();
  NoImu(status);
}

void HAL_GetIMUEulerAnglesLandscape(HAL_EulerAngles3d* angles, int32_t* status) {
  HAL_GetIMUEulerAnglesFlat(angles, status);
}

void HAL_GetIMUEulerAnglesPortrait(HAL_EulerAngles3d* angles, int32_t* status) {
  HAL_GetIMUEulerAnglesFlat(angles, status);
}

void HAL_GetIMUQuaternion(HAL_Quaternion* quat, int32_t* status) {
  *quat = HAL_Quaternion{};
  quat->timestamp = HAL_GetMonotonicTime();
  quat->w = 1.0;  // identity rotation
  NoImu(status);
}

double HAL_GetIMUYawFlat(int64_t* timestamp) {
  *timestamp = HAL_GetMonotonicTime();
  return 0.0;
}

double HAL_GetIMUYawLandscape(int64_t* timestamp) {
  return HAL_GetIMUYawFlat(timestamp);
}

double HAL_GetIMUYawPortrait(int64_t* timestamp) {
  return HAL_GetIMUYawFlat(timestamp);
}

}  // extern "C"
