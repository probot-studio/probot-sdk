// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/imu/OnboardIMU.hpp"

#include "prosdk/hal/IMU.h"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

OnboardIMU::OnboardIMU(MountOrientation mountOrientation)
    : m_mountOrientation{mountOrientation} {
  // TODO: usage reporting
}

prosdk::units::radian_t OnboardIMU::GetYawNoOffset() {
  int64_t timestamp;
  double val;
  switch (m_mountOrientation) {
    case FLAT:
      val = HAL_GetIMUYawFlat(&timestamp);
      break;
    case LANDSCAPE:
      val = HAL_GetIMUYawLandscape(&timestamp);
      break;
    case PORTRAIT:
      val = HAL_GetIMUYawPortrait(&timestamp);
      break;
    default:
      val = 0;
  }
  return prosdk::units::radian_t{val};
}

prosdk::units::radian_t OnboardIMU::GetYaw() {
  return GetYawNoOffset() - m_yawOffset;
}

void OnboardIMU::ResetYaw() {
  m_yawOffset = GetYawNoOffset();
}

prosdk::math::Rotation2d OnboardIMU::GetRotation2d() {
  return prosdk::math::Rotation2d{GetYaw()};
}

prosdk::math::Rotation3d OnboardIMU::GetRotation3d() {
  return prosdk::math::Rotation3d{GetQuaternion()};
}

prosdk::math::Quaternion OnboardIMU::GetQuaternion() {
  HAL_Quaternion val;
  int32_t status = 0;
  HAL_GetIMUQuaternion(&val, &status);
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::math::Quaternion{val.w, val.x, val.y, val.z};
}

prosdk::units::radian_t OnboardIMU::GetAngleX() {
  HAL_EulerAngles3d val;
  int32_t status = 0;
  switch (m_mountOrientation) {
    case FLAT:
      HAL_GetIMUEulerAnglesFlat(&val, &status);
      break;
    case LANDSCAPE:
      HAL_GetIMUEulerAnglesLandscape(&val, &status);
      break;
    case PORTRAIT:
      HAL_GetIMUEulerAnglesPortrait(&val, &status);
      break;
  }
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::units::radian_t{val.x};
}

prosdk::units::radian_t OnboardIMU::GetAngleY() {
  HAL_EulerAngles3d val;
  int32_t status = 0;
  switch (m_mountOrientation) {
    case FLAT:
      HAL_GetIMUEulerAnglesFlat(&val, &status);
      break;
    case LANDSCAPE:
      HAL_GetIMUEulerAnglesLandscape(&val, &status);
      break;
    case PORTRAIT:
      HAL_GetIMUEulerAnglesPortrait(&val, &status);
      break;
  }
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::units::radian_t{val.y};
}

prosdk::units::radian_t OnboardIMU::GetAngleZ() {
  HAL_EulerAngles3d val;
  int32_t status = 0;
  switch (m_mountOrientation) {
    case FLAT:
      HAL_GetIMUEulerAnglesFlat(&val, &status);
      break;
    case LANDSCAPE:
      HAL_GetIMUEulerAnglesLandscape(&val, &status);
      break;
    case PORTRAIT:
      HAL_GetIMUEulerAnglesPortrait(&val, &status);
      break;
  }
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::units::radian_t{val.z};
}

prosdk::units::radians_per_second_t OnboardIMU::GetGyroRateX() {
  HAL_GyroRate3d val;
  int32_t status = 0;
  HAL_GetIMUGyroRates(&val, &status);
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::units::radians_per_second_t{val.x};
}

prosdk::units::radians_per_second_t OnboardIMU::GetGyroRateY() {
  HAL_GyroRate3d val;
  int32_t status = 0;
  HAL_GetIMUGyroRates(&val, &status);
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::units::radians_per_second_t{val.y};
}

prosdk::units::radians_per_second_t OnboardIMU::GetGyroRateZ() {
  HAL_GyroRate3d val;
  int32_t status = 0;
  HAL_GetIMUGyroRates(&val, &status);
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::units::radians_per_second_t{val.z};
}

prosdk::units::meters_per_second_squared_t OnboardIMU::GetAccelX() {
  HAL_Acceleration3d val;
  int32_t status = 0;
  HAL_GetIMUAcceleration(&val, &status);
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::units::meters_per_second_squared_t{val.x};
}

prosdk::units::meters_per_second_squared_t OnboardIMU::GetAccelY() {
  HAL_Acceleration3d val;
  int32_t status = 0;
  HAL_GetIMUAcceleration(&val, &status);
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::units::meters_per_second_squared_t{val.y};
}

prosdk::units::meters_per_second_squared_t OnboardIMU::GetAccelZ() {
  HAL_Acceleration3d val;
  int32_t status = 0;
  HAL_GetIMUAcceleration(&val, &status);
  WPILIB_CheckErrorStatus(status, "Onboard IMU");
  return prosdk::units::meters_per_second_squared_t{val.z};
}
