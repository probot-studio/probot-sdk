// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/OnboardIMUSim.hpp"

#include "prosdk/hal/simulation/IMUData.h"

namespace prosdk::sim {

void OnboardIMUSim::SetAngleX(prosdk::units::radian_t angle) {
  HALSIM_SetIMUAngleX(angle.to<double>());
}
void OnboardIMUSim::SetAngleY(prosdk::units::radian_t angle) {
  HALSIM_SetIMUAngleY(angle.to<double>());
}
void OnboardIMUSim::SetAngleZ(prosdk::units::radian_t angle) {
  HALSIM_SetIMUAngleZ(angle.to<double>());
}

void OnboardIMUSim::SetGyroRateX(prosdk::units::radians_per_second_t rate) {
  HALSIM_SetIMUGyroRateX(rate.to<double>());
}

void OnboardIMUSim::SetGyroRateY(prosdk::units::radians_per_second_t rate) {
  HALSIM_SetIMUGyroRateY(rate.to<double>());
}

void OnboardIMUSim::SetGyroRateZ(prosdk::units::radians_per_second_t rate) {
  HALSIM_SetIMUGyroRateZ(rate.to<double>());
}

void OnboardIMUSim::SetAccelX(prosdk::units::meters_per_second_squared_t accel) {
  HALSIM_SetIMUAccelX(accel.to<double>());
}

void OnboardIMUSim::SetAccelY(prosdk::units::meters_per_second_squared_t accel) {
  HALSIM_SetIMUAccelY(accel.to<double>());
}

void OnboardIMUSim::SetAccelZ(prosdk::units::meters_per_second_squared_t accel) {
  HALSIM_SetIMUAccelZ(accel.to<double>());
}

void OnboardIMUSim::SetYaw(prosdk::units::radian_t angle) {
  HALSIM_SetIMUYaw(angle.to<double>());
}

}  // namespace prosdk::sim
