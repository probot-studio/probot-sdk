// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_velocity.hpp"

namespace prosdk::sim {

class OnboardIMUSim {
 public:
  void SetAngleX(prosdk::units::radian_t angle);
  void SetAngleY(prosdk::units::radian_t angle);
  void SetAngleZ(prosdk::units::radian_t angle);

  void SetGyroRateX(prosdk::units::radians_per_second_t rate);
  void SetGyroRateY(prosdk::units::radians_per_second_t rate);
  void SetGyroRateZ(prosdk::units::radians_per_second_t rate);

  void SetAccelX(prosdk::units::meters_per_second_squared_t accel);
  void SetAccelY(prosdk::units::meters_per_second_squared_t accel);
  void SetAccelZ(prosdk::units::meters_per_second_squared_t accel);

  void SetYaw(prosdk::units::radian_t angle);
};

}  // namespace prosdk::sim
