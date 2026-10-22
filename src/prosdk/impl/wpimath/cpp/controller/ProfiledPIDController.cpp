// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/ProfiledPIDController.hpp"

int prosdk::math::detail::IncrementAndGetProfiledPIDControllerInstances() {
  static int instances = 0;
  return ++instances;
}
