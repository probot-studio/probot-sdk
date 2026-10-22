// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/motor/MotorController.hpp"

#include "prosdk/system/RobotController.hpp"

using namespace prosdk;

void MotorController::SetVoltage(prosdk::units::volt_t voltage) {
  // NOLINTNEXTLINE(bugprone-integer-division)
  SetThrottle(voltage / RobotController::GetBatteryVoltage());
}
