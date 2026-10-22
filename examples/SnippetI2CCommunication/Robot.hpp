// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/bus/I2C.hpp>

/**
 * This is a sample program demonstrating how to communicate to a light
 * controller from the robot code using the ESP32-S3's I2C port.
 */
class Robot : public prosdk::TimedRobot {
 public:
  void RobotPeriodic() override;

  static constexpr prosdk::I2C::Port PORT = prosdk::I2C::Port::PORT_0;

 private:
  static constexpr int deviceAddress = 4;
  prosdk::I2C arduino{PORT, deviceAddress};
};
