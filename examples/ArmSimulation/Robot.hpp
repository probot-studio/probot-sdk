// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Arm.hpp"
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/framework/TimedRobot.hpp>

/**
 * This is a sample program to demonstrate the use of arm simulation.
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {}
  void SimulationPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;

 private:
  prosdk::Joystick joystick{JOYSTICK_PORT};
  Arm arm;
};
