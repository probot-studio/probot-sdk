// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/discrete/DigitalOutput.hpp>

/**
 * This is a sample program demonstrating how to communicate to a light
 * controller from the robot code using digital outputs (GPIO pins).
 */
class Robot : public prosdk::TimedRobot {
 public:
  // define ports for communication with light controller
  static constexpr int ALLIANCE_PORT = 17;
  static constexpr int ENABLED_PORT = 18;
  static constexpr int AUTONOMOUS_PORT = 42;
  static constexpr int ALERT_PORT = 11;

  void RobotPeriodic() override;

 private:
  prosdk::DigitalOutput allianceOutput{ALLIANCE_PORT};
  prosdk::DigitalOutput enabledOutput{ENABLED_PORT};
  prosdk::DigitalOutput autonomousOutput{AUTONOMOUS_PORT};
  prosdk::DigitalOutput alertOutput{ALERT_PORT};
};
