// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TemplateCommandV2: the "Command v2 Robot" project template (upstream
// wpilibcExamples/src/main/cpp/templates/commandv2), ported to prosdk.
// A TimedRobot that runs the CommandScheduler, with RobotContainer, an example subsystem, an
// example command and an autonomous factory. Gamepad 0 (CommandGamepad): the example trigger
// and the face-right button bindings of upstream. No other hardware.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
