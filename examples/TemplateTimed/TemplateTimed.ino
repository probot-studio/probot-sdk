// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TemplateTimed: the "Timed Robot" project template (upstream
// wpilibcExamples/src/main/cpp/templates/timed), ported to prosdk.
// A TimedRobot (20 ms loop) with every mode method and an autonomous chooser on the Driver
// Station (/Tunables/Auto Modes). No hardware. Copy this sketch to start a timed robot.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
