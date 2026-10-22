// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DriverStationDisplayAnsi: an OpModeRobot whose teleop opmode writes a raw ANSI screen and
// rewrites a colored seconds counter in place (upstream
// wpilibcExamples/src/main/cpp/examples/DriverStationDisplayAnsi), ported to prosdk.
// No wiring. Select "Default Teleop" on the Driver Station and enable.
// On the Probot Driver Station, DriverStationDisplay frames go to the Driver Station console
// with ANSI sequences removed (there is no display panel); a frame equal to the previous one is
// not repeated (design/API-DIFFERENCES.md). So the cursor moves and colors of this example do not
// show on the robot; each rewrite appears as a new console line with the counter text.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
