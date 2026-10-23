// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DriverStationDisplayLines: an OpModeRobot whose teleop opmode shows a runtime and loop count
// with DriverStationDisplay line mode (upstream
// wpilibcExamples/src/main/cpp/examples/DriverStationDisplayLines), ported to prosdk.
// No wiring. Select "Default Teleop" on the Driver Station and enable.
// On the Probot Driver Station there is no display panel: the DriverStationDisplay screen shows
// as plain text in the topic tree, /Telemetry/DriverStationDisplay/text, updated in place
// (design/API-DIFFERENCES.md).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
