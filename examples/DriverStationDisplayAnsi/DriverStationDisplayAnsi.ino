// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DriverStationDisplayAnsi: an OpModeRobot whose teleop opmode writes a raw ANSI screen and
// rewrites a colored seconds counter in place (upstream
// wpilibcExamples/src/main/cpp/examples/DriverStationDisplayAnsi), ported to prosdk.
// No wiring. Select "Default Teleop" on the Driver Station and enable.
// The Probot Driver Station shows the DriverStationDisplay screen (/Telemetry/DriverStationDisplay)
// as a text panel: the Dashboard's "Robot Ekranı" card and the Veri page (probot-core R19; an older
// core shows it as plain text in the topic tree). Core's default PROBOT_DASH_MAX_VALUE (1024 bytes)
// holds this screen; a full 48 x 160 screen needs #define PROBOT_DASH_MAX_VALUE 8192 before
// #include <prosdk.h> (design/API-DIFFERENCES.md).
// The counter is rewritten in place; its colors are not shown.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
