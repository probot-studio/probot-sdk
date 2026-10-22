// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DSGamepadChooser: two autonomous opmodes whose parameters are picked with gamepad 0 while
// disabled, through DSGamepadChooser and the DriverStationDisplay (upstream
// wpilibcExamples/src/main/cpp/examples/DSGamepadChooser),
// ported to prosdk.
// No wiring. Gamepad 0 drives the chooser; select "Score Auto" or "Balance Auto" on the Driver
// Station, pick options while disabled, then enable.
// On the Probot Driver Station, DriverStationDisplay frames go to the Driver Station console
// with ANSI sequences removed (there is no display panel); a frame equal to the previous one is
// not repeated (design/API-DIFFERENCES.md).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
