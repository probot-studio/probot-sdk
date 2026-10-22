// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetSelectCommand: choose the autonomous command with SelectCommand
// (upstream wpilibcExamples/src/main/cpp/snippets/SelectCommand), ported to prosdk. The chooser is
// on the Driver Station
// (/Tunables/Auto Chooser); the selected command prints to the console. No hardware.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
