// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetDigitalCommunication: signal robot state to an external light controller (such
// as an Arduino) on digital outputs (upstream
// wpilibcExamples/src/main/cpp/snippets/DigitalCommunication), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   GPIO 17 alliance (red = high)   GPIO 18 enabled   GPIO 10 autonomous   GPIO 11 match-time alert
// The alliance comes from the probot Driver Station's "Maç Bilgisi" card (set while the robot is
// disabled; until then the alliance output stays low); match time comes from probot-core (-1
// when unknown).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
