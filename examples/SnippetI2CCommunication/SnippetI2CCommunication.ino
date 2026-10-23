// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetI2CCommunication: send robot state to an external light controller (such as an
// Arduino at address 4) over I2C (upstream wpilibcExamples/src/main/cpp/snippets/I2CCommunication),
// ported to prosdk.
//
// Wiring: I2C port 0 (GPIO 1 SDA, GPIO 2 SCL: J3, the HAL default) with pull-ups. The alliance
// comes from the probot Driver Station's "Maç Bilgisi" card ("U" until it is set there, while the
// robot is disabled) and the match time is -1 when unknown.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
