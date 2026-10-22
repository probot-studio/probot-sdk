// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetAddressableLED: a scrolling rainbow on an addressable LED strip
// (upstream wpilibcExamples/src/main/cpp/snippets/AddressableLED), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 21, data line of a WS2812/WS2812B strip of 60
// LEDs (5 V strips may need a level shifter). GPIO 3 is probot-core's status LED, not usable.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
