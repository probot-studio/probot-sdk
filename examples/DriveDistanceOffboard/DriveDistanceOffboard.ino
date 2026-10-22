// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DriveDistanceOffboard: a command-based differential drive that drives set distances with a
// trapezoid profile whose setpoints go to a "smart" motor controller's onboard PID
// (upstream wpilibcExamples/src/main/cpp/examples/DriveDistanceOffboard), ported to prosdk.
//
// ExampleSmartMotorController is upstream's API stub with no functionality (the kit has no
// smart motor controllers), so this example drives nothing; it shows the command structure.
// Its port numbers are GPIO 4/5 (left) and 12/13 (right) to match the other drive examples.
// Gamepad 0: left Y / right X arcade drive; right bumper held = half output; face down drives
// 3 m forward (encoders reset), face right drives 3 m from the current position.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
