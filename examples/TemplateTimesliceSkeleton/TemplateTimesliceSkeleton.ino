// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TemplateTimesliceSkeleton: the "Timeslice Skeleton (Advanced)" project template (upstream
// wpilibcExamples/src/main/cpp/templates/timesliceskeleton), ported to prosdk.
// An empty TimesliceRobot (5 ms robot slot, two 2 ms controller slots, 10 ms period). No hardware.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
