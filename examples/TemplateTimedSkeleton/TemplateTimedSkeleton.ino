// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TemplateTimedSkeleton: the "Timed Skeleton (Advanced)" project template (upstream
// wpilibcExamples/src/main/cpp/templates/timedskeleton), ported to prosdk.
// An empty TimedRobot with every mode method. No hardware.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
