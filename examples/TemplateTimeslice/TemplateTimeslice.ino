// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TemplateTimeslice: the "Timeslice Robot" project template (upstream
// wpilibcExamples/src/main/cpp/templates/timeslice), ported to prosdk.
// A TimesliceRobot: robot periodic functions get 5 ms of every 10 ms period, two scheduled
// controller slots of 2 ms each follow (core runs its loop every 1 ms for the slots, see
// design/PORT.md 8.1). Autonomous chooser on the Driver Station (/Tunables/Auto Modes).
// No hardware.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
