// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SysIdRoutine: a command-based robot with SysIdRoutine characterization commands for a
// differential drive and a shooter flywheel (upstream
// wpilibcExamples/src/main/cpp/examples/SysIdRoutine), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   Drive PWMSparkMax: GPIO 4/5 left (leader/follower), GPIO 12/13 right
//   Drive encoders A/B: GPIO 6/7 left, GPIO 15/16 right
//   Shooter PWMSparkMax GPIO 14, feeder PWMSparkMax GPIO 21, shooter encoder A/B GPIO 38/39
//   (three of the four PCNT units)
// Gamepad 0: arcade drive on the sticks, shooter on the left trigger. Right bumper + face
// down/right/left/up runs the drive's quasistatic forward/reverse and dynamic forward/reverse
// tests; left bumper + the same buttons runs them for the shooter.
// The SysId log goes to the DataLog: on the robot to /littlefs/logs. prosdk does not mount
// LittleFS; Robot's first member does (LittleFS.begin(true): formats the partition only when it
// cannot be mounted, e.g. blank on the first boot), and the constructor calls
// DataLogManager::Start() so the "logs" folder is shared from boot on, not only after the first
// test. Without the mount the log is discarded with a console line (design/API-DIFFERENCES.md,
// DataLogManager).
// While the robot is idle the .wpilog files can be downloaded from the Driver Station's Logs page,
// "Dosyalar" card ("logs"), also after a reboot or an e-stop restart.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
