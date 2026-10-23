// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SysIdRoutine: a command-based robot with SysIdRoutine characterization commands for a
// differential drive and a shooter flywheel (upstream
// wpilibcExamples/src/main/cpp/examples/SysIdRoutine), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   Drive PWMSparkMax: GPIO 38/37 left (leader/follower), GPIO 36/35 right
//   Drive encoders A/B: GPIO 39/40 left, GPIO 41/42 right
//   Shooter PWMSparkMax GPIO 47, feeder PWMSparkMax GPIO 48, shooter encoder A/B GPIO 15/16
//   (three of the four PCNT units)
// Gamepad 0: arcade drive on the sticks, shooter on the left trigger. Right bumper + face
// down/right/left/up runs the drive's quasistatic forward/reverse and dynamic forward/reverse
// tests; left bumper + the same buttons runs them for the shooter.
// The SysId log goes to the DataLog: on the robot to /littlefs/logs. The Robot constructor calls
// DataLogManager::Start(), which mounts LittleFS (the "spiffs" partition; formatted only while it
// is blank, on the first boot; a partition holding other data is left alone and the log is
// discarded with a console line), so the "logs" folder is shared from boot on, not only after the
// first test (design/API-DIFFERENCES.md, DataLogManager). A sketch that uses Arduino's LittleFS
// library itself mounts it first instead: #include <LittleFS.h> and a first Robot member
// bool m_fs = LittleFS.begin(true);
// While the robot is idle the .wpilog files can be downloaded from the Driver Station's Logs page,
// "Dosyalar" card ("logs"), also after a reboot or an e-stop restart.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
