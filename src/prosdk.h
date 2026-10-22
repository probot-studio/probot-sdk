// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// prosdk umbrella header (library.properties includes=prosdk.h). Later waves add the robot
// framework, commands, math and hardware headers here.
//
// On the robot it also brings in probot-core: include <prosdk.h> INSTEAD of <probot.h> (or before
// it). prosdk runs as a probot-core framework, so this header defines PROBOT_FRAMEWORK (the OpMode
// hooks become optional) and a larger user-task stack before <probot.h> is seen. Core
// configuration macros (PROBOT_WIFI_AP_PASSWORD, NEOPIXEL_PIN, PROBOT_RSL_PIN, ...) go before
// this include, exactly as they would before <probot.h>.
//
// A robot program includes its framework header (prosdk/framework/TimedRobot.hpp,
// TimesliceRobot.hpp or OpModeRobot.hpp) and ends with PROSDK_ROBOT_MAIN(Robot) instead of
// main() (prosdk/platform/RobotMain.hpp).

#pragma once

#define PROSDK_VERSION "2027.0.0-alpha.1"

#include "prosdk/platform/config.hpp"

#if PROSDK_TARGET_HW
#if defined(PROBOT_VERSION) && !defined(PROBOT_FRAMEWORK)
#error "Include <prosdk.h> before <probot.h> (prosdk must define PROBOT_FRAMEWORK first)."
#endif
#ifndef PROBOT_FRAMEWORK
#define PROBOT_FRAMEWORK
#endif
// core's default (8 KiB) is too small for the WPILib-style robot loop; the user may set it lower
#ifndef PROBOT_USER_STACK
#define PROBOT_USER_STACK 24576
#endif
#include <probot.h>

#include "prosdk/probot/CoreBridge.hpp"
#endif

#include "prosdk/platform/dashboard/Dashboard.hpp"
#include "prosdk/platform/RobotMain.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/util/Alert.hpp"
