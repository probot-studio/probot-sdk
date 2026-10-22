// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Stack sizes of the threads wpilibc starts (design/PORT.md section 7). Every site creates its
// std::thread inside a prosdk::platform::ScopedThreadConfig with these sizes, the default
// priority (below the probot-core loop) and core 1; on the host the values are unused.
//
// They size threads, not class layouts. The notifier, watchdog and Driver Station mode threads
// are created in the precompiled archive (a -D override needs an archive rebuild,
// tools/build/build.sh); the robot thread is created by the StartRobot() template, i.e. in the
// sketch.

#pragma once

#include "prosdk/platform/config.hpp"

/** prosdk::Notifier callback thread (runs user code). */
#ifndef PROSDK_NOTIFIER_THREAD_STACK_SIZE
#define PROSDK_NOTIFIER_THREAD_STACK_SIZE 8192
#endif
/** prosdk::Watchdog timeout thread (formats and reports the overrun message). */
#ifndef PROSDK_WATCHDOG_THREAD_STACK_SIZE
#define PROSDK_WATCHDOG_THREAD_STACK_SIZE 6144
#endif
/** prosdk::internal::DriverStationModeThread (Driver Station refresh, HAL_ObserveUserProgram). */
#ifndef PROSDK_DS_MODE_THREAD_STACK_SIZE
#define PROSDK_DS_MODE_THREAD_STACK_SIZE 6144
#endif
/** Robot thread of prosdk::StartRobot() when the HAL owns the main thread (HAL_HasMain()). */
#ifndef PROSDK_ROBOT_THREAD_STACK_SIZE
#define PROSDK_ROBOT_THREAD_STACK_SIZE 16384
#endif
