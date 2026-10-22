// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Stack sizes of the threads wpilibc starts (design/PORT.md section 7) and DataLogManager limits.
// Every thread site creates its std::thread inside a prosdk::platform::ScopedThreadConfig with
// these sizes, the default priority (below the probot-core loop) and core 1; on the host the stack
// sizes are unused.
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
/**
 * prosdk::DataLogManager thread (wpiutil SafeThread with a per-thread stack). The largest frame is
 * the log rename, std::format("WPILIB_{:%Y%m%d_%H%M%OS}...", system_clock::now()) with
 * libstdc++'s chrono formatter; the old-log cleanup (directory_iterator, sort) is next. Estimated on
 * the host (x86-64 g++ 13, -Os, painted stack, net of the thread's TLS): chrono rename ~3.2 KB,
 * cleanup ~2.5 KB, plain std::format ~0.1 KB, so the 4096-byte SafeThread default is too tight.
 * Not measured on the ESP32: check with uxTaskGetStackHighWaterMark() if this is lowered.
 */
#ifndef PROSDK_DATALOGMANAGER_THREAD_STACK_SIZE
#define PROSDK_DATALOGMANAGER_THREAD_STACK_SIZE 8192
#endif

// DataLogManager old-log cleanup (upstream: below 50 MB free, delete the oldest WPILIB_*.wpilog
// files until 50 MB are free or 10 files remain; warn below 100 MB). The robot's LittleFS
// partition is under 1 MB (huge_app: 896 KiB), so the limits are scaled: 128 KiB (at most half the
// partition; warning below twice that) and 3 files.
#ifndef PROSDK_DATALOGMANAGER_FREE_SPACE_THRESHOLD
#if PROSDK_TARGET_HW
#define PROSDK_DATALOGMANAGER_FREE_SPACE_THRESHOLD 131072
#else
#define PROSDK_DATALOGMANAGER_FREE_SPACE_THRESHOLD 50000000
#endif
#endif
#ifndef PROSDK_DATALOGMANAGER_FILE_COUNT_THRESHOLD
#if PROSDK_TARGET_HW
#define PROSDK_DATALOGMANAGER_FILE_COUNT_THRESHOLD 3
#else
#define PROSDK_DATALOGMANAGER_FILE_COUNT_THRESHOLD 10
#endif
#endif

/**
 * DataLogManager console log: bytes of console output buffered between two drains by the manager's
 * thread (every 20-250 ms); more is dropped and counted.
 */
#ifndef PROSDK_DATALOGMANAGER_CONSOLE_BUFFER
#define PROSDK_DATALOGMANAGER_CONSOLE_BUFFER 2048
#endif
