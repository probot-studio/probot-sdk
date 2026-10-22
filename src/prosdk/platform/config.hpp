// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Build-target configuration shared by vendored patches and hand-written code.
//
// PROSDK_TARGET_HW is 1 when compiling for the robot controller (ESP32-S3, Arduino-ESP32 /
// ESP-IDF) and 0 for the host build (unit tests, simulation). It replaces every upstream
// __FIRST_SYSTEMCORE__ test (design/PORT.md section 7). The build passes it explicitly; the
// fallback below covers sketches that include prosdk headers without the build's flags.

#pragma once

#ifndef PROSDK_TARGET_HW
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
#define PROSDK_TARGET_HW 1
#else
#define PROSDK_TARGET_HW 0
#endif
#endif

// Math library configuration (design/PORT.md sections 7 and 7.1). Every Eigen module header and
// every Sleipnir header includes this file before anything else (tools/upstream/wpilib.py
// `config_include` transform), so sketch translation units and the precompiled archive always
// see the same values; no -D flag is needed or used.
//
// EIGEN_STACK_ALLOCATION_LIMIT: dynamic-size temporaries above 4 KiB go to the heap instead of
//   the (8-16 KiB) task stack; still covers fixed-size matrices up to 22x22 doubles.
// PROSDK_EIGEN_FIXED_SIZE_STACK_LIMIT: compile-time cap for fixed-size Eigen objects (tools/upstream/
//   patches/eigen/0001). 8 KiB: Sleipnir's slp::solve instantiates an 8x8 triangular-product buffer
//   of slp::Variable (4608 bytes on ESP32, 5120 on a 64-bit host), which the 4 KiB runtime limit
//   above would reject. Such an object lives on the calling task's stack.
// EIGEN_NO_IO: no iostream operators (wpimath formats Eigen types with std::formatter).
// EIGEN_MPL2_ONLY: refuse any Eigen code that is not MPL-2.0.
// SLEIPNIR_DISABLE_DIAGNOSTICS: no solver iteration printing, spy files or profiler output.
#ifndef EIGEN_STACK_ALLOCATION_LIMIT
#define EIGEN_STACK_ALLOCATION_LIMIT 4096
#endif
#ifndef PROSDK_EIGEN_FIXED_SIZE_STACK_LIMIT
#define PROSDK_EIGEN_FIXED_SIZE_STACK_LIMIT 8192
#endif
#ifndef EIGEN_NO_IO
#define EIGEN_NO_IO 1
#endif
#ifndef EIGEN_MPL2_ONLY
#define EIGEN_MPL2_ONLY 1
#endif
#ifndef SLEIPNIR_DISABLE_DIAGNOSTICS
#define SLEIPNIR_DISABLE_DIAGNOSTICS 1
#endif
