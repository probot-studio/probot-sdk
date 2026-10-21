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
