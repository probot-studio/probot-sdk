// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// File system locations used by wpilibc (prosdk::filesystem, DataLogManager), replacing the
// SystemCore paths (/home/systemcore, /home/systemcore/deploy, /home/systemcore/logs).
//
// ESP32: the robot's file system is LittleFS, mounted by the sketch with the Arduino LittleFS
//   library (LittleFS.begin(), mount point /littlefs). prosdk never mounts or formats it.
//     operating directory  /littlefs
//     deploy directory     /littlefs/deploy  (the sketch's data/deploy folder after a LittleFS upload)
//     log directory        /littlefs/logs if LittleFS is mounted, otherwise "" (DataLogManager
//                          then keeps a log that discards its data)
// Host: the upstream simulation paths.
//     operating directory  the current working directory
//     deploy directory     <current working directory>/src/main/deploy
//     log directory        <operating directory>/logs
//
// Every location can be overridden at run time (before the code that uses it runs); an empty
// string restores the default.

#pragma once

#include <string>
#include <string_view>

#include "prosdk/platform/config.hpp"

namespace prosdk::platform {

/** Mount point of the Arduino LittleFS library (LittleFS.begin() default base path). */
inline constexpr const char* kLittleFsBasePath = "/littlefs";

/** ESP32: true when a file system is mounted at kLittleFsBasePath. Host: always false. */
bool IsLittleFsMounted();

/** Directory returned by prosdk::filesystem::GetOperatingDirectory(). */
std::string GetOperatingDirectory();
void SetOperatingDirectory(std::string_view dir);

/** Directory returned by prosdk::filesystem::GetDeployDirectory(). */
std::string GetDeployDirectory();
void SetDeployDirectory(std::string_view dir);

/**
 * Default directory of prosdk::DataLogManager (used when DataLogManager::Start() gets no
 * directory). An empty result means "no storage": DataLogManager logs to a discarding writer.
 */
std::string GetLogDirectory();
void SetLogDirectory(std::string_view dir);

}  // namespace prosdk::platform
