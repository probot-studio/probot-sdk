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

#include <cstdint>
#include <optional>
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

/** Size and free space of a file system, in bytes. */
struct StorageSpace {
  std::uintmax_t capacity = 0;
  std::uintmax_t available = 0;
};

/**
 * Size and free space of the file system that holds path (DataLogManager's old-log cleanup).
 *   ESP32: for a path under kLittleFsBasePath, the mounted LittleFS partition
 *          (esp_littlefs_partition_info; the first mounted SPIFFS/LittleFS data partition, i.e.
 *          the one LittleFS.begin() mounted, whatever its label). Other paths (an SD card, ...):
 *          nullopt; the ESP32 C library has no statvfs.
 *   Host:  prosdk::fs::space(path).
 * nullopt when the space is unknown.
 */
std::optional<StorageSpace> GetStorageSpace(std::string_view path);

}  // namespace prosdk::platform
