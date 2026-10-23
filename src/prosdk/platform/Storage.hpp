// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// File system locations used by wpilibc (prosdk::filesystem, DataLogManager), replacing the
// SystemCore paths (/home/systemcore, /home/systemcore/deploy, /home/systemcore/logs).
//
// ESP32: the robot's file system is LittleFS at /littlefs on the data partition "spiffs" (the
//   huge_app scheme's 896 KiB; the partition and mount point of Arduino's LittleFS.begin()).
//   The sketch may mount it itself with LittleFS.begin(). Otherwise GetLogDirectory() (the default
//   directory of DataLogManager) mounts it once through ESP-IDF's esp_littlefs, unless
//   SetLogDirectory() named another directory or SetLogAutoMount(false) was called:
//     - a LittleFS file system on the partition is mounted as it is;
//     - a blank partition (every byte 0xFF: never written since it was erased, the first boot) is
//       formatted first;
//     - a partition that holds anything else (SPIFFS, data written by another program, a
//       damaged file system) is never formatted: nothing is mounted, one console line says how to
//       erase it (LittleFS.begin(true)), and DataLogManager discards its data;
//     - a sketch that includes <LittleFS.h> (Arduino's LittleFS library linked) and has not
//       mounted it is left to mount it itself (one console line): after a mount by prosdk,
//       Arduino-ESP32 3.3.10's LittleFS.begin() returns true without giving the LittleFS object
//       its mount point, so LittleFS.open() would fail.
//     operating directory  /littlefs
//     deploy directory     /littlefs/deploy  (the sketch's data/deploy folder after a LittleFS upload)
//     log directory        /littlefs/logs if LittleFS is mounted (by the sketch or the auto-mount),
//                          otherwise "" (DataLogManager then keeps a log that discards its data)
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

/**
 * Whether GetLogDirectory() may mount LittleFS on the robot when nothing is mounted at
 * kLittleFsBasePath and no log directory was set (default true; see the top of this file). Call
 * SetLogAutoMount(false) before the first DataLogManager use to keep the partition untouched;
 * DataLogManager then logs only to a directory that the sketch mounted or set. The host ignores
 * it (no partition).
 */
void SetLogAutoMount(bool enabled);
bool GetLogAutoMount();

/** Size and free space of a file system, in bytes. */
struct StorageSpace {
  std::uintmax_t capacity = 0;
  std::uintmax_t available = 0;
};

/**
 * Size and free space of the file system that holds path (DataLogManager's old-log cleanup).
 *   ESP32: for a path under kLittleFsBasePath, the mounted LittleFS partition
 *          (esp_littlefs_partition_info; the first mounted SPIFFS/LittleFS data partition, i.e.
 *          the one LittleFS.begin() or the auto-mount mounted, whatever its label). Other
 *          paths (an SD card, ...): nullopt; the ESP32 C library has no statvfs.
 *   Host:  prosdk::fs::space(path).
 * nullopt when the space is unknown.
 */
std::optional<StorageSpace> GetStorageSpace(std::string_view path);

}  // namespace prosdk::platform
