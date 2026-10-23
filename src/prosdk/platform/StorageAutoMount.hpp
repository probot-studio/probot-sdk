// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Internal: the LittleFS auto-mount behind prosdk::platform::GetLogDirectory() (Storage.hpp). The
// decision is separate from the flash access so that the host tests can drive it with a fake
// device (test/prosdk/StorageAutoMountTest.cpp); the ESP32 device (Storage.cpp) uses ESP-IDF's
// esp_littlefs on the data partition that Arduino's LittleFS.begin() uses by default.

#pragma once

#include <cstdint>

namespace prosdk::platform::detail {

/** Partition label that Arduino's LittleFS.begin() mounts by default (huge_app: 896 KiB). */
inline constexpr const char* kLittleFsPartitionLabel = "spiffs";

/** Outcome of an auto-mount attempt. */
enum class LittleFsAutoMount : std::uint8_t {
  kNotTried,        // no attempt yet (no DataLogManager default directory asked for, or opted out)
  kAlreadyMounted,  // something is mounted at /littlefs (the sketch's LittleFS.begin(), ...)
  kDisabled,        // SetLogAutoMount(false)
  kArduinoLibrary,  // the sketch links Arduino's LittleFS library and has not mounted it
  kNoPartition,     // no data partition labelled kLittleFsPartitionLabel
  kMounted,         // a LittleFS file system on the partition, mounted without formatting
  kFormatted,       // the partition was blank (all 0xFF): formatted, then mounted
  kNotBlank,        // the partition holds data that is not LittleFS: left alone, not mounted
  kFailed,          // mount or format error (out of memory, partition in use elsewhere, ...)
};

/** Result of a mount attempt without formatting. */
enum class LittleFsMountStatus : std::uint8_t {
  kOk,
  kNoFileSystem,  // the partition does not hold a LittleFS file system (ESP_FAIL)
  kError,         // any other error
};

/** The flash access the decision needs. */
class LittleFsDevice {
 public:
  virtual ~LittleFsDevice() = default;
  /** Whether a file system is mounted at /littlefs. */
  virtual bool IsMounted() = 0;
  /** Whether the program links Arduino's LittleFS library (the sketch includes <LittleFS.h>). */
  virtual bool ArduinoLibraryLinked() = 0;
  /** Whether the data partition exists. */
  virtual bool HasPartition() = 0;
  /** Whether every byte of the partition is 0xFF (never written since the last erase). */
  virtual bool IsBlank() = 0;
  /** Formats the partition as LittleFS. */
  virtual bool Format() = 0;
  /** Mounts the partition at /littlefs without formatting it. */
  virtual LittleFsMountStatus Mount() = 0;
};

/**
 * The auto-mount decision. Never formats a partition that holds data: it formats only a blank
 * one (the first boot after the partition was erased) and mounts an existing LittleFS as it is.
 *   mounted already          -> kAlreadyMounted (whatever enabled says)
 *   !enabled                 -> kDisabled
 *   Arduino library linked   -> kArduinoLibrary (the sketch mounts it with LittleFS.begin(): a
 *                               LittleFS.begin() after a mount by prosdk returns true but leaves
 *                               the Arduino LittleFS object without its mount point)
 *   no partition             -> kNoPartition
 *   blank                    -> Format() and Mount(): kFormatted, or kFailed
 *   Mount() kOk              -> kMounted
 *   Mount() kNoFileSystem    -> kNotBlank (no format)
 *   Mount() kError           -> kFailed
 */
LittleFsAutoMount AutoMountLittleFs(LittleFsDevice& device, bool enabled);

/**
 * The result of the platform's own attempt (GetLogDirectory() on the ESP32; kNotTried on the
 * host, which has no partition).
 */
LittleFsAutoMount GetLittleFsAutoMountResult();

}  // namespace prosdk::platform::detail
