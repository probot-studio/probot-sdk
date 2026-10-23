// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/Storage.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <system_error>

#include "prosdk/platform/StorageAutoMount.hpp"
#include "prosdk/util/fs.hpp"

#if PROSDK_TARGET_HW
#include <dirent.h>
#include <esp_err.h>
#include <esp_littlefs.h>
#include <esp_partition.h>
#include <esp_spiffs.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "prosdk/platform/ConsoleFormat.hpp"

// Arduino's LittleFS library object (libraries/LittleFS: `fs::LittleFSFS LittleFS`, a global with
// an unmangled name). It is linked only when a sketch file includes <LittleFS.h>; otherwise this
// weak reference is null. Only its address is used (no dependency on the library).
extern "C" char LittleFS __attribute__((weak));  // NOLINT(readability-identifier-naming)
// ESP-IDF's SPIFFS mount (component spiffs, in the link only when something calls it: Arduino's
// SPIFFS library, linked when a sketch file includes <SPIFFS.h>, or the sketch itself). Weak, so
// prosdk does not pull SPIFFS in; null when nothing else does.
extern "C" esp_err_t esp_vfs_spiffs_register(const esp_vfs_spiffs_conf_t* conf)
    __attribute__((weak));
#endif

namespace prosdk::platform {
namespace {
struct Overrides {
  std::mutex mutex;
  std::string operatingDir;
  std::string deployDir;
  std::string logDir;
};

Overrides& GetOverrides() {
  static Overrides overrides;
  return overrides;
}

std::string Get(std::string Overrides::*member) {
  auto& o = GetOverrides();
  std::scoped_lock lock{o.mutex};
  return o.*member;
}

void Set(std::string Overrides::*member, std::string_view value) {
  auto& o = GetOverrides();
  std::scoped_lock lock{o.mutex};
  (o.*member).assign(value.data(), value.size());
}

std::atomic<bool> gLogAutoMount{true};
std::atomic<detail::LittleFsAutoMount> gAutoMountResult{detail::LittleFsAutoMount::kNotTried};

#if PROSDK_TARGET_HW
// The partition Arduino's LittleFS.begin() mounts by default, through ESP-IDF's esp_littlefs (the
// same component Arduino's library calls), at the same base path.
class Esp32LittleFsDevice final : public detail::LittleFsDevice {
 public:
  bool IsMounted() override { return IsLittleFsMounted(); }

  bool ArduinoLibraryLinked() override { return &LittleFS != nullptr; }

  bool SpiffsLinked() override { return &esp_vfs_spiffs_register != nullptr; }

  bool HasPartition() override {
    m_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
                                      detail::kLittleFsPartitionLabel);
    return m_part != nullptr;
  }

  // The first sector holds a LittleFS superblock (or SPIFFS/FAT data), so a used partition is
  // usually told apart after one 4 KiB read; a blank one is read whole (896 KiB, once).
  bool IsBlank() override {
    constexpr std::size_t kChunk = 4096;
    std::unique_ptr<std::uint32_t[]> buf{new (std::nothrow) std::uint32_t[kChunk / 4]};
    if (!buf) {
      return false;
    }
    for (std::size_t offset = 0; offset < m_part->size; offset += kChunk) {
      std::size_t n = std::min<std::size_t>(kChunk, m_part->size - offset);
      if (esp_partition_read(m_part, offset, buf.get(), n) != ESP_OK) {
        return false;
      }
      const auto* bytes = reinterpret_cast<const std::uint8_t*>(buf.get());
      if (!std::all_of(bytes, bytes + n, [](std::uint8_t b) { return b == 0xFF; })) {
        return false;
      }
    }
    return true;
  }

  bool Format() override {
    // as Arduino's LittleFS.format(): core 0's idle task may starve during the flash writes
    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCore(0);
    bool unwatched = idle0 != nullptr && esp_task_wdt_status(idle0) == ESP_OK &&
                     esp_task_wdt_delete(idle0) == ESP_OK;
    m_err = esp_littlefs_format_partition(m_part);
    if (unwatched) {
      esp_task_wdt_add(idle0);
    }
    return m_err == ESP_OK;
  }

  detail::LittleFsMountStatus Mount() override {
    esp_vfs_littlefs_conf_t conf{};
    conf.base_path = kLittleFsBasePath;
    conf.partition_label = detail::kLittleFsPartitionLabel;
    conf.format_if_mount_failed = false;
    conf.grow_on_mount = true;  // as LittleFS.begin()
    m_err = esp_vfs_littlefs_register(&conf);
    if (m_err == ESP_OK) {
      return detail::LittleFsMountStatus::kOk;
    }
    return m_err == ESP_FAIL ? detail::LittleFsMountStatus::kNoFileSystem
                             : detail::LittleFsMountStatus::kError;
  }

  const char* ErrorName() const { return esp_err_to_name(m_err); }

 private:
  const esp_partition_t* m_part = nullptr;
  esp_err_t m_err = ESP_OK;
};

// One console line per outcome that matters to the user (none when the sketch mounted it or
// opted out); DataLogManager then says that it discards its data.
void ReportAutoMount(detail::LittleFsAutoMount result, const Esp32LittleFsDevice& device) {
  using enum detail::LittleFsAutoMount;
  const char* label = detail::kLittleFsPartitionLabel;
  switch (result) {
    case kMounted:
      ConsolePrint(ConsoleLevel::kInfo,
                   "[prosdk] LittleFS (\"{}\" partition) mounted at {} for the logs\n", label,
                   kLittleFsBasePath);
      break;
    case kFormatted:
      ConsolePrint(ConsoleLevel::kInfo,
                   "[prosdk] LittleFS: the \"{}\" partition was blank; formatted and mounted at "
                   "{} for the logs\n",
                   label, kLittleFsBasePath);
      break;
    case kNotBlank:
      ConsolePrint(ConsoleLevel::kWarning,
                   "[prosdk] LittleFS: the \"{}\" partition is not blank and holds no mountable "
                   "LittleFS (other data, or a damaged file system); prosdk does not format it and "
                   "logs nothing. To erase it for the logs, call LittleFS.begin(true) "
                   "(#include <LittleFS.h>) before DataLogManager::Start()\n",
                   label);
      break;
    case kArduinoLibrary:
      ConsolePrint(ConsoleLevel::kWarning,
                   "[prosdk] LittleFS: the sketch uses the Arduino LittleFS library, so prosdk "
                   "does not mount it; call LittleFS.begin(true) before DataLogManager::Start() "
                   "(it formats the partition when it holds no LittleFS)\n");
      break;
    case kSpiffs:
      ConsolePrint(ConsoleLevel::kWarning,
                   "[prosdk] LittleFS: the program uses SPIFFS, which may own the \"{}\" "
                   "partition, so prosdk does not mount LittleFS there and logs nothing; "
                   "SetLogDirectory() names another log storage (an SD card)\n",
                   label);
      break;
    case kNoPartition:
      ConsolePrint(ConsoleLevel::kWarning,
                   "[prosdk] LittleFS: no \"{}\" data partition (partition scheme); no log "
                   "storage\n",
                   label);
      break;
    case kFailed:
      ConsolePrint(ConsoleLevel::kError,
                   "[prosdk] LittleFS: mounting the \"{}\" partition at {} failed ({})\n", label,
                   kLittleFsBasePath, device.ErrorName());
      break;
    default:
      break;
  }
}

// Mounts the log storage once, at the first DataLogManager default-directory lookup (again after
// an opt-out is lifted).
void AutoMountForLogs() {
  static std::mutex mutex;
  std::scoped_lock lock{mutex};
  auto previous = gAutoMountResult.load();
  if (previous != detail::LittleFsAutoMount::kNotTried &&
      previous != detail::LittleFsAutoMount::kDisabled) {
    return;
  }
  Esp32LittleFsDevice device;
  auto result = detail::AutoMountLittleFs(device, gLogAutoMount.load());
  gAutoMountResult.store(result);
  ReportAutoMount(result, device);
}
#endif

#if !PROSDK_TARGET_HW
std::string CurrentPath() {
  std::error_code ec;
  auto path = prosdk::fs::current_path(ec);
  return ec ? std::string{"."} : path.string();
}
#endif
}  // namespace

bool IsLittleFsMounted() {
#if PROSDK_TARGET_HW
  DIR* dir = opendir(kLittleFsBasePath);
  if (dir == nullptr) {
    return false;
  }
  closedir(dir);
  return true;
#else
  return false;
#endif
}

std::string GetOperatingDirectory() {
  if (auto dir = Get(&Overrides::operatingDir); !dir.empty()) {
    return dir;
  }
#if PROSDK_TARGET_HW
  return kLittleFsBasePath;
#else
  return CurrentPath();
#endif
}

void SetOperatingDirectory(std::string_view dir) {
  Set(&Overrides::operatingDir, dir);
}

std::string GetDeployDirectory() {
  if (auto dir = Get(&Overrides::deployDir); !dir.empty()) {
    return dir;
  }
#if PROSDK_TARGET_HW
  return std::string{kLittleFsBasePath} + "/deploy";
#else
  return CurrentPath() + "/src/main/deploy";
#endif
}

void SetDeployDirectory(std::string_view dir) {
  Set(&Overrides::deployDir, dir);
}

std::string GetLogDirectory() {
  if (auto dir = Get(&Overrides::logDir); !dir.empty()) {
    return dir;
  }
#if PROSDK_TARGET_HW
  AutoMountForLogs();
  if (!IsLittleFsMounted()) {
    return {};
  }
  return std::string{kLittleFsBasePath} + "/logs";
#else
  return GetOperatingDirectory() + "/logs";
#endif
}

void SetLogDirectory(std::string_view dir) {
  Set(&Overrides::logDir, dir);
}

void SetLogAutoMount(bool enabled) {
  gLogAutoMount.store(enabled);
}

bool GetLogAutoMount() {
  return gLogAutoMount.load();
}

namespace detail {

LittleFsAutoMount AutoMountLittleFs(LittleFsDevice& device, bool enabled) {
  if (device.IsMounted()) {
    return LittleFsAutoMount::kAlreadyMounted;
  }
  if (!enabled) {
    return LittleFsAutoMount::kDisabled;
  }
  if (device.ArduinoLibraryLinked()) {
    return LittleFsAutoMount::kArduinoLibrary;
  }
  if (device.SpiffsLinked()) {
    return LittleFsAutoMount::kSpiffs;
  }
  if (!device.HasPartition()) {
    return LittleFsAutoMount::kNoPartition;
  }
  // only a partition that was never written is formatted; anything else is mounted as it is
  if (device.IsBlank()) {
    if (!device.Format() || device.Mount() != LittleFsMountStatus::kOk) {
      return LittleFsAutoMount::kFailed;
    }
    return LittleFsAutoMount::kFormatted;
  }
  switch (device.Mount()) {
    case LittleFsMountStatus::kOk:
      return LittleFsAutoMount::kMounted;
    case LittleFsMountStatus::kNoFileSystem:
      return LittleFsAutoMount::kNotBlank;
    default:
      return LittleFsAutoMount::kFailed;
  }
}

LittleFsAutoMount GetLittleFsAutoMountResult() {
  return gAutoMountResult.load();
}

}  // namespace detail

std::optional<StorageSpace> GetStorageSpace(std::string_view path) {
#if PROSDK_TARGET_HW
  std::string_view base{kLittleFsBasePath};
  if (path.substr(0, base.size()) != base ||
      (path.size() > base.size() && path[base.size()] != '/')) {
    return std::nullopt;
  }
  // LittleFS.begin() (or the auto-mount) mounts a data partition of subtype spiffs (Arduino's
  // default label "spiffs") or littlefs; esp_littlefs reports the mounted one whatever its label
  std::optional<StorageSpace> result;
  for (auto subtype : {ESP_PARTITION_SUBTYPE_DATA_SPIFFS, ESP_PARTITION_SUBTYPE_DATA_LITTLEFS}) {
    // esp_partition_next() releases the iterator when it returns nullptr
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, subtype, nullptr);
    while (it != nullptr && !result) {
      const esp_partition_t* part = esp_partition_get(it);
      size_t total = 0;
      size_t used = 0;
      if (esp_littlefs_partition_mounted(part) &&
          esp_littlefs_partition_info(part, &total, &used) == ESP_OK) {
        result = StorageSpace{total, total > used ? total - used : 0};
      } else {
        it = esp_partition_next(it);
      }
    }
    esp_partition_iterator_release(it);
    if (result) {
      break;
    }
  }
  return result;
#else
  std::error_code ec;
  auto info = prosdk::fs::space(std::string{path}, ec);
  if (ec) {
    return std::nullopt;
  }
  return StorageSpace{info.capacity, info.available};
#endif
}

}  // namespace prosdk::platform
