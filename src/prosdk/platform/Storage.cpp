// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/Storage.hpp"

#include <mutex>
#include <string>
#include <system_error>

#include "prosdk/util/fs.hpp"

#if PROSDK_TARGET_HW
#include <dirent.h>
#include <esp_littlefs.h>
#include <esp_partition.h>
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

std::optional<StorageSpace> GetStorageSpace(std::string_view path) {
#if PROSDK_TARGET_HW
  std::string_view base{kLittleFsBasePath};
  if (path.substr(0, base.size()) != base ||
      (path.size() > base.size() && path[base.size()] != '/')) {
    return std::nullopt;
  }
  // LittleFS.begin() mounts a data partition of subtype spiffs (Arduino's default label
  // "spiffs") or littlefs; esp_littlefs reports the mounted one whatever its label
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
