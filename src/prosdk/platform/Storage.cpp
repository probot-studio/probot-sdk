// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/Storage.hpp"

#include <mutex>
#include <string>
#include <system_error>

#include "prosdk/util/fs.hpp"

#if PROSDK_TARGET_HW
#include <dirent.h>
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

}  // namespace prosdk::platform
