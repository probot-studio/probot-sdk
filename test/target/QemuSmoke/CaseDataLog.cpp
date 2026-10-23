// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (f) DataLogManager on LittleFS (the huge_app "spiffs" data partition). Nothing mounts it before
// this case: the first default-directory lookup (prosdk::platform::GetLogDirectory) auto-mounts
// it. Boot 1 starts from a freshly merged flash image whose partition is blank (0xFF), so it is
// formatted; every later boot finds the file system and mounts it without formatting (the
// non-blank refusal is covered by test/prosdk/StorageAutoMountTest.cpp). Start() picks
// /littlefs/logs, the writer thread writes a .wpilog, Stop() flushes it; the file is then read
// back with stdio. Boot 2 checks that boot 1's log survived the restart. With the fake core's file
// sharing (FakeCore.hpp, CoreOps ABI 8) the manager's own task offers /littlefs/logs as "logs" and
// Stop() takes it back on the caller's task.

#include <dirent.h>
#include <sys/stat.h>

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <prosdk/datalog/DataLog.hpp>
#include <prosdk/platform/Storage.hpp>
#include <prosdk/platform/StorageAutoMount.hpp>
#include <prosdk/system/DataLogManager.hpp>

#include "FakeCore.hpp"
#include "Harness.hpp"

namespace qemu {

namespace {

std::string ReadFile(const std::string& path) {
  std::string data;
  if (FILE* f = std::fopen(path.c_str(), "rb")) {
    char buf[512];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
      data.append(buf, n);
    }
    std::fclose(f);
  }
  return data;
}

std::vector<std::string> ListLogs(const std::string& dir) {
  std::vector<std::string> names;
  if (DIR* d = opendir(dir.c_str())) {
    while (dirent* e = readdir(d)) {
      std::string name = e->d_name;
      if (name.size() > 7 && name.ends_with(".wpilog")) {
        names.push_back(name);
      }
    }
    closedir(d);
  }
  return names;
}

}  // namespace

void CaseDataLog(Ctx& ctx) {
  using prosdk::platform::detail::LittleFsAutoMount;
  const int boot = BootNumber();
  QCHECK_MSG(ctx, !prosdk::platform::IsLittleFsMounted(), "{} mounted before the auto-mount",
             prosdk::platform::kLittleFsBasePath);
  QCHECK(ctx, prosdk::platform::detail::GetLittleFsAutoMountResult() ==
                  LittleFsAutoMount::kNotTried);
  const int64_t t0 = esp_timer_get_time();
  const std::string dir = prosdk::platform::GetLogDirectory();
  const int64_t mountMs = (esp_timer_get_time() - t0) / 1000;
  QCHECK_MSG(ctx, dir == "/littlefs/logs", "log dir '{}'", dir);
  QCHECK_MSG(ctx, prosdk::platform::IsLittleFsMounted(), "LittleFS not mounted at {}",
             prosdk::platform::kLittleFsBasePath);
  // boot 1: blank partition, formatted; later boots: mounted as it is (never formatted again)
  const auto mount = prosdk::platform::detail::GetLittleFsAutoMountResult();
  const auto expected = boot == 1 ? LittleFsAutoMount::kFormatted : LittleFsAutoMount::kMounted;
  QCHECK_MSG(ctx, mount == expected, "auto-mount result {} (expected {})",
             static_cast<int>(mount), static_cast<int>(expected));
  ctx.Note("automount={} automount_ms={}", mount == LittleFsAutoMount::kFormatted ? "formatted"
                                           : mount == LittleFsAutoMount::kMounted ? "mounted"
                                                                                  : "other",
           mountMs);
  if (auto space = prosdk::platform::GetStorageSpace(dir)) {
    ctx.Note("fs_capacity={} fs_free={}", space->capacity, space->available);
  } else {
    QCHECK_MSG(ctx, false, "GetStorageSpace({}) unknown", dir);
  }

  // the previous boot's log must still be there
  if (boot > 1) {
    std::string prev = std::format("{}/qemu_smoke_{}.wpilog", dir, boot - 1);
    std::string data = ReadFile(prev);
    QCHECK_MSG(ctx, data.starts_with("WPILOG"), "{}: {} bytes", prev, data.size());
    QCHECK(ctx, data.find(std::format("qemu-smoke boot {}", boot - 1)) != std::string::npos);
    ctx.Note("prev_log_bytes={}", data.size());
  }

  const std::string name = std::format("qemu_smoke_{}.wpilog", boot);
  fakecore::TakeFileCalls();
  prosdk::DataLogManager::Start("", name);
  prosdk::DataLogManager::Log(std::format("qemu-smoke boot {}", boot));
  auto& log = prosdk::DataLogManager::GetLog();
  prosdk::log::IntegerLogEntry counter{log, "/qemu/counter"};
  prosdk::log::DoubleLogEntry value{log, "/qemu/value"};
  for (int i = 0; i < 200; ++i) {
    counter.Append(i);
    value.Append(i * 0.5);
  }
  vTaskDelay(pdMS_TO_TICKS(400));  // > the writer's 0.25 s flush period
  prosdk::DataLogManager::Stop();

  // R11: shared once by the datalogmgr task (the fake accepts a folder stat() finds), and unshared
  // on this task, which only happens for a share the manager made
  auto calls = fakecore::TakeFileCalls();
  QCHECK_MSG(ctx, calls.size() == 2, "{} share/unshare calls", calls.size());
  if (calls.size() == 2) {
    QCHECK(ctx, calls[0].share && calls[0].name == "logs" && calls[0].dir == dir);
    QCHECK_MSG(ctx, calls[0].task == "datalogmgr", "share on task '{}'", calls[0].task);
    QCHECK(ctx, !calls[1].share && calls[1].name == "logs");
    QCHECK_MSG(ctx, calls[1].task != "datalogmgr", "unshare on task '{}'", calls[1].task);
    ctx.Note("share_task={} unshare_task={}", calls[0].task, calls[1].task);
  }

  const std::string path = dir + "/" + name;
  std::string data = ReadFile(path);
  QCHECK_MSG(ctx, data.starts_with("WPILOG"), "{}: {} bytes", path, data.size());
  QCHECK_MSG(ctx, data.size() > 200 * 2 * 8, "{} bytes", data.size());
  QCHECK(ctx, data.find("/qemu/counter") != std::string::npos);
  QCHECK(ctx, data.find(std::format("qemu-smoke boot {}", boot)) != std::string::npos);
  auto logs = ListLogs(dir);
  QCHECK_MSG(ctx, static_cast<int>(logs.size()) >= std::min(boot, 2), "{} logs", logs.size());
  ctx.Note("log_bytes={} logs_in_dir={}", data.size(), logs.size());
}

}  // namespace qemu
