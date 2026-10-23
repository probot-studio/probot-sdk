// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// A fake probot-core for the QEMU sketches (QemuSmoke, QemuBench): the CoreOps table
// (prosdk/platform/CoreOps.hpp) that the sketch-side bridge (prosdk/probot/CoreBridge.hpp) would
// fill from <probot.h>, installed at static initialization exactly as the bridge does it.
// probot-core itself needs WiFi, which QEMU does not emulate, so the sketches drive core's
// framework hooks (detail::Boot / PreCycle / PostCycle / EmergencyStop) themselves.
//
// Header-only: Arduino compiles only the files inside a sketch folder, so both sketches reach
// this one copy through the -I<repo>/test/target/shared that tools/qemu/run.sh passes. The
// inline variables below exist once per program; the table is installed by whichever sketch
// translation unit includes this header (at least one must).

#pragma once

#include <stdint.h>
#include <sys/stat.h>

#include <atomic>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <prosdk/platform/CoreOps.hpp>

namespace qemu::fakecore {

struct State {
  std::atomic<uint8_t> phase{0};         // CorePhase
  std::atomic<uint8_t> selectedMode{1};  // CoreMode (teleop)
  std::atomic<bool> outputs{false};
  std::atomic<bool> estop{false};
  std::atomic<bool> attached{true};
  std::atomic<uint32_t> packetSeq{0};
  std::atomic<uint32_t> periodMs{20};
  std::atomic<uint32_t> periodRequests{0};
  std::atomic<uint32_t> lastPeriodRequest{0};
  std::atomic<uint32_t> overruns{0};
  // opmode list (setOpModeOptions): the last list's size and its UTILITY options named "Utility"
  std::atomic<uint32_t> opModeOptions{0};
  std::atomic<uint32_t> utilityOptions{0};
  // dashboard store
  std::atomic<uint32_t> topics{0};
  std::atomic<uint32_t> sets{0};
  std::atomic<uint32_t> schemas{0};
};

/** The fake file sharing: every share()/unshare() call and the FreeRTOS task that made it. */
struct FileCall {
  bool share;
  std::string name;
  std::string dir;
  std::string task;
};

namespace detail {

namespace pf = prosdk::platform;

inline State gState;
inline std::mutex gConsoleMutex;
inline std::string gConsole;
inline std::mutex gFilesMutex;
inline std::vector<FileCall> gFileCalls;

inline uint8_t Phase() { return gState.phase.load(); }
inline uint8_t SelectedMode() { return gState.selectedMode.load(); }
inline bool Outputs() { return gState.outputs.load() && !gState.estop.load(); }
inline bool EStopped() { return gState.estop.load(); }
inline bool Attached() { return gState.attached.load(); }
inline float MatchTime() { return -1.0f; }
inline uint32_t PacketSeq() { return gState.packetSeq.load(); }
inline float Battery() { return 12.5f; }

inline void ConsoleWrite(const char* text, size_t len) {
  std::scoped_lock lock{gConsoleMutex};
  if (gConsole.size() < 16384) {
    gConsole.append(text, len);
  }
}

inline uint16_t DashPublish(const char*, uint8_t, const char*, const char*, uint8_t) {
  uint32_t id = gState.topics.fetch_add(1);
  return id < 0xFFF0 ? static_cast<uint16_t>(id) : pf::kCoreDashboardInvalid;
}
inline void DashUnpublish(uint16_t) {}
inline bool DashSet(uint16_t, const void*, size_t) {
  gState.sets.fetch_add(1);
  return true;
}
inline bool DashSetProperties(uint16_t, const char*) { return true; }
inline bool DashSetFlags(uint16_t, uint8_t) { return true; }
inline bool DashHasSchema(const char*) { return false; }
inline bool DashAddSchema(const char*, const char*, const void*, size_t) {
  gState.schemas.fetch_add(1);
  return true;
}
inline size_t DashDrain(void (*)(const pf::CoreDashboardWrite&, void*), void*) { return 0; }
inline void DashReject(uint16_t, uint8_t) {}
inline void DashStats(pf::CoreDashboardStats* out) {
  if (out != nullptr) {
    *out = pf::CoreDashboardStats{static_cast<uint16_t>(gState.topics.load()), 512, 0, 65536, 0};
  }
}

inline bool SetLoopPeriod(uint32_t ms) {
  if (ms > pf::kCoreLoopPeriodMaxMs) {
    return false;
  }
  gState.periodRequests.fetch_add(1);
  gState.lastPeriodRequest.store(ms);
  gState.periodMs.store(ms == 0 ? pf::kCoreLoopPeriodDefaultMs : ms);
  return true;
}
inline uint32_t LoopPeriod() { return gState.periodMs.load(); }
inline uint32_t Overruns() { return gState.overruns.load(); }
inline bool SetOpModeOptions(const pf::CoreOpModeOption* options, size_t count) {
  uint32_t utility = 0;
  for (size_t i = 0; i < count; ++i) {
    if (options[i].mode == pf::kCoreModeUtility && std::string_view{options[i].name} == "Utility") {
      ++utility;
    }
  }
  gState.opModeOptions.store(static_cast<uint32_t>(count));
  gState.utilityOptions.store(utility);
  return true;
}
inline uint64_t NoOpMode() { return 0; }
inline bool NoUtility() { return false; }

inline void RecordFileCall(bool share, const char* name, const char* dir) {
  const char* task = pcTaskGetName(nullptr);
  std::scoped_lock lock{gFilesMutex};
  gFileCalls.push_back({share, name, dir != nullptr ? dir : "", task != nullptr ? task : ""});
}
// core's file task is not here (no WiFi); what it would read must be a folder of a mounted VFS
inline bool FilesShare(const char* name, const char* vfsDir) {
  RecordFileCall(true, name, vfsDir);
  struct stat st {};
  return vfsDir != nullptr && vfsDir[0] == '/' && ::stat(vfsDir, &st) == 0 && S_ISDIR(st.st_mode);
}
inline bool FilesUnshare(const char* name) {
  RecordFileCall(false, name, nullptr);
  return true;
}

inline bool Install() {
  static constexpr pf::CoreOps kOps{
      pf::kCoreOpsAbiVersion, sizeof(pf::CoreOps), 400, "0.4.0-qemu-fake",
      &Phase, &SelectedMode, &Outputs, &EStopped, &Attached, &MatchTime, &PacketSeq,
      nullptr,  // rawGamepad: no gamepad (archive default)
      &Battery,
      &ConsoleWrite,
      &DashPublish, &DashUnpublish, &DashSet, &DashSetProperties, &DashSetFlags, &DashHasSchema,
      &DashAddSchema, &DashDrain, &DashReject, &DashStats,
      &SetLoopPeriod, &LoopPeriod, &Overruns,
      &SetOpModeOptions, &NoOpMode, &NoOpMode, &NoUtility,
      // ABI 4 (R9 consoleWriteLevel, R10 systemTimeValid) null, as the bridge leaves them for a
      // core without R9/R10: the archive's fallbacks run (console text through consoleWrite
      // without its level, the 2024-01-01 clock heuristic)
      nullptr, nullptr,
      // ABI 5 (R12 requestStop, rslState, commsDisableCount, setLowVoltage; R13
      // dashMaxSchemaBytes) null, as the bridge leaves them for a core without R12/R13: a halt
      // stops nothing, HAL_GetRSLState/HAL_GetCommsDisableCount report HAL_HANDLE_ERROR, no
      // low-voltage threshold, refused schemas are reported against PROBOT_DASH_MAX_VALUE
      nullptr, nullptr, nullptr, nullptr,
      nullptr,
      // ABI 6 (R12 dashClearPersisted) null, as the bridge leaves it for a core without
      // dashboard persistence: no tunable is published PERSISTENT
      nullptr,
      // ABI 7 (R15 loopGridRestarts) null, as the bridge leaves it for a core without R15: the
      // robot host re-aligns on counted overruns only
      nullptr,
      // ABI 8 (R11 filesShare/filesUnshare) recorded, so DataLogManager's share of its LittleFS
      // folder runs on the target (the null fallback is covered by prosdk_files_legacy_tests)
      &FilesShare, &FilesUnshare,
      // ABI 9 (R18 matchInfoSeq/matchInfo) null, as the bridge leaves them for a core without
      // R18: unknown alliance station, empty game data and match info (the values and the
      // counter are covered by prosdk_match_tests)
      nullptr, nullptr,
  };
  bool ok = pf::InstallCoreOps(&kOps);
  // what CoreBridge.hpp reports after probot::framework::install succeeded
  pf::NoteFrameworkInstall(true);
  return ok;
}

// static initialization, like the bridge's inline variable
inline const bool gInstalled = Install();

}  // namespace detail

inline State& Get() { return detail::gState; }

/** Everything written through CoreOps::consoleWrite (the Driver Station console). */
inline std::string Console() {
  std::scoped_lock lock{detail::gConsoleMutex};
  return detail::gConsole;
}

/** The share()/unshare() calls so far, cleared. */
inline std::vector<FileCall> TakeFileCalls() {
  std::scoped_lock lock{detail::gFilesMutex};
  return std::exchange(detail::gFileCalls, {});
}

/** True when the table was installed at static initialization. */
inline bool Installed() { return detail::gInstalled; }

}  // namespace qemu::fakecore
