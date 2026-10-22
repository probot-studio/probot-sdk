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

#include <atomic>
#include <mutex>
#include <string>

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
  // dashboard store
  std::atomic<uint32_t> topics{0};
  std::atomic<uint32_t> sets{0};
  std::atomic<uint32_t> schemas{0};
};

namespace detail {

namespace pf = prosdk::platform;

inline State gState;
inline std::mutex gConsoleMutex;
inline std::string gConsole;

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
inline bool SetOpModeOptions(const pf::CoreOpModeOption*, size_t) { return true; }
inline uint64_t NoOpMode() { return 0; }
inline bool NoUtility() { return false; }

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

/** True when the table was installed at static initialization. */
inline bool Installed() { return detail::gInstalled; }

}  // namespace qemu::fakecore
