// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "FakeCore.hpp"

#include <mutex>

#include <prosdk/platform/CoreOps.hpp>

namespace pf = prosdk::platform;

namespace qemu::fakecore {
namespace {

State gState;
std::mutex gConsoleMutex;
std::string gConsole;

uint8_t Phase() { return gState.phase.load(); }
uint8_t SelectedMode() { return gState.selectedMode.load(); }
bool Outputs() { return gState.outputs.load() && !gState.estop.load(); }
bool EStopped() { return gState.estop.load(); }
bool Attached() { return gState.attached.load(); }
float MatchTime() { return -1.0f; }
uint32_t PacketSeq() { return gState.packetSeq.load(); }
float Battery() { return 12.5f; }

void ConsoleWrite(const char* text, size_t len) {
  std::scoped_lock lock{gConsoleMutex};
  if (gConsole.size() < 16384) {
    gConsole.append(text, len);
  }
}

uint16_t DashPublish(const char*, uint8_t, const char*, const char*, uint8_t) {
  uint32_t id = gState.topics.fetch_add(1);
  return id < 0xFFF0 ? static_cast<uint16_t>(id) : pf::kCoreDashboardInvalid;
}
void DashUnpublish(uint16_t) {}
bool DashSet(uint16_t, const void*, size_t) {
  gState.sets.fetch_add(1);
  return true;
}
bool DashSetProperties(uint16_t, const char*) { return true; }
bool DashSetFlags(uint16_t, uint8_t) { return true; }
bool DashHasSchema(const char*) { return false; }
bool DashAddSchema(const char*, const char*, const void*, size_t) {
  gState.schemas.fetch_add(1);
  return true;
}
size_t DashDrain(void (*)(const pf::CoreDashboardWrite&, void*), void*) { return 0; }
void DashReject(uint16_t, uint8_t) {}
void DashStats(pf::CoreDashboardStats* out) {
  if (out != nullptr) {
    *out = pf::CoreDashboardStats{static_cast<uint16_t>(gState.topics.load()), 512, 0, 65536, 0};
  }
}

bool SetLoopPeriod(uint32_t ms) {
  if (ms > pf::kCoreLoopPeriodMaxMs) {
    return false;
  }
  gState.periodRequests.fetch_add(1);
  gState.lastPeriodRequest.store(ms);
  gState.periodMs.store(ms == 0 ? pf::kCoreLoopPeriodDefaultMs : ms);
  return true;
}
uint32_t LoopPeriod() { return gState.periodMs.load(); }
uint32_t Overruns() { return gState.overruns.load(); }
bool SetOpModeOptions(const pf::CoreOpModeOption*, size_t) { return true; }
uint64_t NoOpMode() { return 0; }
bool NoUtility() { return false; }

bool Install() {
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
  };
  bool ok = pf::InstallCoreOps(&kOps);
  // what CoreBridge.hpp reports after probot::framework::install succeeded
  pf::NoteFrameworkInstall(true);
  return ok;
}

// static initialization, like the bridge's inline variable
const bool gInstalled = Install();

}  // namespace

State& Get() { return gState; }

std::string Console() {
  std::scoped_lock lock{gConsoleMutex};
  return gConsole;
}

bool Installed() { return gInstalled; }

}  // namespace qemu::fakecore
