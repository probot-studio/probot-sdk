// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Archive side of the probot-core bridge (CoreOps.hpp). No core header is included here.

#include "prosdk/platform/CoreOps.hpp"

#include <atomic>
#include <cstdio>
#include <string_view>

#include "prosdk/platform/Console.hpp"

namespace prosdk::platform {

namespace {

// ---- safe defaults (no core) ----
uint8_t DefPhase() { return kCorePhaseStopped; }
uint8_t DefMode() { return kCoreModeTeleop; }
bool DefFalse() { return false; }
float DefMatchTime() { return -1.0f; }
uint32_t DefSeq() { return 0; }
bool DefGamepad(int, CoreGamepad*) { return false; }
float DefBattery() { return 0.0f; }
void DefConsole(const char*, size_t) {}
uint16_t DefPublish(const char*, uint8_t, const char*, const char*, uint8_t) {
  return kCoreDashboardInvalid;
}
void DefUnpublish(uint16_t) {}
bool DefSet(uint16_t, const void*, size_t) { return false; }
bool DefSetProps(uint16_t, const char*) { return false; }
bool DefSetFlags(uint16_t, uint8_t) { return false; }
bool DefHasSchema(const char*) { return false; }
bool DefAddSchema(const char*, const char*, const void*, size_t) { return false; }
size_t DefDrain(void (*)(const CoreDashboardWrite&, void*), void*) { return 0; }
void DefReject(uint16_t, uint8_t) {}
void DefStats(CoreDashboardStats* out) {
  if (out != nullptr) {
    *out = CoreDashboardStats{};
  }
}

constexpr CoreOps kDefaultOps{
    kCoreOpsAbiVersion, sizeof(CoreOps), 0, "",
    DefPhase, DefMode, DefFalse, DefFalse, DefFalse, DefMatchTime, DefSeq, DefGamepad,
    DefBattery, DefConsole,
    DefPublish, DefUnpublish, DefSet, DefSetProps, DefSetFlags, DefHasSchema, DefAddSchema,
    DefDrain, DefReject, DefStats,
};

// Written once (static initialization, single-threaded) before gInstalled is set.
constinit CoreOps gOps = kDefaultOps;
constinit std::atomic<bool> gInstalled{false};
constinit std::atomic<uint64_t> gReservedPins{0};
constinit std::atomic<bool> gEStopped{false};
constinit std::atomic<bool> gBooted{false};

// Console sink once core is present: Driver Station console plus the default stderr (UART0).
void CoreConsoleSink(std::string_view text) {
  gOps.consoleWrite(text.data(), text.size());
  std::fwrite(text.data(), 1, text.size(), stderr);
}

// ---- callback tables: append-only slots, lock-free reads ----
struct CallbackSlot {
  std::atomic<CoreCallback> fn{nullptr};
  std::atomic<void*> ctx{nullptr};
};

struct CallbackTable {
  CallbackSlot slots[kMaxCoreCallbacks];
  std::atomic<int> count{0};
  std::atomic_flag writeLock = ATOMIC_FLAG_INIT;

  bool Add(CoreCallback fn, void* ctx) noexcept {
    if (fn == nullptr) {
      return false;
    }
    while (writeLock.test_and_set(std::memory_order_acquire)) {
    }
    int n = count.load(std::memory_order_relaxed);
    // reuse a removed slot first
    for (int i = 0; i < n; ++i) {
      if (slots[i].fn.load(std::memory_order_relaxed) == nullptr) {
        slots[i].ctx.store(ctx, std::memory_order_relaxed);
        slots[i].fn.store(fn, std::memory_order_release);
        writeLock.clear(std::memory_order_release);
        return true;
      }
    }
    bool ok = n < kMaxCoreCallbacks;
    if (ok) {
      slots[n].ctx.store(ctx, std::memory_order_relaxed);
      slots[n].fn.store(fn, std::memory_order_relaxed);
      count.store(n + 1, std::memory_order_release);
    }
    writeLock.clear(std::memory_order_release);
    return ok;
  }

  void Remove(CoreCallback fn, void* ctx) noexcept {
    while (writeLock.test_and_set(std::memory_order_acquire)) {
    }
    int n = count.load(std::memory_order_relaxed);
    for (int i = 0; i < n; ++i) {
      if (slots[i].fn.load(std::memory_order_relaxed) == fn &&
          slots[i].ctx.load(std::memory_order_relaxed) == ctx) {
        slots[i].fn.store(nullptr, std::memory_order_release);
        break;
      }
    }
    writeLock.clear(std::memory_order_release);
  }

  void Run() noexcept {
    int n = count.load(std::memory_order_acquire);
    for (int i = 0; i < n; ++i) {
      CoreCallback fn = slots[i].fn.load(std::memory_order_acquire);
      if (fn != nullptr) {
        fn(slots[i].ctx.load(std::memory_order_relaxed));
      }
    }
  }
};

constinit CallbackTable gPre;
constinit CallbackTable gPost;
constinit CallbackTable gBoot;
// boot callbacks registered after boot run once at the next cycle
constinit CallbackTable gLateBoot;

constinit std::atomic<EmergencyStopCallback> gEStopCallbacks[kMaxCoreCallbacks]{};
constinit std::atomic<int> gEStopCount{0};

}  // namespace

bool InstallCoreOps(const CoreOps* ops) noexcept {
  if (ops == nullptr || ops->abiVersion != kCoreOpsAbiVersion || ops->size < sizeof(CoreOps)) {
    ConsoleWrite(
        "[prosdk] probot-core bridge ABI mismatch: rebuild the sketch against this prosdk "
        "release (CoreOps)\n");
    return false;
  }
  bool expected = false;
  if (!gInstalled.compare_exchange_strong(expected, true)) {
    return false;
  }
  CoreOps copy = *ops;
  // never keep a null entry
#define PROSDK_COREOPS_FILL(field) \
  if (copy.field == nullptr) copy.field = kDefaultOps.field
  PROSDK_COREOPS_FILL(phase);
  PROSDK_COREOPS_FILL(selectedMode);
  PROSDK_COREOPS_FILL(outputsEnabled);
  PROSDK_COREOPS_FILL(isEStopped);
  PROSDK_COREOPS_FILL(isAttached);
  PROSDK_COREOPS_FILL(matchTimeSeconds);
  PROSDK_COREOPS_FILL(packetSeq);
  PROSDK_COREOPS_FILL(rawGamepad);
  PROSDK_COREOPS_FILL(batteryVoltage);
  PROSDK_COREOPS_FILL(consoleWrite);
  PROSDK_COREOPS_FILL(dashPublish);
  PROSDK_COREOPS_FILL(dashUnpublish);
  PROSDK_COREOPS_FILL(dashSet);
  PROSDK_COREOPS_FILL(dashSetProperties);
  PROSDK_COREOPS_FILL(dashSetFlags);
  PROSDK_COREOPS_FILL(dashHasSchema);
  PROSDK_COREOPS_FILL(dashAddSchema);
  PROSDK_COREOPS_FILL(dashDrainWrites);
  PROSDK_COREOPS_FILL(dashRejectWrite);
  PROSDK_COREOPS_FILL(dashStats);
#undef PROSDK_COREOPS_FILL
  if (copy.coreVersionString == nullptr) {
    copy.coreVersionString = "";
  }
  copy.size = sizeof(CoreOps);
  gOps = copy;
  SetConsoleSink(CoreConsoleSink);
  return true;
}

bool HasCoreOps() noexcept {
  return gInstalled.load(std::memory_order_acquire);
}

const CoreOps& GetCoreOps() noexcept {
  return gOps;
}

void ReserveCorePins(const int* pins, size_t count) noexcept {
  if (pins == nullptr) {
    return;
  }
  uint64_t mask = 0;
  for (size_t i = 0; i < count; ++i) {
    if (pins[i] >= 0 && pins[i] < 64) {
      mask |= uint64_t{1} << pins[i];
    }
  }
  gReservedPins.fetch_or(mask, std::memory_order_acq_rel);
}

uint64_t GetCoreReservedPins() noexcept {
  return gReservedPins.load(std::memory_order_acquire);
}

bool AddCycleCallback(CyclePhase phase, CoreCallback callback, void* context) noexcept {
  return (phase == CyclePhase::kPre ? gPre : gPost).Add(callback, context);
}

void RemoveCycleCallback(CyclePhase phase, CoreCallback callback, void* context) noexcept {
  (phase == CyclePhase::kPre ? gPre : gPost).Remove(callback, context);
}

bool AddBootCallback(CoreCallback callback, void* context) noexcept {
  if (gBooted.load(std::memory_order_acquire)) {
    return gLateBoot.Add(callback, context);
  }
  return gBoot.Add(callback, context);
}

bool AddEmergencyStopCallback(EmergencyStopCallback callback) noexcept {
  if (callback == nullptr) {
    return false;
  }
  int n = gEStopCount.fetch_add(1, std::memory_order_acq_rel);
  if (n >= kMaxCoreCallbacks) {
    gEStopCount.fetch_sub(1, std::memory_order_acq_rel);
    return false;
  }
  gEStopCallbacks[n].store(callback, std::memory_order_release);
  return true;
}

bool EmergencyStopped() noexcept {
  return gEStopped.load(std::memory_order_acquire) || gOps.isEStopped();
}

namespace detail {

void Boot() {
  gBoot.Run();
  gBooted.store(true, std::memory_order_release);
}

void PreCycle() {
  // boot callbacks registered after boot: run once, then drop
  if (gLateBoot.count.load(std::memory_order_acquire) != 0) {
    for (int i = 0; i < kMaxCoreCallbacks; ++i) {
      CoreCallback fn = gLateBoot.slots[i].fn.exchange(nullptr, std::memory_order_acq_rel);
      if (fn != nullptr) {
        fn(gLateBoot.slots[i].ctx.load(std::memory_order_relaxed));
      }
    }
  }
  gPre.Run();
}

void PostCycle() {
  gPost.Run();
}

void EmergencyStop() {
  gEStopped.store(true, std::memory_order_release);
  int n = gEStopCount.load(std::memory_order_acquire);
  if (n > kMaxCoreCallbacks) {
    n = kMaxCoreCallbacks;
  }
  for (int i = 0; i < n; ++i) {
    EmergencyStopCallback fn = gEStopCallbacks[i].load(std::memory_order_acquire);
    if (fn != nullptr) {
      fn();
    }
  }
}

}  // namespace detail

}  // namespace prosdk::platform
