// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Archive side of the probot-core bridge (CoreOps.hpp). No core header is included here.

#include "prosdk/platform/CoreOps.hpp"

#include <atomic>
#include <cstdio>
#include <ctime>
#include <format>
#include <string>
#include <string_view>
#include <vector>

#include "prosdk/platform/Console.hpp"

namespace prosdk::platform {

namespace {

constinit std::atomic<bool> gInstalled{false};
constinit std::atomic<bool> gFrameworkInstalled{false};
// loop period stored by the default SetLoopPeriodMs (no core): 0 = default
constinit std::atomic<uint32_t> gDefaultLoopPeriodMs{0};

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
// No core: behave like core's setLoopPeriodMs with nothing to drive. Core without R6: refuse.
bool DefSetLoopPeriod(uint32_t ms) {
  if (gInstalled.load(std::memory_order_acquire)) {
    return false;
  }
  if (ms != 0 && (ms < kCoreLoopPeriodMinMs || ms > kCoreLoopPeriodMaxMs)) {
    return false;
  }
  gDefaultLoopPeriodMs.store(ms, std::memory_order_relaxed);
  return true;
}
uint32_t DefLoopPeriod() {
  uint32_t ms = gDefaultLoopPeriodMs.load(std::memory_order_relaxed);
  return ms != 0 ? ms : kCoreLoopPeriodDefaultMs;
}
uint32_t DefOverruns() { return 0; }
// No core: accept (nothing to show). Core without R7: refuse.
bool DefSetOpModeOptions(const CoreOpModeOption*, size_t) {
  return !gInstalled.load(std::memory_order_acquire);
}
uint64_t DefOpModeId() { return 0; }
void DefConsoleLevel(uint8_t level, const char* text, size_t len);  // below: needs gOps
// No core, or core without R10: valid once something moved the clock past 2024-01-01 (the
// ESP32's clock starts at 1970 after boot).
bool DefSystemTimeValid() {
  constexpr std::time_t k20240101 = 1704067200;
  return std::time(nullptr) >= k20240101;
}

constexpr CoreOps kDefaultOps{
    kCoreOpsAbiVersion, sizeof(CoreOps), 0, "",
    DefPhase, DefMode, DefFalse, DefFalse, DefFalse, DefMatchTime, DefSeq, DefGamepad,
    DefBattery, DefConsole,
    DefPublish, DefUnpublish, DefSet, DefSetProps, DefSetFlags, DefHasSchema, DefAddSchema,
    DefDrain, DefReject, DefStats,
    DefSetLoopPeriod, DefLoopPeriod, DefOverruns,
    DefSetOpModeOptions, DefOpModeId, DefOpModeId, DefFalse,
    DefConsoleLevel, DefSystemTimeValid,
};

// Written once (static initialization, single-threaded) before gInstalled is set.
constinit CoreOps gOps = kDefaultOps;
constinit std::atomic<uint64_t> gReservedPins{0};
constinit std::atomic<bool> gEStopped{false};
constinit std::atomic<bool> gBooted{false};

// Core without R9: the text goes to core's telemetry ring, without its level.
void DefConsoleLevel(uint8_t, const char* text, size_t len) {
  gOps.consoleWrite(text, len);
}

// Console sink once core is present: Driver Station console (leveled with R9) plus the default
// stderr (UART0).
void CoreConsoleSink(ConsoleLevel level, std::string_view text) {
  if (!text.empty()) {
    gOps.consoleWriteLevel(static_cast<uint8_t>(level), text.data(), text.size());
  }
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
        "release (CoreOps)\n",
        ConsoleLevel::kError);
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
  PROSDK_COREOPS_FILL(setLoopPeriodMs);
  PROSDK_COREOPS_FILL(loopPeriodMs);
  PROSDK_COREOPS_FILL(loopOverruns);
  PROSDK_COREOPS_FILL(setOpModeOptions);
  PROSDK_COREOPS_FILL(selectedOpModeId);
  PROSDK_COREOPS_FILL(activeOpModeId);
  PROSDK_COREOPS_FILL(utilityAvailable);
  PROSDK_COREOPS_FILL(consoleWriteLevel);
  PROSDK_COREOPS_FILL(systemTimeValid);
#undef PROSDK_COREOPS_FILL
  if (copy.coreVersionString == nullptr) {
    copy.coreVersionString = "";
  }
  copy.size = sizeof(CoreOps);
  gOps = copy;
  SetLeveledConsoleSink(CoreConsoleSink);
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

void NoteFrameworkInstall(bool installed) noexcept {
  gFrameworkInstalled.store(installed, std::memory_order_release);
  if (!installed) {
    ConsoleWrite(
        "[prosdk] ERROR: probot-core already has another framework installed "
        "(probot::framework::install returned false). prosdk's robot loop, Driver Station "
        "input and output gate will not run. Use only one framework per sketch.\n",
        ConsoleLevel::kError);
  }
}

bool FrameworkInstalled() noexcept {
  return gFrameworkInstalled.load(std::memory_order_acquire);
}

bool SetLoopPeriodMs(uint32_t ms) noexcept {
  return gOps.setLoopPeriodMs(ms);
}

uint32_t LoopPeriodMs() noexcept {
  return gOps.loopPeriodMs();
}

uint32_t LoopOverruns() noexcept {
  return gOps.loopOverruns();
}

namespace {

// Longest prefix of `text` of at most `max` bytes that does not split a UTF-8 sequence.
std::string_view TruncateUtf8(std::string_view text, size_t max) {
  if (text.size() <= max) {
    return text;
  }
  size_t n = max;
  while (n > 0 && (static_cast<unsigned char>(text[n]) & 0xC0) == 0x80) {
    --n;
  }
  return text.substr(0, n);
}

bool ValidCoreMode(uint8_t mode) {
  return mode == kCoreModeAuto || mode == kCoreModeTeleop || mode == kCoreModeUtility;
}

}  // namespace

bool SetOpModeOptions(const CoreOpModeOption* options, size_t count) {
  if (options == nullptr) {
    count = 0;
  }
  struct Copy {
    uint64_t id;
    uint8_t mode;
    std::string name, group, description;
  };
  std::vector<Copy> copies;
  copies.reserve(count < kCoreOpModeMaxOptions ? count : kCoreOpModeMaxOptions);
  auto warn = [](uint64_t id, std::string_view name, std::string_view what) {
    ConsoleWrite(std::format("[prosdk] Driver Station opmode {} \"{}\": {}\n", id, name, what),
                 ConsoleLevel::kWarning);
  };
  for (size_t i = 0; i < count; ++i) {
    const CoreOpModeOption& o = options[i];
    std::string_view name = o.name != nullptr ? std::string_view{o.name} : std::string_view{};
    if (copies.size() == kCoreOpModeMaxOptions) {
      ConsoleWrite(std::format(
          "[prosdk] Driver Station opmodes: {} options given, probot-core shows at most {}; "
          "the rest are not selectable\n",
          count, kCoreOpModeMaxOptions),
          ConsoleLevel::kWarning);
      break;
    }
    if (o.id == 0) {
      warn(o.id, name, "id 0 is reserved (no opmode); not listed");
      continue;
    }
    if (!ValidCoreMode(o.mode)) {
      warn(o.id, name, "unknown robot mode; not listed");
      continue;
    }
    if (name.empty()) {
      warn(o.id, name, "empty name; not listed");
      continue;
    }
    bool duplicate = false;
    for (const Copy& c : copies) {
      duplicate = duplicate || c.id == o.id;
    }
    if (duplicate) {
      warn(o.id, name, "duplicate id; not listed");
      continue;
    }
    std::string_view group = o.group != nullptr ? std::string_view{o.group} : std::string_view{};
    std::string_view description =
        o.description != nullptr ? std::string_view{o.description} : std::string_view{};
    auto fit = [&](std::string_view text, size_t max, const char* field) {
      std::string_view cut = TruncateUtf8(text, max);
      if (cut.size() != text.size()) {
        warn(o.id, name,
             std::format("{} truncated to {} bytes (probot-core limit {})", field, cut.size(),
                         max));
      }
      return std::string{cut};
    };
    copies.push_back(Copy{o.id, o.mode, fit(name, kCoreOpModeMaxName, "name"),
                          fit(group, kCoreOpModeMaxGroup, "group"),
                          fit(description, kCoreOpModeMaxDescription, "description")});
  }
  std::vector<CoreOpModeOption> list;
  list.reserve(copies.size());
  for (const Copy& c : copies) {
    list.push_back(CoreOpModeOption{c.id, c.mode, c.name.c_str(), c.group.c_str(),
                                    c.description.c_str()});
  }
  bool ok = gOps.setOpModeOptions(list.empty() ? nullptr : list.data(), list.size());
  if (!ok) {
    ConsoleWrite(std::format(
        "[prosdk] the Driver Station opmode list ({} options) was refused by probot-core ({}); "
        "the previous list stays\n",
        list.size(),
        HasCoreOps() ? "out of memory, or this probot-core has no opmode list (R7)"
                     : "invalid list"),
        ConsoleLevel::kError);
  }
  return ok;
}

uint64_t SelectedOpModeId() noexcept {
  return gOps.selectedOpModeId();
}

uint64_t ActiveOpModeId() noexcept {
  return gOps.activeOpModeId();
}

bool UtilityAvailable() noexcept {
  return gOps.utilityAvailable();
}

bool SystemTimeValid() noexcept {
  return gOps.systemTimeValid();
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
