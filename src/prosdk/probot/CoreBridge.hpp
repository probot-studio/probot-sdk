// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// The probot-core bridge, sketch side (design/PORT.md section 8). Included by <prosdk.h> after
// <probot.h>; never include it from the precompiled archive.
//
// probot-core's API is inline C++ that only exists where <probot.h> is visible, i.e. in the
// user's sketch. This header turns it into the plain function table of
// prosdk/platform/CoreOps.hpp and, at static initialization:
//   - installs the table into the archive (InstallCoreOps),
//   - installs core's framework hooks (boot / preCycle / postCycle / emergencyStop) so prosdk runs
//     on core's user task in every phase,
//   - reserves the GPIO pins core owns (status LED, RSL, enable line, battery sensor) so the HAL
//     refuses them.
//
// probot::framework::install accepts one framework per program. If another framework was
// installed first, the bridge reports it through NoteFrameworkInstall (an error on the console at
// boot, repeated by HAL_Initialize) and the sketch keeps running without prosdk's per-cycle work:
// it does not abort, so the robot still boots and the Driver Station can show the error.
//
// Optional core features are detected with __has_include and compiled out when missing (their
// CoreOps entries stay null and the archive's defaults apply):
//   - R6 loop rate (<probot/core/loop_timing.hpp>): PROSDK_CORE_HAS_LOOP_RATE
//   - R7 opmode list and UTILITY mode (<probot/robot/opmodes.hpp>): PROSDK_CORE_HAS_OPMODES
//   - R8 six gamepads, analog triggers, 8-way POV, names (<probot/io/joystick_frame.hpp>):
//     PROSDK_CORE_HAS_MULTI_GAMEPAD. Without it the CoreGamepad fields of ABI 3 are filled the way
//     core 0.4.0 behaved (trigger buttons 0/1, core's 4-way POV, the mapping profile's name,
//     standard mapping) and only port 0 has data.
//   - R9 leveled console (<probot/telemetry/console.hpp>): PROSDK_CORE_HAS_CONSOLE. Without it the
//     console text goes to core's telemetry ring (telemetry::writeBytes) without its level.
//   - R10 wall clock (<probot/core/wall_clock.hpp>, ds::systemTimeValid()):
//     PROSDK_CORE_HAS_WALL_CLOCK. Without it the archive's 2024-01-01 heuristic applies.
//   - R12 small items (<probot/io/battery_status.hpp>: requestStop(), ds::rslState(),
//     ds::commsDisableCount(), io::battery::setLowVoltage()): PROSDK_CORE_HAS_SMALL_ITEMS. The
//     header is the last of R12 part 1 (core PR #37), so a core that has it has all four. Without it
//     a halted robot program does not stop the Driver Station, HAL_GetRSLState and
//     HAL_GetCommsDisableCount report HAL_HANDLE_ERROR and core's low-voltage flag stays as core
//     set it.
//   - R13 schema size limit (dashboard::Limits::maxSchemaBytes, no new header: detected on the type
//     at compile time). Without it the archive reports a refused schema against
//     PROBOT_DASH_MAX_VALUE.
//   - R12 dashboard persistence (PROBOT_HAS_DASH_PERSIST from <probot/dashboard/dashboard.hpp>,
//     and a requires-check on probot::dashboard::clearPersisted(): kCoreHasDashPersist). The
//     macro marks core's final rule (only Driver Station writes the robot accepted are stored);
//     earlier drafts of core PR #39 had clearPersisted() but stored the robot's own values too,
//     which the tunable backend's republish rule does not follow. The PERSISTENT flag value
//     existed before (core stored it and did nothing), so it is not what is detected. Without it
//     dashClearPersisted stays null and the archive publishes no tunable PERSISTENT.
//   - R15 grid restarts (probot::loopGridRestarts() in <probot/core/loop_timing.hpp>, which older
//     cores already have: detected with a requires-check on the call, kCoreHasGridRestarts, as core
//     intends). Without it loopGridRestarts stays null and the robot host re-aligns its schedule
//     on counted overruns only.
//   - R15 dashboard limits (probot::dashboard::limits(), same commit, detected the same way):
//     dashMaxSchemaBytes reads it. Without it the bridge reads the store's limits from
//     dashboard::detail::g_store under the store's lock, as before.
//   - R11 file sharing (PROBOT_HAS_FILES_SHARE from <probot/files/files.hpp>, and a requires-check
//     on the VFS-path overload probot::files::share(name, vfsDir) and on unshare(name):
//     kCoreHasFilesShare). Without it filesShare/filesUnshare stay null and nothing is shared
//     (DataLogManager's folder does not appear on the Driver Station).
//   - R18 match info (PROBOT_HAS_DS_MATCH_INFO from <probot/robot/match_info.hpp>, which
//     <probot/ds.hpp> includes, and requires-checks on probot::ds::matchInfoSeq() and
//     probot::ds::matchInfo(): kCoreHasMatchInfo). Without it matchInfoSeq/matchInfo stay null:
//     the HAL reports an unknown alliance station and empty game data and match info.
//   - R20 Home/Guide button: no detection either. The active mapping profile is copied as it is
//     (CopyCoreMapping); with R20 its Options is button 16 in the standard profiles, before R20 -1,
//     and the HAL reports WPILib's GUIDE button available only for an Options index the frame has.
// The dashboard value limit (dashMaxValue, CoreOps ABI 10) needs no detection: every supported
// core has it (dashboard::limits() with R15, the store's detail::maxValue() before).
//
// ODR: every function here only calls core's config-free headers, so all definitions are the same
// in every translation unit. The pin reservation depends on the sketch's configuration macros and
// therefore runs from an internal-linkage initializer (one per translation unit, the archive keeps
// the union). The install runs once (inline variable).

#pragma once

#if !defined(PROBOT_VERSION) || !__has_include(<probot/framework.hpp>) || \
    !__has_include(<probot/dashboard/dashboard.hpp>)
// One error only: the rest of the file is skipped, so a missing core header does not add a second one.
#error "prosdk requires probot-core 0.5.0 or later (until its release: the probot-core draft PR stack with the framework API, <probot/framework.hpp>). Install that probot-core library and include <prosdk.h> (it includes <probot.h> itself)."
#else

#include <probot/dashboard/dashboard.hpp>
#include <probot/ds.hpp>
#include <probot/framework.hpp>
#include <probot/io/joystick_api.hpp>
#include <probot/io/joystick_mappings/mapping.hpp>
#include <probot/robot/state.hpp>
#include <probot/telemetry/telemetry.hpp>

#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/CoreOps.hpp"

#if PROBOT_VERSION < PROBOT_VERSION_OF(0, 4, 0)
#error "prosdk requires probot-core 0.5.0 or later (found a PROBOT_VERSION below 0.4.0)."
#endif

#if __has_include(<probot/core/loop_timing.hpp>)
#include <probot/core/loop_timing.hpp>
#define PROSDK_CORE_HAS_LOOP_RATE 1
#else
#define PROSDK_CORE_HAS_LOOP_RATE 0
#endif

#if __has_include(<probot/robot/opmodes.hpp>)
#include <probot/robot/opmodes.hpp>
#define PROSDK_CORE_HAS_OPMODES 1
#else
#define PROSDK_CORE_HAS_OPMODES 0
#endif

#if __has_include(<probot/io/joystick_frame.hpp>)
#define PROSDK_CORE_HAS_MULTI_GAMEPAD 1
#else
#define PROSDK_CORE_HAS_MULTI_GAMEPAD 0
#endif

#if __has_include(<probot/telemetry/console.hpp>)
#include <probot/telemetry/console.hpp>
#define PROSDK_CORE_HAS_CONSOLE 1
#else
#define PROSDK_CORE_HAS_CONSOLE 0
#endif

#if __has_include(<probot/core/wall_clock.hpp>)
#define PROSDK_CORE_HAS_WALL_CLOCK 1
#else
#define PROSDK_CORE_HAS_WALL_CLOCK 0
#endif

#if __has_include(<probot/io/battery_status.hpp>)
#include <probot/io/battery_status.hpp>
#define PROSDK_CORE_HAS_SMALL_ITEMS 1
#else
#define PROSDK_CORE_HAS_SMALL_ITEMS 0
#endif

#if __has_include(<probot/files/files.hpp>)
#include <probot/files/files.hpp>
#endif

#if defined(PROBOT_BATTERY_INA)
#include <Wire.h>
#endif

#include <concepts>
#include <new>

namespace prosdk::corebridge {

namespace pf = prosdk::platform;

// The archive's copies of core's enums and limits must match core.
static_assert(static_cast<uint8_t>(probot::robot::Phase::STOPPED) == pf::kCorePhaseStopped &&
              static_cast<uint8_t>(probot::robot::Phase::AUTO_INIT) == pf::kCorePhaseAutoInit &&
              static_cast<uint8_t>(probot::robot::Phase::AUTO_RUN) == pf::kCorePhaseAutoRun &&
              static_cast<uint8_t>(probot::robot::Phase::TELEOP_INIT) ==
                  pf::kCorePhaseTeleopInit &&
              static_cast<uint8_t>(probot::robot::Phase::TELEOP_RUN) == pf::kCorePhaseTeleopRun &&
              static_cast<uint8_t>(probot::robot::Phase::TRANSITION) == pf::kCorePhaseTransition,
              "probot-core Phase values changed");
static_assert(static_cast<uint8_t>(probot::robot::OpMode::AUTO) == pf::kCoreModeAuto &&
                  static_cast<uint8_t>(probot::robot::OpMode::TELEOP) == pf::kCoreModeTeleop,
              "probot-core OpMode values changed");
#if PROSDK_CORE_HAS_OPMODES
static_assert(static_cast<uint8_t>(probot::robot::Phase::UTILITY_INIT) ==
                      pf::kCorePhaseUtilityInit &&
                  static_cast<uint8_t>(probot::robot::Phase::UTILITY_RUN) ==
                      pf::kCorePhaseUtilityRun &&
                  static_cast<uint8_t>(probot::robot::OpMode::UTILITY) == pf::kCoreModeUtility,
              "probot-core UTILITY values changed");
static_assert(probot::robot::OpModeRegistry::kMaxOptions == pf::kCoreOpModeMaxOptions &&
                  probot::robot::OpModeRegistry::kMaxName == pf::kCoreOpModeMaxName &&
                  probot::robot::OpModeRegistry::kMaxGroup == pf::kCoreOpModeMaxGroup &&
                  probot::robot::OpModeRegistry::kMaxDescription ==
                      pf::kCoreOpModeMaxDescription,
              "probot-core opmode list limits changed");
#endif
#if PROSDK_CORE_HAS_MULTI_GAMEPAD
static_assert(probot::io::kMaxGamepads == pf::kCoreMaxGamepads &&
                  probot::io::kMaxGamepadName == pf::kCoreGamepadMaxName,
              "probot-core gamepad limits changed");
#endif
#if PROSDK_CORE_HAS_CONSOLE
static_assert(static_cast<uint8_t>(probot::console::Level::Info) ==
                      static_cast<uint8_t>(pf::ConsoleLevel::kInfo) &&
                  static_cast<uint8_t>(probot::console::Level::Warning) ==
                      static_cast<uint8_t>(pf::ConsoleLevel::kWarning) &&
                  static_cast<uint8_t>(probot::console::Level::Error) ==
                      static_cast<uint8_t>(pf::ConsoleLevel::kError),
              "probot-core console levels changed");
#endif
// the archive's dashboard flags (prosdk/platform/dashboard/DashboardStore.hpp kFlag*)
static_assert(probot::dashboard::TELEMETRY == 1 && probot::dashboard::TUNABLE == 2 &&
                  probot::dashboard::MUTABLE == 4 && probot::dashboard::ROBUST == 8 &&
                  probot::dashboard::KEEP_DUPLICATES == 16 && probot::dashboard::PERSISTENT == 32,
              "probot-core dashboard flag values changed");
#if PROSDK_CORE_HAS_LOOP_RATE
static_assert(probot::core::kMinLoopPeriodMs == pf::kCoreLoopPeriodMinMs &&
                  probot::core::kMaxLoopPeriodMs == pf::kCoreLoopPeriodMaxMs,
              "probot-core loop period limits changed");
#endif

inline uint8_t Phase() { return static_cast<uint8_t>(probot::ds::phase()); }
inline uint8_t SelectedMode() { return static_cast<uint8_t>(probot::ds::selectedMode()); }
inline bool OutputsEnabled() { return probot::ds::outputsEnabled(); }
inline bool IsEStopped() { return probot::ds::isEStopped(); }
inline bool IsAttached() { return probot::ds::isAttached(); }
inline float MatchTimeSeconds() { return probot::ds::matchTimeSeconds(); }
inline uint32_t PacketSeq() { return probot::ds::packetSeq(); }

// core's joystick API on one snapshot, so the values of ABI 3 come from the same frame as the
// raw arrays
struct SnapshotSource final : probot::io::IGamepadSource {
  SnapshotSource(int p, const probot::io::GamepadSnapshot& s) : port(p), snap(s) {}
  probot::io::GamepadSnapshot read() const override { return snap; }
#if PROSDK_CORE_HAS_MULTI_GAMEPAD
  bool info(char* name, size_t cap, bool* standard) const override {
    return probot::io::rawGamepad(port).info(name, cap, standard);
  }
#endif
  int port;
  probot::io::GamepadSnapshot snap;
};

// Analog value 0..255 of raw button i, as core's getRawButtonValue() reads it.
inline uint8_t RawButtonValue(const probot::io::GamepadSnapshot& s, int i) {
  if (i < 0 || static_cast<uint32_t>(i) >= s.buttonCount) {
    return 0;
  }
#if PROSDK_CORE_HAS_MULTI_GAMEPAD
  if (static_cast<uint32_t>(i) < s.valueCount) {
    return s.buttonValues[i];
  }
#endif
  return s.buttons[i] ? 255 : 0;
}

inline void FillCoreValues(int port, const probot::io::GamepadSnapshot& s,
                           const probot::io::joystick_mapping::MappingProfile& m,
                           pf::CoreGamepad* out) {
  SnapshotSource source{port, s};
  probot::io::joystick_api::Joystick joystick{&source};
  out->leftTriggerRaw = RawButtonValue(s, m.LT);
  out->rightTriggerRaw = RawButtonValue(s, m.RT);
  out->pov = static_cast<int16_t>(joystick.getPOV());
#if PROSDK_CORE_HAS_MULTI_GAMEPAD
  out->leftTrigger = joystick.getLeftTriggerAxis();
  out->rightTrigger = joystick.getRightTriggerAxis();
  bool standard = false;
  if (source.info(out->name, sizeof(out->name), &standard)) {
    out->standardMapping = standard ? 1 : 0;
  } else {
    out->name[0] = '\0';
    out->standardMapping = 0;
  }
#else
  out->leftTrigger = out->leftTriggerRaw != 0 ? 1.0f : 0.0f;
  out->rightTrigger = out->rightTriggerRaw != 0 ? 1.0f : 0.0f;
  out->standardMapping = 1;
  const char* name = m.name != nullptr ? m.name : "";
  size_t n = 0;
  while (name[n] != '\0' && n < pf::kCoreGamepadMaxName) {
    out->name[n] = name[n];
    ++n;
  }
  out->name[n] = '\0';
#endif
}

inline bool RawGamepad(int port, pf::CoreGamepad* out) {
  if (out == nullptr) {
    return false;
  }
  probot::io::GamepadSnapshot s{};
  if (!probot::ds::rawGamepad(port, s)) {
    return false;
  }
  out->timestampMs = s.ms;
  out->seq = s.seq;
  uint32_t na = s.axisCount < pf::kCoreGamepadMaxAxes ? s.axisCount : pf::kCoreGamepadMaxAxes;
  uint32_t nb =
      s.buttonCount < pf::kCoreGamepadMaxButtons ? s.buttonCount : pf::kCoreGamepadMaxButtons;
  out->axisCount = static_cast<uint8_t>(na);
  out->buttonCount = static_cast<uint8_t>(nb);
  for (uint32_t i = 0; i < pf::kCoreGamepadMaxAxes; ++i) {
    out->axes[i] = i < na ? s.axes[i] : 0.0f;
  }
  uint32_t buttons = 0;
  for (uint32_t i = 0; i < nb; ++i) {
    if (s.buttons[i]) {
      buttons |= uint32_t{1} << i;
    }
  }
  out->buttons = buttons;
  const auto& m = probot::io::joystick_mapping::getActive();
  pf::CopyCoreMapping(m, &out->mapping);
  FillCoreValues(port, s, m, out);
  return true;
}

inline float BatteryVoltage() { return probot::robot::state().read().batteryVoltage; }

inline void ConsoleWrite(const char* text, size_t len) {
  if (text != nullptr && len != 0) {
    probot::telemetry::writeBytes(text, len);
  }
}

#if PROSDK_CORE_HAS_CONSOLE
// One console entry per write (core drops a trailing newline and splits text over 512 bytes).
inline void ConsoleWriteLevel(uint8_t level, const char* text, size_t len) {
  if (text != nullptr && len != 0) {
    probot::console::write(static_cast<probot::console::Level>(level <= 2 ? level : 0), text,
                           len);
  }
}
#define PROSDK_CORE_CONSOLE_OP .consoleWriteLevel = &ConsoleWriteLevel
#else
#define PROSDK_CORE_CONSOLE_OP .consoleWriteLevel = nullptr
#endif

#if PROSDK_CORE_HAS_WALL_CLOCK
inline bool SystemTimeValid() { return probot::ds::systemTimeValid(); }
#define PROSDK_CORE_WALL_CLOCK_OP .systemTimeValid = &SystemTimeValid
#else
#define PROSDK_CORE_WALL_CLOCK_OP .systemTimeValid = nullptr
#endif

#if PROSDK_CORE_HAS_SMALL_ITEMS
inline bool RequestStop() { return probot::requestStop(); }
inline bool RslState() { return probot::ds::rslState(); }
inline uint32_t CommsDisableCount() { return probot::ds::commsDisableCount(); }
inline bool SetLowVoltage(float volts) { return probot::io::battery::setLowVoltage(volts); }
#define PROSDK_CORE_SMALL_ITEMS_OPS \
  .requestStop = &RequestStop, .rslState = &RslState, .commsDisableCount = &CommsDisableCount, \
  .setLowVoltage = &SetLowVoltage
#else
#define PROSDK_CORE_SMALL_ITEMS_OPS \
  .requestStop = nullptr, .rslState = nullptr, .commsDisableCount = nullptr, \
  .setLowVoltage = nullptr
#endif

// R15 dashboard limits, looked up like clearPersisted below (persist_probe).
namespace limits_probe {
using namespace probot::dashboard;
template <typename L, typename... A>
inline L CurrentLimits(A... a) {
  return limits(a...);
}
template <typename... A>
inline constexpr bool kHasLimits = requires(A... a) {
  { limits(a...) } -> std::same_as<probot::dashboard::Limits>;
};
}  // namespace limits_probe

// R13: the store's per-schema limit as configured, 0 before runtime_setup(). Read on every call
// (the archive never keeps it), so a zero read before core configured the store does not stick.
// With R15 from core's dashboard::limits(); an older core has no accessor, so the store's limits
// are read under its lock, as core's own detail::maxValue() does. L is a template parameter so
// that a core without Limits::maxSchemaBytes never instantiates it.
template <typename L>
inline uint32_t DashMaxSchemaBytes() {
  if constexpr (limits_probe::kHasLimits<>) {
    const L lim = limits_probe::CurrentLimits<L>();
    return lim.maxSchemaBytes;  // all limits are 0 before runtime_setup()
  } else {
    auto& st = probot::dashboard::detail::g_store;
    probot::core::MuxGuard g(st.mux);
    const L& lim = st.lim;
    return st.configured ? lim.maxSchemaBytes : 0;
  }
}
// The store's per-value limit (PROBOT_DASH_MAX_VALUE as configured), 0 before runtime_setup().
// Every core has it: with R15 from dashboard::limits(), before that from the store's own
// detail::maxValue() (what core's set() checks against). Read on every call, as above.
template <typename L>
inline uint32_t DashMaxValue() {
  if constexpr (limits_probe::kHasLimits<>) {
    const L lim = limits_probe::CurrentLimits<L>();
    return lim.maxValue;
  } else {
    return probot::dashboard::detail::maxValue();
  }
}

template <typename L>
constexpr uint32_t (*DashMaxSchemaBytesOp())() {
  if constexpr (requires(const L& lim) { lim.maxSchemaBytes; }) {
    return &DashMaxSchemaBytes<L>;
  } else {
    return nullptr;
  }
}

// R12 persistence. clearPersisted is looked up from a namespace that sees probot::dashboard
// through a using-directive and called with a dependent (empty) argument pack, so a core without
// it only makes the requires-check false instead of failing to compile. Nothing here declares a
// name in core's namespaces.
namespace persist_probe {
using namespace probot::dashboard;
template <typename... A>
inline bool ClearPersisted(A... a) {
  return clearPersisted(a...);
}
template <typename... A>
inline constexpr bool kHasClearPersisted = requires(A... a) {
  { clearPersisted(a...) } -> std::same_as<bool>;
};
}  // namespace persist_probe

/** True when the sketch's probot-core keeps PERSISTENT dashboard values across reboots. */
#if defined(PROBOT_HAS_DASH_PERSIST)
inline constexpr bool kCoreHasDashPersist = persist_probe::kHasClearPersisted<>;
#else
inline constexpr bool kCoreHasDashPersist = false;
#endif

constexpr bool (*DashClearPersistedOp())() {
  if constexpr (kCoreHasDashPersist) {
    return &persist_probe::ClearPersisted<>;
  } else {
    return nullptr;
  }
}

inline uint16_t DashPublish(const char* path, uint8_t vt, const char* typeString,
                            const char* propertiesJson, uint8_t flags) {
  return probot::dashboard::publish(path, static_cast<probot::dashboard::ValueType>(vt),
                                    typeString, propertiesJson, flags);
}
inline void DashUnpublish(uint16_t id) { probot::dashboard::unpublish(id); }
inline bool DashSet(uint16_t id, const void* data, size_t len) {
  return probot::dashboard::set(id, data, len);
}
inline bool DashSetProperties(uint16_t id, const char* json) {
  return probot::dashboard::setProperties(id, json);
}
inline bool DashSetFlags(uint16_t id, uint8_t flags) {
  return probot::dashboard::setFlags(id, flags);
}
inline bool DashHasSchema(const char* name) { return probot::dashboard::hasSchema(name); }
inline bool DashAddSchema(const char* name, const char* type, const void* data, size_t len) {
  return probot::dashboard::addSchema(name, type, data, len);
}

struct DrainContext {
  void (*fn)(const pf::CoreDashboardWrite&, void*);
  void* ctx;
};

inline void DrainOne(const probot::dashboard::Write& w, void* ctx) {
  auto* d = static_cast<DrainContext*>(ctx);
  pf::CoreDashboardWrite cw{w.id, static_cast<uint8_t>(w.vt), w.len, w.data};
  d->fn(cw, d->ctx);
}

inline size_t DashDrainWrites(void (*fn)(const pf::CoreDashboardWrite&, void*), void* ctx) {
  if (fn == nullptr) {
    return 0;
  }
  DrainContext d{fn, ctx};
  return probot::dashboard::drainWrites(&DrainOne, &d);
}
inline void DashRejectWrite(uint16_t id, uint8_t reason) {
  probot::dashboard::rejectWrite(id, reason);
}
inline void DashStats(pf::CoreDashboardStats* out) {
  if (out == nullptr) {
    return;
  }
  probot::dashboard::Stats s = probot::dashboard::stats();
  *out = pf::CoreDashboardStats{s.topics, s.maxTopics, s.bytesUsed, s.bytesMax, s.dropped};
}

#if PROSDK_CORE_HAS_LOOP_RATE
inline bool SetLoopPeriodMs(uint32_t ms) { return probot::setLoopPeriodMs(ms); }
inline uint32_t LoopPeriodMs() { return probot::loopPeriodMs(); }
inline uint32_t LoopOverruns() { return probot::loopOverruns(); }
#define PROSDK_CORE_LOOP_RATE_OPS \
  .setLoopPeriodMs = &SetLoopPeriodMs, .loopPeriodMs = &LoopPeriodMs, \
  .loopOverruns = &LoopOverruns
#else
#define PROSDK_CORE_LOOP_RATE_OPS \
  .setLoopPeriodMs = nullptr, .loopPeriodMs = nullptr, .loopOverruns = nullptr
#endif

// R15 grid restarts, looked up like clearPersisted (persist_probe). probot::loopGridRestarts() is
// in <probot/core/loop_timing.hpp>, included above when core has it.
namespace grid_probe {
using namespace probot;
template <typename... A>
inline uint32_t LoopGridRestarts(A... a) {
  return loopGridRestarts(a...);
}
template <typename... A>
inline constexpr bool kHasLoopGridRestarts = requires(A... a) {
  { loopGridRestarts(a...) } -> std::same_as<uint32_t>;
};
}  // namespace grid_probe

/** True when the sketch's probot-core counts every restart of its pass grid (R15). */
inline constexpr bool kCoreHasGridRestarts = grid_probe::kHasLoopGridRestarts<>;

constexpr uint32_t (*LoopGridRestartsOp())() {
  if constexpr (kCoreHasGridRestarts) {
    return &grid_probe::LoopGridRestarts<>;
  } else {
    return nullptr;
  }
}

// R11 file sharing, looked up like clearPersisted (persist_probe). The macro marks the feature;
// the requires-check makes sure the VFS-path overload (the one prosdk calls; the fs::FS overload
// came first in core's drafts) and unshare() exist.
#if defined(PROBOT_HAS_FILES_SHARE)
namespace files_probe {
using namespace probot::files;
template <typename... A>
inline bool Share(A... a) {
  return share(a...);
}
template <typename... A>
inline bool Unshare(A... a) {
  return unshare(a...);
}
template <typename... A>
inline constexpr bool kHasShare = requires(A... a) {
  { share(a...) } -> std::same_as<bool>;
};
template <typename... A>
inline constexpr bool kHasUnshare = requires(A... a) {
  { unshare(a...) } -> std::same_as<bool>;
};
}  // namespace files_probe

/** True when the sketch's probot-core offers folders on the Driver Station (R11). */
inline constexpr bool kCoreHasFilesShare =
    files_probe::kHasShare<const char*, const char*> && files_probe::kHasUnshare<const char*>;
#else
inline constexpr bool kCoreHasFilesShare = false;
#endif

constexpr bool (*FilesShareOp())(const char*, const char*) {
#if defined(PROBOT_HAS_FILES_SHARE)
  if constexpr (kCoreHasFilesShare) {
    return &files_probe::Share<const char*, const char*>;
  }
#endif
  return nullptr;
}

constexpr bool (*FilesUnshareOp())(const char*) {
#if defined(PROBOT_HAS_FILES_SHARE)
  if constexpr (kCoreHasFilesShare) {
    return &files_probe::Unshare<const char*>;
  }
#endif
  return nullptr;
}

// R18 match info, looked up like clearPersisted (persist_probe). The macro marks the feature; the
// requires-checks make sure the two calls prosdk makes exist with the expected shapes.
#if defined(PROBOT_HAS_DS_MATCH_INFO)
static_assert(probot::ds::MatchInfo::kMaxText == pf::kCoreMatchInfoMaxText &&
                  sizeof(probot::ds::MatchInfo{}.eventName) == pf::kCoreMatchInfoMaxText + 1 &&
                  sizeof(probot::ds::MatchInfo{}.gameData) == pf::kCoreMatchInfoMaxText + 1,
              "probot-core match text limits changed");
static_assert(static_cast<uint8_t>(probot::ds::AllianceStation::Unknown) == 0 &&
                  static_cast<uint8_t>(probot::ds::AllianceStation::Red1) == 1 &&
                  static_cast<uint8_t>(probot::ds::AllianceStation::Blue1) == 4 &&
                  static_cast<uint8_t>(probot::ds::AllianceStation::Blue3) == 6 &&
                  static_cast<uint8_t>(probot::ds::MatchType::None) == 0 &&
                  static_cast<uint8_t>(probot::ds::MatchType::Elimination) == 3,
              "probot-core match info numbering changed (prosdk expects the WPILib HAL values)");
namespace match_probe {
using namespace probot::ds;
template <typename... A>
inline uint32_t MatchInfoSeq(A... a) {
  return matchInfoSeq(a...);
}
template <typename... A>
inline void ReadMatchInfo(pf::CoreMatchInfo* out, A... a) {
  if (out != nullptr) {
    pf::CopyCoreMatchInfo(matchInfo(a...), out);
  }
}
template <typename... A>
inline constexpr bool kHasMatchInfoSeq = requires(A... a) {
  { matchInfoSeq(a...) } -> std::same_as<uint32_t>;
};
template <typename... A>
inline constexpr bool kHasMatchInfo = requires(A... a) {
  { matchInfo(a...) } -> std::same_as<probot::ds::MatchInfo>;
};
}  // namespace match_probe

/** True when the sketch's probot-core has the Driver Station's match information (R18). */
inline constexpr bool kCoreHasMatchInfo =
    match_probe::kHasMatchInfoSeq<> && match_probe::kHasMatchInfo<>;
#else
inline constexpr bool kCoreHasMatchInfo = false;
#endif

constexpr uint32_t (*MatchInfoSeqOp())() {
#if defined(PROBOT_HAS_DS_MATCH_INFO)
  if constexpr (kCoreHasMatchInfo) {
    return &match_probe::MatchInfoSeq<>;
  }
#endif
  return nullptr;
}

constexpr void (*MatchInfoOp())(pf::CoreMatchInfo*) {
#if defined(PROBOT_HAS_DS_MATCH_INFO)
  if constexpr (kCoreHasMatchInfo) {
    return &match_probe::ReadMatchInfo<>;
  }
#endif
  return nullptr;
}

#if PROSDK_CORE_HAS_OPMODES
inline bool SetOpModeOptions(const pf::CoreOpModeOption* options, size_t count) {
  if (count > pf::kCoreOpModeMaxOptions || (count != 0 && options == nullptr)) {
    return false;
  }
  if (count == 0) {
    return probot::ds::setOpModeOptions(nullptr, 0);
  }
  auto* list = new (std::nothrow) probot::ds::OpModeOption[count];
  if (list == nullptr) {
    return false;
  }
  for (size_t i = 0; i < count; ++i) {
    list[i] = probot::ds::OpModeOption{options[i].id,
                                       static_cast<probot::ds::OpMode>(options[i].mode),
                                       options[i].name, options[i].group,
                                       options[i].description};
  }
  bool ok = probot::ds::setOpModeOptions(list, count);
  delete[] list;
  return ok;
}
inline uint64_t SelectedOpModeId() { return probot::ds::selectedOpModeId(); }
inline uint64_t ActiveOpModeId() { return probot::ds::activeOpModeId(); }
inline bool UtilityAvailable() { return probot::ds::utilityAvailable(); }
#define PROSDK_CORE_OPMODE_OPS \
  .setOpModeOptions = &SetOpModeOptions, .selectedOpModeId = &SelectedOpModeId, \
  .activeOpModeId = &ActiveOpModeId, .utilityAvailable = &UtilityAvailable
#else
#define PROSDK_CORE_OPMODE_OPS \
  .setOpModeOptions = nullptr, .selectedOpModeId = nullptr, .activeOpModeId = nullptr, \
  .utilityAvailable = nullptr
#endif

// Core's boot hook. With USB CDC On Boot Enabled (the Pulse settings) Arduino's Serial is HWCDC on
// USB-C. From Arduino-ESP32 3.3.9, while the host keeps the port open without reading (a serial
// monitor opened, then closed, cable still in), each write that finds the TX ring full waits up to
// 20 times the TX timeout (100 ms by default, so about 2 s per write), and core's sysloop, which
// also runs the emergency stop, logs. probot-core sets the timeout to 5 ms in runtime_setup() since
// its PR #46 (about 100 ms per write); this hook runs after runtime_setup() and applies the same
// rule, so a core before #46 gets it too (on #46 it repeats core's value). Not lower: the TX lock
// take and flush() use the same timeout, so at 1 ms a second printing task loses its whole line
// and flush() drops the ring with a reading host. Before 3.3.9 a write stops waiting after one
// timeout by itself (core's review), so the rule leaves those cores alone.
inline void Boot() {
#if defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE && defined(ARDUINO_USB_CDC_ON_BOOT) && \
    ARDUINO_USB_CDC_ON_BOOT
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 9)
  HWCDCSerial.setTxTimeoutMs(5);
#endif
#endif
  pf::detail::Boot();
}

// Designated initializers: several fields share a type (matchInfoSeq and dashMaxValue are both
// uint32_t (*)()), so a positional table could swap two of them and still compile; a name that is
// wrong or out of the struct's order does not.
inline bool Install() {
  static constexpr pf::CoreOps kOps{
      .abiVersion = pf::kCoreOpsAbiVersion,
      .size = sizeof(pf::CoreOps),
      .coreVersion = PROBOT_VERSION,
      .coreVersionString = PROBOT_VERSION_STRING,
      .phase = &Phase,
      .selectedMode = &SelectedMode,
      .outputsEnabled = &OutputsEnabled,
      .isEStopped = &IsEStopped,
      .isAttached = &IsAttached,
      .matchTimeSeconds = &MatchTimeSeconds,
      .packetSeq = &PacketSeq,
      .rawGamepad = &RawGamepad,
      .batteryVoltage = &BatteryVoltage,
      .consoleWrite = &ConsoleWrite,
      .dashPublish = &DashPublish,
      .dashUnpublish = &DashUnpublish,
      .dashSet = &DashSet,
      .dashSetProperties = &DashSetProperties,
      .dashSetFlags = &DashSetFlags,
      .dashHasSchema = &DashHasSchema,
      .dashAddSchema = &DashAddSchema,
      .dashDrainWrites = &DashDrainWrites,
      .dashRejectWrite = &DashRejectWrite,
      .dashStats = &DashStats,
      PROSDK_CORE_LOOP_RATE_OPS,
      PROSDK_CORE_OPMODE_OPS,
      PROSDK_CORE_CONSOLE_OP,
      PROSDK_CORE_WALL_CLOCK_OP,
      PROSDK_CORE_SMALL_ITEMS_OPS,
      .dashMaxSchemaBytes = DashMaxSchemaBytesOp<probot::dashboard::Limits>(),
      .dashClearPersisted = DashClearPersistedOp(),
      .loopGridRestarts = LoopGridRestartsOp(),
      .filesShare = FilesShareOp(),
      .filesUnshare = FilesUnshareOp(),
      .matchInfoSeq = MatchInfoSeqOp(),
      .matchInfo = MatchInfoOp(),
      .dashMaxValue = &DashMaxValue<probot::dashboard::Limits>,
  };
  bool ok = pf::InstallCoreOps(&kOps);
  bool hooks = probot::framework::install(probot::framework::Hooks{
      &Boot, &pf::detail::PreCycle, &pf::detail::PostCycle,
      &pf::detail::EmergencyStop});
  pf::NoteFrameworkInstall(hooks);
  return ok && hooks;
}

#undef PROSDK_CORE_LOOP_RATE_OPS
#undef PROSDK_CORE_OPMODE_OPS
#undef PROSDK_CORE_CONSOLE_OP
#undef PROSDK_CORE_WALL_CLOCK_OP
#undef PROSDK_CORE_SMALL_ITEMS_OPS

/**
 * True once the bridge is installed: the CoreOps table and prosdk's framework hooks (evaluated
 * once per program during static initialization; an inline variable, so every translation unit
 * that includes <prosdk.h> shares it).
 */
inline const bool kInstalled = Install();

// probot-core #46 and later default the INA pins to SDA 1 / SCL 2 (the Pulse J3 header) in
// battery.hpp, which <probot.h> includes before this header, so PROBOT_BATTERY_INA_SDA/SCL are
// always defined with PROBOT_BATTERY_INA there. An older core leaves them undefined and calls
// Wire.begin() without pins: the esp32s3 variant's SDA 8 / SCL 9.
#if defined(PROBOT_BATTERY_INA) && !defined(PROBOT_BATTERY_INA_SDA)
#warning "This probot-core (before PR #46) starts the INA battery sensor's I2C bus on GPIO 8/9, which are not on a Pulse header, and prosdk I2C port 0 then shares that bus. Define PROBOT_BATTERY_INA_SDA 1 and PROBOT_BATTERY_INA_SCL 2 (J3), or 8/9 on another board."
#endif

#if defined(PROBOT_BATTERY_INA)
// Core's Wire.begin for its INA sensor, so prosdk's I2C port 0 starts the bus the same way
// (pf::SetCoreI2CStart).
inline bool StartCoreI2C() {
#if defined(PROBOT_BATTERY_INA_SDA) && defined(PROBOT_BATTERY_INA_SCL)
  return Wire.begin(PROBOT_BATTERY_INA_SDA, PROBOT_BATTERY_INA_SCL);
#else
  return Wire.begin();  // core before #46: the variant's 8 / 9
#endif
}
#endif

namespace {
// Per translation unit: the core pins this TU's configuration macros name.
inline bool ReservePinsOfThisUnit() {
  const int pins[] = {
#ifdef NEOPIXEL_PIN
      NEOPIXEL_PIN,
#endif
#ifdef PROBOT_RSL_PIN
      PROBOT_RSL_PIN,
#endif
#ifdef PROBOT_ESTOP_ENABLE_PIN
      PROBOT_ESTOP_ENABLE_PIN,
#endif
#ifdef PROBOT_BATTERY_ADC_PIN
      PROBOT_BATTERY_ADC_PIN,
#endif
#if defined(PROBOT_BATTERY_INA_SDA) && defined(PROBOT_BATTERY_INA_SCL)
      PROBOT_BATTERY_INA_SDA, PROBOT_BATTERY_INA_SCL,
#elif defined(PROBOT_BATTERY_INA)
      8, 9,  // core before #46 calls Wire.begin() without pins: the esp32s3 variant's SDA 8 /
             // SCL 9 (not on a Boardoza Pulse header; see the #warning above)
#endif
      -1,
  };
  pf::ReserveCorePins(pins, sizeof(pins) / sizeof(pins[0]));
#if defined(PROBOT_BATTERY_INA)
  pf::SetCoreI2CStart(&StartCoreI2C);
#endif
  return kInstalled;
}
const bool kPinsReserved = ReservePinsOfThisUnit();
}  // namespace

}  // namespace prosdk::corebridge

#endif  // probot-core with the framework API
