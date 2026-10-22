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
//
// ODR: every function here only calls core's config-free headers, so all definitions are the same
// in every translation unit. The pin reservation depends on the sketch's configuration macros and
// therefore runs from an internal-linkage initializer (one per translation unit, the archive keeps
// the union). The install runs once (inline variable).

#pragma once

#if !defined(PROBOT_VERSION) || !__has_include(<probot/framework.hpp>) || \
    !__has_include(<probot/dashboard/dashboard.hpp>)
#error "prosdk needs probot-core >= 0.4.0 with the framework API (probot/framework.hpp, probot/dashboard/dashboard.hpp, probot/version.hpp: probot-core PRs #26-#28, branch sdk-target). Install that probot-core library and include <prosdk.h> (it includes <probot.h> itself)."
#endif

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
#error "prosdk needs probot-core >= 0.4.0 (found older PROBOT_VERSION)."
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

inline int8_t Index(int i) {
  return (i < 0 || i > 127) ? int8_t{-1} : static_cast<int8_t>(i);
}

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
  pf::CoreGamepadMapping& o = out->mapping;
  o.name = m.name != nullptr ? m.name : "";
  o.leftX = Index(m.leftX);
  o.leftY = Index(m.leftY);
  o.rightX = Index(m.rightX);
  o.rightY = Index(m.rightY);
  o.invertLeftY = m.invertLeftY ? 1 : 0;
  o.invertRightY = m.invertRightY ? 1 : 0;
  o.a = Index(m.A);
  o.b = Index(m.B);
  o.x = Index(m.X);
  o.y = Index(m.Y);
  o.leftBumper = Index(m.LB);
  o.rightBumper = Index(m.RB);
  o.leftTrigger = Index(m.LT);
  o.rightTrigger = Index(m.RT);
  o.back = Index(m.Back);
  o.start = Index(m.Start);
  o.options = Index(m.Options);
  o.leftStick = Index(m.LStick);
  o.rightStick = Index(m.RStick);
  o.dpadFromAxis = m.dpadFromAxis ? 1 : 0;
  o.dpadAxis = Index(m.dpadAxis);
  o.dpadUp = Index(m.DPadUp);
  o.dpadDown = Index(m.DPadDown);
  o.dpadLeft = Index(m.DPadLeft);
  o.dpadRight = Index(m.DPadRight);
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
#define PROSDK_CORE_CONSOLE_OP &ConsoleWriteLevel
#else
#define PROSDK_CORE_CONSOLE_OP nullptr
#endif

#if PROSDK_CORE_HAS_WALL_CLOCK
inline bool SystemTimeValid() { return probot::ds::systemTimeValid(); }
#define PROSDK_CORE_WALL_CLOCK_OP &SystemTimeValid
#else
#define PROSDK_CORE_WALL_CLOCK_OP nullptr
#endif

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
#define PROSDK_CORE_LOOP_RATE_OPS &SetLoopPeriodMs, &LoopPeriodMs, &LoopOverruns
#else
#define PROSDK_CORE_LOOP_RATE_OPS nullptr, nullptr, nullptr
#endif

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
  &SetOpModeOptions, &SelectedOpModeId, &ActiveOpModeId, &UtilityAvailable
#else
#define PROSDK_CORE_OPMODE_OPS nullptr, nullptr, nullptr, nullptr
#endif

inline bool Install() {
  static constexpr pf::CoreOps kOps{
      pf::kCoreOpsAbiVersion, sizeof(pf::CoreOps), PROBOT_VERSION, PROBOT_VERSION_STRING,
      &Phase, &SelectedMode, &OutputsEnabled, &IsEStopped, &IsAttached, &MatchTimeSeconds,
      &PacketSeq, &RawGamepad,
      &BatteryVoltage,
      &ConsoleWrite,
      &DashPublish, &DashUnpublish, &DashSet, &DashSetProperties, &DashSetFlags, &DashHasSchema,
      &DashAddSchema, &DashDrainWrites, &DashRejectWrite, &DashStats,
      PROSDK_CORE_LOOP_RATE_OPS,
      PROSDK_CORE_OPMODE_OPS,
      PROSDK_CORE_CONSOLE_OP,
      PROSDK_CORE_WALL_CLOCK_OP,
  };
  bool ok = pf::InstallCoreOps(&kOps);
  bool hooks = probot::framework::install(probot::framework::Hooks{
      &pf::detail::Boot, &pf::detail::PreCycle, &pf::detail::PostCycle,
      &pf::detail::EmergencyStop});
  pf::NoteFrameworkInstall(hooks);
  return ok && hooks;
}

#undef PROSDK_CORE_LOOP_RATE_OPS
#undef PROSDK_CORE_OPMODE_OPS
#undef PROSDK_CORE_CONSOLE_OP
#undef PROSDK_CORE_WALL_CLOCK_OP

/**
 * True once the bridge is installed: the CoreOps table and prosdk's framework hooks (evaluated
 * once per program during static initialization; an inline variable, so every translation unit
 * that includes <prosdk.h> shares it).
 */
inline const bool kInstalled = Install();

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
      8, 9,  // the board's default I2C pins (Wire), used by core's INA sensor
#endif
      -1,
  };
  pf::ReserveCorePins(pins, sizeof(pins) / sizeof(pins[0]));
  return kInstalled;
}
const bool kPinsReserved = ReservePinsOfThisUnit();
}  // namespace

}  // namespace prosdk::corebridge
