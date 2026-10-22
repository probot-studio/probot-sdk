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
#include <probot/io/joystick_mappings/mapping.hpp>
#include <probot/robot/state.hpp>
#include <probot/telemetry/telemetry.hpp>

#include "prosdk/platform/CoreOps.hpp"

#if PROBOT_VERSION < PROBOT_VERSION_OF(0, 4, 0)
#error "prosdk needs probot-core >= 0.4.0 (found older PROBOT_VERSION)."
#endif

namespace prosdk::corebridge {

namespace pf = prosdk::platform;

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
  return true;
}

inline float BatteryVoltage() { return probot::robot::state().read().batteryVoltage; }

inline void ConsoleWrite(const char* text, size_t len) {
  if (text != nullptr && len != 0) {
    probot::telemetry::writeBytes(text, len);
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

inline bool Install() {
  static constexpr pf::CoreOps kOps{
      pf::kCoreOpsAbiVersion, sizeof(pf::CoreOps), PROBOT_VERSION, PROBOT_VERSION_STRING,
      &Phase, &SelectedMode, &OutputsEnabled, &IsEStopped, &IsAttached, &MatchTimeSeconds,
      &PacketSeq, &RawGamepad,
      &BatteryVoltage,
      &ConsoleWrite,
      &DashPublish, &DashUnpublish, &DashSet, &DashSetProperties, &DashSetFlags, &DashHasSchema,
      &DashAddSchema, &DashDrainWrites, &DashRejectWrite, &DashStats,
  };
  bool ok = pf::InstallCoreOps(&kOps);
  probot::framework::install(probot::framework::Hooks{
      &pf::detail::Boot, &pf::detail::PreCycle, &pf::detail::PostCycle,
      &pf::detail::EmergencyStop});
  return ok;
}

/** True once the bridge is installed (evaluated during static initialization). */
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
