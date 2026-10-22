// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// The probot-core bridge, archive side (design/PORT.md section 8).
//
// probot-core is header-only and its API is inline C++ that only exists inside the user's sketch
// (where <probot.h> is visible). The precompiled archive therefore never includes a core header:
// everything prosdk needs from core goes through CoreOps, a table of plain function pointers with
// a C-compatible layout. The sketch-side half (prosdk/probot/CoreBridge.hpp, included by
// <prosdk.h>) fills the table from core's API and installs core's framework hooks, both at static
// initialization time.
//
// Until a table is installed (host build, unit tests, a sketch without <prosdk.h>) the defaults
// are safe: phase STOPPED, outputs disabled, not attached, no gamepad, no battery reading, no
// dashboard. Every entry is always callable; GetCoreOps() never returns a table with null
// pointers.
//
// ABI rules: only append fields, bump kCoreOpsAbiVersion on any change of meaning or layout. The
// archive refuses a table whose abiVersion differs or whose size is smaller than its own.
//
// Callbacks: the archive registers its per-cycle work here (Driver Station pump, output gate,
// later the robot loop). Boot/PreCycle/PostCycle run on core's user task (core 1, between user
// loop iterations); EmergencyStop runs on a fresh core-1 task after core deleted the user task
// and must not block (no mutex, no allocation, no logging).

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace prosdk::platform {

/** Layout/meaning version of CoreOps; both sides must agree. */
inline constexpr uint32_t kCoreOpsAbiVersion = 1;

/** probot-core robot::Phase values (probot/robot/state.hpp). */
enum CorePhase : uint8_t {
  kCorePhaseStopped = 0,
  kCorePhaseAutoInit = 1,
  kCorePhaseAutoRun = 2,
  kCorePhaseTeleopInit = 3,
  kCorePhaseTeleopRun = 4,
  kCorePhaseTransition = 5,
};

/** probot-core robot::OpMode values. */
enum CoreMode : uint8_t {
  kCoreModeAuto = 0,
  kCoreModeTeleop = 1,
};

/** Size of the arrays in CoreGamepad (core's GamepadSnapshot holds 20 axes and 20 buttons). */
inline constexpr int kCoreGamepadMaxAxes = 20;
inline constexpr int kCoreGamepadMaxButtons = 20;

/**
 * Where each logical control is in the raw gamepad arrays (copy of core's active
 * joystick_mapping::MappingProfile). -1 means "not available".
 */
struct CoreGamepadMapping {
  const char* name;  // static string owned by core, never null when filled
  int8_t leftX, leftY, rightX, rightY;
  // core inverts these axes for its own "up is positive" API; raw values are W3C (up negative)
  uint8_t invertLeftY, invertRightY;
  int8_t a, b, x, y;
  int8_t leftBumper, rightBumper;
  int8_t leftTrigger, rightTrigger;  // buttons (core has no analog trigger values yet)
  int8_t back, start, options;
  int8_t leftStick, rightStick;
  uint8_t dpadFromAxis;  // 1: the D-pad is one hat axis (dpadAxis), else four buttons
  int8_t dpadAxis;
  int8_t dpadUp, dpadDown, dpadLeft, dpadRight;
};

/** One raw gamepad snapshot (browser order) plus the mapping that was active when it was read. */
struct CoreGamepad {
  uint32_t timestampMs;  // core millis() of the packet
  uint32_t seq;          // core gamepad sequence number, never 0 for a valid snapshot
  uint8_t axisCount;     // <= kCoreGamepadMaxAxes
  uint8_t buttonCount;   // <= kCoreGamepadMaxButtons
  float axes[kCoreGamepadMaxAxes];
  uint32_t buttons;      // bit i = raw button i
  CoreGamepadMapping mapping;
};

/** One Driver Station write to a dashboard topic (core dashboard::Write). */
struct CoreDashboardWrite {
  uint16_t id;
  uint8_t valueType;
  uint16_t len;
  const uint8_t* data;
};

/** Dashboard store usage (core dashboard::Stats). */
struct CoreDashboardStats {
  uint16_t topics;
  uint16_t maxTopics;
  uint32_t bytesUsed;
  uint32_t bytesMax;
  uint32_t dropped;
};

/** Invalid dashboard topic id (core dashboard::kInvalid). */
inline constexpr uint16_t kCoreDashboardInvalid = 0xFFFF;

/** Everything prosdk calls in probot-core. All pointers are non-null in an installed table. */
struct CoreOps {
  uint32_t abiVersion;  // kCoreOpsAbiVersion
  uint32_t size;        // sizeof(CoreOps) as the sketch saw it
  uint32_t coreVersion;  // PROBOT_VERSION (major * 10000 + minor * 100 + patch)
  const char* coreVersionString;

  // Driver Station state (probot::ds)
  uint8_t (*phase)();         // CorePhase; STOPPED after an e-stop
  uint8_t (*selectedMode)();  // CoreMode chosen on the Driver Station
  bool (*outputsEnabled)();   // RUN phase, started, no deadline miss, not e-stopped
  bool (*isEStopped)();       // latched until reboot
  bool (*isAttached)();       // a Driver Station client is connected
  float (*matchTimeSeconds)();  // remaining autonomous time, -1 when unknown
  uint32_t (*packetSeq)();      // increments with every Driver Station packet
  bool (*rawGamepad)(int port, CoreGamepad* out);  // false: no data on that port

  // robot state
  float (*batteryVoltage)();  // volts; 0 until a battery reading exists (no sensor configured)

  // console: text for the Driver Station console (core telemetry ring)
  void (*consoleWrite)(const char* text, size_t len);

  // dashboard topic store (probot::dashboard)
  uint16_t (*dashPublish)(const char* path, uint8_t valueType, const char* typeString,
                          const char* propertiesJson, uint8_t flags);
  void (*dashUnpublish)(uint16_t id);
  bool (*dashSet)(uint16_t id, const void* data, size_t len);
  bool (*dashSetProperties)(uint16_t id, const char* propertiesJson);
  bool (*dashSetFlags)(uint16_t id, uint8_t flags);
  bool (*dashHasSchema)(const char* name);
  bool (*dashAddSchema)(const char* name, const char* type, const void* data, size_t len);
  size_t (*dashDrainWrites)(void (*fn)(const CoreDashboardWrite& write, void* ctx), void* ctx);
  void (*dashRejectWrite)(uint16_t id, uint8_t reason);
  void (*dashStats)(CoreDashboardStats* out);
};

/**
 * Installs the table filled by the sketch-side bridge (copied; the pointer need not outlive the
 * call). Only the first valid table is taken. Returns false (and keeps the defaults) for a second
 * call, a null table or an ABI mismatch. Safe during static initialization.
 */
bool InstallCoreOps(const CoreOps* ops) noexcept;

/** True once a table was installed (i.e. the sketch includes <prosdk.h> with probot-core). */
bool HasCoreOps() noexcept;

/** The installed table, or the safe defaults. Never null entries. */
const CoreOps& GetCoreOps() noexcept;

/**
 * Marks GPIO pins that probot-core owns (status LED, RSL, enable line, battery ADC, battery I2C)
 * so the HAL refuses them. Called by the bridge from every sketch translation unit; the reserved
 * set is the union. Safe during static initialization. Negative or out-of-range pins are ignored.
 */
void ReserveCorePins(const int* pins, size_t count) noexcept;

/** Bit mask of the GPIO pins reserved by ReserveCorePins (bit n = GPIO n). */
uint64_t GetCoreReservedPins() noexcept;

/** Where a cycle callback runs relative to core's user loop iteration. */
enum class CyclePhase : uint8_t { kPre = 0, kPost = 1 };

/** Callback type for boot and cycle callbacks. */
using CoreCallback = void (*)(void* context);

/** Callback type for emergency-stop callbacks: must be fast and lock-free. */
using EmergencyStopCallback = void (*)();

/** Maximum number of callbacks of each kind. */
inline constexpr int kMaxCoreCallbacks = 8;

/**
 * Registers a callback run by core's user task before (kPre) or after (kPost) every loop
 * iteration, in every phase. Returns false when the table is full. Callbacks run in registration
 * order and must not block for long (they share core's loop deadline).
 */
bool AddCycleCallback(CyclePhase phase, CoreCallback callback, void* context) noexcept;

/** Unregisters a cycle callback (same callback and context). */
void RemoveCycleCallback(CyclePhase phase, CoreCallback callback, void* context) noexcept;

/**
 * Registers a callback run once on core's user task before its first loop iteration. If boot
 * already happened, the callback runs at the start of the next cycle instead.
 */
bool AddBootCallback(CoreCallback callback, void* context) noexcept;

/**
 * Registers an emergency-stop callback. It runs on core's e-stop task (after the user task was
 * deleted) and must only touch hardware registers and atomics: no mutex, no allocation, no
 * logging, no blocking.
 */
bool AddEmergencyStopCallback(EmergencyStopCallback callback) noexcept;

/** True after core reported an emergency stop through EmergencyStop() (latched). */
bool EmergencyStopped() noexcept;

namespace detail {
// Entry points for probot::framework::Hooks (installed by CoreBridge.hpp).
void Boot();
void PreCycle();
void PostCycle();
void EmergencyStop();
}  // namespace detail

}  // namespace prosdk::platform
