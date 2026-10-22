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
// dashboard, no opmode selected, loop period 20 ms. Every entry is always callable; GetCoreOps()
// never returns a table with null pointers. Entries the sketch's probot-core does not provide
// (R6 loop rate, R7 opmode list: compiled out by CoreBridge.hpp) are left null by the bridge and
// get these defaults too. Without R8 (more gamepads) the bridge reports port 0 only and fills the
// CoreGamepad fields of ABI 3 the way core 0.4.0 behaved (trigger buttons as 0/1, 4-way POV,
// the mapping profile's name, standard mapping). Without R9 (leveled console) the console text
// goes to consoleWrite (core's telemetry ring) without its level; without R10 (wall clock) the
// system time counts as valid once the clock is past 2024-01-01 (SystemTimeValid()). Without R12
// (small items) RequestStop() stops nothing, the RSL state and the comms-disable count are unknown
// and core's low-voltage threshold is not set; without R13 (schema size limit) the dashboard
// reports a refused schema against PROBOT_DASH_MAX_VALUE; without R15 (grid restarts) the robot
// host re-aligns its schedule only when loopOverruns changes.
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
inline constexpr uint32_t kCoreOpsAbiVersion = 7;

/** probot-core robot::Phase values (probot/robot/state.hpp). */
enum CorePhase : uint8_t {
  kCorePhaseStopped = 0,
  kCorePhaseAutoInit = 1,
  kCorePhaseAutoRun = 2,
  kCorePhaseTeleopInit = 3,
  kCorePhaseTeleopRun = 4,
  kCorePhaseTransition = 5,
  kCorePhaseUtilityInit = 6,  // probot-core R7
  kCorePhaseUtilityRun = 7,   // probot-core R7
};

/** probot-core robot::OpMode values. */
enum CoreMode : uint8_t {
  kCoreModeAuto = 0,
  kCoreModeTeleop = 1,
  kCoreModeUtility = 2,  // probot-core R7
};

/** True for the INIT phases (an opmode is initialized, outputs off). */
constexpr bool IsCoreInitPhase(uint8_t phase) {
  return phase == kCorePhaseAutoInit || phase == kCorePhaseTeleopInit ||
         phase == kCorePhaseUtilityInit;
}

/** True for the RUN phases. */
constexpr bool IsCoreRunPhase(uint8_t phase) {
  return phase == kCorePhaseAutoRun || phase == kCorePhaseTeleopRun ||
         phase == kCorePhaseUtilityRun;
}

/** probot-core limits of the opmode list (robot::OpModeRegistry, R7). */
inline constexpr size_t kCoreOpModeMaxOptions = 64;
inline constexpr size_t kCoreOpModeMaxName = 63;
inline constexpr size_t kCoreOpModeMaxGroup = 63;
inline constexpr size_t kCoreOpModeMaxDescription = 255;

/**
 * One opmode offered on the Driver Station (prosdk-owned copy of probot-core's
 * ds::OpModeOption; CoreBridge.hpp converts it). Strings are null-terminated UTF-8 and only
 * need to live for the duration of the call (core copies them).
 */
struct CoreOpModeOption {
  uint64_t id;              // non-zero, unique within one list
  uint8_t mode;             // CoreMode
  const char* name;         // non-empty, <= kCoreOpModeMaxName bytes
  const char* group;        // optional (nullptr or ""), <= kCoreOpModeMaxGroup bytes
  const char* description;  // optional (nullptr or ""), <= kCoreOpModeMaxDescription bytes
};

/** probot-core loop period limits and default (probot::setLoopPeriodMs, R6). */
inline constexpr uint32_t kCoreLoopPeriodMinMs = 1;
inline constexpr uint32_t kCoreLoopPeriodMaxMs = 1000;
inline constexpr uint32_t kCoreLoopPeriodDefaultMs = 20;

/** Size of the arrays in CoreGamepad (core's GamepadSnapshot holds 20 axes and 20 buttons). */
inline constexpr int kCoreGamepadMaxAxes = 20;
inline constexpr int kCoreGamepadMaxButtons = 20;

/** Driver Station gamepad ports (probot-core R8: io::kMaxGamepads; WPILib HAL_MAX_JOYSTICKS). */
inline constexpr int kCoreMaxGamepads = 6;

/** Longest gamepad name in bytes, UTF-8, without the terminator (core io::kMaxGamepadName). */
inline constexpr size_t kCoreGamepadMaxName = 63;

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
  int8_t leftTrigger, rightTrigger;  // buttons; analog values in CoreGamepad (ABI 3)
  int8_t back, start, options;
  int8_t leftStick, rightStick;
  uint8_t dpadFromAxis;  // 1: the D-pad is one hat axis (dpadAxis), else four buttons
  int8_t dpadAxis;
  int8_t dpadUp, dpadDown, dpadLeft, dpadRight;
};

/**
 * One raw gamepad snapshot (browser order) plus the mapping that was active when it was read.
 * axisCount and buttonCount are 0 when core neutralized the port (frame older than core's input
 * timeout, Driver Station owner released): the port then has no data.
 */
struct CoreGamepad {
  uint32_t timestampMs;  // core millis() of the packet
  uint32_t seq;          // core gamepad sequence number, never 0 for a valid snapshot
  uint8_t axisCount;     // <= kCoreGamepadMaxAxes
  uint8_t buttonCount;   // <= kCoreGamepadMaxButtons
  float axes[kCoreGamepadMaxAxes];
  uint32_t buttons;      // bit i = raw button i
  CoreGamepadMapping mapping;

  // ---- ABI 3 (probot-core R8) ----
  // Computed on the sketch side by core's joystick API (io::joystick_api::Joystick) on this
  // snapshot, so they are exactly what core's own API reports.
  float leftTrigger;        // getLeftTriggerAxis(): 0..1, core's stick deadzone applied
  float rightTrigger;       // getRightTriggerAxis()
  uint8_t leftTriggerRaw;   // the trigger button's analog value 0..255 (no deadzone; 255/0 from
  uint8_t rightTriggerRaw;  // the pressed state when the Driver Station sent no value)
  int16_t pov;              // getPOV(): -1 centered, else 0..315 degrees in steps of 45
  uint8_t standardMapping;  // 1: the browser reports the W3C standard mapping
  char name[kCoreGamepadMaxName + 1];  // the gamepad's name, UTF-8; "" when core has none
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
  // ports 0..kCoreMaxGamepads-1; false: no Driver Station frame ever arrived on that port
  // (ABI 3: the CoreGamepad layout above)
  bool (*rawGamepad)(int port, CoreGamepad* out);

  // robot state
  float (*batteryVoltage)();  // volts; 0 until a battery reading exists (no sensor configured)

  // console: text for the Driver Station console (core telemetry ring). With probot-core R9 the
  // archive uses consoleWriteLevel (ABI 4) instead.
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

  // ---- ABI 2 ----
  // user loop rate (probot-core R6: probot::setLoopPeriodMs/loopPeriodMs/loopOverruns)
  bool (*setLoopPeriodMs)(uint32_t ms);  // 1..1000, 0 = core default; false otherwise
  uint32_t (*loopPeriodMs)();
  uint32_t (*loopOverruns)();  // since boot

  // opmode list and UTILITY mode (probot-core R7: probot::ds)
  bool (*setOpModeOptions)(const CoreOpModeOption* options, size_t count);  // within core limits
  uint64_t (*selectedOpModeId)();  // Driver Station choice, 0 = none
  uint64_t (*activeOpModeId)();    // latched at INIT, 0 = none
  bool (*utilityAvailable)();

  // ---- ABI 4 ----
  // leveled Driver Station console (probot-core R9: probot::console::write). level is a
  // ConsoleLevel value (0 info, 1 warning, 2 error); core keeps one entry per call (a trailing
  // newline dropped), splits text over 512 bytes at newlines and never inside a UTF-8 character.
  // Any task, not an ISR. Null (core without R9): the archive calls consoleWrite.
  void (*consoleWriteLevel)(uint8_t level, const char* text, size_t len);

  // wall clock (probot-core R10: probot::ds::systemTimeValid): true once a Driver Station set the
  // robot's clock since boot. Null (core without R10): the archive's 2024-01-01 heuristic.
  bool (*systemTimeValid)();

  // ---- ABI 5 ----
  // small items (probot-core R12). Null (core without R12): the archive's fallbacks (no stop, RSL
  // state and comms-disable count unknown, no low-voltage threshold).
  // probot::requestStop(): stops the active OpMode like the Driver Station STOP button
  // (cooperative; cancels a pending INIT or START). false: nothing to stop, a stop already in
  // progress, or e-stopped. Any task.
  bool (*requestStop)();
  // ds::rslState(): the robot signal light is lit (blinking in a RUN phase that may move the
  // robot, solid on otherwise; false before core's runtime started)
  bool (*rslState)();
  // ds::commsDisableCount(): times a lost Driver Station link stopped an enabled (START) robot
  // since boot (PB-E303)
  uint32_t (*commsDisableCount)();
  // io::battery::setLowVoltage(): threshold of core's isLowVoltage() flag in volts, 0 = off (core's
  // default). false (threshold unchanged) for a negative, NaN or >= 100 V value.
  bool (*setLowVoltage)(float volts);

  // dashboard schema size limit (probot-core R13): the store's per-schema byte limit as
  // configured (PROBOT_DASH_MAX_SCHEMA_BYTES, default 4096), 0 before core's runtime_setup().
  // Null (core without R13): schemas are limited by PROBOT_DASH_MAX_VALUE.
  uint32_t (*dashMaxSchemaBytes)();

  // ---- ABI 6 ----
  // dashboard persistence (probot-core R12 part 2: probot::dashboard::clearPersisted()). Its
  // presence also says that core keeps the values of PERSISTENT topics across reboots (NVS): a
  // publish() that creates the topic queues the stored value as a write, delivered by the next
  // dashDrainWrites(); dashRejectWrite() on that write erases the record. clearPersisted() erases
  // every stored value (writes flash: call it while the robot is stopped); false without storage
  // or when the erase failed. Null (core without persistence): PERSISTENT is not used.
  bool (*dashClearPersisted)();

  // ---- ABI 7 ----
  // pass grid restarts (probot-core R15: probot::loopGridRestarts()): times core's pass schedule
  // restarted since boot, i.e. every pass that ended late, including the one-shot INIT/START/STOP
  // passes that loopOverruns leaves out. Core steps it after the late pass, before its sleep, so
  // the next pass (and its pre-cycle hook) already sees the new value; the passes from that one
  // on start on a new grid. A period change is not counted. Null (core without R15): the robot
  // host re-aligns only when loopOverruns changes.
  uint32_t (*loopGridRestarts)();
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

/**
 * Called once by the bridge with the result of probot::framework::install. On false (another
 * framework took core's hooks) it prints an error to the console; prosdk's per-cycle work (Driver
 * Station pump, output gate pre-cycle, robot loop) then never runs. Safe during static
 * initialization.
 */
void NoteFrameworkInstall(bool installed) noexcept;

/** True once the bridge installed prosdk's hooks into probot-core (false on the host). */
bool FrameworkInstalled() noexcept;

// ---- robot framework accessors (probot-core R6 loop rate, R7 opmode list) ----
//
// Thin wrappers over the CoreOps table for the robot framework (TimedRobot, OpModeRobot, the HAL
// Driver Station backend). All of them may be called from any task (not from an ISR or an
// emergency-stop callback) and at any time, including static initialization.
//
// Without probot-core (host build, tests, a sketch without <prosdk.h>) they behave like core
// with nothing attached: SetLoopPeriodMs validates and stores the period (LoopPeriodMs returns
// it, default 20 ms), LoopOverruns is 0, SetOpModeOptions validates and accepts, no opmode is
// selected or active and UTILITY is not available. With a probot-core that lacks R6 (or R7),
// SetLoopPeriodMs (SetOpModeOptions) returns false.

/**
 * Sets core's user-loop period in ms, effective from the next pass (fixed-rate schedule: passes
 * start on a tick grid; a late pass is counted as an overrun and restarts the schedule, no
 * catch-up). 1..1000 ms, or 0 for the sketch's USER_LOOP_PERIOD_MS. Returns false (and changes
 * nothing) for other values.
 */
bool SetLoopPeriodMs(uint32_t ms) noexcept;

/** The effective user-loop period in ms. */
uint32_t LoopPeriodMs() noexcept;

/** User-loop passes since boot that ended after their deadline (one-shot hook passes excluded). */
uint32_t LoopOverruns() noexcept;

/**
 * Times core's pass schedule restarted since boot (probot-core R15 loopGridRestarts(): every late
 * pass, one-shot hook passes included). Returns false (and leaves *count unchanged) without core
 * or with a core without R15.
 */
bool GetLoopGridRestarts(uint32_t* count) noexcept;

/**
 * Replaces the opmode list the Driver Station offers. Options that core would refuse are fixed
 * or dropped with a console warning instead of failing the whole list: id 0, an unknown mode, a
 * duplicate id or an empty name drops the option; name/group/description longer than core's
 * limits are truncated (at a UTF-8 character boundary); options past kCoreOpModeMaxOptions are
 * dropped. Strings are copied. nullptr/0 clears the list. Returns false when core refused the
 * list (out of memory, or no R7 support); the previous list then stays. Allocates; not
 * real-time.
 */
bool SetOpModeOptions(const CoreOpModeOption* options, size_t count);

/**
 * The opmode id picked on the Driver Station (0 = none). Changes only while the robot is idle
 * (STOPPED/TRANSITION), except that autonomous expiry may preselect a teleop option.
 */
uint64_t SelectedOpModeId() noexcept;

/** The opmode id latched by the last INIT (0 before the first INIT or when none was selected). */
uint64_t ActiveOpModeId() noexcept;

/** True when the Driver Station offers UTILITY (a UTILITY option is listed, or the sketch
 * defines core's utilityLoop() hook). */
bool UtilityAvailable() noexcept;

/**
 * True when the system clock (std::chrono::system_clock, time()) holds real UTC time
 * (HAL_GetSystemTimeValid on the robot). With probot-core R10: a Driver Station set the clock
 * since boot (core's ds::systemTimeValid()). Without core or without R10: the clock is past
 * 2024-01-01 (something such as the sketch's SNTP or settimeofday set it).
 */
bool SystemTimeValid() noexcept;

// ---- probot-core R12 accessors ----
//
// Any task (not an ISR or an emergency-stop callback), any time. Without core (host build, tests)
// or with a core without R12 they report "not available" and change nothing.

/**
 * Stops the active opmode as if the Driver Station pressed STOP (core's probot::requestStop():
 * cooperative, cancels a pending INIT or START). Returns false when there was nothing to stop, a
 * stop was already in progress, the robot is e-stopped, or core has no R12.
 */
bool RequestStop() noexcept;

/**
 * Whether the robot signal light is lit (HAL_GetRSLState): blinking while a RUN phase may move
 * the robot, solid on otherwise. Returns false (and leaves *lit unchanged) when core has no R12.
 */
bool GetRslState(bool* lit) noexcept;

/**
 * Times a lost Driver Station link stopped the enabled robot since boot (HAL_GetCommsDisableCount).
 * Returns false (and leaves *count unchanged) when core has no R12.
 */
bool GetCommsDisableCount(uint32_t* count) noexcept;

/**
 * Sets the threshold of core's low-voltage flag (io::battery::setLowVoltage, volts, 0 = off).
 * Returns false when core refused the value or has no R12.
 */
bool SetLowVoltage(double volts) noexcept;

// ---- probot-core R12 dashboard persistence ----

/**
 * True when probot-core keeps PERSISTENT dashboard values across reboots (CoreOps ABI 6
 * dashClearPersisted present). False without core or with a core without persistence.
 */
bool DashPersistenceSupported() noexcept;

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

/**
 * The monotonic time (HAL_GetMonotonicTime, ns) at which core's current loop pass started: taken
 * when core's pre-cycle hook is entered, before late boot callbacks and the pre-cycle callbacks,
 * so it does not include their run time. 0 before the first pass. Core's user task only.
 */
int64_t CycleStartTime() noexcept;

namespace detail {
// Entry points for probot::framework::Hooks (installed by CoreBridge.hpp).
void Boot();
void PreCycle();
void PostCycle();
void EmergencyStop();
}  // namespace detail

}  // namespace prosdk::platform
