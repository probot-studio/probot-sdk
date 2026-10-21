# What prosdk needs from probot-core

From: probot-sdk agent (prosdk = WPILib 2027 C++ API on ESP32-S3, hosted on probot-core).
Spec behind it: `design/PORT.md` section 8; details in `design/reports/core.md` (sections 11-12) and
`design/reports/telemetry_tunables.md` (section 7).

Naming below is a proposal. The core owner picks the final names and tells probot-sdk. prosdk compiles
each integration out (`#if`) until the matching core version exists, so nothing here blocks core releases.

## P0: needed for the robot framework to work at all

### R1. Framework registration (all phases, sdk-owned loop)

prosdk drives WPILib's robot loop from core's user task. Core 0.4.0 calls only the active mode's user
hook and nothing in STOPPED/TRANSITION, so WPILib `RobotPeriodic`, disabled-mode logic,
`CommandScheduler::Run()` while disabled (which cancels commands on disable) and tunables cannot work.

```cpp
namespace probot::framework {
  struct Hooks {
    void (*boot)();          // once, on the user task, before the first cycle
    void (*preCycle)();      // every user-task iteration, ALL phases, before the user hook
    void (*postCycle)();     // every user-task iteration, ALL phases, after the user hook
    void (*emergencyStop)(); // on the e-stop path, before the user task is deleted; short, non-blocking
                             // (prosdk zeroes PWM/motor outputs here)
  };
  bool install(const Hooks&);   // callable during static initialization (before setup())
}
```

- Any field may be null. Hooks run under the same stall heartbeat as user hooks.
- When a framework is installed, the user hooks become optional (no undefined-reference link error
  if the sketch defines none of them). Sketches that do define hooks keep working unchanged, and the
  framework cycle runs around them. Mechanism is up to core (for example a `PROBOT_FRAMEWORK` macro
  that prosdk defines before including `<probot.h>`).

### R2. Typed status and ungated input (config-free header)

```cpp
namespace probot::ds {
  Phase    phase();              // fresh also after e-stop
  OpMode   selectedMode();
  bool     outputsEnabled();     // RUN && START && !estop && !stall  (actuators allowed)
  bool     isEStopped();
  bool     isAttached();         // a DS client owns the robot
  float    matchTimeSeconds();   // remaining time in the current period, auto and teleop; <0 if unknown
  uint32_t packetSeq();          // increments on every DS control/joystick packet ("new data" event)
  bool     rawGamepad(int port, GamepadSnapshot& out);  // last received frame in ANY phase; false if none
}
```

The gated gamepad stays as it is for actuator safety. WPILib code reads buttons while disabled
(auto chooser, bindings), which is why the raw frame is needed.

### R3. Configurable user task stack

`PROBOT_USER_STACK` (default stays 8192). prosdk sets 24576 before including `<probot.h>`.

### R4. Multi-TU-safe API headers and a version macro

- A config-free header (for example `<probot/api.hpp>`) that any .cpp can include without the WiFi
  macros: state, `probot::ds`, `probot::framework`, telemetry print, gamepad types, dashboard store.
  The WiFi/runtime instantiation stays in `<probot.h>` (sketch TU only).
- `PROBOT_VERSION_MAJOR`, `PROBOT_VERSION_MINOR`, `PROBOT_VERSION_PATCH` macros.

## P1: needed for full WPILib feature parity

### R5. Dashboard topic store + WebSocket protocol + DS widgets (telemetry and tunables)

WPILib 2027 replaced SmartDashboard/Sendable with `telemetry` (robot to dashboard, push) and
`tunables` (dashboard to robot, edited values such as PID gains, auto chooser `Selectable`,
"run command" buttons). prosdk implements both backends on top of a core topic store. The full spec,
including the value encoding, frame layouts and limits, is in `design/reports/telemetry_tunables.md`
section 7 items A, B, D. Summary:

- `probot::dashboard::publish/unpublish/set/setProperties/setFlags/addSchema/hasSchema/drainWrites/stats`,
  a fixed-size thread-safe slot table (limits by macro: max topics, max value bytes, pool bytes,
  write queue bytes). Writes from the DS are queued on core 0 and drained on the user task.
- WS frames robot to DS: announce `'A'`, value batch `'V'` (dirty only, ~20 Hz, byte budget per tick so
  heartbeats never starve), remove `'R'`, schema `'C'`, write ack `'K'`. DS to robot: tune write `'W'`,
  resync `'Q'`. The 47-byte cap stays for `'J'`/`'P'` only.
- DS page: topic tree (`/Telemetry` read-only, `/Tunables` editable), editors per type, generic struct
  decoder driven by the published schema (covers Pose2d etc. without per-type code), and typed widgets
  chosen by the sibling `.type` topic: `Selectable` dropdown, `Command` run/cancel button,
  `PIDController` gains, `Alerts` list, then `Field2d` and `Mechanism2d` canvases.

### R6. Fixed-rate loop

`vTaskDelayUntil` instead of a delay after the work; period settable at runtime (default 20 ms);
overruns counted and exposed.

### R7. OpMode list and utility mode

WPILib 2027 `OpModeRobot` registers named opmodes (autonomous / teleoperated / utility, with groups).
Needed: the robot publishes its opmode list, the DS shows a selector, and the selected opmode id is
part of the state. `UTILITY` as a third mode next to AUTO/TELEOP (replaces WPILib's old test mode).

### R8. More gamepad data

Up to 6 gamepads (port index in the reserved byte of `'J'`), analog triggers, 8-way POV, gamepad
name/type string. Rumble (robot to DS, Gamepad `vibrationActuator`) is nice to have.

### R9. Console

Larger configurable telemetry text buffer, levels (error/warning/info), no 127-char printf limit.
WPILib `ReportError`/`ReportWarning` and alerts route here.

### R10. Wall clock

Set the robot's wall clock (`settimeofday`) from the browser on DS connect, so data logs get real
timestamps and file names.

### R11. File download

DS lists and downloads files that prosdk writes to LittleFS/SD (WPILib DataLog `.wpilog`). A generic
`registerUri()` extension point would also do.

### R12. Small items

- `probot::requestStop()`: cooperative stop from code.
- `battery::currentAmps()` declared in every configuration (NaN when there is no sensor); brownout or
  low-battery flag.
- Optional persistence of DS-written tunables in NVS (robust tunables), separate namespace.
