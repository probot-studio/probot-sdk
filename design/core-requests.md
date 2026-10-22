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

Status: delivered as draft PR #31 (see "P1 R8 delivered" below); prosdk uses it (CoreOps ABI 3).
Rumble is still open.

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

- `probot::requestStop()`: cooperative stop from code. The prosdk robot host would call it when the robot
  program halts (constructor or loop exception, unsupported period): today it only keeps prosdk's outputs
  disabled and prints the error, and the DS still shows the robot as enableable.
- `battery::currentAmps()` declared in every configuration (NaN when there is no sensor); brownout or
  low-battery flag.
- Optional persistence of DS-written tunables in NVS (robust tunables), separate namespace.
- `rslState()`: current state of the robot signal light / status LED core drives (WPILib `HAL_GetRSLState`).
- `commsDisableCount()`: number of times outputs were disabled because the DS link was lost (since boot;
  WPILib `HAL_GetCommsDisableCount`).

### R13. Dashboard schemas larger than a value

`addSchema()` refuses data longer than `PROBOT_DASH_MAX_VALUE` (default 1024). The protobuf file descriptors
prosdk sends are bigger: `wpimath/protobuf/geometry2d.proto` alone is about 1.3 KB, so with the defaults no
protobuf-typed topic (`proto:` type strings, e.g. a `TunableProtobuf` or telemetry logged as protobuf) can be
decoded on the DS page. Struct schemas are small and unaffected. Wanted: a separate schema byte limit
(e.g. `PROBOT_DASH_MAX_SCHEMA`, default 4096, still counted in `PROBOT_DASH_POOL_BYTES`), or schemas exempt
from the value limit. Workaround today: the sketch defines `PROBOT_DASH_MAX_VALUE` 2048 or more before
`#include <prosdk.h>`. prosdk reports a refused schema once on the console and keeps publishing the values.

Related, still open in R12: persistence of DS-written robust tunables. Core keeps no value across a reboot,
so prosdk publishes a robust tunable with its code default (upstream NT restores the last tuned value).

### R14. Compressed Driver Station page (flash)

`index_html.h` keeps the DS page as uncompressed HTML: `MAIN_page` is 138,439 B of flash in every sketch
(7 % of a prosdk robot sketch, more than any prosdk library; `design/size.md`). `gzip -9` makes it
39,421 B. Wanted: store the page gzip-compressed (generated from the HTML by a build step) and serve it
from `handleRoot` with `Content-Encoding: gzip` (every browser that runs the DS page accepts it). Saves
about 99 KB of flash per sketch; no protocol or API change.

## Status

### P0 delivered (2026-10-22): probot-core branch `feat/framework-api`, draft PR #26 to `dev`

Not merged until 0.4.0 reaches stable; prosdk builds against that branch (a read-only clone, never the
core agent's working tree). Final names:

- R1 `<probot/framework.hpp>`: `probot::framework::Hooks{boot, preCycle, postCycle, emergencyStop}`,
  `bool install(const Hooks&)` (once, before runtime start; static init OK), `bool installed()`.
  `PROBOT_FRAMEWORK` defined before `<probot.h>` makes all OpMode hooks optional.
  Difference from the request: `emergencyStop` runs AFTER the user task is deleted, on a fresh task on
  core 1, before the stop hook; together with that hook it must finish within `PROBOT_ESTOP_END_MS`
  (500 ms) or the chip reboots (PB-E305). No stall detection in STOPPED/TRANSITION.
  The registration must be referenced from the sketch (prosdk does it in header code compiled in the
  sketch TU, so the archive cannot drop it).
- R2 `<probot/ds.hpp>` `probot::ds`: `phase()` (STOPPED after e-stop), `selectedMode()`,
  `outputsEnabled()`, `isEStopped()`, `isAttached()`, `matchTimeSeconds()` (AUTO_RUN only, otherwise -1),
  `packetSeq()` (joystick frames and accepted mode/INIT/START/STOP), `rawGamepad(int port,
  io::GamepadSnapshot&)` (port 0 only; false if no frame). Also `probot::io::rawGamepad()`. The raw
  frame is cleared on timeout and owner release.
- R3 `PROBOT_USER_STACK` (default 8192; below 4096 is compile error PB-E107).
- R4 `<probot/api.hpp>` config-free (version, state, lifecycle, gamepad, joystick_api, telemetry, ds,
  framework). `<probot/version.hpp>`: `PROBOT_VERSION_MAJOR/MINOR/PATCH/STRING`, `PROBOT_VERSION`,
  `PROBOT_VERSION_OF(a,b,c)` (= a*10000+b*100+c). Gate in prosdk:
  `#if defined(PROBOT_VERSION) && PROBOT_VERSION >= PROBOT_VERSION_OF(0,5,0)`; until the release number is
  fixed, `#ifdef PROBOT_VERSION` identifies the branch.
- Details: core `API.md`, sections "Tipli durum" and "Framework katmani".

### P1 R5 delivered (2026-10-22): draft PR #27 (robot side) and #28 (DS page), stacked on #26

Header `<probot/dashboard/dashboard.hpp>` (also in `<probot/api.hpp>`), namespace `probot::dashboard`:
- `TopicId = uint16_t`, `kInvalid = 0xFFFF`; id = slot | gen<<8; removed ids are not reused before their 'R' is sent.
- `ValueType` 1..11 as requested (StringArray element = u16 LE len + bytes); `Flags` TELEMETRY=1, TUNABLE=2,
  MUTABLE=4, ROBUST=8, KEEP_DUPLICATES=16, PERSISTENT=32 (stored only, nothing persists yet).
- `publish(path, vt, typeString, propsJson, flags)` idempotent for same path+vt+type; `kInvalid` on type
  conflict, name/type > 255 bytes, full table/pool, or before `runtime_setup()`.
- `unpublish`, `bool setProperties`, `bool setFlags`, `bool set(id, data, len)` (raw LE bytes, len <= max value,
  equal bytes deduped unless KEEP_DUPLICATES), `bool hasSchema(name)`, `bool addSchema(name, type, data, len)`.
- `struct Write{id, vt, len, data}`; `size_t drainWrites(fn, ctx)` from our own task (preCycle); last-writer-wins
  per topic; every drained topic re-sends its current value; payloads already validated. Accept = `set()`
  the new value; refuse = `rejectWrite(id, reason)` with `WriteStatus` Ok=0, UnknownId=1, Immutable=2,
  TypeMismatch=3, DecodeError=4, QueueFull=5 (own codes from 16 up).
- `Stats stats()` {topics, maxTopics, bytesUsed, bytesMax, dropped}.
- Limit macros (sketch TU, PB-E108 on bad values): PROBOT_DASH_MAX_TOPICS 128, PROBOT_DASH_MAX_SCHEMAS 32,
  PROBOT_DASH_MAX_VALUE 1024, PROBOT_DASH_POOL_BYTES 32768, PROBOT_DASH_WRITE_QUEUE 2048, PROBOT_DASH_PERIOD_MS 50,
  PROBOT_DASH_MAX_BYTES_PER_TICK 8192. Tables allocated on first publish.
- Wire and DS page ("Veri" tab): as specified; widgets Selectable and Command done; PIDController, Alerts,
  Field2d, Mechanism2d pending.

Widget layouts sent to probot-core (2026-10-22):
- PIDController (upstream `PIDController.cpp`): tunable table `.type`="PIDController" with mutable doubles
  `p`, `i`, `d`, `izone`, `setpoint`; telemetry table `.type`="PIDController" with doubles `p`, `i`, `d`, `izone`,
  `setpoint`, `measurement`, `error`, `error derivative`, `previous error`, `total error`. Same relative name
  under `/Tunables/...` and `/Telemetry/...`.
- Alerts (prosdk-defined; WPILib 2027 has no telemetry layout, this is the WPILib 2025/2026 NT layout that
  AdvantageScope and Elastic know): telemetry table `/Telemetry/Alerts/<group>` with `.type`="Alerts" and
  string[] `errors` (level HIGH), `warnings` (MEDIUM), `infos` (LOW), active alerts only, newest first.

Widget child layout confirmed by probot-core (2026-10-22), matches upstream `Selectable.cpp` and `Command.cpp`:
- `X/.type`="Selectable": required `X/options` (string[]) and `X/selected` (string, TUNABLE|MUTABLE);
  optional `X/default`, `X/active` (strings, info). Missing or mistyped children fall back to the plain tree.
- `X/.type`="Command": required `X/running` (bool, TUNABLE|MUTABLE); optional `X/name`.
- Consequence for `ProbotTunableBackend`: a robust tunable (upstream `selected` is robust) is published as ONE
  topic at its own path (flag ROBUST), never split into `value`/`tune` sub-topics as the NT backend does.
- Wire correction (2026-10-22): 'V' = {u32 LE robot millis} then repeated {u16 id, u16 len, bytes}. prosdk uses
  the C++ API only, so this does not affect the backends.
- PIDController and Alerts widgets are in #28 with the layouts above. R5 remaining: Field2d, Mechanism2d,
  Scheduler (layout below, added 2026-10-22 with commandsv2). probot-core continues with R7.
- Scheduler (upstream `CommandScheduler.cpp` `LogTo`/`PublishTunable`): telemetry table `.type`="Scheduler"
  with `Names` (string[], running command names) and `Ids` (int64[], same order); tunable table
  `.type`="Scheduler" with `Names` and `Ids` (read-only, TUNABLE) and `Cancel` (int64[], TUNABLE|MUTABLE|ROBUST).
  Widget: the running commands, each with a cancel button that writes `[id]` to `Cancel` (the robot cancels
  those ids that are still scheduled and clears `Cancel`). Until it exists the DS shows the plain tree.

### P1 R7 delivered (2026-10-22): draft PR #29 (branch `feat/opmode-list`, stacked on #28)

Held until core 0.4.0 is on stable, like #26-#28. Final C++ API (`<probot/api.hpp>`, `probot::ds`):
- `struct OpModeOption { uint64_t id; OpMode mode; const char* name; const char* group; const char* description; }`:
  id non-zero and unique; mode AUTO|TELEOP|UTILITY; name non-empty, <=63; group optional, <=63; description
  optional, <=255.
- `bool setOpModeOptions(const OpModeOption*, size_t count)`: <=64 options, strings copied, false keeps the
  old list, callable any time.
- `uint64_t selectedOpModeId()`: DS choice, 0 = none; changes at auto expiry when TeleOp is preselected.
- `uint64_t activeOpModeId()`: latched at INIT; dispatch on this in hooks/preCycle.
- `bool utilityAvailable()`, `OpMode selectedMode()`.
- `OpMode::UTILITY = 2`; phases `UTILITY_INIT = 6`, `UTILITY_RUN = 7` (0-5 unchanged). Optional weak hooks
  utilityInit/InitLoop/Start/Loop/Stop. UTILITY is offered when utilityLoop() is defined or a UTILITY option is
  listed. No match timer; inputs on in UTILITY_RUN; e-stop calls utilityStop and the framework emergencyStop.
- Selection: a mode change picks the last-picked option of that mode if still listed, else the first. Changes
  only while idle. A selection dropped from the list is corrected while idle or at the next INIT. If UTILITY
  disappears while selected, the selection falls back to TeleOp.
- Wire (for reference): GET /opmodes; cmd=opmode&id=N; cmd=mode&val=utility; S frame and /getState add
  "opMode", "omv" (bumps on every set), "util".
- prosdk use (wave 3): `OpModeRobot` maps its registered opmodes to `OpModeOption` (id = WPILib opmode id),
  dispatches on `activeOpModeId()`, and maps UTILITY to WPILib's utility mode. The read-only core clone moves to
  #29 after wave 2 finishes (not mid-build).

### P1 R6 delivered (2026-10-22): draft PR #30 (branch `feat/loop-rate`, stacked on #29)

Held like #26-#29. Final API (config-free, `<probot/api.hpp>`, namespace `probot`):
- `bool setLoopPeriodMs(uint32_t ms)`: 1-1000, effective from the next pass; 0 restores USER_LOOP_PERIOD_MS;
  false for anything else. Works from static init.
- `uint32_t loopPeriodMs()`; `uint32_t loopOverruns()` (since boot).
- Semantics: passes start on a fixed tick grid (vTaskDelayUntil style). A late pass yields 1 tick and restarts the
  schedule (no catch-up bursts). Passes that ran a one-shot init/start/stop are not counted as overruns. The sleep
  never counts toward the stall deadline, so periods longer than PROBOT_LOOP_DEADLINE_MS are fine.
- Wire: /health and S gain "loopMs" and "overruns"; the DS Debug panel shows them.
- Breaking: USER_LOOP_PERIOD_MS outside 1-1000 is now PB-E109 (0 used to be allowed).
- prosdk use (wave 3): `TimedRobot(period)` calls `setLoopPeriodMs(round(period in ms))` and keeps upstream
  `AddPeriodic` callbacks on its own schedule inside preCycle (they fire on the first pass at or after their due
  time). Loop overruns feed the Watchdog/`loop time overrun` warning. Sub-millisecond periods are rejected with a
  reported error.

### P1 R8 delivered (2026-10-22): draft PR #31 (branch `feat/multi-gamepad`, 4484a97, stacked on #30 71a5c19)

Held like #26-#30. API:
- `probot::io::gamepad(int port = 0)`, `io::rawGamepad(int port = 0)`; `ds::rawGamepad(0..5, out)` returns true only for a
  port that really received a frame. `joystick_api::makeForPort(int port)`.
- `Joystick::getRawButtonValue(i)` 0..1 (analog), `getName(char*, size_t)` (<=63 bytes UTF-8, false when no gamepad),
  `isStandardMapping()`. Trigger axes are analog. `getPOV` is 8-way from the D-pad buttons (opposite directions
  cancel); `getDpad*` is also true on diagonals.
- Constants `kMaxGamepads = 6`, `kMaxGamepadName = 63`.
- DS ports are sticky: port 0 is the selected source; others take the lowest free of 1-5 and keep it until unplugged
  (no shifting); a freed port is held for 600 ms. No rumble.
- Wire (reference): 'J' byte 3 = port plus a value block; new 'N' frame (names). See core API.md.
- Amend 4484a97 (same names): trigger axes get the stick deadzone (`getRawButtonValue` stays raw); when the DS
  keyboard/touch source is port 0, the selected gamepad takes a port 1-5; a gamepad waiting for a port never moves
  to a higher port.
- prosdk use: HAL DS joysticks 0-5 from `ds::rawGamepad(port)`, analog triggers, 8-way POV, joystick name and
  standard-mapping flag into the HAL joystick descriptor (wired: CoreOps ABI 3, `CoreGamepad` carries the trigger
  values, POV, name and flag; prosdk/platform/hal/CoreJoystick.cpp; CI examples job builds against
  `feat/multi-gamepad`). With a core without R8 the bridge keeps the R2 behaviour (port 0, digital triggers).

### P1 R9 delivered (2026-10-22): draft PR #33 (branch `feat/console`, rebased 301f190, stacked on #31 4484a97)

API preview from probot-core (config-free, `<probot/api.hpp>`): `probot::console::info/warning/error(fmt, ...)`,
`console::write(Level, text, len)`, `console::print(Level, text)`; `enum Level { Info = 0, Warning = 1, Error = 2 }`.
Callable from any task (not an ISR). Lines up to 512 bytes; write() splits longer text at newlines, never inside a
UTF-8 character. prosdk use: ConsoleWrite moves from `telemetry::writeBytes` to `console::write` with a level (ReportError -> Error,
ReportWarning/MathShared warnings -> Warning, everything else Info). Wired once the PR exists. Scheduler widget was
added to core's R5 remainder (after R12/R13).

### P1 R10 delivered (2026-10-22): draft PR #34 (branch `feat/wall-clock`, 29466db, stacked on #33 301f190)

The DS page sends the browser's UTC time on every WebSocket open; the robot calls settimeofday. The first valid time
always sets the clock; later ones only while phase STOPPED and status STOP (never mid-match, including the
auto->teleop TRANSITION). `probot::ds::systemTimeValid()`; `time()`/`gettimeofday()` give UTC (no TZ set).
prosdk use: HAL_GetSystemTimeValid -> CoreOps systemTimeValid (replaces the time > 2024-01-01 heuristic), so
DataLogManager gets real file names and systemTime entries. Wired after wave 5 together with R9.
