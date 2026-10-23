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

Status: delivered as draft PR #38 (`feat/ds-files` 352b555 at delivery, eab956a after the rebase into
the pinned 15defc1; agreed design under "R12 part 2 and R11
design" below; the rebase added `share(name, vfsDir)` for ESP-IDF VFS mounts, `unshare(name)`, replacing a
share by name, and the `PROBOT_HAS_FILES_SHARE` macro); in the pinned core since 2026-10-22, wired
(CoreOps ABI 8), see "P1 R11 wired" below.

### R12. Small items

- `probot::requestStop()`: cooperative stop from code. The prosdk robot host would call it when the robot
  program halts (constructor or loop exception, unsupported period): today it only keeps prosdk's outputs
  disabled and prints the error, and the DS still shows the robot as enableable.
- `battery::currentAmps()` declared in every configuration (NaN when there is no sensor); brownout or
  low-battery flag.
- Optional persistence of DS-written tunables in NVS (opt-in per tunable), separate namespace.
- `rslState()`: current state of the robot signal light / status LED core drives (WPILib `HAL_GetRSLState`).
- `commsDisableCount()`: number of times outputs were disabled because the DS link was lost (since boot;
  WPILib `HAL_GetCommsDisableCount`).

Status: part 1 (everything except persistence) delivered as draft PR #37 and wired (CoreOps ABI 5, see
"P1 R12 part 1 delivered" below). Persistence is delivered as draft PR #39 (`feat/dash-persist` 3e9189b)
and wired (CoreOps ABI 6, see "R12 part 2 persistence: prosdk side wired" below), in the pinned core since
2026-10-22. Core's final rule is the one prosdk asked for: only Driver Station writes the robot accepted
are stored, never a value the robot sets, and `PROBOT_HAS_DASH_PERSIST` marks it; prosdk follows both
("Core stack rebased again" below; earlier drafts stored the topic's current value and had no macro).

### R13. Dashboard schemas larger than a value

`addSchema()` refuses data longer than `PROBOT_DASH_MAX_VALUE` (default 1024). The protobuf file descriptors
prosdk sends are bigger: `wpimath/protobuf/geometry2d.proto` alone is about 1.3 KB, so with the defaults no
protobuf-typed topic (`proto:` type strings, e.g. a `TunableProtobuf` or telemetry logged as protobuf) can be
decoded on the DS page. Struct schemas are small and unaffected. Wanted: a separate schema byte limit
(e.g. `PROBOT_DASH_MAX_SCHEMA`, default 4096, still counted in `PROBOT_DASH_POOL_BYTES`), or schemas exempt
from the value limit. Workaround today: the sketch defines `PROBOT_DASH_MAX_VALUE` 2048 or more before
`#include <prosdk.h>`. prosdk reports a refused schema once on the console and keeps publishing the values.

Related, in R12 part 2: persistence of tunable values. A core without it keeps no value across a
reboot, so every tunable starts from its code default. Persistence is a per-tunable opt-in
(`properties["persistent"] = true`), not a property of robust tunables (see "R12 part 2 persistence: prosdk
side wired" below).

Status: delivered as draft PR #36 (`PROBOT_DASH_MAX_SCHEMA_BYTES`) and wired, see "P1 R13 delivered" below.
The `PROBOT_DASH_MAX_VALUE` workaround above is only for a core without it, and 2048 is not enough for
every descriptor: `kinematics.proto` is 4332 B (the others 639-1999 B).

### R14. Compressed Driver Station page (flash)

`index_html.h` keeps the DS page as uncompressed HTML: `MAIN_page` is 138,439 B of flash in every sketch
(7 % of a prosdk robot sketch, more than any prosdk library; `design/size.md`). `gzip -9` makes it
39,421 B. Wanted: store the page gzip-compressed (generated from the HTML by a build step) and serve it
from `handleRoot` with `Content-Encoding: gzip` (every browser that runs the DS page accepts it). Saves
about 99 KB of flash per sketch; no protocol or API change.

Status: delivered as draft PR #35, see "P1 R14 delivered" below.

### R15. Loop grid restarts the robot host cannot see (low priority)

Core's `core::LoopPacer::afterCycle()` (`probot/core/loop_timing.hpp`) restarts the pass grid after every
late cycle (`_last = now + 1`) but reports an overrun only when `!oneShot`. A pass that ran a one-shot
INIT/START/STOP hook (a phase change) and ended late therefore moves the grid without anything prosdk can
read: `loopOverruns()` does not change, so the robot host keeps its schedule aligned to the old grid until
the next counted overrun or period change (design/PORT.md 8.1 "Schedule", prosdk commit 536cdda; the
robot's mode init runs in the pass after the phase change, which is counted, so the effect is a schedule
up to one pass off the grid, not lost callbacks). Wanted: a grid-restart counter, e.g.
`uint32_t loopGridRestarts()` (every restart, one-shot or not; `loopOverruns()` keeps its meaning), or
count one-shot restarts in a separate counter, so the host can re-align on every restart. Optional: start
the first pass after boot on a tick boundary (today it starts right after the boot hook, off the grid, so
the host skips aligning on it).

Also nice to have (no prosdk change needed): a public accessor of the dashboard store's configured limits
(`dashboard::Limits limits()`); the bridge read `Limits::maxSchemaBytes` from `dashboard::detail::g_store`
under its lock.

Status: delivered as draft PR #41 (all three items) and wired (CoreOps ABI 7), see "P1 R15 delivered"
below.

### R16. Driver Station signal for a value refused for size (low priority)

`dashboard::set()` refuses a value longer than `PROBOT_DASH_MAX_VALUE` (or one the pool cannot hold): the
robot gets `false` and prosdk reports it once on the console, but nothing reaches the Driver Station,
which keeps showing the previous value as if it were current. The case prosdk users meet first is a
`Field2d` trajectory: `struct:Pose2d[]` is 24 bytes per pose, so with the default 1024 an object of more
than 42 poses silently stops updating on the DS. Wanted: a wire error or flag frame for "the robot's
last value of this topic was refused" (cleared by the next stored value), so the DS can mark the topic
as stale (for example the same row status a refused DS write gets). Core proposed it as a separate item
from R15 (2026-10-23); no prosdk change is needed beyond keeping its console report.

Until then (documented in design/API-DIFFERENCES.md section 3): `PROBOT_DASH_MAX_VALUE` is global (one
limit for every topic, 16-16384); a sketch defines it before `#include <prosdk.h>` as the power of two at
or above 24 x the largest object's poses (4096: 170, 8192: 341, 16384: 682). It has three costs, read
from 15defc1. Core keeps each value in a buffer rounded up to the next power of two (at most the limit,
`dashboard.hpp` `set()`: `grow = 16; while (grow < len) grow <<= 1`) counted in `PROBOT_DASH_POOL_BYTES`
(32768, shared with names, types, properties and schemas); a `Field2d` object that is telemetry and
tunable holds two such buffers, so one object of 342-682 poses takes the whole default pool and its
second value is refused like an oversized one: the pool has to grow by the rounded size of every large
value. The limit sizes two heap buffers outside the pool whatever is live: the DS push frame
(`frameCapacity()`, the larger of limit + 519 B and the schema frame, 4611 B with the default schema
limit) and the WebSocket receive buffer for DS writes (limit + 4 B): about 5.6 KB at 1024, 8.7 KB at
4096, 16.9 KB at 8192, 33 KB at 16384. A DS write of an object (the whole array) also counts against
`PROBOT_DASH_WRITE_QUEUE` (2048 B, 85 poses) and is refused with `QueueFull` above it. Persistent topics
are stored only up to 2048 bytes whatever the limit.

Status: delivered as draft PR #42 (`feat/dash-set-refused` 62e5dbf) and documented (no prosdk API), see
"P1 R16 and R17 delivered" below.

### R17. `unshare()` that reports whether the file task let go (low priority)

`probot::files::unshare()` (and `share()` replacing a name) waits for the DS file task to release the
old share, but `waitReleased()` gives up after `kReleaseWaitMs` (1000 ms), while the task checks
`shareCurrent()` only between chunks and each `httpd_resp_send_chunk()` can block for the server's
`send_wait_timeout` (2 s, per socket send) on a slow or stalled browser. So `unshare()` can return while
`FileReader` still holds the file open, and core's own comment ("so the file system can be unmounted
afterwards") promises more than the 1 s cap gives. Wanted, any of: `unshare()` returning false (or a
status) when the wait timed out with the share still marked active; a `probot::files::released(name)` /
`busy()` query a sketch can poll before unmounting; or the comment and `API.md` saying the 1 s is a cap.
prosdk never unmounts (`MountSdCard` lasts until the program ends), so nothing breaks today; prosdk's
docs (DataLogManager, `UnshareFiles()`, design/API-DIFFERENCES.md) tell a sketch that unmounts after
`DataLogManager::Stop()` to wait a few seconds first. Once core reports it, `UnshareFiles()` can pass the
result on.

Status: delivered in draft PR #38 (`feat/ds-files` ba2500b, `probot::files::busy()`) and documented, see
"P1 R16 and R17 delivered" below.

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
  Field2d, Mechanism2d pending at #28. Update: the pinned core has the Alerts widget and the PIDController
  order, and #40 (`feat/dash-widgets`) adds Scheduler, Field2d and Mechanism2d (layouts below).

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
- PIDController and Alerts widgets are in #28 with the layouts above. probot-core continues with R7; the
  R5 remainder (Field2d, Mechanism2d, Scheduler widgets) follows with the layouts below.

### Widget layouts for Field2d, Mechanism2d, Scheduler (captured from prosdk, 2026-10-22)

Captured from the store by test/prosdk/commands2/WidgetLayoutTest.cpp (golden: path, value type, type
string, properties, flags, value). `<X>` is the name the program logs/publishes the object under; the
telemetry table is `/Telemetry/<X>/...`, the tunable table `/Tunables/<X>/...` (same relative layout).
Telemetry topics have flags TELEMETRY and empty properties; tunable topics have flags TUNABLE (+MUTABLE,
ROBUST as listed) and properties `{"mutable":true|false}` (+`"robust":true`). Every `.type` topic is
value type string, type string "string". All numbers little-endian. These differ from the upstream WPILib
2025 NetworkTables layouts (double[] [x, y, deg] Field2d objects; Mechanism2d roots with separate `x`/`y`):
the DS must decode the layouts below, not the 2025 ones.

Field2d (telemetry AND tunable; `Field2d.cpp`, `FieldObject2d.cpp`):
- `<X>/.type` = "Field2d" (tunable side: TUNABLE, `{"mutable":false}`).
- One topic per field object, named by the object: `<X>/Robot` always (the robot pose, first object), then
  every `GetObject(name)` (e.g. `<X>/Targets`); new objects can appear at any time.
- Object topic: value type raw (6), type string `struct:Pose2d[]`, value = n x 24 bytes, n >= 0 (0 = object
  has no pose, draw nothing). Each Pose2d = {f64 x, f64 y, f64 rotation}: x, y in meters (field
  coordinates, WPILib blue-origin), rotation in RADIANS (counter-clockwise positive). Telemetry: TELEMETRY.
  Tunable: TUNABLE|MUTABLE, `{"mutable":true}` (not robust, not persistent).
- Struct schemas published with it (addSchema, schema type "structschema", name => text):
  `struct:Pose2d` => `Translation2d translation;Rotation2d rotation`,
  `struct:Translation2d` => `double x;double y`, `struct:Rotation2d` => `double value`.
  (Packed: Pose2d = Translation2d{x,y} then Rotation2d{value} = 3 doubles, 24 bytes.)
- Example (golden): `/Telemetry/Field/Robot` = 24 bytes {(1.5, 2.25, 1.5707963267948966)};
  `/Telemetry/Field/Targets` = 48 bytes {(4, 0.5, 3.141592653589793), (6.25, 3, -0.7853981633974483)}.
- DS write-back (pose drag): write the WHOLE array to the tunable object topic (`/Tunables/<X>/<object>`,
  raw, n x 24 bytes, radians). The robot replaces that object's poses (1 pose replaces 2; empty array clears
  them), `GetRobotPose()`/`GetPoses()` return the written value and the topic echoes it. A length that is not a
  multiple of 24 is refused with `rejectWrite(DecodeError=4)`. Telemetry topics are not writable.

Mechanism2d (telemetry only, no `/Tunables` side, no schemas; `Mechanism2d.cpp`, `MechanismRoot2d.cpp`,
`MechanismLigament2d.cpp`):
- `<X>/.type` = "Mechanism2d".
- `<X>/dims` = double[] (10) `[width, height]` (canvas size, user units; same units as root positions and
  ligament lengths).
- `<X>/backgroundColor` = string `"#RRGGBB"` (uppercase hex; default `"#000020"`).
- Each root is a subtable `<X>/<root>` WITHOUT a `.type`; it is a root because it has `<X>/<root>/position` =
  double[] `[x, y]` (canvas units, from the bottom-left corner, y up, as upstream Glass draws it). Upstream 2025 had `x` and `y` doubles.
- Each ligament is a subtable under a root or under another ligament (any depth): `<parent>/<name>/.type` =
  "line", `angle` (double, DEGREES, relative to the parent's direction; a root's direction is +x),
  `length` (double, canvas units), `color` (string `"#RRGGBB"`, uppercase; default `"#EB8934"`), `weight`
  (double, line width in px; default 6).
- Children of a node are any subtables with `.type`="line"; names are free. All topics TELEMETRY, empty
  properties; values change on the next log.
- Example (golden, `Mechanism2d{3, 2, Color8Bit{0,0,32}}`):
  `/Telemetry/Arm/.type`="Mechanism2d", `dims`=[3, 2], `backgroundColor`="#000020",
  `shoulder/position`=[1, 0.5], `shoulder/arm/{.type="line", angle=45, length=1.25, color="#FF0000",
  weight=4}`, `shoulder/arm/wrist/{.type="line", angle=-30, length=0.5, color="#0080FF", weight=2.5}`,
  `base/position`=[2.5, 0], `base/post/{.type="line", angle=90, length=1.5, color="#EB8934", weight=6}`.

Scheduler (telemetry AND tunable; commandsv2 `CommandScheduler::LogTo`/`PublishTunable`):
- Telemetry: `<X>/.type`="Scheduler", `<X>/Names` string[] (11) (scheduled command names), `<X>/Ids` int[]
  (8, i64) (same order; an id is an opaque per-command number, the Command's address). No `Cancel` here.
- Tunable: `<X>/.type`="Scheduler" (`{"mutable":false}`), `<X>/Names` string[] and `<X>/Ids` int[] (TUNABLE,
  `{"mutable":false}`, refreshed by every tunables update, i.e. each robot loop), `<X>/Cancel` int[] (TUNABLE|MUTABLE|ROBUST,
  `{"mutable":true,"robust":true}`, normally `[]`, never persistent).
- DS write-back (cancel button): write int[] `[id, ...]` to `/Tunables/<X>/Cancel`. The robot cancels the
  ids that are still scheduled (unknown ids are ignored), then sets `Cancel` back to `[]`; `Names`/`Ids`
  drop the command on the next loop. Writes to `Names`/`Ids` are refused with `Immutable=2`.
- Example (golden): `Names`=["Drive", "Intake"], `Ids`=[<id Drive>, <id Intake>], `Cancel`=[]; after the DS
  writes `[<id Intake>, 12345]`: Intake cancelled, Drive still scheduled, `Cancel`=[], `Names`=["Drive"].

Cross-check against core's DS widgets (#40 e725330, as in the pinned 15defc1 `ui/index.html`, 2026-10-23):
no mismatch. Read side by side: Field2d takes every child of type `struct:Pose2d[]` with value type raw
(6) as an object ("Robot" first), 24 bytes per pose `{f64 x, f64 y, f64 rad}` LE (a length that is not a
multiple of 24 shows "Geçersiz uzunluk", an empty array draws nothing), editable when the topic has
TUNABLE|MUTABLE, and writes the whole array; Mechanism2d needs `dims` double[] (10), reads
`backgroundColor`, roots = children without `.type` holding `position` double[] [x, y], ligaments =
children with `.type` "line" whose `angle` (degrees) adds to the parent's direction (a root points along
+x), `length`, `color` `#RRGGBB` in either case, `weight` px, any depth; Scheduler needs `Names` string[]
(11) and `Ids` int[] (8), offers cancel buttons only when `Cancel` is int[] with TUNABLE|MUTABLE, writes
every pending id as exact int64 (text through BigInt past 2^53) and never reads Cancel's value (the
robot's echo `[]` is fine). Run: `tools/build/ds_widget_check.js` extracts core's own functions (`dkDec`,
`dkEnc`, `dkPoses`, `dkPoseBytes`, `dkMech`, ...) from `ui/index.html` and runs them under node on the
goldens above: the poses decode to the same values (90 deg for the robot), core's pose encoder
`dkPoseBytes([[3, 1, 0.5]])` (what the typed pose list writes; a real drag keeps each pose's rotation and
moves only x and y, so it cannot produce this golden) and the cancel write of `[id, 12345]` are byte-identical to what the C++ test pushes, a 64-bit host id
round-trips exactly, and the Mechanism2d summary reads `base/post: 90°, 1.5; shoulder/arm: 45°, 1.25;
shoulder/arm/wrist: 15°, 0.5` with the strokes where the golden puts them on a 300 x 200 canvas. The
goldens in the script are copied from the C++ test; re-run it when core's DS page changes.

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
ReportWarning/MathShared warnings -> Warning, everything else Info). Scheduler widget was added to core's R5
remainder (after R12/R13).

Wired (prosdk, CoreOps ABI 4): `prosdk::platform::ConsoleWrite(text, level)` carries a level (kInfo/kWarning/kError,
same values as `console::Level`); the bridge (src/prosdk/probot/CoreBridge.hpp) detects R9 with
`__has_include(<probot/telemetry/console.hpp>)` and fills CoreOps `consoleWriteLevel` with `console::write`. HAL
errors and ReportError are Error; ReportWarning, MathShared, telemetry/tunables (alerts) and other warnings are
Warning; everything else Info. A core without R9 leaves the field null and the text goes through
`telemetry::writeBytes` without its level. On the robot a ConsoleWrite from an ISR is dropped. Tests:
test/prosdk/ConsoleLevelTest.cpp, test/prosdk/CoreOpsTest.cpp.

### P1 R10 delivered (2026-10-22): draft PR #34 (branch `feat/wall-clock`, b46069e, stacked on #33 301f190)

The DS page sends the browser's UTC time on every WebSocket open; the robot calls settimeofday. The first valid time
always sets the clock; later ones only while phase STOPPED and status STOP (never mid-match, including the
auto->teleop TRANSITION). `probot::ds::systemTimeValid()`; `time()`/`gettimeofday()` give UTC (no TZ set).
prosdk use: HAL_GetSystemTimeValid -> CoreOps systemTimeValid (replaces the time > 2024-01-01 heuristic), so
DataLogManager gets real file names and systemTime entries.

Wired (prosdk, CoreOps ABI 4): the bridge detects R10 with `__has_include(<probot/core/wall_clock.hpp>)` and fills
CoreOps `systemTimeValid` with `ds::systemTimeValid()`; HAL_GetSystemTimeValid reads it. A core without R10 keeps
the heuristic (time() at or after 2024-01-01). CI builds the sketches against `feat/wall-clock`
(PROBOT_CORE_REF). Tests: test/prosdk/CoreOpsTest.cpp, test/prosdk/DataLogManagerTimeTest.cpp (log renamed to
WPILIB_<UTC>.wpilog and systemTime entries with a valid clock).

### Core branches force-pushed (2026-10-22): same trees, new shas

probot-core rewrote its stacked branches to drop a committed `tools/__pycache__/sync_version.cpython-312.pyc`.
Every tree is otherwise identical (`git diff` old new: only that file), so nothing in prosdk changes. New
shas: #30 `feat/loop-rate` 71a5c19 -> e30a98d, #31 `feat/multi-gamepad` 4484a97 -> e10c697, #33
`feat/console` 301f190 -> 3985e07, #34 `feat/wall-clock` b46069e -> 3ebde55 (`feat/framework-api` is now
96f449d, `feat/opmode-list` fe3ad65). The sizes in design/examples.md, size.md and PORT.md that name
b46069e were measured on the same tree as 3ebde55.

### P1 R14 delivered (2026-10-22): draft PR #35 (branch `feat/ds-gzip`, 29e7d56, stacked on #34 3ebde55)

The DS page's source moved to `ui/index.html`; `src/driverstation/esp32s3/index_html_gz.h` is generated by
`tools/gen_ui.py` (`make ui`, CI checks it matches) and served from `handleRoot` with
`Content-Encoding: gzip`. No API or protocol change; prosdk needs no code change (verified: the bridge, the
HAL link test and the examples build unchanged against it). Measured (core `feat/small-items`, all three PRs): 99,856-100,940 B less flash in each of eight
sketches, design/size.md section 9.

### P1 R13 delivered (2026-10-22): draft PR #36 (branch `feat/dash-schema-limit`, 8419db4, stacked on #35)

`PROBOT_DASH_MAX_SCHEMA_BYTES` (default 4096, 16-32768, PB-E108; counted in `PROBOT_DASH_POOL_BYTES`) limits
each schema; `PROBOT_DASH_MAX_VALUE` no longer applies to schemas. `dashboard::Limits` gets
`maxSchemaBytes` as its last field. With the defaults `geometry2d.proto` (1360 B) and every other wpimath
descriptor are accepted except `kinematics.proto` (4332 B, the largest): a kinematics type sent as
protobuf needs `#define PROBOT_DASH_MAX_SCHEMA_BYTES 8192` (or at least 4332). Not asked of core as a
larger default: the DS frame buffer is sized from this limit (`frameCapacity()`), so every sketch would
pay about 4 KB more heap for a path only explicit protobuf users take (telemetry prefers the struct).

Wired (prosdk, CoreOps ABI 5): CoreOps `dashMaxSchemaBytes` returns the store's configured limit (0 before
`runtime_setup()`). There is no new header to detect, so the bridge checks `Limits::maxSchemaBytes` on the
type at compile time (a constexpr `requires` check; a core without it leaves the field null). A refused
schema is reported once: larger than the limit -> its size and `PROBOT_DASH_MAX_SCHEMA_BYTES`; otherwise
the schema table or the pool; with a core without R13 the old message, advising
`#define PROBOT_DASH_MAX_VALUE <the schema's size> or more`. Tests (test/prosdk/dashboard, the fake store
follows R13 with 4096 by default): a Translation2d protobuf descriptor (over 1024 B) is stored with core
defaults (telemetry and tunable), `kinematics.proto` (ChassisVelocities) is refused with the defaults and
reported with 4332 bytes and stored with 8192, a too-small `PROBOT_DASH_MAX_SCHEMA_BYTES` and an older core
are reported once with the right advice, and `PROBOT_DASH_MAX_VALUE` 2048 still stores geometry2d.proto on
an older core.

### P1 R12 part 1 delivered (2026-10-22): draft PR #37 (branch `feat/small-items`, 09a6fe4, stacked on #36)

API (config-free, `<probot/api.hpp>`): `probot::requestStop()` (stops the active OpMode like the DS STOP
button, cooperative, cancels a pending INIT/START; false when there is nothing to stop, a stop in progress
or e-stop; any task, including a hook), `ds::rslState()` (the signal light core draws, with or without
`PROBOT_RSL_PIN`; false before the runtime started), `ds::commsDisableCount()` (DS timeouts that stopped an
enabled START robot since boot, PB-E303 only), `<probot/io/battery_status.hpp>` with
`io::battery::currentAmps()` in every configuration (NaN instead of 0 without an INA sensor, before its
first sample and after a failed read), `setLowVoltage(volts)` / `isLowVoltage()` (a flag on the DS
voltage, threshold 0 = off by default, no hysteresis, outputs not disabled).

Wired (prosdk, CoreOps ABI 5: `requestStop`, `rslState`, `commsDisableCount`, `setLowVoltage`,
`dashMaxSchemaBytes`). The bridge detects R12 with `__has_include(<probot/io/battery_status.hpp>)` (the
last header of PR #37); without it the four fields stay null and the archive keeps the old behaviour:
- Robot host halts (constructor, startup or loop exception, unsupported period, a robot class without a
  hosted loop, failed HAL initialization, full callback table): the error print and the closed output gate
  as before, plus one `requestStop()` at the halt and a pre-cycle callback that calls it every pass, so
  every later INIT or START is stopped (one console line each). Without R12 the DS can still start the
  halted program (nothing runs).
- `HAL_GetRSLState` -> `ds::rslState()`, `HAL_GetCommsDisableCount` -> `ds::commsDisableCount()`
  (`HAL_SUCCESS`); without R12 both report `HAL_HANDLE_ERROR` as on systemcore.
- Brownout: `RobotController::SetBrownoutVoltages` / `IsBrownedOut` keep prosdk's computation (default
  6.75 V, recovery 7.25 V, hysteresis, reported only). An accepted `SetBrownoutVoltages` (in range) also
  sets core's `setLowVoltage()` threshold to the brownout voltage, so core's `isLowVoltage()` agrees with
  prosdk's entry voltage. The 6.75 V default is not forwarded at HAL initialization (it was in the first
  wiring): core's flag is a low-battery warning, off by default, and 6.75 V would turn it on in every
  sketch, far below a 3S LiPo cutoff, and overwrite a threshold the sketch set from a static
  initializer. WPILib 2027 has no `GetBrownoutVoltage`.
- `currentAmps()`: not wrapped. WPILib 2027 has no robot input current (`GetInputCurrent`/`HAL_GetVinCurrent`
  are gone; total current is a PowerDistribution reading, which the ESP32 HAL does not support), and no
  prosdk code read it. A sketch reads it from core and checks `std::isnan()`; NaN logged through prosdk
  telemetry is stored as NaN (repeats dropped as equal bytes; host test).
Tests: test/prosdk/CoreOpsTest.cpp (no-core defaults, the mapping with a fake core),
test/prosdk/robot/RobotHostTest.cpp (halts stop the DS, later starts are stopped, fields left null report
"not available"), test/target QemuSmoke (FakeCore leaves ABI 5 null: `HAL_HANDLE_ERROR` for RSL and
comms-disable count). CI builds the sketches against `feat/small-items` (PROBOT_CORE_REF).

### R12 part 2 and R11 design (agreed 2026-10-22; delivered as #39 and #38)

Superseded status (kept for history): written while both were core work in progress, with sdk-target and
CI on `feat/small-items`. Both are now draft PRs in the pinned core (#38, #39; "Core stack rebased
again" below) and both are wired (persistence ABI 6, files ABI 8, "P1 R11 wired" below). The DataLogManager
folder is `/littlefs/logs` (a VFS path, shared with `share(name, vfsDir)`), not LittleFS `/logs`.

- R12 tunable persistence (`feat/dash-persist`): a topic published with `PERSISTENT` gets its stored value
  back after a reboot. Core saves values of such topics to NVS (own namespace) with a debounced write, and
  on `publish()` of a `PERSISTENT` topic with a saved value queues that value as a normal `Write`.
  Correction (2026-10-22): the first version of this design said prosdk publishes every robust tunable
  with `ROBUST | PERSISTENT`. It does not: ROBUST only means "echo the applied value", and the robust
  topics prosdk itself creates (the scheduler's Cancel, Selectable's `selected`, the `/Preferences`
  mirrors) must start from their code value after a reboot. `PERSISTENT` is set only for a mutable tunable
  that opts in, and prosdk applies the restored value inside `Publish()`, not in a later
  `TunableRegistry::Update()` (see below). Non-persistent topics are unchanged; `/Preferences` keeps its
  own NVS store.
- R11 files (`feat/ds-files`): `probot::files::share(name, fs, dir)` registers a directory of a mounted
  filesystem under a name; the DS gets a "Dosyalar" card listing the shared folders and their files.
  Download only while the robot is idle or e-stopped (no file traffic while it can move); no delete in v1.
  prosdk shares its DataLogManager folder as "logs" (LittleFS `/logs`, or the SD card folder when
  DataLogManager logs there), so `.wpilog` files can be downloaded from the DS.

### R12 part 2 persistence: prosdk side wired (CoreOps ABI 6), 2026-10-22

First wired against the local core commit `feat/dash-persist` 5513bb0 (then 25a34e8), with sdk-target and
CI on `feat/small-items`; since "P1 R15 delivered" the pinned core has #39. The store rule and the
republish rule below were revised for core's final rule ("Core stack rebased again" below): the bullets
"Core's answer", "Republish rule", "Detection" and "Tests" describe the current state, except the first
answer, kept for history.

Core's first answer to prosdk's two questions (2026-10-22; superseded by #39 3e9189b, which stores only
DS writes the robot accepted and adds `PROBOT_HAS_DASH_PERSIST`):
- Store only DS-accepted values: declined. Core keeps storing the topic's current value, i.e. what the
  robot last `set()` on the topic (a DS write changes it only once the robot applies and echoes it),
  2 s after it last changed and only while no OpMode runs, the same as an NT persistent topic stores
  its value whoever set it. A stored value therefore wins over the code default after a reboot, also
  one the robot program set; after changing a default, call `ClearPersistedTunables()`. Core's advice
  for prosdk's republish memory, followed (below): remember the last value `set()` on the path, not the
  last applied tune.
- Feature macro or header: none. The requires-check on `probot::dashboard::clearPersisted()` is the
  intended detection; after a core release with persistence `PROBOT_VERSION >= PROBOT_VERSION_OF(0,5,0)`
  also works.

- Opt-in: a mutable tunable with `TunableConfig::properties["persistent"] = true` (the topic property name)
  is published with `TUNABLE | MUTABLE | PERSISTENT` (plus `ROBUST` when robust). Robust alone never sets
  `PERSISTENT`; immutable tunables, ComplexTunable `.type` topics, the scheduler's Cancel, Selectable's
  `selected` and the `/Preferences` mirrors never do. Documented as a prosdk addition in
  design/API-DIFFERENCES.md section 7.
- Restore: `Publish()` of a new PERSISTENT topic calls `drainWrites()` right after core's `publish()`, takes
  the restored write for that topic (other topics' drained writes stay queued for `Update()`) and applies
  it through the DS write validation (type check, set, RecordTuneApplied, onRemoteSet), then
  `UpdateNetwork(true)` and onTune, as upstream applies a robust tunable's retained value. So the tunable
  has its stored value when `Publish()` returns (a subsystem constructor reads it). A value that does not
  decode or type-check is answered with `rejectWrite()`, which erases the record. A tunable published
  before the store is ready gets its value when it is published by `FlushPending()` or `Update()`. The
  dashboard router lock is recursive so a participant's `FlushPending()` can drain.
- Republish rule (core's final rule: only DS writes the robot accepted are stored): published again in the
  same run (Remove and Publish, or a new tunable object on the same path), the tunable gets the last value
  core accepted on that path in this run: the backend remembers, under its value type and type string,
  core's restore (applied, or equal to the tunable's own value) and every DS write the robot applied (a
  write prosdk refuses, in `OfferWrite` during the drain or with `rejectWrite()` in the `Update()` that
  follows it, is neither stored by core nor remembered), never a value the robot program sets. Core's
  record can be older (it saves an accepted write only while no OpMode runs, and drops it when the topic
  is unpublished first: a tune of an OpMode-scoped tunable removed when the OpMode ends may not reach the
  next reboot; prosdk has no way to store it), so core's restored write is then dropped without
  `rejectWrite()` (the record is not erased). Like a restore, the remembered value is applied through the
  DS write path: tune revision, onRemoteSet and onTune, and it overwrites a `Set()` made while the tunable
  was not published or since the tune, as after a reboot. A restored or remembered value equal to the
  tunable's own is not applied (no onTune or tune revision, the record stays; e.g. Remove and Publish of
  the same object, or a stored value equal to the code default); a tunable of another value type or type
  string on the path takes core's record. `ClearPersistedTunables()` also forgets the remembered values (a
  counter the backend compares under its lock), as core erases its records and the accepted writes not
  stored yet and stores the next DS write: a republish after the clear keeps the new tunable's own value
  until it is tuned again. (Between 4940136 and this revision the backend remembered every value the
  store took, robot `Set()` included, to follow core's first answer.)
- Detection: CoreBridge.hpp requires core's `PROBOT_HAS_DASH_PERSIST` macro (the final rule; the earlier
  drafts with the old rule have `clearPersisted()` but no macro and are not wired) and looks up
  `probot::dashboard::clearPersisted()` with a requires-expression on a
  dependent call (nothing is declared in core's namespaces) and fills the ABI 6 field `dashClearPersisted`.
  A core without it leaves the field null: the opt-in is a no-op with one console warning per path (a
  republish does not repeat it; "properties.persistent: this probot-core does not keep dashboard values
  across reboots ..."), the flag is not set. `prosdk::platform::ClearPersistedTunables()` (Dashboard.hpp)
  calls `clearPersisted()` (false without it); `prosdk::platform::DashPersistenceSupported()` reports the
  field.
- Tests: test/prosdk/dashboard/TunablePersistenceTest.cpp (the fake store follows core 3e9189b: a drained
  DS write to a PERSISTENT topic is a candidate until `rejectWrite()` or a newer drained write; the idle
  save stores accepted writes, never set values; unpublish and `clearPersisted()` drop what is not stored
  yet. Cases: synchronous restore with value, onTune and revision; no or mismatching record; refused
  restore erases the record; robust/Selectable/immutable not persistent; other drained writes kept;
  published before ready via FlushPending and Update; republish keeps the run's tune; a robot `Set()` is
  neither stored nor remembered, also for an always-get tunable; a robot `Set()` after a tune: the
  republish and core both get the tune; DS writes refused during and after the drain are not stored or
  remembered; a stored value equal to the code value is not a tune and is still what the path gets;
  ClearPersistedTunables, also before a republish, and it drops a tune not stored yet; refused restore on
  the before-ready path; republish with another type takes core's record), mutation-checked (dropping the
  remembered tune or the remembered equal restore fails 3 cases),
  test/prosdk/dashboard_legacy/PersistFallbackTest.cpp (own executable, field null: no flag, one warning
  per path also across a republish, DS tunes still work), Cancel and Preferences mirrors not PERSISTENT,
  CoreOpsTest (no-core and fake defaults), FakeCore.hpp leaves ABI 6 null. A single-TU compile of
  CoreBridge.hpp detects persistence against #39 3e9189b and later, and not against abef559 (old rule,
  no macro) or `feat/small-items`.

### P1 R15 delivered (2026-10-22): draft PR #41 (branch `feat/loop-grid-restarts`, dbf9d1f, stacked on #40)

Core rebased its stacked drafts for a docs-only `API.md` line in #37 (`git diff 09a6fe4 2eb1a72`: that
line only). Shas at delivery (rebased again since: "Core stack rebased again" below): #37 `feat/small-items` 2eb1a72 (was 09a6fe4), #38 `feat/ds-files` f48ef22, #39
`feat/dash-persist` abef559, #40 `feat/dash-widgets` 960c63c, #41 `feat/loop-grid-restarts` dbf9d1f.
`.upstream/probot-core` `sdk-target` and CI (`PROBOT_CORE_REF`) now pin `feat/loop-grid-restarts`, the top
of the stack, so the pinned core also has R11 files (#38, not wired yet), R12 part 2 persistence (#39) and
the R5 DS widgets for Field2d, Mechanism2d and Scheduler (#40, built from the layouts above).
`PROBOT_CORE_OLDEST_REF` stays `feat/dashboard-store`.

API (core `API.md`, `CHANGELOG.md`):
- `probot::loopGridRestarts()` (`<probot/core/loop_timing.hpp>`, also through `<probot.h>`): times the pass
  schedule restarted since boot, every late pass, including the one-shot INIT/START/STOP passes that
  `loopOverruns()` leaves out (unchanged). Core steps it after the late pass, before its sleep, so the next
  pass and its pre-cycle hook already see it. A period change is not counted.
- The first pass after boot starts on a tick boundary (`vTaskDelay(1)` after the boot hook).
- `dashboard::limits()`: the `Limits` in effect (`PROBOT_DASH_*` after clamping), all zero before
  `runtime_setup()`.
- Detection: a requires-expression on `probot::loopGridRestarts` (no feature macro).

Wired (prosdk 9ec2023, CoreOps ABI 7: `loopGridRestarts`):
- Bridge: `kCoreHasGridRestarts`, a requires-check on a dependent call of `loopGridRestarts()` looked up
  through a using-directive (as for `clearPersisted()`); without it the field stays null.
  `dashMaxSchemaBytes` reads `dashboard::limits().maxSchemaBytes` when `limits()` exists (same kind of
  check), else `detail::g_store` under the store's lock as before. That was prosdk's only read of core's
  store internals; outside core's `API.md` the bridge still uses `probot::core::kMinLoopPeriodMs` /
  `kMaxLoopPeriodMs` (a `static_assert` against prosdk's copies) and `probot::core::MuxGuard` (the lock
  of that `g_store` fallback).
  The archive never keeps the value: it is read when a refused schema is reported, which happens only
  once the store is configured (a schema refused before that is kept and retried), so a zero read before
  `runtime_setup()` never sticks.
- Robot host (`RobotMain.cpp`): re-aligns its schedule on the pass where `GetLoopGridRestarts()` changed,
  i.e. the first pass of core's new grid, also after a late one-shot pass. The overrun warning still
  follows `loopOverruns()` only (a one-shot restart is not reported). Without R15
  (`GetLoopGridRestarts()` false) it re-aligns on overrun changes, as before. Alignment still starts at
  pass 1: with R15 core's first pass is on the grid too, so pass 1 is the same grid; before R15 pass 0 was
  off it, and a late registration constructs the robot inside pass 0 either way.
- Tests: test/prosdk/robot/RobotHostTest.cpp, built twice (`prosdk_robot_tests` with the count,
  `prosdk_robot_legacy_tests` with the field null): a late one-shot pass moves core's grid by 2.4 ms of a
  5 ms pass, then passes start up to 1.5 ms late; with R15 every pass runs the loop exactly once and
  nothing is reported as an overrun. The legacy build pins the degraded case PORT.md describes: the same
  passes put the loop in the wrong pass (some pass runs it twice or not at all) until a counted overrun,
  after which every pass runs it once (revised after review: it first injected the overrun with the
  one-shot restart, which only re-ran the overrun path). CoreOpsTest: no core and a null field report
  "not available". QEMU FakeCore leaves ABI 7 null, and the QEMU sketches drive the hooks themselves, so
  the R15 path is checked on the host fake and by the target link only, never against core's real pacer.
- R12 part 2 against #39 abef559 (superseded: #39 changed its store rule at 45112ae/3e9189b, see the next
  section): the ABI 6 wiring builds unchanged against the pinned core (archive,
  HAL link test and examples, no worktree). Core changes since 25a34e8 that prosdk follows without code:
  stored values are limited to 2048 bytes (PB-E308 above that), a restored value is erased by
  `rejectWrite()` only before another write to that topic is delivered (prosdk rejects it right after the
  publish's drain), and `clearPersisted()` also drops restores not drained yet.
- Checked: the HAL link test links against the pinned core (`grid_probe::LoopGridRestarts<>` and
  `persist_probe::ClearPersisted<>` in the ELF) and against `feat/dashboard-store` (neither); a host
  syntax check of `CoreBridge.hpp` with core's test stubs detects R15 on dbf9d1f, not on #40 960c63c
  (the `g_store` path of `dashMaxSchemaBytes` compiles there) nor on `feat/dashboard-store`. Sizes:
  design/size.md section 10.

### Core stack rebased again (2026-10-22, after 9ec2023): R12 part 2 final rule, repinned to 15defc1

Core force-pushed #38-#41 three more times after the R15 wiring (23:32, 23:36 and 23:57). Current shas
(`gh pr list`, and the clone after `git fetch`): #37 `feat/small-items` 2eb1a72 (unchanged), #38
`feat/ds-files` eab956a, #39 `feat/dash-persist` 3773ea1, #40 `feat/dash-widgets` e725330, #41
`feat/loop-grid-restarts` 15defc1 (between: 352b555 / 3e9189b / 634cd40 / e40fa3d, where most of the
checks below ran). `.upstream/probot-core` `sdk-target` is at 15defc1; CI's `PROBOT_CORE_REF` names the
branch (`feat/loop-grid-restarts`), so it already builds that. The R15 API is unchanged (`git diff dbf9d1f
15defc1` touches no loop timing file; `limits()`, `Limits`, `clearPersisted()` and `loopGridRestarts()`
keep their signatures). e40fa3d -> 15defc1 leaves `dashboard.hpp` alone (PB-E308 texts, a 404 from
`/files` without shares, the WPILib 2027 Field2d `struct:Pose2d[]` and Mechanism2d `position` layouts,
which prosdk's Field2d and Mechanism2d already publish, and the DS page). The core stack is still moving: a
later force-push needs the same check.

What changed for prosdk:
- #39 (abef559 -> 45112ae -> 3e9189b) reversed core's first persistence answer: only Driver Station
  writes the robot code accepted are stored. A drained write is a candidate until `rejectWrite()`
  refuses it (inside the drain callback, or before the next write for the topic is drained and within
  about 2 s), then it is stored once no OpMode runs; a value the robot `set()`s is never stored and does
  not change the record. `clearPersisted()` also drops accepted writes not stored yet, and the next DS
  write is stored again; unpublishing a topic drops them too. Stored values are at most 2048 bytes and
  `PROBOT_DASH_MAX_VALUE`. `PROBOT_HAS_DASH_PERSIST` (in `<probot/dashboard/dashboard.hpp>`) marks the
  feature.
- #38 adds `PROBOT_HAS_FILES_SHARE`, `share(name, vfsDir)` and `unshare(name)` (R11, not wired).
- #40 and #41 changed only by the rebase (and `API.md` lines).

prosdk changes (d641c18, CoreOps ABI unchanged at 7):
- Tunable backend: the republish memory holds the last value core accepted on the path (restore or applied
  DS write), not every value the store took; `UpdateNetwork()` no longer reports what it set. This is the
  rule before 4940136, plus remembering a restore equal to the tunable's own value (a later object with
  another default gets core's record, as after a reboot). Docs: section "R12 part 2 persistence: prosdk
  side wired" above, design/API-DIFFERENCES.md section 7, design/PORT.md, the backend and Dashboard.hpp
  comments.
- Detection: `kCoreHasDashPersist` also requires `PROBOT_HAS_DASH_PERSIST`, so the drafts with the old
  rule (no macro) are not wired.
- Fake store (test/prosdk/dashboard/FakeDashboardStore.*) models the new rule; tests above.
- Known edge, not fixed (core has no way to undo it): prosdk keeps only the newest undrained DS write per
  topic until `Update()`. If two writes for one topic are drained before an `Update()` and the newer one
  does not decode, core has already accepted the older one (a newer write was drained) and stores it,
  though the robot never applied it.

Checked against e40fa3d: host tests, the incremental archive, the HAL link test (persistence, grid
restart and limits probes in the ELF) and the four size examples; the HAL link test against
`feat/dashboard-store` 9217ae4 (no probes); a host syntax check of `CoreBridge.hpp` with core's test
stubs (persistence detected on e40fa3d and 634cd40, not on abef559; R15 only on e40fa3d); QEMU smoke and
bench. Against 15defc1: the syntax check (all three detected) and the HAL link test (all three probes in
the ELF). Sizes: design/size.md section 11.

### P1 R11 wired (CoreOps ABI 8), 2026-10-23

Core's R11 (#38 `feat/ds-files` eab956a, in the pinned 15defc1; API in core `API.md` "Dosya paylaşımı"):
`probot::files::share(name, fs, dir)` (Arduino `fs::FS`) and `share(name, vfsDir)` (POSIX
`opendir`/`readdir`/`stat`/`fopen` on an ESP-IDF VFS path), `unshare(name)`, `PROBOT_HAS_FILES_SHARE`.
Only the files directly in a shared folder are offered, at most 4 shares, names 1-15 of `A-Z a-z 0-9 _ -`,
absolute paths of at most 95 bytes without `.`/`..` segments. Sharing a name again replaces it (true even
with 4 shares). A running transfer of a replaced or removed share stops at its next file call, and
`share()`/`unshare()` wait for the file task to release it, but at most 1 s: a chunk send to a slow
browser can block 2 s, so unmounting right after `unshare()` is not guaranteed safe (R17). The DS "Dosyalar" card (Logs page) lists and downloads only while the robot is idle
and not after an e-stop; no delete in v1; `/files` answers 404 to every client while nothing is shared.

Wired:
- CoreOps ABI 8: `filesShare(name, vfsDir)`, `filesUnshare(name)`. The bridge fills them when core defines
  `PROBOT_HAS_FILES_SHARE` and requires-checks find the VFS-path overload `share(const char*, const
  char*)` and `unshare(const char*)` (looked up through a using-directive in `files_probe`, as for
  `clearPersisted()`); `<probot/files/files.hpp>` is included when present. Older cores leave both null.
- Archive accessors (`CoreOps.hpp`): `FileSharingSupported()`, `ShareFiles(name, vfsDir)`,
  `UnshareFiles(name)`; false and silent without core or without R11. No public helper beyond these:
  a sketch that wants to share another folder calls core's `probot::files::share()` itself.
- DataLogManager (`wpilibc/0015`): the manager's thread shares its log directory as `"logs"` with the
  VFS-path overload once the directory exists (right after its old-log cleanup, then once a second while
  it is missing; the default directory is created by `Start()`, an explicit one may appear later). A
  refusal (relative path, `.`/`..`, over 95 bytes, 4 other shares) prints one warning and is tried again
  once a second like a missing directory, so the share is made once the sketch frees a slot (with 4
  sketch shares, which of the fifth `share()` calls wins depends on timing; DataLogManager catches up). `Stop()` joins the thread, whose destructor unshares (only a share it
  made), so `Stop()` returns after the share is gone; a later `Start()` with another directory (e.g.
  `SetLogDirectory("/sd/logs")` after mounting a card) shares that one under the same name. prosdk never
  unmounts a file system (`MountSdCard` lasts until the program ends), so there is no other unshare
  point.
- Threading: `share()` runs on the manager's own thread (`datalogmgr`), never on core's user task, so its
  wait for a replaced share (a sketch's own `"logs"` being downloaded) only delays that thread's loop;
  the DataLog writer is a separate thread and keeps writing. `unshare()` runs on the caller of `Stop()`
  and can block it up to 1 s, only while the DS downloads from the folder, which core allows only while
  the robot is idle; a download stalled longer can keep its file open after `Stop()` returns (R17), so
  the docs tell a sketch that unmounts to wait a few seconds first. A log file grows while it is offered; core sends a file with its size at open.
- Examples: SysIdRoutine and BoardozaSdLog mention the "Dosyalar" card.
- Tests: test/prosdk/files/DataLogShareTest.cpp, built twice. `prosdk_files_tests` (a fake core that
  records each call and its thread, then passes it on to core's own header-only `probot::files::share()`
  / `unshare()` when the core clone is present at configure time, `PROSDK_TEST_PROBOT_CORE_FILES`; CI's
  host job checks out `PROBOT_CORE_REF` for it): an existing folder is shared as `"logs"` on the
  manager's thread (not the caller of `Start()`) and unshared on the caller before `Stop()` returns; after
  a restart a folder created after `Start()` is shared once it exists, once; a `share()` that blocks 1 s
  delays neither `Start()` nor `Log()`, and the message reaches the file meanwhile; a relative folder, a
  folder over 95 bytes and one with `..` are refused by core's rules with exactly one warning, tried again
  and never unshared; with 4 sketch shares the manager is refused, then takes the slot the sketch frees,
  and `Stop()` leaves the sketch's shares alone. `prosdk_files_legacy_tests` (both fields null):
  `FileSharingSupported()`/`ShareFiles()`/`UnshareFiles()` false, the log is written, no warning.
  CoreOpsTest: no-core and null-field defaults. QEMU FakeCore records share/unshare with the FreeRTOS task
  name (accepting an absolute folder `stat()` finds): its DataLog case checks on the ESP32 that
  `/littlefs/logs` is shared once on `datalogmgr` and unshared on the case task. Not covered: core's
  file task itself (no WiFi in QEMU) and the 1 s release wait; a DS download needs a hardware run.
- Checked against 15defc1: host tests (`build.sh host all header_check --test`), the incremental archive,
  the HAL link test (`files_probe::Share<char const*, char const*>` and `Unshare<char const*>` in the ELF
  next to the grid-restart and persistence probes; 1,708,993 B flash / 75,996 B RAM) and against
  `feat/dashboard-store` 9217ae4 (links, no probe), QEMU smoke 8/8 on both boots (FakeCore, fields null),
  examples TemplateTimed 1,796,884 / 72,308, BoardozaSdLog 1,930,877 / 75,752, HalSmoke 1,612,901 /
  74,380 (flash / static RAM).
- Review fixes (2026-10-23): the unmount wording (the 1 s is a cap, R17), a refusal retried once a second,
  the R16 sizing advice (power-of-two buffers in the pool, the limit-sized heap buffers, the write queue),
  the tests above. Re-checked against 15defc1: host tests, the incremental archive, the HAL link test
  (same size and probes) and against `feat/dashboard-store` 9217ae4 (links, no probe), QEMU smoke 8/8 on
  both boots with the recording FakeCore (`share_task=datalogmgr unshare_task=user`).


### P1 R16 and R17 delivered, repinned to #42 62e5dbf (2026-10-23)

Core rebased its stack again and added draft PR #42 on top: #38 `feat/ds-files` ba2500b, #39
`feat/dash-persist` 29c282a, #40 `feat/dash-widgets` 589262e, #41 `feat/loop-grid-restarts` 784a94a, #42
`feat/dash-set-refused` 62e5dbf. `.upstream/probot-core` `sdk-target` is at 62e5dbf (`git reset --keep`);
CI's `PROBOT_CORE_REF` names `feat/dash-set-refused` (`PROBOT_CORE_OLDEST_REF` stays
`feat/dashboard-store`). 15defc1 -> 62e5dbf changes `dashboard.hpp` and `files.hpp` only by the two
items below; the signatures the bridge probes (`limits()`, `clearPersisted()`, `loopGridRestarts()`,
`files::share()`/`unshare()`) are unchanged.

R16 (#42): a refused `set()` (longer than `PROBOT_DASH_MAX_VALUE`: reason 1 TooLarge; no room in the pool
or heap: 2 NoMemory; a failed heap allocation is now counted in `stats().dropped` too, a full pool
already was) is sent to the DS in a new robot -> DS frame `'E'` 0x45 (repeated `{u16 id, u8 reason, u32
last refused length}`, LE) after the topic's remaining value: at once when the reason changes, then after
every resend of that value (resync, re-announce, write echo). The next stored `set()` clears it and sends
`'V'` even for an unchanged value. The DS shows "Güncel değil: <sebep>" on the row (widget topics without
a row are listed under the widget) and logs it once; older pages ignore the frame. Core also added
prosdk's sizing note to `API.md`. The robot side gets only the public enum
`probot::dashboard::SetRefusal`; `set()` still returns `bool` and there is no accessor of a topic's
reason. prosdk could still tell the two reasons apart itself: core's TooLarge test is only the length
against `limits().maxValue` (R15, which the bridge already probes), so one more CoreOps field such as
`dashMaxValue` would let `SetTopicValue()` split the message as `ReportSchemaRefused()` does for schemas;
no core request is needed. Not done (minimal): with #42 the DS shows the reason, and the console report
(`Limit::kValue`, once) keeps naming both causes. Documented in design/API-DIFFERENCES.md section 3 (the
`Field2d` advice: the DS now marks the topic not current with #42; an older core still shows the old
value as current) and the `DashboardStore.hpp` store notes.

R17 (folded into #38, same `PROBOT_HAS_FILES_SHARE`): `probot::files::busy()` is true while a DS listing
or download holds a folder or file of any share; `unshare()` waits at most 1 s (core's comment and
`API.md` now call it a cap). Before unmounting: unshare, then wait until `busy()` is false. `busy()` is a
separate query, not a result of `unshare()`, so R17's "`UnshareFiles()` can pass the result on" does not
apply. prosdk never unmounts, so no CoreOps field: a sketch that unmounts already calls ESP-IDF (prosdk
has no unmount) and sees core's `<probot.h>`, so `probot::files::busy()` does not make it mix layers more
than it does. DataLogManager.hpp (and its patch `wpilibc/0015`), `UnshareFiles()` in CoreOps.hpp and
design/API-DIFFERENCES.md now say: after `Stop()`/`UnshareFiles()`, poll `probot::files::busy()` until
false. Since the macro is unchanged, an earlier #38 draft (such as the 15defc1 stack) has file sharing
without `busy()` and a sketch cannot detect that at compile time; the docs name the sha (`busy()` from
ba2500b on, before it wait a few seconds). A core without file sharing shares nothing, so there is
nothing to wait for. Merged #38 will have `busy()`.

- DS widget check (`node tools/build/ds_widget_check.js .upstream/probot-core/ui/index.html`): all golden
  layouts decode as before (core's #42 page changes add the `'E'` handler and stale marks; no decoder
  renamed).
- Checked against 62e5dbf: host tests (`build.sh host all header_check --test`, all 25 binaries pass),
  the incremental archive, the HAL link test (1,709,877 B flash / 75,996 B RAM), examples TemplateTimed
  1,797,756 / 72,308 and DriversShowcase 1,924,612 / 73,900 (design/size.md section 12). The bridge did
  not change, so neither the oldest-core link test nor QEMU was re-run.
- Vendoring check (`python3 tools/upstream/wpilib.py --check`): all 2406 generated files match; the R17
  comment in DataLogManager.hpp is made in its patch `wpilibc/0015`.
