# prosdk: WPILib 2027 port to ESP32-S3 on top of probot-core

This is the engineering contract for the port. Every agent and contributor follows it.
Investigation reports that back these decisions are listed at the end.

## 1. What prosdk is

- prosdk is the C++ API of WPILib 2027 (allwpilib `main`), running on an ESP32-S3 under Arduino-ESP32 3.3.10.
- It sits on top of probot-core (`depends=probot`). probot-core owns `setup()`/`loop()`, WiFi, the browser
  Driver Station, the robot state machine and e-stop. prosdk never duplicates them.
- A user sketch includes only `<prosdk.h>`. It pulls in `<probot.h>` itself.
- The API matches WPILib: same class names, method names and semantics. Only the root namespace and
  include root change: `wpi::` becomes `prosdk::` and `<wpi/...>` becomes `<prosdk/...>`. Sub-namespaces
  stay: `prosdk::math`, `prosdk::units`, `prosdk::util`, `prosdk::cmd`, `prosdk::telemetry`,
  `prosdk::tunables`, `prosdk::hal`, `prosdk::log`.
- Not a FIRST/FRC product. The words FIRST, FRC and WPILib never appear in the product name or marketing
  (BSD-3 clause 3). Attribution goes in `NOTICE` and the upstream license files.

## 2. Upstream pin and vendoring (no fork)

- Upstream: `https://github.com/wpilibsuite/allwpilib`, branch `main`, commit
  `b68effca4ddc9742ac208daa7dd1d11816137882` (2026-10-20). The pin lives in `tools/upstream/wpilib.lock`.
- `tools/upstream/wpilib.py` produces every vendored file. It follows WPILib's own `upstream_utils` pattern:
  1. fetch the pinned commit into a cache (`.upstream/allwpilib`, gitignored; sparse checkout);
  2. copy the selected files of each module (selection data in `tools/upstream/modules/<module>.py`);
  3. apply the mechanical rename (section 4) and the Arduino-macro guard (section 5);
  4. apply the patches in `tools/upstream/patches/<module>/NNNN-*.patch` in order (`git apply`);
  5. write `tools/upstream/generated.txt`, the list of every file it produced.
- Re-running the script with the same pin gives byte-identical output. It deletes only files listed in the
  previous `generated.txt`, never anything else.
- **Vendored files are never edited by hand.** Every change to upstream code is a patch file. A CI check
  re-runs the script and fails on any diff.
- Hand-written code lives only in paths the script never writes (section 3).
- Updating upstream = bump the pin, re-run, refresh patches that no longer apply.

## 3. Repository layout

```
library.properties        name=prosdk, depends=probot, precompiled=full, includes=prosdk.h
NOTICE                    attribution (WPILib BSD-3, Eigen MPL-2.0, gcem Apache-2.0, Sleipnir BSD-3, nanopb zlib,
                          LLVM Apache-2.0 w/ LLVM exception, double-conversion BSD-3, JSON for Classic C++ Apache-2.0, htrie MIT)
LICENSE, LICENSE-commercial  unchanged
src/
  prosdk.h                umbrella header (hand-written)
  prosdk/                 include root for everything (only src/ is on the Arduino include path)
    util/ math/ units/ telemetry/ tunables/ commands2/ hal/ datalog/ ...   vendored (generated)
    framework/ driverstation/ hardware/ system/ event/ drive/ simulation/ ... vendored wpilibc (generated)
    thirdparty/{eigen,gcem,sleipnir,nanopb,...}/                         vendored thirdparty (generated)
    cxx/                  hand-written C++23 compatibility shims (section 6)
    platform/             hand-written ESP32 and host platform glue (HAL backend, threads, time, console)
      platform/hal/       hand-written replacements for backend-independent HAL files (DashboardOpMode no-op)
      platform/hal/esp32/ the ESP32 HAL backend (target only; contract: design/hal-esp32-functions.txt)
    probot/               hand-written probot-core bridge (header side compiled in the sketch, section 8)
    drivers/              hand-written prosdk devices (BTS7960, VNH5019, MPU6050, ProHub, Servo, ...)
  esp32s3/libprosdk.a     precompiled archive (build output, section 9)
tools/upstream/           vendoring script, module selection, patches, lock
tools/build/              CMake toolchain + build wrapper
host/hal/sim/             WPILib HAL simulation backend (generated), compiled only in the host build; never
                          under src/, so the Arduino tree never sees it
test/                     host tests (upstream unit tests + prosdk tests); never under src/
examples/                 Arduino sketches
design/                   this file and the probot-core contract
```

Upstream code that already lives under `wpi/` (llvm, json, htrie, sigslot, double-conversion) follows the
normal rename into `src/prosdk/`. Thirdparty headers with their own include root live under `src/prosdk/thirdparty/<lib>/` so they cannot collide with other Arduino
libraries (for example ArduinoEigen). The script rewrites their angle includes, for example
`<Eigen/Core>` becomes `<prosdk/thirdparty/eigen/Eigen/Core>`; quoted relative includes are left alone.

## 4. Mechanical rename (applied to every vendored file, in this order)

1. Include paths: `<wpi/` and `"wpi/` become `<prosdk/` and `"prosdk/`. This includes `__has_include(<wpi/...>)`
   probes (for example in `units/base.hpp`).
2. Namespaces: `namespace wpi` becomes `namespace prosdk`; `wpi::` becomes `prosdk::`; `::wpi::` likewise.
3. wpiutil's top-level `namespace fs` becomes `prosdk::fs` (Arduino `FS.h` declares `namespace fs`).
4. Thirdparty include roots move under `prosdk/thirdparty/` (section 3).
5. **Kept unchanged** (wire formats and C ABI): protobuf package and C identifiers (`wpi.proto.*`,
   `wpi_proto_*`), struct type names (`struct:Pose2d`), telemetry type strings, `HAL_*` and `WPI_*` C API
   names, `WPILIB_*` macro names. The exclusion list is data in the script, not ad-hoc sed.
6. Product strings shown to users (error text that says "FRC Driver Station", and so on) are patched only
   where they are wrong for this platform.

## 5. Arduino macro collisions

`Arduino.h` defines object-like macros that break C++ identifiers: `DEFAULT`, `PI`, `HALF_PI`, `TWO_PI`,
`DEG_TO_RAD`, `RAD_TO_DEG`, `EULER`, `HIGH`, `LOW`, `INPUT`, `OUTPUT`, `INPUT_PULLUP`, `INPUT_PULLDOWN`,
`RISING`, `FALLING`, `CHANGE`, `ANALOG`, `B0`..`B11111111`, `abs`, `min`, `max`, `round`, `constrain`,
`radians`, `degrees`, `sq`, `bit`, `F` (function-like), `SERIAL`, `DISPLAY`, and others.

- Every vendored header gets a generated prologue/epilogue that `#pragma push_macro` + `#undef`s the
  colliding macros it actually uses and `pop_macro`s them at the end. The list per header is computed by the
  script (token scan against the macro list taken from the installed core headers), not hand-maintained.
- An identifier that **users must type** and that collides (for example an enum value named `DEFAULT`)
  is renamed by a table in the script. Use the WPILib Java spelling if it differs; otherwise prefix `k`
  (`kDefault`). Every such rename is listed in `design/API-DIFFERENCES.md`.
- CI compiles every public header after `#include <Arduino.h>` with the exact Arduino flags.

## 6. C++ standard

- Arduino-ESP32 3.3.10 compiles sketches as **gnu++20** (`-std=gnu++2b` is followed by `-std=gnu++2a` in
  `cpp_flags`). prosdk headers must compile as gnu++20 with no `build_opt.h`.
- The precompiled archive is built with exactly the same standard (gnu++20), so header and archive agree on
  every type layout.
- C++23 library features used upstream are provided by hand-written shims in `src/prosdk/cxx/`, and the
  script rewrites the uses: `std::expected`/`std::unexpected`/`std::unexpect` to `prosdk::cxx::...`
  (own implementation, always, never an alias to `std::expected`), `std::print`/`std::println` to
  `prosdk::cxx::print`/`println` (on top of `std::format` + `fwrite`), `std::byteswap`, `std::to_underlying`,
  `std::invoke_r`, `std::unreachable`, and whatever else the gnu++20 build reports.
- `if consteval` is accepted by GCC 14 in C++20 mode as an extension; it stays.
- If a C++23 *language* feature that GCC 14 rejects in C++20 mode appears, patch it at the use site.
- Multidimensional subscript (`m[i, j]`, C++23) is the exception: gnu++20 does not reject the use, it
  compiles it as a comma expression (`m[j]`), and rejects only multi-parameter `operator[]` declarations.
  Sleipnir and wpimath use it throughout, so the script rewrites it (`multi_subscript` transform:
  `m[i, j]` -> `m(i, j)`, `operator[](a, b)` -> `operator()(a, b)`); the header check shows no
  `-Wcomma-subscript` warning afterwards.
- Fallback, only if the shim approach proves unworkable: build everything as gnu++23 and ship `build_opt.h`
  with `-std=gnu++2b` in every example. This needs a written reason in this file before it is taken.

## 7. Platform rules (ESP32-S3)

- Flags: identical to Arduino's `@flags/cpp_flags` (machine and codegen flags included), `-Os`, `-fno-rtti`,
  `-fexceptions`, `-ffunction-sections -fdata-sections`, no LTO. Archive references nothing whose type
  depends on board menu options (never `Serial`); logging goes through the platform console API.
- Target macro: `PROSDK_TARGET_HW=1` on the ESP32 build, `0` on the host build. It replaces every
  `__FIRST_SYSTEMCORE__` test (priority_mutex.hpp, HandlesInternal.hpp, RobotBase.hpp IsSimulation/IsReal,
  Preferences.cpp, DataLogManager.cpp, MotorSafety.cpp).
- RTTI: off. `GetTypeName(*this)` (Demangle.hpp) is replaced: commands get their name from
  `CommandHelper<Base, CRTP>` using a `__PRETTY_FUNCTION__`-based `prosdk::util::GetTypeName<T>()`
  (patch `commandsv2/0001`); subsystems keep the static base name (`prosdk::cmd::SubsystemBase`, which
  upstream C++ also gives, since it names them in the base constructor) and are named with
  `SubsystemBase(name)` or `SetName()`. No other `typeid`/`dynamic_cast` is used by commandsv2.
- Exceptions: on. Upstream throw sites stay. The robot framework catches exceptions that escape user code,
  reports them to the DS console and moves the robot to a safe stopped state instead of rebooting.
- Threads: no WPILib thread is created with the default pthread config (2 KB stack, priority 5, which
  would preempt the probot-core control loop at priority 4). Every creation site (SafeThread, Notifier,
  DataLog writer, ...) goes through `prosdk::platform::ThreadConfig` (`esp_pthread_set_cfg`): core 1,
  priority below 4, explicit stack size.
- Spinlocks map to mutexes on ESP32 (`spinlock.hpp` patch).
- Time: `prosdk::util::Now()` uses `esp_timer_get_time()`. Monotonic, nanoseconds.
- Stack traces: ESP backtrace PCs (`esp_backtrace_*`) or empty; never `execinfo`.
- Sleipnir: no 16384-block thread-local pool on ESP32 (use plain allocation or a small pool);
  `SLEIPNIR_DISABLE_DIAGNOSTICS`.
- `<random>`: one shared engine seeded from `esp_random()`; no per-call `random_device` + `mt19937` on the stack.
- Eigen: `EIGEN_STACK_ALLOCATION_LIMIT=4096` (runtime alloca of dynamic temporaries), a separate
  compile-time cap of 8192 for fixed-size objects (`PROSDK_EIGEN_FIXED_SIZE_STACK_LIMIT`, patch
  `eigen/0001`; Sleipnir's `slp::solve` needs 4608 bytes), `EIGEN_NO_IO`, `EIGEN_MPL2_ONLY`.
- Memory-heavy defaults are scaled for the MCU and configurable by macro (DataLog buffers, and so on).

### 7.1 Sketch and archive must agree

- Public headers never depend on a `-D` flag that only the archive build gets. Configuration a header needs
  (Eigen limits, Sleipnir switches, prosdk options) lives in a hand-written config header
  (`prosdk/platform/config.hpp`) that vendored headers include (inserted by the script or a patch) before
  Eigen or Sleipnir. The header check compiles every public header exactly as a sketch TU does.
- Flash: probot-core alone is about 1.08 MB, so prosdk sketches need the Huge APP partition scheme
  (3 MB app). Wave 1 measured prosdk's share at about 565 KB for wpiutil + telemetry + tunables, mostly
  libstdc++ (`std::format`) and header templates. Wave 2 (wpimath): a smoke sketch using units,
  geometry, kinematics/odometry, TrapezoidProfile, PID, feedforwards, LQR, KalmanFilter, spline
  trajectory generation and ArmFeedforward (Sleipnir) links to 1,837,133 B flash / 51,468 B RAM
  (core alone 1,083,849 / 45,764). Size is measured after every wave.

## 8. probot-core integration

- probot-core 0.4.0 is header-only and owns the user task (core 1, 20 ms). prosdk is **hosted**: it
  never creates its own robot task. The WPILib robot loop (`IterativeRobotBase::LoopFunc`,
  `TimedRobot` periodic queue, `OpModeRobot`) is driven from core's per-cycle callback. Callbacks that
  are due run; nothing blocks.
- The precompiled archive must not depend on core internals. All calls into core go through a small
  table of function pointers (`prosdk::platform::CoreOps`). The table is filled by header-side code in
  `src/prosdk/probot/` that is compiled inside the user's sketch (where `<probot.h>` is visible) and
  registered at static-init time. Changing core's internals then never requires rebuilding the archive.
- Mapping: core `Phase` to WPILib `RobotMode` + enabled; e-stop latched to `IsEStopped`; core gamepad
  (W3C order) to WPILib HID (SDL order) with the remap in one place; battery to `RobotController`;
  telemetry/tunables backends to core's dashboard store; alerts to the DS alert widget.
- Outputs (PWM, motor controllers) are zeroed whenever core reports "outputs disabled" and immediately
  on e-stop (core calls a registered e-stop callback on its own task).
- Features prosdk needs from probot-core are listed in `design/core-requests.md` and are sent to the
  probot-core agent (`bp msg probot-core`). prosdk code is written against that contract and is compiled
  out (`#if`) until core provides it, so prosdk builds against core 0.4.0 at every step.

### 8.1 Hosted robot program (StartRobot on the robot)

- **Entry point.** A sketch ends with `PROSDK_ROBOT_MAIN(Robot)` (`prosdk/platform/RobotMain.hpp`,
  included by `<prosdk.h>`) instead of `int main() { return StartRobot<Robot>(); }`. On the robot it
  registers a factory during static initialization (`platform::SetRobotMain`, a second one is refused
  and a second macro is a link error); on the host it defines `main()` with upstream `StartRobot`.
  Several sketch files may include `<prosdk.h>`; only one has the macro.
- **Boot** (core's boot hook, once, on core's user task before its first pass; never during static
  init, never on a new task): install the robot host hooks (`RegisterDashboardBackends` at
  `RobotBase` construction, dashboard flush every loop), `RunHALInitialization()` once, construct
  the robot, `RobotBase::StartHosted()` (everything `StartCompetition()` does before its loop:
  `SimulationInit` on the host, the startup banner, `ObserveUserProgramStarting`), set core's loop
  period, then register the robot pass as the last pre-cycle callback (after the HAL output gate,
  the Driver Station pump and the dashboard).
- **Exceptions.** Constructor, startup and loop exceptions are reported as upstream `StartRobot`
  does (`RuntimeError::Report` + "The robot program quit unexpectedly...", or `what()` through
  `HAL_SendError`; anything else is reported too instead of terminating). The program then
  **halts**: its pass callback is removed, the HAL output gate stays closed
  (`platform::RobotProgramHalted()`), core keeps running so the Driver Station shows the error.
  A robot class that derives from `RobotBase` directly (blocking `StartCompetition()`) and a main
  loop period outside 1..1000 ms are reported and halt the same way.
- **Schedule.** The robot's `PeriodicPriorityQueue` keeps the upstream schedule (main loop,
  `AddPeriodic`, `TimesliceRobot` slots, `PeriodicOpMode` callbacks). Core's pass period is the
  greatest common divisor, in whole milliseconds (at most 1000), of the callback periods and of their
  phases relative to the robot's start time; it follows `AddPeriodic`/opmode changes
  (`SetLoopPeriodMs`, R6). A callback whose phase is not whole milliseconds (a `PeriodicOpMode`
  callback is timed from the opmode's construction) is moved to the nearest pass instead of
  shrinking the period; one below 1 ms runs once per pass (reported). The schedule is aligned with
  core's passes (start time and expirations shifted by less than half a pass) at the first pass,
  after a period change and after a late pass (core restarts its grid). Each pass runs
  `PeriodicPriorityQueue::RunDue(now, half a pass)`: every callback due by the middle of the next
  pass, earliest first, rescheduled exactly as `RunCallbacks()` (skips the periods it is behind),
  so jitter of the earlier pre-cycle callbacks never makes a callback miss its pass. Upstream
  watchdogs (`IterativeRobotBase`, `OpModeRobot`) still report loop overruns; core's late-pass
  count is reported as a warning (at most once per second).
- **Modes and opmodes** come only from the HAL Driver Station layer (`hal-esp32.md`): enabled = core
  `outputsEnabled()`, robot mode from the phase, the active opmode id in INIT/RUN and the selected
  one while idle (R7). `OpModeRobot::PublishOpModes()` sends the opmode list to core
  (`HAL_SetOpModeOptions` -> `SetOpModeOptions`, ids = WPILib opmode ids, stable per mode + name).
- **E-stop.** The HAL closes the output gate from core's e-stop callback. Core then deletes the user
  task, so the robot object is never touched again; the robot host's pass checks
  `EmergencyStopped()` and stops as well (host tests). Threads the program started itself
  (`Notifier`, `MotorSafety`, `Watchdog`) keep running, but every output they write is refused by
  the gate.
- **Driver Station display.** `DriverStationDisplay` frames go to the Driver Station console with
  the ANSI sequences removed and repeated frames dropped (the page has no display panel).
- **Tests.** `test/prosdk/robot` (own executable): the robot host on the HAL sim with a fake core
  table, driving core's hooks on simulated time (disabled -> auto -> teleop -> utility -> e-stop,
  callback order, `AddPeriodic`/timeslice timing, jitter, late pass, opmode selection, halts).
- **Examples** (`esp32:esp32:esp32s3:PartitionScheme=huge_app`, 2026-10-22; flash / static RAM):
  `TimedRobotDifferential` 1926441 / 65764 bytes, `OpModeRobotExample` 1904333 / 65772,
  `TimesliceRobotExample` 1902981 / 65708, `CommandBasedRobot` (commandsv2: scheduler, drive
  subsystem, `CommandXboxController`, PID command, autonomous `Selectable`, `SysIdRoutine`)
  2128932 / 69340, `DriversShowcase` (one of each `drivers/` device: Spark, Servo, BTS7960,
  VNH5019, PCA9685 with ProHubMotor and a servo, MPU6050, GoBildaPinpoint, ProHub encoders)
  1998064 / 67412 (of 3145728 / 327680).

## 9. Build (incremental, low load)

- Out-of-Arduino CMake build of `libprosdk.a` with a toolchain file that reproduces the Arduino recipe
  (`tools/build/esp32s3.cmake`). One OBJECT library per module; the archive is assembled from them.
  Build dir `.build/esp32s3` persists; only changed translation units recompile. The full archive
  (`-ggdb` from the recipe, about 1 GB of DWARF) stays in `.build/esp32s3/libprosdk.a` for
  `addr2line`; `src/esp32s3/libprosdk.a` is a copy made with `objcopy --strip-debug`, redone only when
  the full archive changed.
- Host build (`.build/host`) with system g++ in gnu++20 and the sim HAL, for upstream unit tests.
- ccache (static binary in `tools/bin`, cache in `.build/ccache`) and ninja (pip venv) are used when present.
- Every compile runs as `nice -n 19 ionice -c3`, at most `-j2`, under the lock `flock .build/build.lock`, so
  parallel agents never stack compiles on the shared server.
- Sketch builds: `arduino-cli compile --build-path <persistent dir> --jobs 2`, never `--clean`.
  `tools/build/build.sh examples [sketch...]` compiles `examples/*` that way (huge_app, build folders
  `.build/examples/<name>`, probot-core from `$PROBOT_CORE` or `.upstream/probot-core`); CI runs it and
  `tools/build/hal_link_test.py` against probot-core at `PROBOT_CORE_REF` (`.github/workflows/ci.yml`).
- Flash/RAM numbers are measured from real sketch builds (`xtensa-esp32s3-elf-size`) and recorded.

## 10. Scope

Ported (all features, nothing silently dropped): wpiutil (needed set), wpimath (all, incl. Sleipnir,
struct, protobuf, json), units, telemetry, tunables, datalog writer side, hal (common API + ESP32 backend +
sim backend for host), wpilibc (framework incl. TimedRobot/OpModeRobot, driverstation incl. all
controller classes, hardware, motor controllers from `drivers/`, simulation classes, system, event,
drive, LED, Alert, Preferences on NVS, sysid log), commandsv2 (all incl. generated HID wrappers and
SysIdRoutine), GoBildaPinpoint and other `drivers/` devices that fit the hardware, fields (field metadata and built-in AprilTag layouts, the 2027
successor of `AprilTagFieldLayout`) and apriltag (detector, pose estimator, AprilTag C library).

fields and apriltag on the ESP32, measured with `arduino-cli compile` (huge_app) of one TimedRobot
sketch in four variants (flash = "Sketch uses", RAM = static "Global variables"): empty robot 1,864,136 B
flash / 65,908 B RAM; with `AprilTagPoseEstimator::Estimate` +137,980 B flash, +0 RAM; with
`GetField(FieldId::DEFAULT_FIELD)` and `GetTagPose` +79,724 B flash, +0 RAM (all 18 fields link in; their
JSON and tag arrays are 70,239 B of flash rodata, the rest is JSON parsing); with `AprilTagDetector`
(`AddFamily("tag36h11")`, `Detect`) +50,880 B flash, +4,928 B RAM (the C library's non-const tag code
tables: tag36h11 4,696 B, tag16h5 240 B). **Decision: the detector ships.** It costs 1.6 % of the
huge_app partition, only sketches that call it pay for it (archive linking), and a camera board can pass
grayscale frames to `Detect()` although the kit has no camera. Detection heap is not measured (no camera;
estimate in `design/API-DIFFERENCES.md` section 3). The field images are not vendored (9.3 MB; upstream
builds them only for its GUI). `examples/AprilTagFieldPose`: 2,102,184 B flash (66 %), 70,836 B RAM.

Re-implemented for this platform: `NetworkButton` and `NetworkBooleanEvent` over tunables
(`wpilibc/0011`); Preferences on the DS as robust tunables `/Preferences/<key>` written back to NVS
(`src/prosdk/platform/dashboard/DashboardPreferences.*`); `DataLogManager` console capture through a
console tee and old-log deletion from esp_littlefs space (`wpilibc/0012`); DataLog sink to flash/SD and to the DS;
telemetry and tunables backends over probot-core; Servo; prosdk drivers carried over from the old SDK
(BTS7960, VNH5019, MPU6050 gyro, ProHub PCA9685 + UART encoder).

Not ported, with reason: JNI/Java/Python bindings (no JVM), commandsv3 (Java-only, uses JVM
continuations), ntcore/wpinet (replaced by the probot-core DS link; an NT4-compatible server for
AdvantageScope/Elastic is a later work item), cscore/cameraserver (no camera on the kit), DataLog
reader and upb (desktop tool side), argparse, Romi/XRP vendordeps (other robots), and:

- ExpansionHub (`wpilibc/.../hardware/expansionhub`): drives a REV Expansion Hub through SystemCore's USB
  `rhsp` server over NetworkTables topics; the probot has no rhsp server, no Expansion Hub and no ntcore.
  Motors and servos go through the ESP32 HAL and `drivers/` instead.
- `aos_sdk`, `aosnt`, `ntcoreffi`: `aos_sdk` packages AOS (the event loop/IPC framework, flatbuffers,
  libuv via wpinet) for CMake desktop/SystemCore builds; `aosnt` bridges AOS channels to NetworkTables;
  `ntcoreffi` is the C FFI of ntcore plus a C `DataLogManager` for non-WPILib programs. All are
  Linux/desktop libraries built on ntcore/wpinet, none of which exists on the ESP32.
- `simulation/halsim_*` extensions: `halsim_gui` (desktop ImGui simulator), `halsim_ds_socket` (lets
  the FRC Driver Station drive a simulated program over UDP/TCP), `halsim_ws_core`/`halsim_ws_server`/
  `halsim_ws_client` (HAL state over WebSockets for web simulators), `halsim_xrp` (XRP robot over UDP).
  They are dynamically loaded desktop plugins (`HALSIM_EXTENSIONS`, wpinet sockets); the robot has no
  dynamic loading and its Driver Station is probot-core's. Host tests use the plain sim backend.

Every exclusion is listed in `design/API-DIFFERENCES.md`.

## 11. Evidence

Nothing is "done" without: the command that was run, its output (compile log, test summary, size table),
and where it ran. Unmeasured performance is called an estimate.

## Reports behind this design

Copied into the repo:
`design/reports/`
(`wpimath.md`, `wpiutil.md`, `telemetry_tunables.md`, `hal.md`, `wpilibc.md`, `commands.md`, `core.md`,
`toolchain.md`, `critic.md`).
