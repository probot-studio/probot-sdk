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
                          LLVM Apache-2.0 w/ LLVM exception, double-conversion BSD-3, json (jtjson) ISC/MIT, htrie MIT)
LICENSE, LICENSE-commercial  unchanged
src/
  prosdk.h                umbrella header (hand-written)
  prosdk/                 include root for everything (only src/ is on the Arduino include path)
    util/ math/ units/ telemetry/ tunables/ commands2/ hal/ datalog/ ...   vendored (generated)
    framework/ driverstation/ hardware/ system/ event/ drive/ simulation/ ... vendored wpilibc (generated)
    thirdparty/{eigen,gcem,sleipnir,nanopb,...}/                         vendored thirdparty (generated)
    cxx/                  hand-written C++23 compatibility shims (section 6)
    platform/             hand-written ESP32 and host platform glue (HAL backend, threads, time, console)
    probot/               hand-written probot-core bridge (header side compiled in the sketch, section 8)
    drivers/              hand-written prosdk devices (BTS7960, VNH5019, MPU6050, ProHub, Servo, ...)
  esp32s3/libprosdk.a     precompiled archive (build output, section 9)
tools/upstream/           vendoring script, module selection, patches, lock
tools/build/              CMake toolchain + build wrapper
test/                     host tests (upstream unit tests + prosdk tests); never under src/
examples/                 Arduino sketches
design/                   this file and the probot-core contract
```

Thirdparty headers live under `src/prosdk/thirdparty/<lib>/` so they cannot collide with other Arduino
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
  `CommandHelper<Base, CRTP>` using a `__PRETTY_FUNCTION__`-based `prosdk::util::GetTypeName<T>()`;
  subsystems use the same through a helper, with `SetName()` still available.
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
- Eigen: `EIGEN_STACK_ALLOCATION_LIMIT=4096`, `EIGEN_NO_IO`, `EIGEN_MPL2_ONLY`.
- Memory-heavy defaults are scaled for the MCU and configurable by macro (DataLog buffers, and so on).

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

## 9. Build (incremental, low load)

- Out-of-Arduino CMake build of `libprosdk.a` with a toolchain file that reproduces the Arduino recipe
  (`tools/build/esp32s3.cmake`). One OBJECT library per module; the archive is assembled from them.
  Build dir `.build/esp32s3` persists; only changed translation units recompile.
- Host build (`.build/host`) with system g++ in gnu++20 and the sim HAL, for upstream unit tests.
- ccache (static binary in `tools/bin`, cache in `.build/ccache`) and ninja (pip venv) are used when present.
- Every compile runs as `nice -n 19 ionice -c3`, at most `-j2`, under the lock `flock .build/build.lock`, so
  parallel agents never stack compiles on the shared server.
- Sketch builds: `arduino-cli compile --build-path <persistent dir> --jobs 2`, never `--clean`.
- Flash/RAM numbers are measured from real sketch builds (`xtensa-esp32s3-elf-size`) and recorded.

## 10. Scope

Ported (all features, nothing silently dropped): wpiutil (needed set), wpimath (all, incl. Sleipnir,
struct, protobuf, json), units, telemetry, tunables, datalog writer side, hal (common API + ESP32 backend +
sim backend for host), wpilibc (framework incl. TimedRobot/OpModeRobot, driverstation incl. all
controller classes, hardware, motor controllers from `drivers/`, simulation classes, system, event,
drive, LED, Alert, Preferences on NVS, sysid log), commandsv2 (all incl. generated HID wrappers and
SysIdRoutine), GoBildaPinpoint and other `drivers/` devices that fit the hardware.

Re-implemented for this platform: `NetworkButton` over tunables; DataLog sink to flash/SD and to the DS;
telemetry and tunables backends over probot-core; Servo; prosdk drivers carried over from the old SDK
(BTS7960, VNH5019, MPU6050 gyro, ProHub PCA9685 + UART encoder).

Not ported, with reason: JNI/Java/Python bindings (no JVM), commandsv3 (Java-only, uses JVM
continuations), ntcore/wpinet (replaced by the probot-core DS link; an NT4-compatible server for
AdvantageScope/Elastic is a later work item), cscore/cameraserver (no camera on the kit), halsim desktop
GUI extensions, DataLog reader and upb (desktop tool side), argparse, Romi/XRP vendordeps (other robots).
Every exclusion is listed in `design/API-DIFFERENCES.md`.

## 11. Evidence

Nothing is "done" without: the command that was run, its output (compile log, test summary, size table),
and where it ran. Unmeasured performance is called an estimate.

## Reports behind this design

Copied into the repo:
`design/reports/`
(`wpimath.md`, `wpiutil.md`, `telemetry_tunables.md`, `hal.md`, `wpilibc.md`, `commands.md`, `core.md`,
`toolchain.md`, `critic.md`).
