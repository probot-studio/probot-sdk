# Critic report: completeness review of the 8 prosdk investigation reports

Reviewed: `wpimath.md`, `wpiutil.md`, `telemetry_tunables.md`, `hal.md`, `wpilibc.md`, `commands.md`, `core.md`, `toolchain.md`
(all in `/tmp/claude-0/-srv-probot-sdk/8fcba988-962f-4d98-ac5d-540d36df924f/scratchpad/reports/`).
Source: allwpilib b68effca sparse checkout, probot-core `/srv/probot/core`, Arduino-ESP32 3.3.10 in `~/.arduino15`.
Method: read every report in full, then greps and two `g++ -fsyntax-only` probes on 3-line files (host g++, under `nice`).
The probe files are in `.../scratchpad/critic_probe/`. Nothing was built.

---

## 1. Verified load-bearing claims (checked against the source myself)

| # | Claim (report) | Verdict | Evidence |
|---|---|---|---|
| V1 | The effective C++ standard on Arduino-ESP32 3.3.10 is **gnu++20**, not gnu++2b (wpiutil, toolchain) | **TRUE. The task CONTEXT premise "-std=gnu++2b" is wrong.** | `esp32s3-libs/3.3.10/flags/cpp_flags`: token 25 `-std=gnu++2b`, token 28 `-std=gnu++2a`. The last one wins. `<expected>`, `std::print` and the C++23 library parts fail unless the flag is overridden after cpp_flags (sketch `build_opt.h`, or a gnu++20-clean public header set). |
| V2 | probot-core runs **no hook in STOPPED/TRANSITION** and **zeroes the gamepad in every non-RUN iteration**. The user task is prio 1 with an 8 KB stack (core, commands, wpilibc) | **TRUE** | `core/lifecycle.hpp:126-141` (`step()`: `if (cur_.stage != Stage::RUN && neutralize) neutralize();`, hooks only for INIT/RUN). `inputsEnabled()` at `:112` gates WS and HTTP joystick writes (`ws_joystick.hpp:261`, `driver_station_esp32.hpp:733`). `core_config.hpp`: `PRIO_USER=1`, `STACK_USER=8192` (constexpr, not overridable). |
| V3 | The Sleipnir pool is `thread_local PoolResource pool{16384}`. The first expression allocation therefore grabs about 16384 x sizeof(Expression), several hundred KB (wpimath) | **TRUE** | `thirdparty/sleipnir/src/util/pool.cpp`. `pool.hpp:97-99` `new std::byte[bytes_per_block * blocks_per_chunk]`. `expression.hpp:22-27` enables the pool for every OS except `_WIN32`. Members are 2 doubles + intrusive ptrs + vptr, about 40-48 B on xtensa, so 650-800 KB. Must patch. |
| V4 | The only RTTI use is `wpi/util/Demangle.hpp:27` `typeid`, called by commandsv2 `Command.cpp:20`, `Subsystem.cpp:24` and `SubsystemBase.cpp:16` (wpiutil, commands, toolchain) | **TRUE** | Grep confirms 1 `typeid` and 3 `GetTypeName(*this)` call sites. |
| V5 | The 2027 HAL DS sits behind `MrcLibDs` (27 virtuals). There is no SPI/AnalogOutput/FPGA time. `TunableRegistry::Update()` is called per loop (hal, telemetry) | **TRUE** | `hal/.../cpp/MrcLibDs.hpp`: 27 `virtual`. There is no `SPI.h` in `include/wpi/hal/`. `IterativeRobotBase.cpp:158` and `OpModeRobot.cpp:167`. |

Also checked: `CONFIG_PTHREAD_TASK_STACK_SIZE_DEFAULT=2048` and **`CONFIG_PTHREAD_TASK_PRIO_DEFAULT=5`** (sdkconfig).
Also: `CONFIG_SPIRAM=y` (quad) and `CONFIG_COMPILER_CXX_EXCEPTIONS_EMG_POOL_SIZE=0`.

---

## 2. NEW blockers that no report found (verified)

### 2.1 Arduino macros break core WPILib headers, not only the enumerators wpilibc listed
`Arduino.h` is always included first in a sketch TU, so every public prosdk header is parsed with these macros active
(`cores/esp32/esp32-hal.h`/`Arduino.h`/`esp32-hal-gpio.h`). Probes with host g++:

- **`#define DEFAULT 1` breaks `wpi/util/SymbolExports.hpp`** (`critic_probe/a.cpp`: "expected primary-expression").
  The `EXPORT_TEMPLATE_STYLE` machinery expands to the token `DEFAULT`, which is then rescanned to `1`. That gives
  `EXPORT_TEMPLATE_TEST_DEFAULT_1`, and the static_assert fails. **165 wpimath headers** include SymbolExports, and Sleipnir has
  an identical copy (`sleipnir/util/symbol_exports.hpp:146-188`). So almost every prosdk header fails to compile in a sketch.
  Fix: replace both files with trivial empty `WPILIB_DLLEXPORT`/`SLEIPNIR_DLLEXPORT` definitions. DLL export is meaningless on this target.
- **`#define PI 3.14159...` breaks `wpi/units/constants.hpp:52`** (`using PI = unit<...>`; probe b.cpp fails).
  Nothing in the library includes it, but users will (`units::constants::pi`).
- `DEFAULT` also hits `tunables/TunableConfig.hpp:26` (`Polling::DEFAULT`, which is part of the **public API**) and `Selectable.cpp:16`.
  `telemetry_tunables.md` missed this.
- Full set of affected tokens (grep over all the ported modules): `RISING`/`FALLING` (Debouncer, BooleanEvent, Trigger),
  `DEFAULT` (4 files), `LOW`/`HIGH` (`Alert::Level`, AlertSim, DSBackend), `INPUT`/`OUTPUT` (`SimDevice.hpp` + 5 cpp),
  `ANALOG` (CompressorConfigType), `PI` (units). Function-like macros (`radians(`, `degrees(`, `sq(`, `constrain(`, `bit(`) gave no hits.
  **Recommendation:** add a single, enforced rename table to the mechanical rename script. Also add a CI check that compiles every public
  header after `#include <Arduino.h>`.

### 2.2 `IsReal()`/`IsSimulation()` are compile-time on `__FIRST_SYSTEMCORE__`
`RobotBase.hpp:259-278`: `IsSimulation()` is `constexpr true` unless `__FIRST_SYSTEMCORE__` is defined.
- `hal.md` §5.2 claims that "GetRuntimeType = SYSTEMCORE so `RobotBase::IsReal()` is true". **That is wrong.** On ESP32 without the macro, the
  robot reports itself as a simulation. `CommandScheduler::Run` then calls `SimulationPeriodic()` on device (commands.md §2).
- There are only **7 `__FIRST_SYSTEMCORE__` switches** in the ported set: `priority_mutex.hpp:16`, `HandlesInternal.hpp:144`,
  `RobotBase.hpp:30,130,260,273`, `Preferences.cpp:47`, `DataLogManager.cpp:74`, `MotorSafety.cpp:71`.
  Decide per site. Recommended: a new `PROSDK_TARGET_HW` macro, defined in the `.a` build and in the public headers via `prosdk/config.h`,
  so the constexpr result does not differ between TUs.

### 2.3 WPILib background threads would preempt probot-core's supervisor
The pthread default is **prio 5, any core, 2 KB stack**. Core's `sysloop` (the only TWDT subscriber, which also runs e-stop) is **prio 4**,
and the user task is prio 1. Every `std::thread` WPILib creates (Notifier per instance, Watchdog singleton, MotorSafety SafeThread,
DataLogBackgroundWriter, sim Notifier) would therefore outrank both core tasks, unless `esp_pthread_set_cfg` is applied **per thread
creation**. The setting is thread-local and inherited, so it must be applied at every creation site.
- wpiutil.md noted the 2 KB stack, but no report related prio 5 to core's `PRIO_CTRL=4`.
- hal.md says the "Notifier thread priority must be above the robot thread". wpilibc.md says "below PRIO_CTRL (4)". Both are needed:
  a band of **2-3**, pinned to core 1.

---

## 3. Contradictions between reports

| Topic | Report A | Report B | Resolution / who decides |
|---|---|---|---|
| C++ standard | Task context and telemetry_tunables/hal/commands/wpimath assume gnu++2b works | wpiutil + toolchain: effective gnu++20 (verified V1) | wpiutil wants `build_opt.h` in every sketch. toolchain prefers gnu++20-clean public headers (`prosdk::expected` shim, no `std::print` in headers). **Pick toolchain (a).** `std::expected` appears in DARE.hpp (included by 19 headers) and in HAL handle headers. `if consteval` compiles as an extension under gnu++20 (toolchain probe). |
| Who runs the robot loop | hal.md §5.5 option (A): WPILib in its **own task** | wpilibc §3.3, commands §6, core §12, telemetry §6.4: **hosted in core's user task**, plus a new core every-cycle hook. A separate task escapes e-stop and stall detection. | Hosted model. Option A conflicts with core's e-stop (`vTaskDelete(user)` only). This is the #1 architecture decision for Tuna and probot-core. |
| CAN devices (PDP/PDH, PCM/PH, Compressor, Solenoid) | hal.md: reuse the systemcore CAN protocol stack **as-is** (~15k lines) over TWAI | wpilibc.md §2: **DROP** PowerDistribution, PCM, PH, Compressor | The goal is "no missing features", so keep them behind `PROSDK_HAL_CAN` (opt-in). The FRC universal heartbeat frame is **unverified** (hal.md admits it). Without it REV/CTRE devices stay disabled. This needs a source before it is promised. |
| PWM peripheral | hal.md: **MCPWM** (12 outputs) for servo/ESC; LEDC for DigitalPWM | wpilibc.md table: PWM -> **LEDC** | MCPWM avoids the LEDC conflict below. Also see 4.3. |
| Joystick while disabled | wpilibc §4.1: "core zeroes inputs outside RUN, which **matches** WPILib's joysticks are read while disabled" | core §12 #3 and V2: WPILib **does** read real values while disabled. Core gives zeros, so `DSGamepadChooser`, disabled-mode bindings and auto selection by gamepad break | wpilibc is wrong. A core change is needed: a raw ungated buffer for reads, with the gate applied to outputs only. |
| TelemetryLoggable count | wpilibc: 38 classes | telemetry: 27 `LogTo` overrides in 59 files | Both are consistent once you count separately: 25 headers in `src/main` and 15 in `src/generated`. Not a real conflict. |
| `if consteval` count | wpimath: 12 files | toolchain: 7 headers; wpiutil: 13 uses | Different scopes (cpp vs public headers). Harmless; GCC 14 accepts it in gnu++20. |
| Debouncer stack | wpimath: patch `<random>`, mt19937 is ~5 KB of stack per call | – | Nobody else mentioned it. Keep. |

---

## 4. Risky assumptions

1. **Flash budget is unmeasured.** Core already needs "Huge APP (3 MB No OTA)" for the ~105 KB UI. wpimath + Eigen + Sleipnir
   (estimated 100-300 KB) + `std::format` + nanopb + 15 generated HIDs x2 + json + double-conversion is estimated at ">1 MB"
   (wpimath.md). Nobody measured it. The board's flash/PSRAM size is unknown: KIT.md has nothing, and sdkconfig defaults to 16 MB flash
   with quad PSRAM enabled. The real module must be confirmed before choosing the PSRAM-dependent strategies (DataLog buffers,
   Sleipnir pool).
2. **RAM budget is unaggregated.** Each report sizes its own piece: telemetry 30-50 KB, DataLog defaults 1 MiB, HAL handle mutexes
   ~5 KB, 16-24 KB user stack, 3+ WPILib threads (each at least 4-8 KB), WiFi AP + httpd. Nobody summed these against the free
   internal heap (~320 KB claimed, never measured).
3. **Soft-float double performance.** Every report says "measure" (LTV controllers, UKF, ArmFeedforward Newton solve per loop).
   No number exists yet. The ESP32-S3 FPU is single precision only.
4. **`-Os`.** Arduino compiles with `-Os`. The precompiled `.a` can use `-O2` for wpimath; nobody discussed this. It matters
   for soft-double math.
5. **Exceptions under OOM.** `EMG_POOL_SIZE=0`, so throwing `bad_alloc` under memory pressure terminates. 102 wpimath + 12 commands +
   57 telemetry/tunables throw sites rely on `-fexceptions`. The DrivetrainSplineTrajectoryGenerator catch path is control flow
   (wpimath.md).
6. **LEDC conflicts with core guidance.** Core API.md:150-157 tells users to drive motors with `analogWrite` (auto-allocates LEDC
   channels from 0) and servos on LEDC channel 7. hal.md reserves LEDC ch0-5 for DigitalPWM. Mixing them collides. This is one
   more reason PWM should be on MCPWM and DigitalPWM should allocate LEDC dynamically through the Arduino allocator.
7. **Multi-TU and precompiled `.a` vs header-only core** (core §11, toolchain §1.8): prosdk `.cpp` files cannot include `<probot.h>`
   without the WiFi macros. Macro-dependent COMDAT bodies (`USER_LOOP_PERIOD_MS` etc.) can silently diverge, and there is no
   probot-core version macro. Every report that writes "prosdk calls `probot::...`" from a `.cpp` assumes a core refactor
   (core §12 #19) that has not been requested yet.
8. **Strong required hooks.** If prosdk defines `teleopLoop()` etc. for the hosted model, users cannot define them
   (link error). Users of the plain FTC style and users of the WPILib style need separate opt-in headers. wpilibc §3.3 assumes
   prosdk owns all ten hooks.
9. **Wire-name preservation.** wpimath says keep `wpi.proto.*`, `struct:Pose2d` and the `wpi_proto_*` C identifiers through the
   rename. A blanket `wpi` -> `prosdk` sed would also hit `WPI_*` C symbols (HAL/wpiutil C API), `__has_include(<wpi/...>)` probes
   and generated nanopb names. No report specifies the rename script or its exclusion list.

---

## 5. Modules and features no report covered

| Module | C++ content | Assessment |
|---|---|---|
| **drivers/** `odometry/GoBildaPinpoint` (571 hpp + 1090 cpp) | Real I2C driver: Pose2d/Rotation3d, units, `wpi/hardware/bus/I2C.hpp` only | **Not covered by any report; high value.** The goBILDA Pinpoint odometry computer is common on FTC-style robots. It ports directly once HAL I2C (Wire) exists. |
| drivers/ `motor/Koors40` | One more generated PWM controller | wpilibc listed the 9 PWM controllers including Koors40, from `pwm_motor_controllers.json` (jinja templates in `drivers/src/generate`). OK. |
| romiVendordep (4 cpp) / xrpVendordep (6 cpp) | SimDevice-backed classes driven over halsim_ws/halsim_xrp from a desktop | Not covered. On ESP32 they are only useful as API patterns (XRPServo/RomiServo show the missing `Servo` shape). **Drop**, but say so explicitly. |
| simulation/ (halsim_ds_socket, halsim_gui, halsim_ws_client/core/server, halsim_xrp; 96 files) | Desktop extensions | hal.md drops them. **Open question:** `halsim_ws_server` speaks the WPILib "WebSocket sim" protocol. It could connect the host sim build to `/srv/probot/sim` (the web simulator, apps/sim in probot-studio). Not evaluated. |
| ntcore (132 C++ files) / wpinet (280) | NT4 server/client, libuv, mDNS, web server | All reports say "drop". **Unanswered:** the NT4 protocol is WebSocket + JSON + msgpack (mpack is already in wpiutil). A minimal NT4 *server* on the ESP32 httpd would let **AdvantageScope, Elastic and Glass** connect directly, and the telemetry/tunables backends could mirror `NetworkTablesTelemetryBackend` more closely. Nobody weighed this against the bespoke `'A'/'V'/'R'/'C'/'W'/'Q'` protocol proposed in telemetry_tunables.md. Decide before core protocol work starts. |
| datalog (18 files) | DataLog, writers, reader | Covered only inside wpiutil.md §4 (writer portable, reader dropped, 1 MiB buffer default). No report designed DataLogManager/SysIdRoutineLog storage end to end (LittleFS vs SD vs WS download). |
| commandsv3 / epilogue-runtime / wpiunits / wpiannotations | Java only (0 C++ files; verified) | Correctly skipped. epilogue (annotation logging) has no C++ counterpart. Say so in the docs. |
| upstream_utils | Patch sets for Eigen, gcem, htrie, double-conversion, etc. | Not covered. WPILib's vendored thirdparty is **patched** (`eigen_patches/`, `gcem_patches/`...). The port must copy the patched tree from the checkout, not upstream releases. This is also the model for a reproducible prosdk sync script. |
| wpimath `monotonic_clock.hpp` in hal | Small | Not mentioned. Check it with the timestamp work. |
| Examples/templates | not in the sparse checkout | No report looked at wpilibc examples/templates. prosdk needs ported examples for the OpModeRobot/TimedRobot/command-based models. |
| apriltag (not checked out) | `AprilTagFieldLayout` | Only relevant with vision. Out of scope, but state the decision. |

---

## 6. Important questions nobody answered

1. **License compatibility.** WPILib is BSD-3 (`LICENSE.md`, "FIRST and other WPILib contributors"). The Eigen files are MPL-2.0
   (file-level copyleft). prosdk's repo is **public** with `LICENSE` (MIT) + `LICENSE-commercial` (Commons Clause, per sdk
   CLAUDE.md). Questions: can Commons Clause terms be placed on BSD-derived code (yes, but the notices must be kept), and on MPL
   files (no, they stay MPL)? How are `ThirdPartyNotices.txt` and the copyright headers kept through the `wpi`->`prosdk` rename?
   **Tuna needs to decide this.**
2. **Upstream sync strategy.** Is the port a one-shot fork or a scripted transform (rename + patch queue) that can be re-run on new
   allwpilib commits? This determines how the Arduino-macro renames, the C++20 shims and the ESP patches are stored.
3. **Exact board:** module, flash size, PSRAM yes/no, and the available GPIOs after core, NeoPixel, USB and PSRAM pins.
4. **Single robot model for users.** OpModeRobot (closest to core) vs TimedRobot vs command-based: which one is the default
   template? What happens to users of plain FTC hooks with no WPILib class?
5. **Test infrastructure.** wpimath uses gtest and commandsv2 uses Catch2. The existing sdk uses a custom harness (31 tests).
   Pick one host framework, and decide whether the sim HAL (`hal/sim`) is the host backend (hal.md says yes; nobody else confirmed).
6. **Error-code band.** core suggests `PS-Exxx`. No report decided how WPILib's `WPIErrors.mac` codes and Alerts map to the
   DS console.
7. **NT4 server vs custom protocol** (section 5).

---

## 7. Recommended next actions (priority order)

1. Ask probot-core (`bp msg probot-core`, after Tuna approves) for one change package: every-cycle framework hook, init/boot hook,
   ungated raw gamepad + `outputsEnabled()`, configurable user stack, version macro, multi-TU-safe headers, and a frame/URI
   extension API.
2. Write the mechanical port script with: namespace/include rename, exclusion list (wire names, `WPI_*` C API if kept), the
   Arduino-macro rename table (2.1), SymbolExports stubs, the `PROSDK_TARGET_HW` replacement for `__FIRST_SYSTEMCORE__` (2.2),
   and C++20 shims.
3. Run one low-priority probe build of the `.a` (wpiutil + wpimath + telemetry + tunables) to get **real flash/RAM numbers**,
   plus a QEMU timing run for PID/LTV/UKF/ArmFeedforward.
4. Write a thread policy: an `esp_pthread_set_cfg` wrapper at every `std::thread`/SafeThread creation, prio 2-3, core 1, 4-8 KB.
   Plus the HAL output gate (hal §5.4 and wpilibc §4.4 agree on this).
5. Patch Sleipnir's pool, `<random>` per-call engines, `GetStackTrace` and `Demangle`. All reports agree on these.
6. Decide the CAN, NT4 and license questions with Tuna before porting those parts.
