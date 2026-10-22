# Wave 2 completeness audit: wpimath, HAL, DataLog

Date: 2026-10-22. Branch `prosdk-2027` at `b58eb54` (plus another agent's uncommitted edits to
`design/API-DIFFERENCES.md`, `design/hal-esp32.md` and `src/prosdk/platform/hal/esp32/{DIO,PWM,SmartIo}.cpp`,
which were read as the current state). Upstream: allwpilib `b68effca4ddc9742ac208daa7dd1d11816137882`.
Only this file was written by the audit.

Question: for every upstream wpimath, HAL and DataLog C++ public header, class, free function and HAL C
function, is it ported as is, ported with a patch, stubbed, or missing? And which gaps are not justified
by `design/PORT.md` section 10?

Short answer:

- **wpimath (incl. units, struct, protobuf, json, Eigen, gcem, Sleipnir):** every upstream C++ header
  and source is vendored and compiled. Nothing is missing at file, class or function level. Five
  patches change behavior (section 2.2). Two configuration switches remove user-visible features on
  both host and target: Sleipnir diagnostics, and Eigen `operator<<`.
- **HAL:** every public header is vendored except `SystemServer.h` and `cpp/MrcLibAlert.hpp`.
  The host uses the upstream sim backend unchanged. The ESP32 backend defines all 695 backend
  functions; it does not leave any symbol undefined. Of these, 430 `HALSIM_*` functions are inert,
  as on systemcore. Of the 265 other functions, 135 are implemented and 130 are stubs or partial.
  Of the 36 Driver Station functions in the common code, 9 return fixed or empty data on ESP32.
- **DataLog:** the writer side is vendored with 3 patches. The reader side is excluded, which
  PORT.md section 10 covers. `FileLogger` is excluded, which section 10 does not cover.
- **Upstream tests:** all wpimath and HAL C++ tests run on the host. Two DataLog test files
  (10 test cases) are not vendored. No upstream test runs on the ESP32, and the ESP32 branches of
  the patches and the whole ESP32 HAL backend have no automated test.

## 1. Method and evidence

All commands ran in `/srv/probot/sdk` on 2026-10-22.

1. Vendored tree matches the script (with `PROSDK_UPSTREAM` set to the pinned checkout):
   ```
   $ flock .build/vendor.lock nice -n 19 python3 tools/upstream/wpilib.py --check
   check OK: 1770 generated files in 11 modules match (catch2, datalog, eigen, gcem, hal, licenses,
   sleipnir, telemetry, tunables, wpimath, wpiutil)
   ```
2. File-level classification. A script in the session scratchpad loaded `tools/upstream/wpilib.py`
   and `tools/upstream/modules/*.py`. It walked every file under upstream `hal/`, `datalog/` and
   `wpimath/` and assigned each one to a vendoring rule, an `exclude` entry with a reason, a
   `NOT_VENDORED` entry with a reason, or no rule. Result:
   `1216 VENDORED, 73 EXCLUDED, 1033 NOTVEND, 48 UNCOVERED`. Every vendored destination is in
   `generated.txt`. The 48 files with no rule are listed in section 6.
3. ESP32 HAL backend. A script read every `HAL_*` definition in
   `src/prosdk/platform/hal/esp32/*.cpp`, compared it with `design/hal-esp32-functions.txt` and
   printed every short body. Result: all 265 non-`HALSIM` functions are defined, and none is missing.
   The `HALSIM_*` functions are in `SimStubs.cpp`. Each stub was then read by hand and compared with
   the systemcore reference (`hal/src/main/native/systemcore/*.cpp`).
4. Archive contents. `ar t src/esp32s3/libprosdk.a` (built 04:31) contains an object for every
   `.cpp` of the datalog, hal, sleipnir and wpimath modules in `generated.txt`, and for every file in
   `src/prosdk/platform/hal/{,esp32/}`; the check by basename found 0 missing.
5. Host tests. I ran the existing binaries in `.build/host`. They were built at 04:12, after the
   last vendoring at 03:26, so they contain the current vendored tests. I did not rebuild them.
   ```
   $ flock .build/build.lock nice -n 19 ionice -c3 .build/host/hal_tests      -> All tests passed (392 assertions in 13 test cases)
   $ ... .build/host/datalog_tests                                             -> All tests passed (205 assertions in 52 test cases)
   $ ... .build/host/units_tests                                               -> All tests passed (1347 assertions in 177 test cases)
   $ ... .build/host/wpimath_tests                                             -> All tests passed (172412 assertions in 799 test cases)
   ```
   `--list-tests` on the same binaries reports 13, 52, 177 and 799 test cases. With `'[.]'`, only
   `units_tests` has a hidden test (1 test case).

   Upstream test macros (`TEST_CASE`, `TEMPLATE_TEST_CASE`, `TEST_CASE_METHOD`), counted with grep:

   | Module | Upstream | Vendored | Not vendored |
   |---|---|---|---|
   | wpimath | 943 | 943 | 0 |
   | hal | 13 | 13 | 0 |
   | datalog | 62 | 52 | 10 |

   No patch touches a test file. The ESP32 archive, the header check, the link test
   (`tools/build/hal_link_test.py`) and the ESP32 sketch build were **not** re-run for this audit;
   their last logs predate wave 2 (`.build/esp32s3-header-check.log` 01:45).

## 2. wpimath

### 2.1 Ported as is

| Upstream | Files | Disposition |
|---|---|---|
| `wpimath/src/main/native/include/wpi/math/**` | 215 headers in 16 submodules: controller 26, estimator 22, filter 6, fmt 1, geometry 36, interpolation 1, kinematics 53, linalg 7, optimization 1, path 1, random 1, shape 6, spline 9, system 12, trajectory 29, util 4 | vendored to `src/prosdk/math/**`; patched files are in 2.2 |
| `wpimath/src/main/native/include/wpi/units/**` | 45 headers | vendored to `src/prosdk/units/**` |
| `wpimath/src/main/native/cpp/**` (non-JNI) | 134 `.cpp`/`.hpp` (core, `struct/`, `proto/`, json) | `src/prosdk/impl/wpimath/**`, all compiled into the archive |
| `wpimath/src/generated/main/native/cpp/wpimath/protobuf` | 9 `.npb.h` + 9 `.npb.cpp` | vendored; C identifiers and descriptors unchanged |
| `thirdparty/eigen` (Eigen 5.0.1 as WPILib ships it, incl. `unsupported/Eigen/MatrixFunctions`) | 299 headers + 4 license files | `src/prosdk/thirdparty/eigen`; one patch (2.2) |
| `thirdparty/gcem` | 68 headers | `src/prosdk/thirdparty/gcem` |
| `thirdparty/sleipnir` | 53 headers + 10 sources | `src/prosdk/thirdparty/sleipnir`, `src/prosdk/impl/sleipnir`; 3 patches |

Every class and free function in these headers is present with its upstream signature. The only
spelling differences are the documented renames (API-DIFFERENCES.md sections 1 and 2):
`wpi::` becomes `prosdk::`, `std::expected` becomes `prosdk::cxx::expected`, and `m[i, j]`
becomes `m(i, j)` on Sleipnir matrices. One unit type cannot be used (2.3).

### 2.2 Ported with a patch

| Patch | Files | Behavior change? |
|---|---|---|
| `wpimath/0001` | `random/Normal.{hpp,cpp}`, `optimization/SimulatedAnnealing.hpp`, `path/TravelingSalesman.hpp` | Yes, but only the random sequence: one shared mt19937 instead of a new engine per call. Same distributions; the signatures do not change. |
| `wpimath/0002` | `util/MathShared.cpp` | Yes: the default `ReportError`/`ReportWarning` print to the platform console; upstream drops them. |
| `wpimath/0003` | `trajectory/DrivetrainSplineTrajectoryGenerator.cpp` | Yes: the default error output goes to the console instead of stderr. |
| `wpimath/0004` | `estimator/PoseEstimator{,3d}.hpp` | No (build fix for C++23 `optional::transform`). |
| `eigen/0001` | `Eigen/src/Core/DenseStorage.h` | Yes: compile-time cap of 8 KiB on fixed-size objects; upstream allows 128 KiB (2.3). |
| `sleipnir/0001` | `util/symbol_exports.hpp` | No. |
| `sleipnir/0002` | `util/pool.cpp`, `autodiff/expression.hpp` | Allocation only: ESP32 uses plain new/delete instead of the 16384-block pool. Results are the same. |
| `sleipnir/0003` | `optimization/problem.hpp` | No (only reachable with diagnostics on). |

### 2.3 Stubbed or reduced through configuration

`src/prosdk/platform/config.hpp` is included first by every Eigen and Sleipnir header. Its settings
apply to the host build too, so the host tests run with them.

| Item | Effect | Reason recorded |
|---|---|---|
| `SLEIPNIR_DISABLE_DIAGNOSTICS` | `Options::diagnostics` is accepted and ignored. The iteration table, spy files (`spy.hpp`), profiler output (`profiler.hpp`, `print_diagnostics.hpp`) and the warning in `Variable::set_value` on a dependent variable are compiled out. Upstream compiles diagnostics in (design/reports/wpimath.md section 1). | PORT.md section 7; API-DIFFERENCES.md section 3 |
| `EIGEN_NO_IO` | No `operator<<` for Eigen types; use `std::format` (`prosdk/math/fmt/Eigen.hpp`). | PORT.md section 7; API-DIFFERENCES.md section 3 |
| `EIGEN_STACK_ALLOCATION_LIMIT` 4096 and the fixed-size cap `PROSDK_EIGEN_FIXED_SIZE_STACK_LIMIT` 8192 | A fixed-size Eigen object over 8 KiB (for example a 33x33 `double` matrix) does not compile; upstream allows up to 128 KiB. Dynamic temporaries over 4 KiB go to the heap. | PORT.md section 7; API-DIFFERENCES.md section 3 |
| `EIGEN_MPL2_ONLY` | No effect: no vendored Eigen file tests it (`grep -rn EIGEN_MPL2_ONLY src/prosdk/thirdparty/eigen` finds nothing). | — |
| `prosdk::units::PS` (petasiemens) | Cannot be written in a sketch: the Xtensa `specreg.h` defines `PS`. Use `petasiemens_t`. | API-DIFFERENCES.md section 2 |

### 2.4 Missing

None at C++ level. Not vendored, with reasons that PORT.md section 10 covers: `cpp/jni` (13 files),
the Java API, Java sources and generators, Python bindings and tests, `src/dev`, the `.proto` sources
(their nanopb output is vendored) and `docs/`.

## 3. HAL

### 3.1 Headers and backend-independent code

| Upstream | Disposition |
|---|---|
| `include/wpi/hal/*.h`, `*.hpp`, `handles/*`, `simulation/*`, `cpp/MrcLibDs.hpp` | vendored to `src/prosdk/hal/**` as is |
| `include/wpi/hal/handles/HandlesInternal.hpp` | patch `hal/0001`: `__FIRST_SYSTEMCORE__` becomes `PROSDK_TARGET_HW`. On ESP32 the handle version byte is not checked (as on systemcore). |
| `cpp/ErrorHandling.cpp`, `Main.cpp`, `UsageReporting.cpp`, `handles/HandlesInternal.cpp` | vendored as is (`src/prosdk/impl/hal/cpp`) |
| `cpp/FIRSTDriverStation.cpp` | patch `hal/0002`: the default print goes to the platform console instead of stderr |
| `cpp/DashboardOpMode.cpp` | **stub**: hand-written no-op (`src/prosdk/platform/hal/DashboardOpMode.cpp`); `GetDashboardSelectedOpMode` returns 0. Reason: ntcore is not ported (section 10). |
| `include/wpi/hal/SystemServer.h`, `sim/SystemServer.cpp` | **missing**: returns an ntcore `NT_Inst`. Reason: ntcore is not ported (section 10). Consequence for the wpilibc wave: `wpilibc/.../system/SystemServer.cpp` and the ExpansionHub classes (`ExpansionHubMotor`, `ExpansionHubServo`, `ExpansionHubCRServo`, `ExpansionHubVelocityConstants`) have no HAL to build on. Section 10 does not name ExpansionHub. |
| `include/wpi/hal/cpp/MrcLibAlert.hpp`, `cpp/mrclib/*` | **missing**: SystemCore mrclib Driver Station and alert transport. The probot-core DS backend replaces the DS part; for alerts, PORT.md section 8 plans the DS alert widget (wpilibc wave). Section 10 does not mention mrclib. |
| `sim/*` (except `SystemServer.cpp`) | vendored as is to `host/hal/sim`, host build only. It defines every contract function except the four `HAL_*PowerDistributionStream*` functions, which upstream sim does not define either (`design/hal-esp32-functions.txt`, last section). |
| `systemcore/*` | not vendored. It is the reference for the hand-written ESP32 backend. `SimStubs.cpp` is a hand copy of `systemcore/mockdata/*.cpp`. |

### 3.2 ESP32 backend: HAL C functions (`src/prosdk/platform/hal/esp32`)

All 695 contract functions are defined. The 430 `HALSIM_*` functions (`simulation/*.h`) are inert
stubs copied from systemcore: setters do nothing, getters return fixed values and callbacks never
fire.

The table covers the 265 other backend functions. "Implemented" means real hardware or probot-core
behavior, including the documented hardware limits (pin model, unit counts) in
`design/hal-esp32.md`. The "SC" column says whether the systemcore reference implements the
function (impl) or stubs it too (stub).

| Header | Functions | Implemented | Stubbed or partial (SC) |
|---|---|---|---|
| `AddressableLED.h` | 5 | 5 | none |
| `AnalogInput.h` | 9 | 8 | `HAL_SetAnalogInputSimDevice` no-op (stub) |
| `CAN.h` | 6 | 5 | `HAL_CAN_GetCANStatus` partial: bus utilization is always 0 (impl) |
| `CANAPI.h` | 9 | 9 | none |
| `CTREPCM.h` | 22 | 0 | all 22: `HAL_InitializeCTREPCM` fails with a message; every other call returns `HAL_HANDLE_ERROR` or 0 (impl) |
| `Counter.h` | 8 | 8 | none |
| `DIO.h` | 18 | 7 | digital PWM (`HAL_AllocateDigitalPWM`, `HAL_FreeDigitalPWM`, `HAL_SetDigitalPWMRate`, `HAL_SetDigitalPWMDutyCycle`, `HAL_SetDigitalPWMPPS`, `HAL_SetDigitalPWMOutputChannel`), pulses (`HAL_Pulse`, `HAL_PulseMultiple`, `HAL_IsPulsing`, `HAL_IsAnyPulsing`): `HAL_HANDLE_ERROR` with a message (stub); `HAL_SetDIOSimDevice` no-op (stub) |
| `DutyCycle.h` | 6 | 5 | `HAL_SetDutyCycleSimDevice` no-op (stub) |
| `Encoder.h` | 18 | 17 | `HAL_SetEncoderSimDevice` no-op (stub) |
| `Extensions.h` | 6 | 3 | `HAL_LoadExtensions` (loads nothing, returns 0), `HAL_LoadOneExtension` (-1), `HAL_SetShowExtensionsNotFoundMessages` (no-op); the in-process registry works |
| `HAL.h` | 14 | 8 | `HAL_GetRSLState` and `HAL_GetCommsDisableCount`: `HAL_HANDLE_ERROR` (stub); `HAL_GetComments` empty (stub); `HAL_GetTeamNumber` always 0 (impl); `HAL_SimPeriodicBefore`/`After` no-op (no simulator) |
| `I2C.h` | 5 | 5 | none |
| `IMU.h` | 9 | 0 | all 9: zero data; the 6 functions with a status report `HAL_HANDLE_ERROR`; the 3 `HAL_GetIMUYaw*` (no status argument) return 0 (impl) |
| `Notifier.h` | 7 | 7 | none |
| `PWM.h` | 7 | 6 | `HAL_SetPWMSimDevice` no-op (stub) |
| `Ports.h` | 19 | 19 | none; the counts for CTRE/REV modules and channels are upstream values for hardware that is not supported |
| `Power.h` | 9 | 3 | `HAL_GetUserCurrent3V3` and `HAL_SetUserRailEnabled3V3`: `HAL_HANDLE_ERROR` (stub); `HAL_GetUserVoltage3V3` 3.3, `HAL_GetUserActive3V3` true, `HAL_GetUserCurrentFaults3V3` 0 and `HAL_ResetUserCurrentFaults` return fixed values with `HAL_SUCCESS` (systemcore: `HAL_HANDLE_ERROR`) |
| `PowerDistribution.h` | 25 | 0 | all 25: `HAL_InitializePowerDistribution` fails with a message; the rest return `HAL_HANDLE_ERROR` (impl) |
| `REVPH.h` | 26 | 0 | all 26, same pattern (impl) |
| `SerialPort.h` | 20 | 16 | `HAL_SetSerialReadBufferSize`/`WriteBufferSize` no-op, fixed 1 KiB (stub); `HAL_GetSerialFD` -1 (stub); `HAL_SetSerialFlowControl` accepts only "none" (stub). Also refused: mark/space parity and kUSB1/kUSB2 |
| `SimDevice.h` | 9 | 0 | all 9 inert: create returns 0, get returns unassigned (stub) |
| `Threads.h` | 4 | 4 | none (priorities mapped to FreeRTOS) |
| `UsageReporting.h` | 4 | 0 | `HAL_ReportUsage` and `HAL_Publish*Version` accept the call and drop it |

Totals: 135 implemented and 130 stubbed or partial, of which 91 are whole device classes (PCM, PH,
PDP/PDH, IMU, SimDevice).

### 3.3 ESP32 backend: Driver Station (common `FIRSTDriverStation.cpp` over `ProbotCoreDs`)

| HAL function(s) | ESP32 behavior |
|---|---|
| `HAL_GetControlWord`, `HAL_GetUncachedControlWord`, `HAL_RefreshDSData`, `HAL_GetOutputsEnabled`, new-data events, `HAL_GetMatchTime` | implemented from core: phase, `outputsEnabled`, e-stop, attached, match time |
| `HAL_SetOpModeOptions` + opmode in the control word | **partial**: the opmode reported is the first option registered for the robot mode. Core selects only AUTO and TELEOP, so there is no UTILITY mode and no opmode chooser. Core delivered R7 (draft PR #29), but `.upstream/probot-core` (`sdk-target`, `c26c042`) does not contain it and nothing is wired. |
| `HAL_GetJoystickAxes`/`Buttons`/`POVs`/`Descriptor`/`GetAllJoystickData` | implemented for the gamepads core provides. Core R2 `rawGamepad` delivers **port 0 only** (core-requests.md, Status R2); R8 (more gamepads) is pending. `design/hal-esp32.md` says "up to 6": the loop reads 6 ports, but only port 0 can return data today. Triggers are 0 or 1 (core has trigger buttons only). |
| `HAL_GetJoystickTouchpads` | always empty |
| `HAL_SetJoystickRumble`, `HAL_SetJoystickLeds` | accepted and ignored |
| `HAL_GetAllianceStation` | always `HAL_ALLIANCE_STATION_UNKNOWN` |
| `HAL_GetGameData`, `HAL_GetMatchInfo` | always empty, match type NONE |
| `HAL_WriteDisplayAnsi` | accepted and ignored ("no display", code comment only) |
| `HAL_GetSystemTimeValid` | always true, although the clock is time since boot and the wall clock (core R10) is not delivered. The reason is a code comment only. Upstream uses this flag to mean the wall clock is synchronized (DataLogManager names log files from it). |
| `HAL_ObserveUserProgramStarting`, `HAL_ObserveUserProgram` | no-op; core shows the program state itself |
| `HAL_SendError`, `HAL_SendConsoleLine`, `HAL_SendProgramCrash` | implemented (platform console) |

## 4. DataLog

| Upstream | Disposition |
|---|---|
| `DataLog.h`, `DataLog.hpp`, `DataLog.cpp` | ported with patches: `datalog/0001` (ESP32 buffers 2 KiB x 16 outgoing, 4 free; host keeps upstream values) and `datalog/0002` (default message logger writes to the console) |
| `DataLogWriter.hpp/.cpp` | ported as is |
| `DataLogBackgroundWriter.hpp/.cpp` | ported with patch `datalog/0003`: thread config; minimum free space 64 KiB on ESP32. The free-space check is skipped because there is no `statvfs`, so logging stops only when a write fails. Also `fsync` after each flush and `esp_random` file names. |
| `DataLogReader.hpp/.cpp` | not in the library (section 10: DataLog reader). Vendored as host test support only. |
| `DataLogReaderThread.hpp/.cpp` | missing (section 10: reader side; needs upb) |
| `FileLogger.hpp/.cpp` | **missing**: tails a Linux log file with inotify. Section 10 does not cover it: it is not reader code and not a desktop tool, and wpilibc `DataLogManager` uses it for system-log capture. |
| `cpp/jni/*`, Java, Python, `examples/`, `src/dev` | not vendored (section 10) |

## 5. Upstream unit tests not run on the host

| Upstream test | Test cases | Why |
|---|---|---|
| `datalog/src/test/native/cpp/DataLogReaderThreadTest.cpp` | 2 | `DataLogReaderThread` is not vendored (upb) |
| `datalog/src/test/native/cpp/FileLoggerTest.cpp` | 8 | `FileLogger` is not vendored (inotify) |
| `wpimath/.../UnitsTest.cpp` "UnitContainer DISABLED_to_string_locale" | 1 | compiled but hidden (`[.]` tag); upstream hides it too |
| `UnitsTest.cpp` blocks under `UNIT_LIB_ENABLE_IOSTREAM` | not counted | compiled out; upstream does not define the macro either |
| `hal/src/test/java`, `hal/src/test/python`, `datalog/src/test/java`, `datalog/src/test/python`, `wpimath/src/test/java`, `wpimath/src/test/python` | — | Java and Python (section 10) |
| Sleipnir, Eigen, gcem | — | the pinned allwpilib tree has no tests for them (only `include/` and `src/`) |

Everything else runs and passes: wpimath (943 upstream test macros; 799 + 177 cases after template
expansion), hal (13) and datalog (52 of 62).

Limits of the host coverage:

- The host build has `PROSDK_TARGET_HW=0`. No test exercises the ESP32 branches of `sleipnir/0002`
  (no pool), `datalog/0001` and `datalog/0003` (MCU buffers, no `statvfs`, `fsync`, `esp_random`),
  `hal/0001` (no handle version check) or the `esp_random` seeding of `wpimath/0001`.
- `hal_tests` run against the upstream **sim** backend. The hand-written ESP32 backend has no unit
  test. It is checked only by the link test (`tools/build/hal_link_test.py`) and the
  `examples/HalSmoke` hardware sketch, and this audit ran neither.
- No upstream test runs on the ESP32 itself. Software double precision, Eigen stack limits on a
  real task stack, and the Sleipnir solve time are unmeasured on the target.

## 6. Upstream files with no recorded reason

48 upstream files match no rule and no `NOT_VENDORED` entry. None of them is C++ library code:

- build files: `{hal,datalog,wpimath}/{BUILD.bazel,CMakeLists.txt,build.gradle,*-config.cmake.in,robotpy_*_build_info.bzl}`,
  `hal/src/test/native/BUILD.bazel`, `datalog/src/test/{native,java,python}/BUILD.bazel`,
  `wpimath/src/test/native/BUILD.bazel`, `wpimath/generate.bzl`, Eigen `.clang-format`;
- documentation: `hal/doc/usage.adoc`, `datalog/README.md`, `datalog/doc/datalog.adoc`,
  `datalog/doc/wpilog.ksy` (the wpilog format spec), `wpimath/README.md`;
- generators: `wpimath/generate_{nanopb,numbers,quickbuf,wpimath}.py`, `wpimath/src/test/generate_biquad_fixtures.py`;
- Java and Python: `datalog/src/printlog/java/...`, `datalog/src/test/java/**` (4 tests),
  `datalog/src/test/python/test_wpilog.py`;
- JNI headers `hal/src/main/native/cpp/jni/*.hpp` and `jni/simulation/*.hpp`. The `cpp/jni/*`
  exclude covers them in intent, but the include glob `cpp/*.cpp` never selects them.

To keep the module data complete, add `datalog/src/test/java`, `datalog/src/test/python` and
`datalog/src/printlog` to `datalog.py` `NOT_VENDORED`.

## 7. Gaps not justified by PORT.md section 10

Section 10 justifies: JNI/Java/Python, ntcore (and with it `SystemServer.h` and the
`DashboardOpMode` no-op), halsim desktop extensions (the `HAL_LoadExtensions` stubs), and the
DataLog reader and upb (`DataLogReader`, `DataLogReaderThread` and its test). Everything below is
outside section 10.

### 7.1 No reason recorded in any design document (code comment at most)

1. `HAL_GetTeamNumber` always returns 0. Systemcore returns the configured team number. Not in
   `hal-esp32.md` or API-DIFFERENCES.md.
2. `HAL_GetRSLState` returns `HAL_HANDLE_ERROR`, although probot-core drives an RSL
   (`PROBOT_RSL_PIN`). `HAL_GetCommsDisableCount` also returns `HAL_HANDLE_ERROR`. Systemcore stubs
   both too. `design/reports/hal.md` section 6.7 asked core for them; `core-requests.md` has no item.
3. `HAL_GetSystemTimeValid` always returns true with no wall clock (core R10 is pending). The
   reason is a code comment only. This is a semantic difference that DataLogManager depends on.
4. `HAL_WriteDisplayAnsi` is a no-op (code comment only).
5. `HAL_SetSerialReadBufferSize` and `HAL_SetSerialWriteBufferSize` are no-ops; the buffers are
   fixed at 1 KiB (code comment only; systemcore also no-op).
6. `HAL_GetComments` is empty (systemcore stubs it too).
7. The 3.3 V rail: `hal-esp32.md` says fault counts are "not available (`HAL_HANDLE_ERROR`)", but
   `HAL_GetUserCurrentFaults3V3` returns 0 with `HAL_SUCCESS`, `HAL_ResetUserCurrentFaults`
   succeeds, and `HAL_GetUserActive3V3`/`HAL_GetUserVoltage3V3` return fixed true/3.3. The document
   and the code disagree.
8. Joysticks: only gamepad port 0 can return data (core R2 port 0 only; R8 pending), while
   `hal-esp32.md` says "up to 6".
9. Opmode selection: core R7 is delivered (draft PR #29) but not in the `sdk-target` clone and not
   wired. There is no UTILITY mode and no DS opmode chooser; the HAL reports the first registered
   option per mode. `hal-esp32.md` records this as "not wired yet": a pending item, not an exclusion.
10. The 48 upstream non-C++ files in section 6 (bookkeeping only).

Follow-up (wave 3, 2026-10-22): items 1, 2, 7, 8 and 9 are now recorded in `hal-esp32.md`
(team number 0; RSL state and comms-disable count pending core R12; the actual 3.3 V rail
behavior; joysticks 1-5 empty until core R8). Item 9 is wired: the HAL opmode list goes to core
R7, the control word reports core's active/selected opmode id, UTILITY maps to core's UTILITY
phases, and the "first registered option" fallback is gone.

### 7.2 Reason recorded elsewhere (hal-esp32.md, API-DIFFERENCES.md, PORT.md section 7), not in section 10

1. CTRE PCM (22 functions), REV PH (26) and PDP/PDH (25) are stubbed on ESP32. Reason: they need the
   FRC CAN heartbeat and are not in the kit. wpilibc's `Compressor`, `Solenoid`, `DoubleSolenoid`,
   `PneumaticHub`, `PneumaticsControlModule` and `PowerDistribution` will compile but fail at runtime.
2. The IMU is stubbed (9 functions): there is no onboard IMU.
3. Digital PWM generators and DIO pulses are stubbed (10 functions). Systemcore stubs them too.
4. `HALSIM_*` (430 functions), `SimDevice` (9) and `HAL_Set*SimDevice` (5) are inert on ESP32,
   as on systemcore.
5. Serial: no flow control, no `HAL_GetSerialFD`, no mark/space parity, no USB ports. CAN: no CAN FD
   and no FRC CAN heartbeat; bus utilization is 0. Analog: no ADC2. LED: 3 outputs.
6. Driver Station: no alliance station, match info or game data; no joystick rumble, LEDs or
   touchpads; analog triggers are 0/1 only.
7. Brownout is reported only; it does not cut the outputs.
8. Usage reporting and version publishing are accepted and dropped.
9. Sleipnir diagnostics are compiled out (`Options::diagnostics`, spy, profiler, `set_value`
   warning), on host and target (PORT.md section 7).
10. Eigen `operator<<` is not available (`EIGEN_NO_IO`, PORT.md section 7).
11. Fixed-size Eigen objects over 8 KiB do not compile (upstream: 128 KiB); dynamic temporaries over
    4 KiB go to the heap (PORT.md section 7, `eigen/0001`).
12. `prosdk::units::PS` cannot be used (Xtensa `PS` macro; API-DIFFERENCES.md section 2).
13. `FileLogger` is excluded (inotify), with its 8 tests. Section 10 only excludes the DataLog
    reader. DataLogManager system-log capture will be missing in the wpilibc wave.
14. DataLog on ESP32: smaller buffers, and no free-space check (no `statvfs`).
15. `MrcLibAlert.hpp` and `cpp/mrclib` are excluded (SystemCore transport). The replacement alert
    path (PORT.md section 8, DS alert widget) does not exist yet.
16. `SystemServer.h` is excluded because of ntcore, which section 10 covers. The ExpansionHub
    classes that depend on it are not named in section 10 or in API-DIFFERENCES.md.

## 8. Stubs, complete list

- **HAL ESP32, whole device classes:** `CTREPCM.h` (22), `REVPH.h` (26), `PowerDistribution.h` (25),
  `IMU.h` (9), `SimDevice.h` (9), every `HALSIM_*` in `simulation/*.h` (430).
- **HAL ESP32, single functions:** the digital PWM and pulse functions (10); `HAL_Set{AnalogInput,DIO,DutyCycle,Encoder,PWM}SimDevice`;
  `HAL_LoadExtensions`, `HAL_LoadOneExtension`, `HAL_SetShowExtensionsNotFoundMessages`;
  `HAL_GetRSLState`, `HAL_GetCommsDisableCount`, `HAL_GetComments`, `HAL_GetTeamNumber`,
  `HAL_SimPeriodicBefore`, `HAL_SimPeriodicAfter`; `HAL_GetUserCurrent3V3`,
  `HAL_SetUserRailEnabled3V3`, `HAL_GetUserVoltage3V3`, `HAL_GetUserActive3V3`,
  `HAL_GetUserCurrentFaults3V3`, `HAL_ResetUserCurrentFaults`; `HAL_GetSerialFD`,
  `HAL_SetSerialFlowControl` (only "none"), `HAL_SetSerialReadBufferSize`,
  `HAL_SetSerialWriteBufferSize`; `HAL_CAN_GetCANStatus` (bus utilization 0); `HAL_ReportUsage`,
  `HAL_PublishVersion`, `HAL_PublishWpilibVersion`, `HAL_PublishCanVersion`.
- **HAL ESP32, Driver Station:** `HAL_GetAllianceStation`, `HAL_GetGameData`, `HAL_GetMatchInfo`,
  `HAL_GetJoystickTouchpads`, `HAL_SetJoystickRumble`, `HAL_SetJoystickLeds`,
  `HAL_WriteDisplayAnsi`, `HAL_GetSystemTimeValid` (always true), opmode selection (partial; R7 not
  wired), joystick ports 1-5 (no data).
- **HAL common:** `DashboardOpMode.hpp` (all 5 functions, no-op).
- **HAL host sim:** `HAL_StartPowerDistributionStream`, `HAL_GetPowerDistributionStreamData`,
  `HAL_FreePowerDistributionStreamData` and `HAL_StopPowerDistributionStream` are undefined, as
  upstream. Nothing in wpilibc calls them.
- **wpimath/Sleipnir/Eigen:** Sleipnir diagnostics (compiled out), Eigen stream output (compiled out).
- **DataLog:** the free-space check in `DataLogBackgroundWriter` on ESP32 (skipped: no `statvfs`).
