# Wave 4 completeness audit: whole project

Date: 2026-10-22. Branch `prosdk-2027` at `acc5abc`. Upstream: allwpilib
`b68effca4ddc9742ac208daa7dd1d11816137882`. probot-core: `.upstream/probot-core`, branch `sdk-target`
at `71a5c19` (core draft PRs #26-#30). Only this file was written by the audit. Other agents had
uncommitted edits while it ran (`.github/workflows/ci.yml`, `tools/build/{CMakeLists.txt,build.sh,
esp32s3.cmake}`: debug-stripped archive copy and `build.sh examples`); the builds below ran on that
working tree.

Question: for every C++ library in allwpilib at the pin, is it ported fully, ported with documented
differences, partially, excluded with a recorded reason, or missing without one? For the ported
libraries, which public classes or free functions are stubbed or missing, and which upstream C++ tests
do not run on the host? Which wave-2 and wave-3 gaps are closed? What is still missing for a user to
run a robot end to end?

Short answer:

- **Libraries.** 11 upstream C++ libraries are vendored (wpiutil, telemetry, tunables, wpimath incl.
  units and Eigen/gcem/Sleipnir, hal, datalog, wpilibc, commandsv2, drivers, fields, apriltag) plus
  Catch2 for the host tests. All of them are "ported with documented differences": every file that is
  not vendored has a reason in the module data and in `API-DIFFERENCES.md`. Every other upstream
  library with C++ is excluded with a reason in PORT.md section 10, **except** the desktop
  applications and infrastructure `glass`, `wpigui`, `thirdparty/imgui_suite`, `tools/*` (sysid,
  wpical, datalogtool, outlineviewer), `benchmark`, `developerRobot`, `shared/singlelib` and the
  upstream examples `wpilibcExamples`, which no design document mentions (section 2).
- **Files.** The vendored tree matches the script (2406 files, 16 modules). In the 11 vendored
  library directories no C/C++ file is without a rule or a reason; 64 non-C++ files (build files,
  Java/Python tests, test images) in the four wave-4 modules have neither (section 2.3).
- **Builds and tests.** Host: 17 test binaries, all passed. ESP32: archive and header check up to
  date, every source has an archive object, all 7 examples and the 731-function HAL link test link
  (section 1). Nothing ran on an ESP32.
- **Wave-2/3 gaps.** All 12 unjustified items of wave 3 except the upstream-examples decision are
  closed; the wave-2 items still open in wave 3 are closed (section 5).
- **End to end.** Nothing has run on an ESP32 yet. Core P0/P1 work is in unmerged draft PRs, R8-R13
  and three DS widgets are pending, the last 10 commits are not pushed (CI last ran at `48beaf2`), and
  there is no release of either library (section 6).

## 1. Method and evidence

All commands ran in `/srv/probot/sdk` on 2026-10-22 with `PROSDK_UPSTREAM` set to the pinned
checkout.

1. Vendoring drift check:
   ```
   $ flock .build/vendor.lock nice -n 19 python3 tools/upstream/wpilib.py --check
   check OK: 2406 generated files in 16 modules match (apriltag, catch2, commandsv2, datalog, drivers,
   eigen, fields, gcem, hal, licenses, sleipnir, telemetry, tunables, wpilibc, wpimath, wpiutil)
   ```
2. File classification. A script (session scratchpad, not committed) loaded
   `tools/upstream/modules/*.py` and assigned every file of the pinned tree (`git ls-tree -r HEAD`, so
   directories outside the sparse checkout count too) to VENDORED (a rule includes it), EXCLUDED (a
   rule's `exclude` with reason), NOTVEND (a `NOT_VENDORED` entry with reason) or UNCOVERED. C/C++ files
   (`.c .cc .cpp .h .hpp .inc .inl .mac`) per upstream library:

   | Upstream | VENDORED | EXCLUDED | NOTVEND | UNCOVERED |
   |---|---|---|---|---|
   | wpiutil | 217 | 14 | 185 | 0 |
   | telemetry | 19 | 0 | 8 | 0 |
   | tunables | 25 | 0 | 17 | 0 |
   | wpimath (incl. units, Eigen, gcem, Sleipnir) | 1020 | 13 | 48 | 0 |
   | hal | 159 | 52 | 75 | 0 |
   | datalog | 14 | 7 | 4 | 0 |
   | wpilibc | 362 | 22 | 12 | 0 |
   | commandsv2 | 137 | 0 | 1 | 0 |
   | drivers | 23 | 1 | 2 | 0 |
   | fields | 29 | 0 | 2 | 0 |
   | apriltag | 67 | 4 | 2 | 0 |
   | thirdparty/catch2 | 290 | 0 | 0 | 0 |
   | all other libraries (section 2.2) | 0 | 0 | 0 | 1,342 |

   (The NOTVEND C/C++ files of telemetry/tunables are their Python-binding sources.)
3. Host build and tests (incremental):
   ```
   $ tools/build/build.sh host all --test
   == prosdk_tests            All tests passed (188 assertions in 23 test cases)
   == prosdk_dashboard_tests  All tests passed (432 assertions in 61 test cases)
   == prosdk_robot_tests      All tests passed (171 assertions in 7 test cases)
   == wpiutil_tests           All tests passed (23221 assertions in 993 test cases)
   == telemetry_tests         All tests passed (210 assertions in 35 test cases)
   == tunables_tests          All tests passed (485 assertions in 77 test cases)
   == hal_tests               All tests passed (392 assertions in 13 test cases)
   == datalog_tests           All tests passed (205 assertions in 52 test cases)
   == units_tests             All tests passed (1347 assertions in 177 test cases)
   == wpimath_tests           All tests passed (172412 assertions in 799 test cases)
   == fields_tests            All tests passed (46 assertions in 7 test cases)
   == apriltag_tests          All tests passed (9 assertions in 8 test cases)
   == wpilibc_tests           All tests passed (1574968 assertions in 609 test cases)
   == commandsv2_tests        All tests passed (705 assertions in 221 test cases)
   == prosdk_commands2_tests  All tests passed (46 assertions in 4 test cases)
   == drivers_tests           All tests passed (275 assertions in 23 test cases)
   == prosdk_drivers_tests    All tests passed (192 assertions in 13 test cases)
   ```
   `<binary> '[.]' --list-tests`: hidden cases only in `units_tests` (1, hidden upstream too).
4. ESP32 archive and header check (incremental):
   ```
   $ tools/build/build.sh esp32s3 prosdk header_check
   [0/2] Re-checking globbed directories...
   ninja: no work to do.
   ```
   `ar t src/esp32s3/libprosdk.a`: 503 objects; every one of the 503 `.c`/`.cpp` files (generated.txt
   sources plus hand-written `platform/`, `drivers/`, `cxx/`, `probot/`) has an object (by basename,
   0 missing). Header check: 859 TUs in `.build/esp32s3/header-check` = every vendored public header
   except the documented internal ones (Eigen `src/`, gcem `gcem_incl/`, 3 AprilTag C fragments) plus
   the 47 hand-written headers.
5. ESP32 sketches (`tools/build/build.sh examples`, arduino-cli, huge_app, against the archive above
   and `.upstream/probot-core`) and the HAL link test
   (`python3 tools/build/hal_link_test.py --out <scratch> --compile`): results in section 1.1.
6. Upstream test macros (`TEST_CASE`, `TEST_CASE_METHOD`, `TEMPLATE_TEST_CASE`, ... at line start),
   counted with grep in the upstream `src/test/native` and `src/generated/test/native` trees and in
   `test/<module>` after patches:

   | Module | Upstream | Vendored (after patches) |
   |---|---|---|
   | wpiutil | 461 | 457 |
   | telemetry | 35 | 35 |
   | tunables | 77 | 77 |
   | wpimath | 943 | 943 |
   | hal | 13 | 13 |
   | datalog | 62 | 52 |
   | wpilibc | 346 | 297 |
   | commandsv2 | 174 | 174 |
   | drivers | 23 | 23 |
   | fields | 7 | 7 |
   | apriltag | 8 | 8 |
7. probot-core: `git log` and `grep` of the `sdk-target` clone (fetched 09:06) and its remote-tracking
   branches; `gh run list -R probot-studio/probot-sdk` for CI.

### 1.1 ESP32 sketch builds

```
$ tools/build/build.sh examples          # all 7, build folders .build/examples/<name>, exit 0
== AprilTagFieldPose       Sketch uses 2102008 bytes (66%) ... Global variables use 70836 bytes (21%)
== CommandBasedRobot       Sketch uses 2128900 bytes (67%) ... Global variables use 69340 bytes (21%)
== DriversShowcase         Sketch uses 1998016 bytes (63%) ... Global variables use 67412 bytes (20%)
== HalSmoke                Sketch uses 1687057 bytes (53%) ... Global variables use 67148 bytes (20%)
== OpModeRobotExample      Sketch uses 1940332 bytes (61%) ... Global variables use 66268 bytes (20%)
== TimedRobotDifferential  Sketch uses 1946028 bytes (61%) ... Global variables use 66276 bytes (20%)
== TimesliceRobotExample   Sketch uses 1922336 bytes (61%) ... Global variables use 66196 bytes (20%)

$ python3 tools/build/hal_link_test.py --out <scratch>/hallink --compile
HalLinkTest.ino: 731 HAL functions from 42 headers, plus GetDefaultDriverStationImpl and 2 SimDevice constructors
Sketch uses 1773841 bytes (56%) of program storage space. Maximum is 3145728 bytes.
Global variables use 68636 bytes (20%) of dynamic memory, leaving 259044 bytes for local variables.
arduino-cli exit 0
```

All seven examples compile and link; none was run (no board). The three robot examples grew since
PORT.md section 8.1 recorded them (TimedRobotDifferential 1,926,441 -> 1,946,028 B, OpModeRobotExample
1,904,333 -> 1,940,332, TimesliceRobotExample 1,902,981 -> 1,922,336; RAM 65.7 KB -> 66.2 KB): the
section 8.1 figures are stale. The largest sketch uses 67 % of the huge_app partition.

## 2. Library classification

### 2.1 Vendored libraries

| Upstream | Status | Not vendored (reason recorded in module data and API-DIFFERENCES.md section 4) |
|---|---|---|
| wpiutil | ported with documented differences (9 patches) | `jni_util`, `MappedFileRegion`, `MemoryBuffer`, `SmallVectorMemoryBuffer` (mmap), `MessagePack` + mpack (ntcore), `PixelFormat`/`RawFrame` (cscore), `ProtobufMessageDatabase` + upb (desktop reader), argparse, `debugging` (debugger detection), `windows/` |
| telemetry | ported with documented differences (1 patch; DS backend hand-written) | nothing (Java, Python, docs) |
| tunables | ported with documented differences (1 patch; DS backend hand-written) | nothing (Java, Python, docs) |
| wpimath, units, Eigen, gcem, Sleipnir | ported with documented differences (4 + 1 + 3 patches, config switches; audit-wave2.md section 2) | JNI, `src/dev`, `.proto` sources (nanopb output vendored) |
| hal | ported with documented differences: common code + host sim backend vendored, ESP32 backend hand-written (`hal-esp32.md`) | `SystemServer.h` (ntcore), `MrcLibAlert.hpp` + `cpp/mrclib` (SystemCore transport), `DashboardOpMode.cpp` (ntcore; hand-written no-op), `systemcore/` (reference only), JNI |
| datalog | ported with documented differences (writer side, 3 patches) | `DataLogReader` (host test support only), `DataLogReaderThread`, `FileLogger` (inotify; replaced by the console tee of `wpilibc/0012`) |
| wpilibc | ported with documented differences (12 patches) | `NetworkTablesTelemetryBackend`, `NetworkTablesTunableBackend` (replaced by probot-core backends), `SystemServer`, 6 `ExpansionHub*` classes |
| commandsv2 | ported with documented differences (4 patches) | nothing (NT constructors of `NetworkButton` removed by patch) |
| drivers | ported fully (unchanged) | `empty.cpp` |
| fields | ported with documented differences (1 patch) | field images (PNG/JPG) and their registry |
| apriltag | ported with documented differences (2 patches) | `AprilTagDetector_cv.hpp` (OpenCV), `AprilTagImageGenerator` + `AprilTag.cpp` (`RawFrame`) |

prosdk additions (not upstream): `Servo` (WPILib 2026 API), `BTS7960`, `VNH5019`, `PCA9685`,
`ProHubMotor`, `ProHubEncoders`/`ProHubEncoder`, `MPU6050` (API-DIFFERENCES.md section 7).

### 2.2 Libraries not vendored

| Upstream | C/C++ files | Status |
|---|---|---|
| ntcore, wpinet | 166 + 306 | excluded, PORT.md section 10 (NT4 server for AdvantageScope/Elastic: "later work item") |
| cscore, cameraserver | 106 + 11 | excluded, section 10 (no camera on the kit) |
| romiVendordep, xrpVendordep | 11 + 15 | excluded, section 10 (other robots) |
| simulation/halsim_gui, halsim_ds_socket, halsim_ws_core, halsim_ws_server, halsim_ws_client, halsim_xrp | 39 + 4 + 32 + 11 + 8 + 8 | excluded, section 10 (desktop plugins). The plain sim backend (`hal/src/main/native/sim`) is vendored as `host/hal/sim` |
| aos_sdk, aosnt, ntcoreffi | 0 + 8 + 3 | excluded, section 10 |
| commandsv3 | 0 (Java) | excluded, section 10 (Java only) |
| epilogue-runtime, epilogue-processor, wpiunits, wpiannotations | 0 (Java) | no C++ to port; recorded only in audit-wave3.md, not in PORT.md/API-DIFFERENCES.md |
| glass, wpigui, thirdparty/imgui_suite | 127 + 10 + 64 | **not mentioned** in PORT.md or API-DIFFERENCES.md (desktop GUI) |
| tools/sysid, tools/wpical, tools/datalogtool, tools/outlineviewer | 41 + 40 + 10 + 2 | **not mentioned** (desktop applications; sysid analysis and wpical have C++ tests) |
| benchmark | 44 | **not mentioned** (google-benchmark runs of CartPole/TravelingSalesman on wpimath) |
| developerRobot, shared/singlelib, wpilibj/src/dev, wpilibjExamples/src/dev | 1 + 1 + 1 + 1 | **not mentioned** (developer scratch / build helpers) |
| wpilibcExamples (38 examples, 31 snippets, 8 templates, 8 command examples; tests for ArmSimulation, ElevatorSimulation, UnitTest) | 283 | **no decision recorded** (wave-3 item 12). prosdk has 7 examples of its own |

None of the unmentioned items is robot library code; the gap is that the scope document does not say so.

### 2.3 Bookkeeping: upstream files with no rule and no reason

Inside the 11 vendored library directories every C/C++ file is covered. Non-C++ files with no rule and
no `NOT_VENDORED` entry (the wave-2/3 bookkeeping item, now closed for wpiutil, wpimath, hal, datalog,
wpilibc, telemetry, tunables, recurs in the wave-4 modules):

- apriltag (17): `BUILD.bazel`, `CMakeLists.txt`, `README.md`, `apriltag-config.cmake.in`,
  `build.gradle`, `robotpy_*_build_info.bzl`, `src/test/native/BUILD.bazel`, `src/test/java/**`
  (2 Java tests + BUILD), `src/test/python/**` (test_detection.py + 4 images + BUILD);
- commandsv2 (22): build files, `CommandsV2.json` (vendordep descriptor), `generate.bzl`,
  `generate_hids.py`, `src/generated/main/java/**` (15 generated Java controllers);
- fields (18): build files, `README.md`, `generate_fields.bzl`, `publish.gradle`,
  `src/generated/main/java/.../Fields.java`, `src/test/java/**` (4 tests), `src/test/python/**`;
- drivers (1): `src/test/native/BUILD.bazel`;
- thirdparty/catch2 (6): build files and `extras/*.cmake`.

## 3. Ported libraries: stubbed or missing public API

"Stub" = compiles and links, but does nothing useful on the ESP32 (it reports an error or returns a
fixed value). On the host every class runs on the upstream sim HAL.

### 3.1 wpiutil, telemetry, tunables, wpimath, units

- Missing (excluded): `MappedFileRegion`, `MemoryBuffer`, `SmallVectorMemoryBuffer`, `MessagePack`,
  `PixelFormat`, `RawFrame`, `ProtobufMessageDatabase`, `jni_util`, `ArgumentParser` (argparse),
  `debugging.hpp` (`is_debugger_present`, `breakpoint`), mpack, upb.
- Reduced: `GetTypeName(const T&)` static type (no RTTI); `GetStackTrace` addresses only on ESP32;
  Sleipnir diagnostics compiled out; Eigen `operator<<` absent; fixed-size Eigen objects over 8 KiB do
  not compile; `prosdk::units::PS` cannot be spelled in a sketch.
- telemetry/tunables on the DS: telemetry timestamps ignored; robust tunables start from the code
  default after a reboot (core R12 persistence); protobuf schemas over `PROBOT_DASH_MAX_VALUE` refused
  (core R13).
- No stubs otherwise.

### 3.2 hal (ESP32 backend; full list in audit-wave2.md section 8, updated here)

Since wave 2 the digital PWM generators (6 functions, LEDC), `HAL_WriteDisplayAnsi`,
`HAL_GetSystemTimeValid` and the opmode functions became real. Still stubbed:

- whole device classes: `CTREPCM.h` (22), `REVPH.h` (26), `PowerDistribution.h` (25), `IMU.h` (9),
  `SimDevice.h` (9), every `HALSIM_*` (430);
- `HAL_SetDigitalPWMPPS`, `HAL_Pulse`, `HAL_PulseMultiple`, `HAL_IsPulsing`, `HAL_IsAnyPulsing`;
- `HAL_Set{AnalogInput,DIO,DutyCycle,Encoder,PWM}SimDevice`; `HAL_LoadExtensions`,
  `HAL_LoadOneExtension`, `HAL_SetShowExtensionsNotFoundMessages`; `HAL_SimPeriodicBefore/After`;
- `HAL_GetRSLState`, `HAL_GetCommsDisableCount` (`HAL_HANDLE_ERROR`, core R12);
  `HAL_GetTeamNumber` 0; `HAL_GetComments` "";
- `HAL_GetUserCurrent3V3`, `HAL_SetUserRailEnabled3V3` (error), the other 3.3 V getters fixed values;
- `HAL_GetSerialFD`, `HAL_SetSerialFlowControl` (only "none"), `HAL_SetSerial{Read,Write}BufferSize`
  (no effect); `HAL_CAN_GetCANStatus` bus utilization 0;
- `HAL_ReportUsage`, `HAL_Publish*Version` (dropped);
- Driver Station: `HAL_GetAllianceStation`, `HAL_GetGameData`, `HAL_GetMatchInfo` (empty),
  `HAL_GetJoystickTouchpads`, `HAL_SetJoystickRumble`, `HAL_SetJoystickLeds`; joysticks 1-5 have no
  data and triggers are 0/1 (core R8);
- common code: `DashboardOpMode.hpp` (5 functions, no-op).

### 3.3 datalog

Missing: `DataLogReader` (host test support only), `DataLogReaderThread`, `FileLogger`.
ESP32: smaller buffers; the background writer's free-space check is skipped (no `statvfs`).

### 3.4 wpilibc

- Missing: `NetworkTablesTelemetryBackend`, `NetworkTablesTunableBackend`, `SystemServer`,
  `ExpansionHub`, `ExpansionHubMotor`, `ExpansionHubServo`, `ExpansionHubCRServo`,
  `ExpansionHubPositionConstants`, `ExpansionHubVelocityConstants`.
- Changed API: `NetworkBooleanEvent` (tunables; `NetworkTableInstance` overload removed).
- ESP32 stubs through the HAL: `Compressor`, `Solenoid`, `DoubleSolenoid`, `PneumaticsControlModule`,
  `PneumaticHub` (and `PneumaticsBase`), `PowerDistribution`, `OnboardIMU` (the kit IMU is
  `prosdk::MPU6050`, which is not an `OnboardIMU`); `DigitalOutput::Pulse`, `IsPulsing`, `EnablePPS`;
  `RobotController::GetRSLState`, `GetCommsDisableCount`, `GetCurrent3V3`, `SetEnabled3V3`
  (errors), `GetTeamNumber` (0), `GetComments` (""); `SerialPort::SetFlowControl` (only none),
  `SetReadBufferSize`/`SetWriteBufferSize` (no effect); `GenericHID::SetRumble`/`SetLeds`, touchpads;
  `MatchState` (no alliance/match/game data); every HAL-sim wrapper (`*Sim`, `SimHooks`) inert.
- Reduced: `Preferences` (NVS, mirrored to the DS), `DataLogManager` (LittleFS, console tee, flash-
  scaled cleanup; no NT logging), `DriverStationDisplay` (DS console), `GetThreadId()` empty on ESP32,
  `RobotBase` direct subclasses halt on the robot.
- DS widgets: `Field2d`, `Mechanism2d` show as a plain topic tree (core R5 remainder).

### 3.5 commandsv2

No class is missing. `NetworkButton` lost its four NT constructors (tunables path constructors
instead). Commands are named after their CRTP class, subsystems by static name (no RTTI). The
scheduler's `Scheduler` table has no DS widget (core R5 remainder), so running commands cannot be
cancelled from a widget; the `Command` widget works.

### 3.6 drivers

Nothing missing or stubbed (9 PWM motor controllers and `GoBildaPinpoint`, unchanged). Limitation of
the prosdk `PCA9685` (and `ProHubMotor`): outputs are not guaranteed off on an e-stop if the I2C bus
lock is held (`design/drivers.md`).

### 3.7 fields

`GetFieldImages()` / `GetFieldImage()` always empty; the per-year `GetResource_*_png()` / `_jpg()`
functions are declared in `prosdk/fields/frc/*.hpp` and `ftc/*.hpp` but defined nowhere (link error if
called; as upstream without `fieldimages`).

### 3.8 apriltag

Missing: `AprilTagImageGenerator`, `AprilTagDetector_cv.hpp`. Detection heap on the ESP32 is
unmeasured (no camera).

## 4. Upstream C++ tests not run on the host

| Upstream test | Macros | Reason |
|---|---|---|
| `wpiutil/.../JniUtilTest.cpp` | 2 | JNI |
| `wpiutil/.../argparse/ArgumentParserTest.cpp` | 1 | argparse not vendored |
| `wpiutil/.../debugging/DebuggingTest.cpp` | 1 | debugging not vendored |
| `wpiutil/.../priority_mutex_test.cpp` | 5 | vendored, compiled out (`WPI_HAVE_PRIORITY_MUTEX` is SystemCore-only, as upstream on desktop) |
| `datalog/.../DataLogReaderThreadTest.cpp` | 2 | `DataLogReaderThread` not vendored (upb) |
| `datalog/.../FileLoggerTest.cpp` | 8 | `FileLogger` not vendored (inotify) |
| `wpimath/.../UnitsTest.cpp` `DISABLED_to_string_locale` | 1 | hidden `[.]`, hidden upstream too |
| `wpilibc/.../NetworkTablesTunableBackendTest.cpp` | 36 | NT backend not ported; replaced by `prosdk_dashboard_tests` |
| `wpilibc/.../NetworkTablesTelemetryBackendTest.cpp` | 11 | same |
| `wpilibc/.../UnitNetworkTablesTest.cpp` | 3 | unit-typed NT topics |
| `wpilibc/.../hardware/ExpansionHubMotorTest.cpp` | 2 | ExpansionHub not ported |
| `wpilibc` `TimedRobotTest` `ConstructorPublishesProgramStartTime`, `ConstructorRegistersTelemetryAndTunableBackends` | 2 | replaced by `TimedRobotTest RobotHostHooks` (`wpilibc/0004`) |
| `wpilibc/src/test/native/c/test.c` | (compile check) | needs `ntcore.h` |
| `commandsv2` `CommandScheduleTest NetworkTablesTunableCancelClearsValue`, `CommandTunableButtonTest NetworkTablesLifecycleMatchesGlassRunCancel` | 2 | removed (`commandsv2/0004`); same flows in `test/prosdk/commands2/DashboardCommandsTest.cpp` |
| `wpilibcExamples/src/test/cpp/examples/{ArmSimulation,ElevatorSimulation,UnitTest}` | 14 | examples not vendored, **no decision recorded** |
| tests of excluded libraries: ntcore (230), wpinet (134), cscore (3), aosnt (52), glass (12), tools/sysid (24), tools/wpical (9), halsim_ws_server/ds_socket/gui (5), romi/xrp (2) | 471 | library excluded (section 2.2) |

Rewritten rather than dropped: `NetworkBooleanEventTest` (1 -> 5 cases, `wpilibc/0011`),
`NetworkButtonTest` (1 -> 3, `commandsv2/0003`).

Limits: every upstream test runs against the sim HAL with `PROSDK_TARGET_HW=0`. No upstream or prosdk
test runs on the ESP32; the ESP32 HAL backend, the ESP32 branches of the patches (NVS Preferences,
LittleFS paths, Sleipnir without pool, DataLog buffers, `esp_random`) and the probot-core bridge are
checked only by compiling and linking.

## 5. Earlier gaps re-checked

### 5.1 audit-wave3.md section 8

| Wave-3 item | Status at `acc5abc` |
|---|---|
| 1 `fields` missing | closed: vendored (`fields/0001`), PORT.md section 10 |
| 2 `apriltag` missing | closed: vendored; decision by measurement in PORT.md section 10 |
| 3 `aos_sdk`/`aosnt`/`ntcoreffi`, non-GUI halsim not in section 10 | closed: named with reasons |
| 4 `HAL_GetSystemTimeValid` always true | closed: true only past 2024-01-01 (`3f98a46`) |
| 5 DataLogManager never deletes old logs | closed: `wpilibc/0012` (`GetStorageSpace`, flash-scaled thresholds) |
| 6 `LogConsoleOutput()` no effect | closed: console tee (`wpilibc/0012`) |
| 7 Preferences not on the DS | closed: `DashboardPreferences` |
| 8 `NetworkBooleanEvent` not re-implemented | closed: `wpilibc/0011` |
| 9 serial buffer sizes, `GetComments()` undocumented | closed: API-DIFFERENCES.md section 3 |
| 10 ExpansionHub not in section 10 | closed |
| 11 bookkeeping (12 wpilibc + 48 wave-2 files) | closed for those modules; 64 new files in the wave-4 modules (section 2.3) |
| 12 upstream C++ examples: no decision | **open** |

### 5.2 audit-wave2.md section 7

Items 7.1.1, 7.1.2, 7.1.4, 7.1.7, 7.1.8, 7.1.9 were closed in wave 3. The three still open then are
closed: 7.1.3 (system time valid, above), 7.1.5/7.1.6 (documented), 7.1.10 (bookkeeping, above).
Section 7.2 items remain documented limits; 7.2.13 (`FileLogger`) and 7.2.15 (alert path) now have
replacements; 7.2.16 (ExpansionHub) is in section 10.

## 6. Still missing for a user to run a robot end to end

1. **No hardware run.** No sketch (not `HalSmoke`, not a robot example) has been flashed or run on an
   ESP32-S3; this server has no board attached. Unverified on the target: the HAL backend (pins,
   MCPWM, PCNT, capture, RMT, TWAI, UART, I2C, LEDC), the output gate and e-stop timing, the hosted
   loop on core's task (stack 24 KiB, timing, overruns), NVS Preferences, LittleFS DataLog,
   std::format/exception paths, Sleipnir/Eigen on a task stack, and every `drivers/` device. The kit
   electronics are not final (KIT.md section 3), so pin maps (ProHub, I2C/serial defaults) are
   provisional.
2. **probot-core is not released with what prosdk needs.** R1-R7 are in draft PRs #26-#30, held until
   core 0.4.0 is on stable; prosdk builds only against the read-only `sdk-target` clone. A user with
   core from Library Manager (`stable` `e5907a5`) gets the `CoreBridge.hpp` version error.
3. **Pending core requests** (none in `sdk-target`; `feat/multi-gamepad` has no own commits):
   - R5 remainder: DS widgets `Field2d`, `Mechanism2d`, `Scheduler` (Selectable, Command, Alerts,
     PIDController exist). Other WPILib telemetry types (`DifferentialDrive`, `MecanumDrive`,
     `Motor Controller`, encoders, `HID:*`, `Subsystem`, `BangBangController`, ...) show as a plain
     tree; no widget is requested for them.
   - R8: gamepads 1-5, analog triggers, 8-way POV, name/type, rumble.
   - R9: console levels, larger buffer.
   - R10: wall clock from the browser (DataLog file names and `systemTime`).
   - R11: file download (`.wpilog` from LittleFS/SD). Until then a log can only be read by dumping the
     LittleFS partition. PORT.md section 10 nevertheless lists "DataLog sink ... to the DS" as
     re-implemented.
   - R12: `requestStop()` (a halted program still looks enableable on the DS), `rslState()`,
     `commsDisableCount()`, `battery::currentAmps()`, brownout flag, robust-tunable persistence.
   - R13: schema size limit for protobuf topics.
4. **Not pushed, no CI since `48beaf2`.** CI on `dev` passed at `48beaf2` (run 39727874071,
   2026-10-22T05:24Z). The 10 commits after it (commandsv2, fields, apriltag, drivers,
   NetworkBooleanEvent, Preferences on the DS, DataLogManager) and the CI changes that compile the
   examples and run `hal_link_test.py` (uncommitted, another agent's work) have not run in CI. PORT.md
   section 9 already describes that CI step. Pushing needs Tuna's approval.
5. **No release.** `src/esp32s3/libprosdk.a` is gitignored ("not tracked for now"), there is no
   release or Library Manager entry, and building prosdk from sources inside a sketch (no archive) was
   not tested. README says "Not yet usable for robots".
6. **User-facing rough edges seen in this audit:** every sketch that includes tunables prints
   `'if consteval' only available with '-std=c++2b'` warnings (`ComplexTunable.hpp:54,61`;
   accepted by design, PORT.md section 6, but visible to users); host tests write `.wpilog` files into
   the repository's untracked `logs/` directory (6 files after this run).

## 7. Gaps not justified by a design document

1. Upstream C++ examples/templates/snippets (`wpilibcExamples`, 85 programs, 14 test macros): no
   decision on which are ported or why not (wave-3 item 12, still open).
2. Desktop applications and infrastructure with C++ not named in PORT.md section 10 or
   API-DIFFERENCES.md section 5: `glass`, `wpigui`, `thirdparty/imgui_suite`, `tools/sysid`,
   `tools/wpical`, `tools/datalogtool`, `tools/outlineviewer`, `benchmark`, `developerRobot`,
   `shared/singlelib`; Java-only `epilogue-*`, `wpiunits`, `wpiannotations` are recorded only in
   audit-wave3.md.
3. API-DIFFERENCES.md section 3 row "Pneumatics ..., PowerDistribution, OnboardIMU, CAN (...)" says
   the kit has no CAN bus and the ESP32 backend errors on construction; the backend implements CAN on
   TWAI (`hal-esp32.md` "CAN", `src/prosdk/platform/hal/esp32/CAN.cpp`). The row is wrong for CAN.
4. PORT.md section 10 lists "DataLog sink to flash/SD and to the DS" as re-implemented; there is no
   DS sink (needs core R11). Flash/SD works through `DataLogManager`'s directory.
5. Bookkeeping: 64 non-C++ upstream files in apriltag, commandsv2, fields, drivers and catch2 have no
   rule and no `NOT_VENDORED` reason (section 2.3).
6. PORT.md section 8.1 example sizes are stale (section 1.1: the robot examples are 20-36 KB larger
   now). The same section and section 9 describe the `build.sh examples` target and the CI example /
   link-test step, which exist only as another agent's uncommitted edits at `acc5abc`.
