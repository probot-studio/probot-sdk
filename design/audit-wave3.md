# Wave 3 completeness audit: wpilibc

Date: 2026-10-22. Branch `prosdk-2027` at `84e4c43`. Upstream: allwpilib
`b68effca4ddc9742ac208daa7dd1d11816137882`. Only this file was written by the audit. Other agents had
uncommitted edits in the worktree while it ran (`design/hal-esp32.md`,
`src/prosdk/platform/{PreferencesStore,RobotMain}.cpp`, `src/prosdk/platform/hal/esp32/DriverStation.cpp`,
`test/prosdk/{PreferencesTest,robot/RobotHostTest}.cpp`). This audit judges the committed state and names
those edits where they touch a gap.

Question: for every upstream wpilibc C++ public class, free function and test (`src/main/native` and
`src/generated`), is it ported as is, ported with a patch, stubbed, or missing? Which wave-2 gaps
(`design/audit-wave2.md` section 7) are closed? What is still missing for "every WPILib 2027 C++ feature"
(commandsv2 and `drivers/` are wave 4 and out of scope here)?

Short answer:

- **Files.** Every upstream wpilibc header and source is vendored except 25 files, all with a reason
  (ntcore: the two NT backends, `NetworkBooleanEvent`, `SystemServer`, the six ExpansionHub classes, and
  their tests). The vendored tree matches the script, every source is in the ESP32 archive and every
  public header passes the ESP32 header check.
- **Classes.** About 160 upstream class or enum names in 162 public headers (6 headers declare only
  free functions). 10 classes are missing (`NetworkTablesTelemetryBackend`,
  `NetworkTablesTunableBackend`, `NetworkBooleanEvent`, `SystemServer`, 6 `ExpansionHub*`), 17 classes or functions are changed by the 10 `wpilibc/*` patches, and on
  the ESP32 about 15 classes are stubbed fully or partly by the HAL (pneumatics, `PowerDistribution`,
  `OnboardIMU`, digital PWM/pulses of `DigitalOutput`, parts of `RobotController`, `SerialPort`,
  `GenericHID`, `MatchState`, the HAL-sim wrappers). Everything else is upstream code, unchanged apart
  from the mechanical renames.
- **Tests.** 69 of 74 upstream test files (289 of 343 test macros) build and pass on the host
  (`wpilibc_tests`: 604 test cases, all passed). 53 test macros in 5 files are not vendored (ntcore,
  ExpansionHub) and 2 `TimedRobotTest` cases are replaced by 1 prosdk case. No upstream test runs on the
  ESP32.
- **Wave-2 gaps.** 7 of the 10 unjustified items of wave 2 are closed (documented or implemented);
  `HAL_GetSystemTimeValid`, the serial buffer-size setters / `HAL_GetComments` and the bookkeeping item
  are still open.
- **Still missing outside wpilibc** (beyond commandsv2/drivers): the `fields` library and the
  `apriltag` library, which no design document mentions; plus the recorded exclusions (ntcore/wpinet,
  cscore/cameraserver, Romi/XRP, halsim GUI) and pending probot-core requests (R8-R13, R5 widgets).

## 1. Method and evidence

All commands ran in `/srv/probot/sdk` on 2026-10-22, `PROSDK_UPSTREAM` set to the pinned checkout.

1. Vendored tree matches the script:
   ```
   $ flock .build/vendor.lock nice -n 19 python3 tools/upstream/wpilib.py --check
   check OK: 2129 generated files in 12 modules match (catch2, datalog, eigen, gcem, hal, licenses,
   sleipnir, telemetry, tunables, wpilibc, wpimath, wpiutil)
   ```
2. File-level classification: the wave-2 script (loads `tools/upstream/wpilib.py` and
   `tools/upstream/modules/*.py`, walks the upstream tree, assigns each file to a rule, an `exclude`
   with reason, a `NOT_VENDORED` entry or nothing) run on `wpilibc/`:
   `359 VENDORED (152 header, 133 source, 69 test, 5 test-header), 25 EXCLUDED, 249 NOTVEND, 12 UNCOVERED`.
   Every vendored destination is in `generated.txt`.
3. Class inventory: a script listed the namespace-level `class`/`struct`/`enum` names of every upstream
   header under `wpilibc/src/main/native/include` and `wpilibc/src/generated/main/native/include`
   (162 headers). Each was compared with the excludes, the 10 patch files
   (`tools/upstream/patches/wpilibc`, every hunk read) and the ESP32 HAL functions it calls
   (grep of the vendored sources against the stub list of `audit-wave2.md` section 8, re-read in
   `src/prosdk/platform/hal/esp32/*.cpp`).
4. Host build and tests (incremental, nothing was out of date):
   ```
   $ tools/build/build.sh host all --test
   ninja: no work to do.
   == prosdk_tests            All tests passed (121 assertions in 15 test cases)
   == prosdk_dashboard_tests  All tests passed (389 assertions in 55 test cases)
   == prosdk_robot_tests      All tests passed (169 assertions in 7 test cases)
   == wpiutil_tests           All tests passed (23221 assertions in 993 test cases)
   == telemetry_tests         All tests passed (210 assertions in 35 test cases)
   == tunables_tests          All tests passed (485 assertions in 77 test cases)
   == hal_tests               All tests passed (392 assertions in 13 test cases)
   == datalog_tests           All tests passed (205 assertions in 52 test cases)
   == units_tests             All tests passed (1347 assertions in 177 test cases)
   == wpimath_tests           All tests passed (172412 assertions in 799 test cases)
   == wpilibc_tests           All tests passed (1574958 assertions in 604 test cases)
   ```
   `wpilibc_tests '[.]' --list-tests`: 0 hidden test cases. `--list-tests --verbosity high` maps the
   604 cases to 65 source files; the other 4 vendored test files (`main.cpp`, `TestCallbackHelpers.cpp`,
   `motorcontrol/Mock*.cpp`) contain no test cases.
5. ESP32 archive and header check (incremental):
   ```
   $ tools/build/build.sh esp32s3 prosdk header_check
   ninja: no work to do.
   ```
   `ar t src/esp32s3/libprosdk.a`: an object exists for every wpilibc `.cpp` in `generated.txt`
   (0 missing by basename). Every wpilibc public header has a header-check TU
   (`.build/esp32s3/header-check/*.cpp`) except `WPIErrors.mac`/`WPIWarnings.mac`, which are X-macro
   fragments included by `Errors.hpp`.
6. ESP32 sketch link check of the stubbed classes. A scratch sketch (`W3Audit.ino`, session scratchpad,
   not committed) constructs `Compressor`, `DoubleSolenoid`, `PowerDistribution`, `OnboardIMU`,
   `DigitalOutput` (+ `Pulse`, `EnablePWM`), `SerialPort` (+ `SetReadBufferSize`), `CAN`, `Tachometer`,
   `AddressableLED`, `LEDPattern`, `PWMSim`, `DriverStationSim`, `DCMotorSim`, `Preferences`,
   `DataLogManager`, `SysIdRoutineLog`, `Field2d`, `Mechanism2d`, `DriverStationDisplay`,
   `RobotController` in a `TimedRobot` with `PROSDK_ROBOT_MAIN`:
   ```
   $ flock .build/build.lock nice -n 19 ionice -c3 arduino-cli compile \
       --fqbn esp32:esp32:esp32s3:PartitionScheme=huge_app --libraries <scratch libs> \
       --build-path <scratch>/build/W3Audit --jobs 2 <scratch>/audit3/W3Audit
   Sketch uses 2113769 bytes (67%) of program storage space. Maximum is 3145728 bytes.
   Global variables use 68316 bytes (20%) of dynamic memory, leaving 259364 bytes for local variables.
   ```
   `examples/OpModeRobotExample` (same command, its existing build path): 1904409 bytes flash, 65772
   bytes RAM. Neither sketch was run on hardware.

## 2. File-level disposition

| Upstream | Files | Disposition |
|---|---|---|
| `src/main/native/include/wpi/**` | 130 + 2 `.mac` | 120 vendored to `src/prosdk/**`; 10 excluded (section 3.6) |
| `src/generated/main/native/include/wpi/**` | 30 | vendored: 15 controllers, 15 `*ControllerSim` |
| `src/main/native/cpp/**` | 113 | 103 vendored to `src/prosdk/impl/wpilibc/cpp`; 10 excluded |
| `src/generated/main/native/cpp/**` | 30 | vendored to `src/prosdk/impl/wpilibc/generated` |
| `src/test/native/{cpp,include}`, `src/generated/test/native/cpp` | 63 + 5 + 11 | 69 + 5 vendored to `test/wpilibc`; 5 excluded |
| `src/test/native/c` | 1 | not vendored: C compile check of `HAL.h` with `ntcore.h` |
| `src/generate/**` | 10 | not vendored: Jinja templates; their output is vendored. `WPILibVersion.cpp.in` is replaced by hand-written `src/prosdk/platform/WPILibVersion.cpp` |
| `src/main/python`, `src/test/python` | 198 + 38 | not vendored (Python) |
| `src/dev` | 2 | not vendored (developer scratch) |
| `BUILD.bazel`, `CMakeLists.txt`, `build.gradle`, `generate.bzl`, `generate_{hids,first_ds_hids,wpilibc}.py`, `publish.gradle`, `robotpy_*_build_info.bzl`, `wpilibc-config.cmake.in`, `src/test/native/BUILD.bazel` | 12 | no rule and no `NOT_VENDORED` entry (build and generator files; bookkeeping only) |

## 3. Classes and free functions

"As is" = vendored upstream code with only the mechanical renames (API-DIFFERENCES.md sections 1-2).
"ESP32" = behavior of the hand-written ESP32 HAL backend; on the host every class runs on the upstream
sim backend.

### 3.1 framework/, opmode/, internal/

| Class / function | Disposition |
|---|---|
| `RobotBase` | patch `0001`: no NT server, no NT backends, no `/Robot/ProgramStartTime`, calls `platform::OnRobotBaseConstructed()`; `IsReal`/`IsSimulation` on `PROSDK_TARGET_HW`. Patch `0010`: new virtual `StartHosted()` + `HostedLoop`; on ESP32 `GetThreadId()` is `std::thread::id{}`. Behavior change: yes (documented) |
| `StartRobot<T>`, `RunHALInitialization`, `impl::RunRobot`, `impl::ResetMotorSafety` | patch `0001`: robot thread through `ScopedThreadConfig`, console output. On the robot replaced by `PROSDK_ROBOT_MAIN` (hand-written `platform/RobotMain.*`) |
| `IterativeRobotBase` | patch `0002`: `FlushTelemetry()` instead of NT `FlushLocal`; "Override me" lines to the console |
| `TimedRobot` | patches `0002`, `0010` (`StartHosted`, loop unchanged on the host) |
| `TimesliceRobot` | as is |
| `OpModeRobotBase`, `OpModeRobot<T>` | patches `0002`, `0010` |
| `OpMode`, `PeriodicOpMode` | as is |
| `internal::PeriodicPriorityQueue` | patch `0010`: adds `RunDue`, `Update`, `GetVersion`; existing members unchanged |
| `internal::DriverStationModeThread` | patch `0008`: thread config only |

### 3.2 driverstation/ (incl. generated)

| Class | Disposition |
|---|---|
| `internal::DriverStationBackend` | patch `0003`: the `/DriverStation` NT table is not published; rest unchanged |
| `DriverStation`, `RobotState`, `HIDDevice`, `POVDirection`, `TouchpadFinger`, `Alliance`, `MatchType`, `Gamepad`, `Joystick` | as is. ESP32: joystick ports 1-5 have no data until core R8; triggers 0/1 |
| `MatchState` | as is. ESP32: no alliance, match info or game data (empty / unknown) |
| `GenericHID` | as is. ESP32: `SetRumble`/`SetLeds` ignored, touchpads empty |
| `DriverStationDisplay`, `DSGamepadChooser` | as is. ESP32: frames go to the DS console without ANSI (no display panel) |
| 15 generated controllers (`XboxController`, `NiDs{Xbox,PS4,PS5,Stadia}Controller`, `DualShock4Controller`, `DualSense{,Edge}Controller`, `LogitechF310Controller`, `SteamController`, `Switch{Pro,2Pro,2GC,N64}Controller`, `GameCubeController`) | as is; the five PlayStation classes have the `Button::k*` and `PSButton()` renames |

### 3.3 event/

`EventLoop`, `BooleanEvent`: as is (`Debounce` default is `kBoth`, rename). `NetworkBooleanEvent`:
**missing** (section 3.6).

### 3.4 hardware/

| Class | Disposition |
|---|---|
| `ADXL345_I2C`, `AnalogAccelerometer`, `I2C`, `EdgeConfiguration`, `EdgeCounter`, `Tachometer`, `AnalogInput`, `CounterBase`, `DigitalInput`, `PWM`, `AddressableLED`, `LEDPattern`, `MotorController`, `PWMMotorController`, `SharpIR`, `AnalogEncoder`, `AnalogPotentiometer`, `DutyCycle`, `DutyCycleEncoder`, `Encoder` | as is; ESP32 limits in `hal-esp32.md` (4 PCNT units, 12 PWM, 6 capture, ADC1 only, 3 LED outputs, output gate) |
| `CAN`, `CANPort` | as is. ESP32: TWAI bus 0 only, classic CAN, no FRC heartbeat |
| `SerialPort` | as is. ESP32 partial: `SetFlowControl` only "none", `SetReadBufferSize`/`SetWriteBufferSize` silently ignored (1 KiB fixed), USB ports refused |
| `DigitalOutput` | as is. ESP32: `Pulse`, `IsPulsing`, `EnablePWM`, `DisablePWM`, `UpdateDutyCycle`, `SetPWMRate`, `EnablePPS` report `HAL_HANDLE_ERROR` (stub, as systemcore) |
| `MotorSafety` | patch `0001` (`ResetMotorSafety` host only, as upstream SystemCore) |
| `OnboardIMU` | as is. ESP32: **stub** (no onboard IMU; reads report an error, yaw 0) |
| `Compressor`, `Solenoid`, `DoubleSolenoid`, `PneumaticsBase`, `PneumaticsControlModule`, `PneumaticHub`, `CompressorConfigType` (k-renamed), `PneumaticsModuleType` | as is. ESP32: **stub** (construction fails at run time, HAL CTREPCM/REVPH) |
| `PowerDistribution` | as is. ESP32: **stub** |
| `ExpansionHub`, `ExpansionHubMotor`, `ExpansionHubServo`, `ExpansionHubCRServo`, `ExpansionHubPositionConstants`, `ExpansionHubVelocityConstants` | **missing** (section 3.6) |

### 3.5 drive/, smartdashboard/, backend/, preferences/, sysid/, system/, util/, simulation/

| Class / function | Disposition |
|---|---|
| `RobotDriveBase`, `DifferentialDrive`, `MecanumDrive` | as is |
| `Field2d`, `FieldObject2d`, `Mechanism2d`, `MechanismRoot2d`, `MechanismObject2d`, `MechanismLigament2d` | as is. On the DS the data appears in the topic tree; the Field2d/Mechanism2d widgets are still pending in core (R5 remainder) |
| `DataLogTelemetryBackend` | as is |
| `NetworkTablesTelemetryBackend`, `NetworkTablesTunableBackend` | **missing**, replaced by hand-written `ProbotTelemetryBackend`/`ProbotTunableBackend` (`src/prosdk/platform/dashboard`) |
| `Preferences` | patch `0005`: NVS / memory / JSON store instead of the NT table. Not visible or editable on the DS |
| `sysid::SysIdRoutineLog` | as is (logs through `DataLogManager`) |
| `DataLogManager` | patches `0006`, `0007`: log directory `/littlefs/logs` or a discarding log; `LogNetworkTables()` and `LogConsoleOutput()` no effect; console output via `ConsoleWrite`. ESP32: the free-space cleanup never runs (section 7) |
| `filesystem::GetLaunchDirectory/GetOperatingDirectory/GetDeployDirectory` | patch `0006` (LittleFS paths) |
| `Notifier`, `Watchdog` | patch `0008` (thread config only) |
| `RobotController`, `CANStatus` | as is. ESP32 partial: `GetRSLState`, `GetCommsDisableCount` error (core R12); `GetTeamNumber` 0; `GetComments` ""; 3.3 V rail fixed values, `GetCurrent3V3`/`SetEnabled3V3` error; CAN bus utilization 0; `IsSystemTimeValid` always true (section 6) |
| `Errors` (`RuntimeError`, `MakeError`, `ReportError`, `WPILIB_*` macros), `RuntimeType`, `Timer`, `Tracer`, `ScopedTracer`, `GetThreadPriority`/`SetThreadPriority`/`GetCurrentThreadPriority`/`SetCurrentThreadPriority`, `SensorUtil` | as is |
| `GetWPILibVersion()` | hand-written (`prosdk 2027.0.0-alpha.1 (WPILib b68effca)`) |
| `SystemServer` | **missing** (section 3.6) |
| `LinearSystemSim`, `DCMotorSim`, `FlywheelSim`, `ElevatorSim`, `SingleJointedArmSim`, `DifferentialDrivetrainSim`, `BatterySim` | as is; pure math, work on host and ESP32 |
| `AlertSim`, `DriverStationSim` (incl. `OpModeOptions`) | patch `0009`: build fix only (`int32_t` is `long` on Xtensa) |
| HAL-sim wrappers: `AddressableLEDSim`, `AnalogInputSim`, `CTREPCMSim`, `CallbackStore`, `DIOSim`, `DigitalPWMSim`, `DutyCycleSim`, `EncoderSim`, `OnboardIMUSim`, `PWMSim`, `PowerDistributionSim`, `REVPHSim`, `RoboRioSim`, `SimDeviceSim`, `ADXL345Sim`, `AnalogEncoderSim`, `DutyCycleEncoderSim`, `PWMMotorControllerSim`, `SharpIRSim`, `DoubleSolenoidSim`, `SolenoidSim`, `PneumaticsBaseSim`, `GenericHIDSim`, `GamepadSim`, `JoystickSim`, 15 `*ControllerSim`; `SimHooks` functions | as is. Host: work (tests). ESP32: link, but inert (`HALSIM_*` stubs, as systemcore) |

### 3.6 Missing (excluded) and the reason recorded

| Upstream | Reason (API-DIFFERENCES.md section 4) |
|---|---|
| `backend/NetworkTablesTelemetryBackend`, `NetworkTablesTunableBackend` | ntcore not ported; replaced by the probot-core dashboard backends |
| `event/NetworkBooleanEvent` | ntcore not ported; "to be re-implemented over tunables" (PORT.md section 10). **Not re-implemented yet** |
| `system/SystemServer` | returns the SystemCore system server's NT instance |
| `hardware/expansionhub/*` (6 classes) | REV Expansion Hub through the SystemCore USB "rhsp" server over NT topics. PORT.md section 10 does not name them |

## 4. Patches (behavior changes)

| Patch | Behavior change? |
|---|---|
| `0001` RobotBase without NT | yes: no NT server/backends/program start time; `IsReal` on the ESP32; robot thread config |
| `0002` loop flush through RobotHost | yes: `FlushTelemetry()` hook instead of NT flush; console output |
| `0003` DriverStationBackend without NT | yes: no `/DriverStation` table |
| `0004` robot tests without NT | test only: 2 NT tests replaced by `RobotHostHooks` |
| `0005` Preferences on platform store | yes: storage, getter conversions, `Set` replaces type, no DS visibility |
| `0006` filesystem platform storage | yes: paths |
| `0007` DataLogManager platform storage | yes: directory, discarding log, no NT/console logging |
| `0008` thread config | stack/priority/core only |
| `0009` Xtensa int32 build fixes | no |
| `0010` hosted robot loop | additive API (`StartHosted`, `RunDue`, `Update`, `GetVersion`); host loop unchanged; ESP32 `GetThreadId()` empty |

All behavior changes are in API-DIFFERENCES.md section 3.

## 5. Upstream tests

| Upstream | Test macros | Host |
|---|---|---|
| 69 vendored test files (`test/native/cpp`, `generated/test/native/cpp`) | 289 (upstream 290: `TimedRobotTest` 10 -> 9) | built into `wpilibc_tests`, 604 Catch2 cases, all pass |
| `NetworkTablesTunableBackendTest.cpp` | 36 | not vendored: NT backend not ported |
| `NetworkTablesTelemetryBackendTest.cpp` | 11 | not vendored: NT backend not ported |
| `UnitNetworkTablesTest.cpp` | 3 | not vendored: unit-typed NT topics |
| `hardware/ExpansionHubMotorTest.cpp` | 2 | not vendored: ExpansionHub not ported |
| `event/NetworkBooleanEventTest.cpp` | 1 | not vendored: NetworkBooleanEvent not ported |
| `TimedRobotTest` `ConstructorPublishesProgramStartTime`, `ConstructorRegistersTelemetryAndTunableBackends` | 2 | replaced by `TimedRobotTest RobotHostHooks` (patch `0004`) |
| `test/native/c` | (C compile check) | not vendored: needs `ntcore.h` |
| `src/test/python` (38 files) | — | Python |

Replacement coverage: `prosdk_dashboard_tests` (55 cases) test the probot-core dashboard backends that
replace the NT backends; `prosdk_robot_tests` (7 cases) test the hosted robot loop on simulated core time.

Limits:

- The wpilibc tests run against the upstream sim HAL. Pneumatics, `PowerDistribution`, `OnboardIMU` and
  the digital-PWM tests pass on the host, while the same classes are stubs on the ESP32. No test covers
  wpilibc on the ESP32 HAL backend.
- No upstream test runs on the ESP32. The ESP32 branches of patches `0001`, `0005` (NVS), `0006`,
  `0007` (LittleFS) and `0010` (`GetThreadId`) have no automated test. The only on-target evidence is the
  link check (1.6) and the examples, which were compiled and not run.

## 6. Wave-2 gaps re-checked (audit-wave2.md section 7)

| Wave-2 item | Status at `84e4c43` |
|---|---|
| 7.1.1 `HAL_GetTeamNumber` 0 | closed: documented in `hal-esp32.md` |
| 7.1.2 `HAL_GetRSLState`, `HAL_GetCommsDisableCount` | closed as documentation: still `HAL_HANDLE_ERROR`, recorded with core request R12 |
| 7.1.3 `HAL_GetSystemTimeValid` always true | **open**. It now matters: `DataLogManager` renames its file from `system_clock`, which starts at 1970 on the ESP32, so a DS-attached robot logs to `WPILIB_19700101_...wpilog`. An uncommitted edit in the worktree (`DriverStation.cpp`, `hal-esp32.md`) returns true only after 2024-01-01; not counted until committed |
| 7.1.4 `HAL_WriteDisplayAnsi` no-op | closed: implemented (DS console, ANSI removed, repeats dropped) |
| 7.1.5 `HAL_SetSerial{Read,Write}BufferSize` no-ops | **open**: not documented; `SerialPort::SetReadBufferSize`/`SetWriteBufferSize` silently do nothing |
| 7.1.6 `HAL_GetComments` empty | **open**: not documented (`RobotController::GetComments()` is "") |
| 7.1.7 3.3 V rail doc vs code | closed: `hal-esp32.md` matches the code |
| 7.1.8 joysticks: port 0 only | closed as documentation (pending core R8) |
| 7.1.9 opmode selection | closed: wired to core R7 (`SetOpModeOptions`, active/selected id, UTILITY) |
| 7.1.10 upstream files with no rule | **open**: `datalog.py` `NOT_VENDORED` unchanged; wpilibc adds 12 more (section 2) |
| 7.2.1 PCM/PH/PDP stubs | unchanged; documented (API-DIFFERENCES.md section 3) |
| 7.2.13 `FileLogger` excluded | unchanged; consequence documented (`LogConsoleOutput()` no effect) |
| 7.2.15 alert path missing | closed: alerts go to `/Telemetry/Alerts/<group>` (`DashboardAlerts`) |
| 7.2.16 ExpansionHub not named | partly: in API-DIFFERENCES.md section 4, still not in PORT.md section 10 |
| Other 7.2 items (IMU, digital PWM, HALSIM, serial/CAN/analog limits, DS match data, brownout, usage reporting, Sleipnir diagnostics, Eigen limits, `units::PS`, DataLog buffers) | unchanged, documented |

Wave-2 stub list (section 8): `HAL_WriteDisplayAnsi` and the opmode selection are no longer stubs.

## 7. Still missing for "every WPILib 2027 C++ feature" (excluding commandsv2 and drivers/)

### 7.1 Upstream C++ libraries not ported

| Upstream | C/C++ files | Recorded? |
|---|---|---|
| `fields` (`Field`, `FieldTag`, `FieldImage`, `GetField(FieldId)`, generated `fields.hpp` with FRC 2018-2026 and FTC 2024-2025 / 2025-2026 DECODE field metadata and tag poses) | 3 sources, 22 headers, 3 tests + generated | **no**. Pure C++ (json + wpimath geometry), the 2027 successor of `AprilTagFieldLayout`; used by the upstream example `DifferentialDrivePoseEstimator`. Field images (jpg/png) are large and optional |
| `apriltag` (`AprilTagDetector`, `AprilTagPoseEstimator`, `AprilTagDetection`, `AprilTagPoseEstimate`, `AprilTagImageGenerator`, thirdparty apriltag C library) | 73 | **no** (only `design/reports/critic.md` asked for a decision). Plausible reason: no camera on the kit, as for cscore |
| `aos_sdk`, `aosnt` (AOS event loop + NT bridge), `ntcoreffi` | 0 + 7 + 3 | **no**. Linux/desktop, depend on ntcore/wpinet |
| `ntcore`, `wpinet` | 164 + 306 | yes (PORT.md section 10; NT4 server for AdvantageScope/Elastic is a later work item) |
| `cscore`, `cameraserver` | 106 + 11 | yes |
| `romiVendordep`, `xrpVendordep` | 11 + 15 | yes |
| `simulation/halsim_gui` | — | yes ("halsim desktop GUI extensions") |
| `simulation/halsim_ws_core`, `halsim_ws_server`, `halsim_ws_client`, `halsim_ds_socket`, `halsim_xrp` | ~70 | **partly**: section 10 names only GUI extensions; the WebSocket sim protocol (host sim to a web simulator) is an open question of `critic.md` |
| `glass`, `wpigui`, `tools/*` (desktop applications) | — | not mentioned; not library code |
| `commandsv3`, `epilogue-*`, `wpiunits`, `wpiannotations` | 0 C++ | Java only (commandsv3 recorded; the others have no C++ counterpart) |
| `wpilibcExamples` (38 examples, 31 snippets, 8 templates, 8 command examples) | — | not recorded; prosdk has 3 robot examples + `HalSmoke`. Not a library feature; many need `drivers/` or commandsv2 (wave 4) |

### 7.2 wpilibc features missing or reduced

- `NetworkBooleanEvent` over tunables: planned (PORT.md section 10), not implemented.
- ExpansionHub classes: excluded (no rhsp server, ntcore).
- Preferences on the DS: upstream shows and edits `/Preferences` in dashboards; prosdk's store is not
  visible on the DS.
- `DataLogManager`: no NT logging (ntcore), no console capture although prosdk owns the console
  (`ConsoleWrite`); on the ESP32 old logs are never deleted (`fs::space` fails without `statvfs`, so
  `freeSpace` is `UINTMAX_MAX`; upstream's 50 MB threshold would always trigger on a LittleFS partition
  anyway), log files named from a 1970 clock (6, item 7.1.3), no download from the DS (core R11).
- DS widgets for `Field2d`/`Mechanism2d` (core R5 remainder).
- ESP32 stubs of section 3: pneumatics, `PowerDistribution`, `OnboardIMU` (the kit's IMU is a
  `drivers/` device, wave 4), digital PWM/pulses, rumble/LEDs/touchpads, match data, RSL state, comms
  disable count, team number, comments, 3.3 V rail current/switch, serial flow control and buffer sizes,
  CAN bus utilization, HAL-sim wrappers on the target.

### 7.3 Waiting for probot-core (core-requests.md)

R5 remainder (Field2d, Mechanism2d widgets), R8 (gamepads 1-5, analog triggers, rumble), R9 (console
levels and size), R10 (wall clock), R11 (file download), R12 (`requestStop`, `rslState`,
`commsDisableCount`, robust tunable persistence, battery current), R13 (schema size). None is in the
`sdk-target` clone (`.upstream/probot-core` at `ab3697c`); grep for `rslState`, `commsDisableCount`,
`requestStop` finds nothing.

## 8. Gaps not justified by PORT.md section 10 or another design document

1. `fields` library missing, not mentioned anywhere (7.1).
2. `apriltag` library missing, no recorded decision (7.1).
3. `aos_sdk`/`aosnt`/`ntcoreffi` and the non-GUI `simulation/halsim_*` extensions not mentioned in
   PORT.md section 10 (7.1).
4. `HAL_GetSystemTimeValid` always true on the ESP32 (wave-2 7.1.3); `DataLogManager` names log files with
   1970 timestamps. Fix pending as an uncommitted edit.
5. `DataLogManager` on the ESP32 never deletes old log files; not in its API-DIFFERENCES row.
6. `DataLogManager::LogConsoleOutput()` has no effect; the recorded reason is upstream's file-tailing
   mechanism, but prosdk could capture its own console.
7. `Preferences` are not visible or editable on the DS; the API-DIFFERENCES row does not say so.
8. `NetworkBooleanEvent` not re-implemented over tunables, although PORT.md section 10 lists it as
   re-implemented.
9. `SerialPort::SetReadBufferSize`/`SetWriteBufferSize` silently ignored and `RobotController::GetComments()`
   empty on the ESP32, undocumented (wave-2 7.1.5, 7.1.6).
10. ExpansionHub classes: reason in API-DIFFERENCES.md, not in PORT.md section 10.
11. Bookkeeping: 12 wpilibc and 48 wave-2 upstream files with no rule or `NOT_VENDORED` entry.
12. Upstream C++ examples and templates: no decision recorded on which are ported (only 3).
