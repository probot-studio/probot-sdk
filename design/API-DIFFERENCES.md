# prosdk API differences from WPILib 2027 C++

prosdk is WPILib 2027 C++ (allwpilib `b68effca4ddc9742ac208daa7dd1d11816137882`) vendored by
`tools/upstream/wpilib.py` and compiled for the Probot ESP32-S3 kit. Code written for WPILib compiles
against prosdk after the changes in this file. Every rename and every exclusion is listed here
(design/PORT.md sections 4, 5 and 10).

## 1. Mechanical renames (every vendored file)

| WPILib | prosdk | Why |
|---|---|---|
| `#include <wpi/...>` | `#include <prosdk/...>` | library include root |
| `namespace wpi`, `wpi::` | `namespace prosdk`, `prosdk::` | library namespace |
| global `namespace fs` (wpiutil `fs.hpp`), `fs::` | `prosdk::fs`, `prosdk::fs::` | Arduino `FS.h` declares a global `namespace fs` |
| `std::expected`, `std::unexpected`, `std::unexpect`, `std::unexpect_t`, `std::bad_expected_access` | `prosdk::cxx::...` (`<prosdk/cxx/expected.hpp>`) | C++23; Arduino-ESP32 compiles as gnu++20 |
| `std::print`, `std::println` | `prosdk::cxx::print`, `prosdk::cxx::println` (`<prosdk/cxx/print.hpp>`) | C++23 |
| `std::byteswap` | `prosdk::cxx::byteswap` (`<prosdk/cxx/bit.hpp>`) | C++23 |
| `std::to_underlying`, `std::unreachable` | `prosdk::cxx::...` (`<prosdk/cxx/utility.hpp>`) | C++23 |
| `std::invoke_r` | `prosdk::cxx::invoke_r` (`<prosdk/cxx/functional.hpp>`) | C++23 |
| `#include <Eigen/...>`, `<unsupported/Eigen/...>` | `#include <prosdk/thirdparty/eigen/Eigen/...>`, `<prosdk/thirdparty/eigen/unsupported/Eigen/...>` | thirdparty include roots live under `prosdk/thirdparty/` so they cannot collide with other Arduino libraries (for example ArduinoEigen); the C++ namespace stays `Eigen` |
| `#include <gcem.hpp>` | `#include <prosdk/thirdparty/gcem/gcem.hpp>` | same; namespace stays `gcem` |
| `#include <sleipnir/...>`, `<gch/small_vector.hpp>` | `#include <prosdk/thirdparty/sleipnir/sleipnir/...>`, `<prosdk/thirdparty/sleipnir/gch/small_vector.hpp>` | same; namespace stays `slp` |
| `m[i, j]`, `m[rowSlice, colSlice]` (C++23 multidimensional subscript on Sleipnir `VariableMatrix`/`VariableBlock` and Eigen matrices) | `m(i, j)`, `m(rowSlice, colSlice)` | C++23 language feature; the target is gnu++20, where `m[i, j]` silently means `m[j]` (comma operator). Sleipnir's multi-parameter `operator[]` overloads are `operator()` here; single-index `m[i]` is unchanged. Vendored code is rewritten by the `multi_subscript` transform of `tools/upstream/wpilib.py` |
| `#include "wpimath/protobuf/<file>.npb.h"` | `#include <prosdk/generated/wpimath/protobuf/<file>.npb.h>` | generated nanopb headers under the library include root; the C types keep their `wpi_proto_*` names |

Kept unchanged (wire formats and C ABI): `wpi.proto.*`, `wpi_proto_*`, `struct:<Type>` and `proto:<Type>`
type strings, `WPI_*` and `HAL_*` C functions, `WPILIB_*` macros.

## 2. Identifiers renamed because Arduino.h defines them as macros

Every enumerator of an enum that has at least one colliding name is renamed (k prefix, WPILib C++ style;
the Java names are the same upper-case words, so there is no other spelling to use).

| WPILib | prosdk | Colliding Arduino macros |
|---|---|---|
| `wpi::util::Alert::Level::HIGH` / `MEDIUM` / `LOW` | `prosdk::util::Alert::Level::kHigh` / `kMedium` / `kLow` | `HIGH`, `LOW` |
| `wpi::tunables::TunableConfig::Polling::DEFAULT` / `ALWAYS_GET` / `GET_ON_CHANGE` | `prosdk::tunables::TunableConfig::Polling::kDefault` / `kAlwaysGet` / `kGetOnChange` | `DEFAULT` |
| `wpi::hal::SimDevice::Direction::INPUT` / `OUTPUT` / `BIDIR` | `prosdk::hal::SimDevice::Direction::kInput` / `kOutput` / `kBidir` (the WPILib Java spelling) | `INPUT`, `OUTPUT` |
| `wpi::math::Debouncer::DebounceType::RISING` / `FALLING` / `BOTH` | `prosdk::math::Debouncer::DebounceType::kRising` / `kFalling` / `kBoth` (also the default argument of `BooleanEvent::Debounce`) | `RISING`, `FALLING` |
| `wpi::CompressorConfigType::DISABLED` / `DIGITAL` / `ANALOG` / `HYBRID` | `prosdk::CompressorConfigType::kDisabled` / `kDigital` / `kAnalog` / `kHybrid` | `DISABLED`, `ANALOG` |
| `Button::CROSS` ... `Button::RIGHT_FUNCTION` of the PlayStation controllers (`DualShock4Controller`, `DualSenseController`, `DualSenseEdgeController`, `NiDsPS4Controller`, `NiDsPS5Controller`) | `Button::kCross`, `kCircle`, `kSquare`, `kTriangle`, `kShare`/`kCreate`, `kPS`, `kOptions`, `kL1`, `kR1`, `kL2`, `kR2`, `kL3`, `kR3`, `kDpadUp`, `kDpadDown`, `kDpadLeft`, `kDpadRight`, `kTouchpad`, `kMicrophone`, `kRightPaddle1`, `kLeftPaddle1`, `kLeftFunction`, `kRightFunction` (only in these five classes; the other controllers keep their `Button` names) | `PS` (Xtensa `specreg.h`) |
| `BooleanEvent PS(EventLoop*)` of the same five classes | `PSButton(EventLoop*)` (matches `GetPSButton()` / `GetPSButtonPressed()`) | `PS` |
| `Trigger PS(EventLoop*)` of their command wrappers (`CommandDualShock4Controller`, `CommandDualSenseController`, `CommandDualSenseEdgeController`, `CommandNiDsPS4Controller`, `CommandNiDsPS5Controller`) | `PSButton(EventLoop*)`; the wrappers use the renamed `Button::k...` enumerators of their controllers | `PS` |

Other Arduino macro names used inside vendored headers (internal names, parameters, `F`, `likely`, ...)
need no rename: each public header is wrapped in `#pragma push_macro` / `#undef` / `#pragma pop_macro` for
exactly the macros it uses (computed by the script from the installed core headers). Names formed by token
pasting in the units macros are covered too (`guard_pasted`). One such name is user-visible and not
renamed: the abbreviated petasiemens type `prosdk::units::PS` (`P` prefix pasted onto `S`) cannot be
spelled in a sketch, because the Xtensa header `xtensa/config/specreg.h` (included by Arduino.h) defines
`PS`; use `prosdk::units::petasiemens_t` or `petasiemens` instead.

## 3. Behavior differences

| API | Difference | Patch |
|---|---|---|
| `EXPORT_TEMPLATE_DECLARE` / `EXPORT_TEMPLATE_DEFINE` (`SymbolExports.hpp`) | fixed to the non-Windows behavior; the upstream style detection dispatches through the token `DEFAULT` | `wpiutil/0001` |
| `prosdk::util::spinlock`, `recursive_spinlock*` | aliases of `std::mutex` / `std::recursive_mutex` on ESP32 (spinning starves lower-priority FreeRTOS tasks) | `wpiutil/0002` |
| `priority_mutex.hpp` | `__FIRST_SYSTEMCORE__` replaced by `PROSDK_TARGET_HW`. On ESP32 this only defines `WPI_USE_PRIORITY_MUTEX`; the `priority_mutex` classes stay Linux-only (`WPI_HAVE_PRIORITY_MUTEX` is not defined), so callers use `std::mutex`, which on FreeRTOS already inherits priority | `wpiutil/0003` |
| `prosdk::util::GetTypeName(const T&)` | built without RTTI: returns the static type name of `T` (`__PRETTY_FUNCTION__`), not the dynamic type | `wpiutil/0004` |
| `prosdk::util::GetStackTrace` | ESP32: return addresses only (`0x...`, decode with `xtensa-esp32s3-elf-addr2line`) | `wpiutil/0005` |
| `SafeThread` | threads start with an explicit stack (`PROSDK_THREAD_STACK_SIZE`, 4096), priority below the probot-core loop (`PROSDK_THREAD_PRIORITY`, 3) on core 1 (`PROSDK_THREAD_CORE`) | `wpiutil/0006` |
| `prosdk::util::Now()` default | ESP32: `esp_timer_get_time()` (monotonic since boot) | `wpiutil/0007` |
| `prosdk::util::Demangle` | none (build fix: the `__cxa_demangle` status variable is `int`; `int32_t` is `long` on Xtensa) | `wpiutil/0008` |
| Telemetry and tunables default warning reporters | write to the platform console (`prosdk::platform::ConsoleWrite`) instead of `std::print(stderr)`; never throw | `telemetry/0001`, `tunables/0001` |
| HAL backend | host: the WPILib simulation backend (`host/hal/sim`, outside the Arduino tree; `HAL_GetRuntimeType()` is `HAL_RUNTIME_SIMULATION`). ESP32: the prosdk backend in `src/prosdk/platform/hal/esp32` (SystemCore's backend is its reference). The function contract is `design/hal-esp32-functions.txt`; hardware limits and deviations from SystemCore are in `design/hal-esp32.md` | layout |
| HAL handle version check (`getHandleTypedIndex`, `handles/HandlesInternal.hpp`) | `__FIRST_SYSTEMCORE__` replaced by `PROSDK_TARGET_HW`: like SystemCore, the ESP32 does not check the handle version byte; the host build checks it | `hal/0001` |
| `HAL_SetPrintErrorImpl` default (text printed by `HAL_SendError`) | written to the platform console instead of `fwrite(stderr)` | `hal/0002` |
| Telemetry, tunables and alerts on the Driver Station (`prosdk::platform::RegisterDashboardBackends()`, `ProbotTelemetryBackend`, `ProbotTunableBackend`) | backends on probot-core's dashboard topic store instead of NetworkTables (`NetworkTablesTelemetryBackend` / `NetworkTablesTunableBackend` are not ported). Same topic layout (`/Telemetry/...`, `/Tunables/...`, `.type` children, struct/protobuf type strings and schemas, JSON properties) with these differences: a robust tunable is one topic with flag ROBUST instead of `<path>/value` + `<path>/tune`, and its initial value is the code default (core persists nothing, `design/core-requests.md` R12); telemetry timestamps are ignored (the store keeps the latest value); `.type` topics are read-only (properties `{"mutable":false}`, DS writes refused); DS writes are queued by core, drained before each loop and applied by `TunableRegistry::Update()`; writes of the wrong type, to immutable tunables or to unknown topics are refused with core's `WriteStatus`; topics published before core's store exists wait and are sent on the next cycle. Core limits (`PROBOT_DASH_MAX_TOPICS` 128, `PROBOT_DASH_MAX_VALUE` 1024, `PROBOT_DASH_MAX_SCHEMAS` 32): an overflow is reported once on the console per limit and the robot keeps running (a refused telemetry entry is discarded, a refused tunable makes `Publish` return false); protobuf descriptors over 1024 bytes need a larger `PROBOT_DASH_MAX_VALUE` (R13). Alerts: active alerts are logged to `/Telemetry/Alerts/<group>` (`.type` "Alerts", string[] `errors`/`warnings`/`infos`, newest first) after each loop, replacing the SystemCore `MrcLibAlert` transport | hand-written `src/prosdk/platform/dashboard` |
| Dashboard opmode chooser (`DashboardOpMode.hpp`: `InitializeDashboardOpMode`, `SetDashboardOpModeOptions`, `StartDashboardOpMode`, `EnableDashboardOpMode`, `GetDashboardSelectedOpMode`) | no-op: options are ignored and a dashboard never selects an opmode (`GetDashboardSelectedOpMode` returns 0). Upstream publishes ntcore string choosers (`/SmartDashboard/* OpMode`); ntcore is not ported and the probot-core Driver Station selects the opmode | hand-written `src/prosdk/platform/hal/DashboardOpMode.cpp` |
| `prosdk::log::DataLog` buffers (`BLOCK_SIZE`, `MAX_BUFFER_COUNT`, `MAX_FREE_COUNT`) | ESP32: 2 KiB blocks, logging pauses at 16 outgoing buffers (32 KiB), 4 free buffers kept (8 KiB). Upstream (kept on the host): 16 KiB, 64 (1 MiB), 16 (256 KiB). Set by `PROSDK_DATALOG_*` in `prosdk/platform/DataLogConfig.hpp`; a different value needs an archive rebuilt with the same `-D` | `datalog/0001` |
| `DataLog` default message logger | writes `DataLog: ...` lines (info and above) to the platform console instead of stdout/stderr | `datalog/0002` |
| `prosdk::log::DataLogBackgroundWriter` | writer thread started through `ScopedThreadConfig` (name `datalog`, stack `PROSDK_DATALOG_THREAD_STACK_SIZE` = 6144, prosdk thread priority and core). Minimum free space `PROSDK_DATALOG_MIN_FREE_SPACE`: ESP32 64 KiB, host 5 MiB (upstream). The ESP32 toolchain has no `statvfs`, so `fs::space` fails and the free-space check is skipped (the upstream fallback, free space = unlimited): logging stops only when a write fails, which is reported. `fsync` after each flush on ESP32 (upstream syncs only on Linux/macOS). Random file names use `esp_random()` on ESP32 (no `std::random_device` there) | `datalog/0003` |
| `prosdk::math::Normal`, `SimulatedAnnealing::Solve`, `TravelingSalesman` (neighbor step) | draw from one shared `std::mt19937` (`prosdk::platform::RandomEngine`, mutex-protected) seeded once from `esp_random()` (ESP32) or `std::random_device` (host), instead of a new `random_device` + `mt19937` (about 5 KB of stack) per call. Same distributions; the sequence differs from upstream, as upstream's is nondeterministic too | `wpimath/0001` |
| `prosdk::math::MathSharedStore` default (`MathShared` not set) | `ReportError`/`ReportWarning` print `Error: ...` / `Warning: ...` lines to the platform console instead of dropping them; `GetTimestamp()` is `prosdk::util::Now()` as upstream (ESP32: `esp_timer_get_time()`) | `wpimath/0002` |
| `DrivetrainSplineTrajectoryGenerator` default error function | writes to the platform console instead of `print(stderr)`; `SetErrorHandler` works as upstream | `wpimath/0003` |
| Sleipnir `EXPORT_TEMPLATE_DECLARE` / `EXPORT_TEMPLATE_DEFINE` (`symbol_exports.hpp`) | fixed to the non-Windows behavior, token-identical to `wpiutil/0001` (the upstream style detection dispatches through `DEFAULT`) | `sleipnir/0001` |
| Sleipnir expression allocation | ESP32: plain `new`/`delete` instead of the thread-local pool of 16384 blocks (about 800 KB per thread on first use); the pool itself uses 64-block chunks there. Host: upstream pool | `sleipnir/0002` |
| Sleipnir diagnostics (`Options::diagnostics`, spy files, profiler output) | compiled out (`SLEIPNIR_DISABLE_DIAGNOSTICS`, set in `prosdk/platform/config.hpp`); the option is accepted and ignored | config |
| Eigen dynamic-size temporaries | `EIGEN_STACK_ALLOCATION_LIMIT` is 4096: temporaries above 4 KiB are heap-allocated instead of using the task stack (upstream default 128 KiB). Set in `prosdk/platform/config.hpp`, which every Eigen module header includes first. Fixed-size objects are checked against a separate compile-time cap, `PROSDK_EIGEN_FIXED_SIZE_STACK_LIMIT` = 8192 (upstream: both 128 KiB), so Sleipnir's `slp::solve` (an 8x8 buffer of `slp::Variable`, 4608 bytes on ESP32) still compiles | config, `eigen/0001` |
| Eigen stream output (`operator<<` for matrices) | not available (`EIGEN_NO_IO`); format Eigen types with `std::format` (`prosdk/math/fmt/Eigen.hpp`) | config |
| Eigen non-MPL code | refused at compile time (`EIGEN_MPL2_ONLY`) | config |
| `PoseEstimator::SampleAt`, `PoseEstimator3d::SampleAt` | none (build fix: `std::optional::transform` is C++23; written as an explicit check) | `wpimath/0004` |
| Sleipnir `Problem` constraint-type printout | none (build fix: `std::views::zip` is C++23; indexed loop); only reachable with diagnostics, which are compiled out | `sleipnir/0003` |
| `prosdk::RobotBase` constructor | no NetworkTables server, no NT telemetry/tunable backend registration, no `/Robot/ProgramStartTime`, no NT connection usage reports. Calls `prosdk::platform::OnRobotBaseConstructed()` instead (hook of the robot host, `prosdk/platform/RobotHost.hpp`; no-op until installed). MathShared and the warning-to-Alert reporters are set up as upstream | `wpilibc/0001` |
| `RobotBase::IsReal()` / `IsSimulation()`, `prosdk::impl::ResetMotorSafety` | `__FIRST_SYSTEMCORE__` replaced by `PROSDK_TARGET_HW`: the ESP32 is "real", the host is simulation | `wpilibc/0001` |
| `prosdk::StartRobot<Robot>()` | compiles unchanged except that the robot thread (only when `HAL_HasMain()`) is started through `ScopedThreadConfig` (name `robot`, stack `PROSDK_ROBOT_THREAD_STACK_SIZE` = 16384). Host only in practice: on the robot a sketch ends with `PROSDK_ROBOT_MAIN(Robot)` instead of `main()` (next rows) | `wpilibc/0001` |
| Robot framework status lines (`Robot program starting`, `startup complete`, opmode creating/starting/ending/closing, `Default ...() method... Override me!`, HAL init failure, default telemetry/tunable warnings) | written to the platform console (`prosdk::platform::ConsoleWrite`, Driver Station console on the robot) instead of stdout/stderr | `wpilibc/0001`, `wpilibc/0002` |
| `IterativeRobotBase` / `OpModeRobotBase` loop | instead of `NetworkTableInstance::FlushLocal()` each iteration calls `prosdk::platform::FlushTelemetry()` (robot host hook, no-op until installed) | `wpilibc/0002` |
| `prosdk::internal::DriverStationBackend` | the `/DriverStation` NetworkTables table (match data, control word, opmode name, `.type`) is not published; everything else (joystick data, edge flags, alerts, DataLog of DS/joystick data, opmode lookup) is unchanged | `wpilibc/0003` |
| `prosdk::Preferences` | API unchanged; storage is `prosdk/platform/PreferencesStore.hpp` instead of the persistent NT table `/Preferences`. ESP32: one NVS blob (namespace `prosdk`, key `prefs`), rewritten on every change (a `Set` of the stored value writes nothing; do not set changing values every loop). Host: memory, or a JSON file named with `prosdk::platform::prefs::SetPreferencesFile()`. Getters follow the NT entry semantics (numeric getters convert between integer/float/double, other types give the default); a `Set` replaces value and type (in NT a `Set` of another type fails). No `.type` key. Name clash: Arduino-ESP32's Preferences library defines a global `::Preferences`; with `using namespace prosdk;` and `<Preferences.h>` in the same sketch, write `prosdk::Preferences` | `wpilibc/0005` |
| `prosdk::filesystem::GetOperatingDirectory()` / `GetDeployDirectory()` | ESP32: `/littlefs` and `/littlefs/deploy` (LittleFS mounted by the sketch with `LittleFS.begin()`; the sketch's `data/deploy` folder after a LittleFS upload). Host: upstream simulation paths. Both configurable (`prosdk/platform/Storage.hpp`). `GetLaunchDirectory()` returns `/` if the current path is unavailable | `wpilibc/0006` |
| `prosdk::DataLogManager` | default directory `prosdk::platform::GetLogDirectory()`: ESP32 `/littlefs/logs` when LittleFS is mounted, otherwise the log discards its data (a writer whose output goes nowhere; `GetLogDir()` is `""`) and a console line says so; host `<operating directory>/logs`. `LogNetworkTables()` and `LogConsoleOutput()` have no effect (no ntcore; upstream's console capture tails a SystemCore log file with `FileLogger`, not vendored). Random file names come from `prosdk::platform::RandomEngine`. `Log()` echoes to the platform console | `wpilibc/0007` |
| `prosdk::Notifier`, `prosdk::Watchdog`, `prosdk::internal::DriverStationModeThread` | threads started through `ScopedThreadConfig` (names `notifier`, `watchdog`, `dsmode`; stacks `PROSDK_NOTIFIER_THREAD_STACK_SIZE` 8192, `PROSDK_WATCHDOG_THREAD_STACK_SIZE` 6144, `PROSDK_DS_MODE_THREAD_STACK_SIZE` 6144 in `prosdk/platform/WpilibcConfig.hpp`) | `wpilibc/0008` |
| Pneumatics (`Solenoid`, `DoubleSolenoid`, `Compressor`, `PneumaticsControlModule`, `PneumaticHub`, `PneumaticsBase`), `PowerDistribution`, `OnboardIMU`, CAN (`CAN`, CAN motor/device APIs) and their `*Sim` classes | API vendored unchanged and compiled. ESP32: the kit has no PCM/PH/PDP/PDH, no onboard IMU and no CAN bus; the ESP32 HAL backend reports an error when one is constructed (`design/hal-esp32.md`), so they fail at run time, not at compile time. Host: the simulation backend implements them as upstream (the upstream tests run) | HAL stubs |
| `GetWPILibVersion()` | returns `prosdk 2027.0.0-alpha.1 (WPILib b68effca)` (upstream: generated from the Gradle version) | hand-written `src/prosdk/platform/WPILibVersion.cpp` |
| `prosdk::DigitalOutput::EnablePWM()`, `UpdateDutyCycle()`, `SetPWMRate()` (digital PWM generators) | ESP32: implemented on the LEDC peripheral (systemcore stubs them): 6 generators, each on one DIO output, one rate for all (5 Hz to 40 kHz, default 500 Hz, at most 14 bits of duty). The duty is forced to 0 while the output gate is closed (the roboRIO does not gate them) and the emergency stop drives their pins low. `EnablePPS()` and `Pulse()` report `HAL_HANDLE_ERROR`. Host: simulation as upstream (`design/hal-esp32.md`, DIO) | HAL backend |
| Robot program entry point | `PROSDK_ROBOT_MAIN(Robot)` (`prosdk/platform/RobotMain.hpp`, via `<prosdk.h>`) replaces `int main() { return wpi::StartRobot<Robot>(); }`. Robot: the class is constructed on probot-core's user task at boot (not during static initialization, not on a new thread) and its loop runs from core's loop passes (design/PORT.md 8.1). Host: expands to `main()` with `StartRobot`. Only `TimedRobot`, `TimesliceRobot` and `OpModeRobot` run on the robot; a class deriving from `RobotBase` directly is reported and the program halts | hand-written `src/prosdk/platform/RobotMain.*` |
| `RobotBase::StartHosted()` (new, virtual), `TimedRobot` / `OpModeRobotBase::StartCompetition()` | `StartHosted()` does what `StartCompetition()` did before its loop and returns the callback queue; `StartCompetition()` calls it and then loops as upstream. `PeriodicPriorityQueue` gains `RunDue(now, lookahead)` (non-blocking `RunCallbacks`, same order and rescheduling), `Update(func)` and `GetVersion()` | `wpilibc/0010` |
| Robot loop timing on the robot | callbacks run only on core's loop passes. The pass period is the GCD (whole ms, 1..1000) of the callback periods and phases and follows `AddPeriodic`; a period below 1 ms runs once per pass, a phase that is not whole ms (e.g. `PeriodicOpMode::AddPeriodic`) moves to the nearest pass, a non-whole-ms period runs up to one pass late; a loop period outside 1..1000 ms is refused (reported, program halts). A late core pass restarts core's grid; the schedule is re-aligned (no catch-up, like upstream) and a warning is printed at most once per second | hand-written `src/prosdk/platform/RobotMain.cpp` |
| Robot program exceptions | constructor/loop exceptions are reported like upstream `StartRobot` (and a non-`std::exception` one too), then the program halts with outputs disabled instead of terminating (on the robot a terminate reboots). `platform::RobotProgramHalted()` | hand-written `src/prosdk/platform/RobotMain.cpp`, `hal/esp32/SmartIo.cpp` |
| `RobotBase::GetThreadId()` | ESP32: `std::thread::id{}` (the robot runs on core's user task, a FreeRTOS task without a pthread; ESP-IDF's `pthread_self()` aborts there). Host: upstream | `wpilibc/0010` |
| `DriverStationDisplay` (`HAL_WriteDisplayAnsi`) | ESP32: frames go to the Driver Station console with ANSI sequences removed; a frame equal to the previous one is not repeated. Host: simulation backend (upstream) | hand-written `src/prosdk/platform/hal/esp32/DriverStation.cpp` |
| Upstream test `TimedRobotTest` `ConstructorPublishesProgramStartTime`, `ConstructorRegistersTelemetryAndTunableBackends` | replaced by `TimedRobotTest RobotHostHooks` (they read NT topics; ntcore is not ported). The robot tests no longer reset the NT instance | `wpilibc/0004` |
| `prosdk::cmd::CommandHelper<Base, CRTP>` (command names) | built without RTTI: a command is named after its CRTP class when it is constructed (`prosdk::util::GetTypeName<CRTP>()`, for example `prosdk::cmd::InstantCommand` or a user's `DriveDistance`). Upstream C++ names it in the `Command` constructor with `GetTypeName(*this)`, which there yields the base type (`wpi::cmd::Command`). `SetName()` and `WithName()` work as upstream | `commandsv2/0001` |
| `prosdk::cmd::Subsystem::GetName()`, `SubsystemBase` default name | the static type name (`wpiutil/0004`): `prosdk::cmd::Subsystem` for a class that implements `Subsystem` directly (upstream: its dynamic type), `prosdk::cmd::SubsystemBase` for a `SubsystemBase` (as upstream, which names it in the base constructor). Name subsystems with `SubsystemBase(name)` or `SetName()` | none |
| `PrintCommand`, `CommandScheduler` warnings (loop overrun, subsystem registered twice, non-interruptible default command) | written to the platform console (`prosdk::platform::ConsoleWrite`) instead of stdout | `commandsv2/0002` |
| `prosdk::cmd::NetworkButton` | re-implemented over tunables instead of an NT boolean topic: `NetworkButton(path)` and `NetworkButton(table, field)` (path `table/field`) publish one `Tunable<bool>` per normalized path, shared by every button on it and removed with the last one (on the Driver Station a writable boolean under `/Tunables/<path>`). The trigger is true while the value is true and a Driver Station is attached (upstream: while NetworkTables is connected). The constructors taking `nt::BooleanTopic`, `nt::BooleanSubscriber`, `nt::NetworkTable` and `nt::NetworkTableInstance` are removed | `commandsv2/0003` |
| `CommandScheduler` and `Command` on the Driver Station | API unchanged: `telemetry::Log(name, scheduler)` and `tunables::Publish(name, scheduler)` go through the probot-core dashboard backends (`.type` `Scheduler`: `Names`, `Ids`, writable `Cancel`); commands publish `.type` `Command` (`name`, `running`). The Driver Station has a `Command` widget but no `Scheduler` widget yet (design/core-requests.md R5) | none |
| Upstream tests `CommandScheduleTest NetworkTablesTunableCancelClearsValue`, `CommandTunableButtonTest NetworkTablesLifecycleMatchesGlassRunCancel`, `NetworkButtonTest` | the first two read the NT tunable backend (ntcore is not ported) and are removed; the same flows on the Driver Station backend are tested in `test/prosdk/commands2/DashboardCommandsTest.cpp`. `NetworkButtonTest` is rewritten over a mock tunable backend | `commandsv2/0004`, `commandsv2/0003` |
| `AprilTagFieldLayout`, `AprilTagFields` | do not exist at the pinned upstream (b68effca): WPILib 2027 replaced them with the fields library, ported as `prosdk::fields` (`Field`, `FieldTag`, `FieldImage`, `GetField(FieldId)`, `GetFields()`, `GetFieldName()`, `FieldId::DEFAULT_FIELD` = 2026 Rebuilt Welded; `Field::GetTagPose(id)`, `GetTags()`, `GetOrigin()`, `Serialize(path)`) | none |
| Built-in field files (`prosdk::fields::GetResource_<field>_json()`, extern "C" `FIELDSGetResource_<field>_json(size_t*)`) | generated when vendoring (`wpilib.py` rule kind `resource`, `src/prosdk/impl/fields/resources/*.cpp`) instead of at build time by `cmake/scripts/GenResource.cmake`; same symbols and contents. Const arrays in flash rodata: the JSON and tag arrays of all 18 fields are 70,239 bytes in a sketch that calls `GetField` (it switches over every `FieldId`, so all of them link in) | `wpilib.py` |
| `prosdk::fields::Field(std::string_view path)` | reads the file with `prosdk::util::raw_fd_istream` instead of `MemoryBuffer::GetFile` (`MemoryBuffer` is not vendored, it maps files with mmap); same errors (`Cannot open file: ...`, JSON parse errors). On the robot the path is on a mounted file system (e.g. `/littlefs/deploy/...`) | `fields/0001` |
| `prosdk::fields::GetFieldImages()`, `GetFieldImage(path)` | always empty (the documented "not available" result): the field images are not vendored (section 4). Image metadata (`Field::GetImage()`: path, corners, size) is kept. The per-year headers `prosdk/fields/frc/*.hpp`, `ftc/*.hpp` still declare their `GetResource_*_png()` / `_jpg()`, which are defined nowhere (as upstream without its `fieldimages` library): calling one is a link error | hand-written `src/prosdk/platform/FieldImages.cpp` |
| `prosdk::apriltag::AprilTagDetector::Detect()` | with `Config::numThreads` > 1 the C library's worker threads (`pthread_create`) start through `ScopedThreadConfig` (name `apriltag`, stack `PROSDK_APRILTAG_THREAD_STACK_SIZE` = 8192, prosdk thread priority and core). The default `numThreads` is 1 (no threads), as upstream. Heap use during detection is not measured on the robot (no camera on the kit): the union-find alone takes 8 bytes per pixel of the decimated image (about 150 KB for 320x240 at the default `quadDecimate` 2, about 600 KB without decimation), so camera robots need PSRAM or decimation | `apriltag/0001` |
| AprilTag C library `pjpeg-idct.c` | none (build fix: the IDCT helpers take `int32_t*` instead of `int*`; `int32_t` is `long` on Xtensa) | `apriltag/0002` |
| Upstream fields and apriltag tests | vendored unchanged (`test/fields`, `test/apriltag`); prosdk adds `test/prosdk/FieldsAprilTagTest.cpp` (default field from the compiled-in resources, no field images, file round trip, detection and pose of a drawn tag36h11 tag) | none |

## 4. Excluded files (vendored modules)

| Module | Upstream path | Reason |
|---|---|---|
| wpiutil | `wpiutil/src/main/native/include/wpi/util/jni_util.hpp` | JNI (no JVM on the MCU) |
| wpiutil | `wpiutil/src/main/native/include/wpi/util/MappedFileRegion.hpp` | mmap is not available on ESP32; only the datalog reader used it |
| wpiutil | `wpiutil/src/main/native/include/wpi/util/MessagePack.hpp` | only ntcore uses it (mpack not vendored) |
| wpiutil | `wpiutil/src/main/native/include/wpi/util/PixelFormat.h`, `PixelFormat.hpp` | camera support (cscore) is not ported |
| wpiutil | `wpiutil/src/main/native/include/wpi/util/RawFrame.h`, `RawFrame.hpp` | camera support (cscore) is not ported |
| wpiutil | `wpiutil/src/main/native/include/wpi/util/protobuf/ProtobufMessageDatabase.hpp` | dynamic protobuf (upb) for the desktop log reader |
| wpiutil | `wpiutil/src/main/native/thirdparty/llvm/include/wpi/util/MemoryBuffer.hpp` | needs MappedFileRegion (mmap); desktop log reader only |
| wpiutil | `wpiutil/src/main/native/thirdparty/llvm/include/wpi/util/SmallVectorMemoryBuffer.hpp` | needs MemoryBuffer |
| wpiutil | `wpiutil/src/main/native/cpp/jni/*` | JNI (no JVM on the MCU) |
| wpiutil | `wpiutil/src/main/native/cpp/MappedFileRegion.cpp` | mmap is not available on ESP32 |
| wpiutil | `wpiutil/src/main/native/cpp/MessagePack.cpp` | only ntcore uses it (mpack not vendored) |
| wpiutil | `wpiutil/src/main/native/cpp/RawFrame.cpp` | camera support (cscore) is not ported |
| wpiutil | `wpiutil/src/main/native/cpp/protobuf/ProtobufMessageDatabase.cpp` | dynamic protobuf (upb), desktop only |
| wpiutil | `wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/MemoryBuffer.cpp` | needs MappedFileRegion (mmap) |
| wpiutil (tests) | `wpiutil/src/test/native/cpp/argparse/*` | argparse is not vendored (CLI parser) |
| wpiutil (tests) | `wpiutil/src/test/native/cpp/debugging/*` | debugging thirdparty is not vendored |
| wpiutil (tests) | `wpiutil/src/test/native/cpp/JniUtilTest.cpp` | JNI |
| hal | `hal/src/main/native/include/wpi/hal/SystemServer.h` | returns an ntcore instance (`NT_Inst`) for the SystemCore system server; ntcore is not ported and there is no system server (its only users are the ExpansionHub classes) |
| hal | `hal/src/main/native/include/wpi/hal/cpp/MrcLibAlert.hpp` | SystemCore mrclib alert transport; its implementation (`cpp/mrclib`) is not ported |
| hal | `hal/src/main/native/include/wpi/hal/.clang-tidy` | lint configuration, not source |
| hal | `hal/src/main/native/cpp/DashboardOpMode.cpp` | publishes the opmode choosers over ntcore (not ported); replaced by a hand-written no-op (section 3) |
| hal | `hal/src/main/native/cpp/mrclib/*` | SystemCore mrclib Driver Station and alert transport; the probot-core Driver Station backend replaces it |
| hal | `hal/src/main/native/cpp/jni/*` | JNI (no JVM on the MCU) |
| hal | `hal/src/main/native/sim/SystemServer.cpp` | implements `HAL_GetSystemServerHandle` (`SystemServer.h`, not vendored) |
| datalog | `datalog/src/main/native/include/wpi/datalog/DataLogReader.hpp`, `cpp/DataLogReader.cpp` | reader side (desktop log tools); needs `MemoryBuffer` (mmap). Vendored only as host test support (`test/datalog/support`, with `MemoryBuffer`, `SmallVectorMemoryBuffer` and `MappedFileRegion`) so the upstream writer tests can read back what they wrote |
| datalog | `datalog/src/main/native/include/wpi/datalog/DataLogReaderThread.hpp`, `cpp/DataLogReaderThread.cpp` | reader side; needs upb, sigslot and DynamicStruct (desktop log viewer) |
| datalog | `datalog/src/main/native/include/wpi/datalog/FileLogger.hpp`, `cpp/FileLogger.cpp` | tails a Linux log file with inotify (no inotify on ESP32); DataLogManager uses it for system logs |
| datalog | `datalog/src/main/native/cpp/jni/*` | JNI (no JVM on the MCU) |
| datalog (tests) | `datalog/src/test/native/cpp/DataLogReaderThreadTest.cpp` | DataLogReaderThread is not vendored (upb) |
| datalog (tests) | `datalog/src/test/native/cpp/FileLoggerTest.cpp` | FileLogger is not vendored (inotify) |
| wpimath | `wpimath/src/main/native/cpp/jni/*` | JNI (no JVM on the MCU) |
| wpilibc | `wpilibc/src/main/native/include/wpi/backend/NetworkTablesTelemetryBackend.hpp`, `cpp/backend/NetworkTablesTelemetryBackend.cpp` | NetworkTables telemetry backend (ntcore is not ported); the robot host registers the probot-core dashboard backend (`src/prosdk/platform/dashboard`) instead |
| wpilibc | `wpilibc/src/main/native/include/wpi/backend/NetworkTablesTunableBackend.hpp`, `cpp/backend/NetworkTablesTunableBackend.cpp` | NetworkTables tunable backend; replaced like the telemetry backend |
| wpilibc | `wpilibc/src/main/native/include/wpi/event/NetworkBooleanEvent.hpp`, `cpp/event/NetworkBooleanEvent.cpp` | BooleanEvent over an NT boolean topic; to be re-implemented over tunables |
| wpilibc | `wpilibc/src/main/native/include/wpi/system/SystemServer.hpp`, `cpp/system/SystemServer.cpp` | returns the SystemCore system server's ntcore instance (HAL `SystemServer.h` is not vendored) |
| wpilibc | `wpilibc/src/main/native/include/wpi/hardware/expansionhub/*`, `cpp/hardware/expansionhub/*` | REV Expansion Hub through the SystemCore USB `rhsp` server, driven over NT topics; no rhsp server, ntcore is not ported |
| wpilibc (tests) | `wpilibc/src/test/native/cpp/NetworkTablesTelemetryBackendTest.cpp`, `NetworkTablesTunableBackendTest.cpp` | the NT backends are not vendored |
| wpilibc (tests) | `wpilibc/src/test/native/cpp/event/NetworkBooleanEventTest.cpp` | NetworkBooleanEvent is not vendored |
| wpilibc (tests) | `wpilibc/src/test/native/cpp/UnitNetworkTablesTest.cpp` | unit-typed NT topics (ntcore is not ported) |
| wpilibc (tests) | `wpilibc/src/test/native/cpp/hardware/ExpansionHubMotorTest.cpp` | ExpansionHub is not vendored |
| fields | `fields/src/main/native/resources/org/wpilib/fields/**/*.png`, `**/*.jpg` | field images (PNG/JPG, 9.3 MB, about 55 MB as a C array): upstream builds them only into the optional `fieldimages` library (`WPILIB_WITH_GUI`); the robot has no display and the flash has no room. `GetFieldImages()`/`GetFieldImage()` report no image (section 3) |
| fields | `fields/src/generated/main/native/cpp/fieldimages/field_images.cpp` | registry of the field images (not vendored); replaced by `src/prosdk/platform/FieldImages.cpp` |
| apriltag | `apriltag/src/main/native/include/wpi/apriltag/AprilTagDetector_cv.hpp` | OpenCV `cv::Mat` adapter; OpenCV is not available on the MCU |
| apriltag | `apriltag/src/main/native/include/wpi/apriltag/AprilTagImageGenerator.hpp`, `cpp/AprilTag.cpp` | tag image generator: returns a wpiutil `RawFrame`, the cscore camera frame type, which is not vendored (cscore is not ported); printing tags is a desktop task |
| apriltag | `apriltag/src/main/native/cpp/jni/*` | JNI (no JVM on the MCU) |

Whole upstream trees not vendored:

| Upstream path | Reason |
|---|---|
| `wpiutil/src/main/native/thirdparty/argparse` | command-line parser for desktop tools; uses std::any/RTTI |
| `wpiutil/src/main/native/thirdparty/debugging` | debugger detection has no ESP32 branch; unused by ported code |
| `wpiutil/src/main/native/thirdparty/mpack` | MessagePack for ntcore (NT4 is not ported yet) |
| `wpiutil/src/main/native/thirdparty/upb` | dynamic protobuf for the desktop DataLog reader |
| `wpiutil/src/main/native/thirdparty/nanopb/generator` | protoc plugin; generated .npb files are vendored instead |
| `wpiutil/src/main/native/windows` | Windows only |
| `wpiutil/src/main/native/resources` | desktop GUI icon |
| `hal/src/main/native/systemcore` | SystemCore (Linux) hardware backend; reference for the hand-written ESP32 backend (`src/prosdk/platform/hal/esp32`), never compiled as is |
| `hal/src/main/java`, `hal/src/test/java` | Java bindings and tests (no JVM on the MCU) |
| `hal/src/main/python`, `hal/src/test/python` | Python bindings and tests |
| `hal/src/dev` | desktop development executable |
| `datalog/src/main/java`, `datalog/src/main/python` | Java and Python bindings |
| `datalog/examples`, `datalog/src/dev` | desktop example programs (printlog, writelog) and development executable |
| `wpimath/src/main/native/cpp/jni` | JNI (no JVM on the MCU) |
| `wpimath/src/main/java`, `wpimath/src/generated/main/java`, `wpimath/src/generate` | Java API and its generators |
| `wpimath/src/main/python`, `wpimath/src/test/python` | Python bindings and their tests |
| `wpimath/src/test/java` | Java tests |
| `wpimath/src/dev` | upstream developer scratch executable |
| `wpimath/src/main/proto` | `.proto` sources; the committed nanopb output (`src/generated/.../wpimath/protobuf`) is vendored instead |
| `wpimath/docs` | design notes (not code) |
| `wpimath/src/main/native/thirdparty/eigen/include/BUILD.bazel`, `MODULE.bazel` | Bazel build files |
| `wpilibc/src/test/native/c` | C compile check of `HAL.h` together with `ntcore.h` (ntcore is not ported) |
| `wpilibc/src/generate` | Jinja templates of the HID classes; their output (`wpilibc/src/generated`, 15 controllers and 15 `*ControllerSim` classes plus their tests) is committed upstream and vendored instead. `WPILibVersion.cpp.in`: hand-written `src/prosdk/platform/WPILibVersion.cpp` |
| `wpilibc/src/main/python`, `wpilibc/src/test/python` | Python bindings and tests |
| `wpilibc/src/dev` | upstream developer scratch executable |
| `commandsv2/src/generate` | Jinja templates of the `Command*Controller` wrappers; their output (`commandsv2/src/generated`) is committed upstream and vendored instead |
| `commandsv2/src/main/java`, `commandsv2/src/test/java`, `commandsv2/src/test/resources` | Java API, Java tests and their JUnit service registration (no JVM on the MCU) |
| `commandsv2/src/main/python`, `commandsv2/src/test/python` | Python bindings (RobotPy commands2) and tests |
| `commandsv2/src/test/native/BUILD.bazel` | Bazel build file |
| `commandsv2/src/dev` | upstream developer scratch executable |
| `fields/src/generate`, `fields/generate_fields.py`, `fields/convert_apriltag_layouts.py` | Jinja templates, generator input and the official AprilTag CSVs with their converters; their output (`fields/src/generated` and the field JSON files) is committed upstream and vendored instead |
| `fields/src/test/resources`, `apriltag/src/test/resources` | custom layout, field image and tag image files used only by the Java tests |
| `fields/src/main/java`, `fields/src/main/python`, `apriltag/src/main/java`, `apriltag/src/main/python` | Java API and Python bindings |
| `apriltag/src/dev` | desktop development executable |
| `apriltag/AprilTag.json` | vendordep descriptor for the desktop/roboRIO build system |
| `drivers/src/main/native/cpp/empty.cpp` | empty translation unit (upstream build placeholder) |
| `drivers/src/generate`, `drivers/generate_pwm_motor_controllers.py` | Jinja templates, `pwm_motor_controllers.json` and their generator; the output (`drivers/src/generated`) is committed upstream and vendored instead |
| `drivers/src/main/java`, `drivers/src/generated/main/java`, `drivers/src/test/java` | Java API, generated Java motor controllers and Java tests (no JVM on the MCU) |
| `drivers/src/main/python`, `drivers/src/test/python` | Python bindings (robotpy) and tests |
| `drivers/src/dev` | upstream developer scratch executables |
| `drivers/BUILD.bazel`, `build.gradle`, `CMakeLists.txt`, `drivers-config.cmake.in`, `generate.bzl`, `robotpy_*.bzl` | upstream build files (prosdk builds with `tools/build`) |

The source of truth for these lists is the module data in `tools/upstream/modules/*.py`; this table is
updated with it.

Vendored but not compiled on their own by the header check (`tools/build/header_check.py`): Eigen's
internal headers (`Eigen/src/**`, IWYU-private, they error unless included through their module header)
and gcem's internal headers (`gcem_incl/*`, they need declarations that only `gcem.hpp` provides). Both
are checked through the module headers that include them. The AprilTag C library's
`common/doubles.h`, `common/floats.h` (C-only: their static inline functions convert `void*`
implicitly, which is invalid C++; nothing includes them) and `common/doubles_floats_impl.h` (a C
template fragment that needs `TNAME` from those two) are not compiled as C++ either.

## 5. Modules not ported (design/PORT.md section 10)

| WPILib | Reason |
|---|---|
| JNI, Java and Python bindings | no JVM or Python on the MCU |
| commandsv3 | Java-only; uses JVM continuations |
| ntcore, wpinet | replaced by the probot-core Driver Station link (an NT4-compatible server for AdvantageScope/Elastic is a later work item) |
| cscore, cameraserver | no camera on the kit |
| halsim desktop GUI extensions | desktop only |
| DataLog reader, upb | desktop tool side |
| argparse | desktop command-line tools |
| Romi and XRP vendordeps | other robots |

## 6. Status of this file

Wave 1 (foundation) covers wpiutil, telemetry and tunables. Wave 2 adds wpimath (wpi/math, wpi/units,
struct, protobuf, json), Eigen, gcem and Sleipnir, the HAL (common code, the host simulation backend
and the hand-written ESP32-S3 backend, `design/hal-esp32.md`) and the DataLog writer. Wave 3 adds
wpilibc (framework, opmodes, Driver Station and the generated HID classes, events, hardware, drive,
LED, system, sysid, preferences, simulation, smartdashboard), with NetworkTables removed by the
`wpilibc/*` patches and the robot host hooks in `prosdk/platform/RobotHost.hpp`. The robot host
(`PROSDK_ROBOT_MAIN`, hosted loop on probot-core, `wpilibc/0010`) runs robot classes on the robot. commandsv2 (the
command-based framework, `commandsv2/*` patches) is vendored on top of wpilibc. fields (field metadata and the built-in AprilTag
layouts, `fields/0001`) and apriltag (detector, pose estimator and the AprilTag C library,
`apriltag/*`) are vendored on wpimath. drivers (the generated PWM motor controllers and
GoBildaPinpoint, unchanged) is vendored on wpilibc; the hand-written devices are listed in section 7. Later waves add their
renames and exclusions here.

## 7. prosdk additions (not in WPILib 2027)

Hand-written classes in `src/prosdk/drivers/` (`design/drivers.md`), tested on the host simulation
HAL (`test/prosdk/drivers`). They use only prosdk classes (`PWM`, `DigitalOutput`, `I2C`,
`SerialPort`), no Arduino or probot-core API.

| Class (header) | What it is |
|---|---|
| `prosdk::Servo` (`prosdk/drivers/servo/Servo.hpp`) | the WPILib 2026 `frc::Servo` API (removed in 2027) on a GPIO (`prosdk::PWM`, 20 ms) or on a PCA9685 channel. `SetBounds()` takes the 2026 five-argument form and uses only max and min, as 2026 |
| `prosdk::BTS7960` (`prosdk/drivers/motor/BTS7960.hpp`) | BTS7960/IBT-2 H-bridge on digital PWM (PWM on RPWM/LPWM or on the bridged enable); `MotorController`, `MotorSafety` |
| `prosdk::VNH5019` (`prosdk/drivers/motor/VNH5019.hpp`) | VNH5019 H-bridge (INA/INB/PWM, optional EN/DIAG), brake or coast at 0; `MotorController`, `MotorSafety` |
| `prosdk::PCA9685` (`prosdk/drivers/pwm/PCA9685.hpp`) | 16-channel I2C PWM controller with buffered writes and its own output gate (all outputs off while disabled; emergency-stop limitation in `design/drivers.md`) |
| `prosdk::ProHubMotor` (`prosdk/drivers/motor/ProHubMotor.hpp`), `prosdk::prohub` (`prosdk/drivers/ProHub.hpp`) | ProHub motor port (two PCA9685 channels) and the board constants and pin setup |
| `prosdk::ProHubEncoders`, `prosdk::ProHubEncoder` (`prosdk/drivers/encoder/ProHubEncoders.hpp`) | ProHub encoder stream (108-byte frames over `SerialPort`) and an `Encoder`-like view of one channel |
| `prosdk::MPU6050` (`prosdk/drivers/imu/MPU6050.hpp`) | MPU-6050 gyro/accelerometer with the `OnboardIMU` getters, integrated yaw, bias calibration |

ESP32 HAL addition used by BTS7960 and VNH5019: digital PWM generators on LEDC (section 3).
