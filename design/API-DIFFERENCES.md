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
| `if consteval { ... }`, `if !consteval { ... }` | `if (std::is_constant_evaluated()) { ... }`, `if (!std::is_constant_evaluated()) { ... }` | C++23 language feature; GCC accepts it in gnu++20 but warns in every sketch that includes the header. Same behavior in the vendored uses (no immediate function is called inside the branch). Rewritten by the `cxx23` transform of `tools/upstream/wpilib.py` |
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
| `SafeThread` | threads start with an explicit stack (`PROSDK_THREAD_STACK_SIZE`, 4096), priority below the probot-core loop (`PROSDK_THREAD_PRIORITY`, 3) on core 1 (`PROSDK_THREAD_CORE`). A `SafeThreadBase` subclass may set `m_stackSize` and `m_threadName` in its constructor (new members, read by `SafeThreadOwner::Start()`; 0 / `nullptr` keep the defaults) | `wpiutil/0006`, `wpiutil/0009` |
| `prosdk::util::Now()` default | ESP32: `esp_timer_get_time()` (monotonic since boot) | `wpiutil/0007` |
| `prosdk::util::Demangle` | none (build fix: the `__cxa_demangle` status variable is `int`; `int32_t` is `long` on Xtensa) | `wpiutil/0008` |
| `prosdk::util::json(unsigned long)` | none (warning fix: delegates to the `unsigned long long` constructor; the upstream range check is always true where `unsigned long` is 32 bit and warned with `-Wtype-limits` in every sketch using json) | `wpiutil/0010` |
| Telemetry and tunables default warning reporters | write to the platform console (`prosdk::platform::ConsoleWrite`) instead of `std::print(stderr)`; never throw | `telemetry/0001`, `tunables/0001` |
| HAL backend | host: the WPILib simulation backend (`host/hal/sim`, outside the Arduino tree; `HAL_GetRuntimeType()` is `HAL_RUNTIME_SIMULATION`). ESP32: the prosdk backend in `src/prosdk/platform/hal/esp32` (SystemCore's backend is its reference). The function contract is `design/hal-esp32-functions.txt`; hardware limits and deviations from SystemCore are in `design/hal-esp32.md` | layout |
| HAL handle version check (`getHandleTypedIndex`, `handles/HandlesInternal.hpp`) | `__FIRST_SYSTEMCORE__` replaced by `PROSDK_TARGET_HW`: like SystemCore, the ESP32 does not check the handle version byte; the host build checks it | `hal/0001` |
| `HAL_SetPrintErrorImpl` default (text printed by `HAL_SendError`) | written to the platform console instead of `fwrite(stderr)` | `hal/0002` |
| Sim I2C: `HALSIM_GetI2CLastAddress`, `HALSIM_SetI2CResultCallback` (new, `hal/simulation/I2CData.h`), sim `HAL_TransactionI2C` / `HAL_WriteI2C` / `HAL_ReadI2C` | prosdk additions to the sim API for the Boardoza driver tests: the 7-bit address of the transfer in progress (-1 before the first and after `HALSIM_ResetI2CData`), and one result callback per port, called at the start of every transfer, whose negative result (an address NACK) fails the transfer and skips the read/write callbacks. Without a callback every transfer returns 0, as upstream. The sim serializes the transfers of a port, and `HAL_TransactionI2C` splits its phases like the ESP32 backend (no empty write before a read, no empty read after a write; upstream called both callbacks, also with size 0). ESP32: inert stubs like every `HALSIM_*` function (`HALSIM_GetI2CLastAddress` returns -1). Details: `design/boardoza/F2-sim-i2c.md` | `hal/0004` |
| Console levels (`prosdk::platform::ConsoleWrite(text, level)`) | every console line carries a level, shown on the Driver Station Logs page (probot-core R9 `console::write`): `HAL_SendError` errors (ReportError, HAL errors) and the `MathShared`, `DrivetrainSplineTrajectoryGenerator`, DataLog error defaults are errors; `HAL_SendError` warnings (ReportWarning), the `MathShared` warning default, the telemetry/tunables warning reporters (alerts on the robot), DataLog warnings and the hand-written backends' warnings are warnings; everything else (status lines, command scheduler messages, `DataLogManager::Log`) is info. A core without R9 gets the text without its level. On the robot a write from an ISR is dropped | `hal/0002`, `wpimath/0002`, `wpimath/0003`, `datalog/0002`, `telemetry/0001`, `tunables/0001`, `wpilibc/0001`, `wpilibc/0007`; hand-written `src/prosdk/platform/Console.cpp`, `CoreOps.cpp` |
| `RobotController::GetRSLState()`, `GetCommsDisableCount()` (`HAL_GetRSLState`, `HAL_GetCommsDisableCount`) | ESP32: probot-core R12 `ds::rslState()` (the signal light core draws: blinking in a RUN phase that may move the robot, solid on otherwise, with or without `PROBOT_RSL_PIN`) and `ds::commsDisableCount()` (times the Driver Station timeout stopped an enabled robot since boot). Upstream systemcore reports `HAL_HANDLE_ERROR` for both; so does a core without R12. Host: simulation | hand-written `src/prosdk/platform/hal/esp32/HAL.cpp`, `CoreOps.cpp` |
| `RobotController::SetBrownoutVoltages()`, `IsBrownedOut()` | ESP32: computed from core's battery reading with hysteresis (default 6.75 V / recovery 7.25 V, 4..13 V), reported only (outputs are not cut). An accepted `SetBrownoutVoltages` call (in range) also sets probot-core's low-voltage threshold to the brownout voltage (R12 `io::battery::setLowVoltage`), so core's `isLowVoltage()` agrees; the 6.75 V default is not forwarded, so without that call core's flag stays off (core's default) or at the sketch's own `setLowVoltage()`. No battery sensor: never browned out | hand-written `src/prosdk/platform/hal/esp32/Power.cpp` |
| Battery current | WPILib 2027 has no robot input current (no `GetInputCurrent`; PowerDistribution is not supported on the ESP32). A sketch reads probot-core's `io::battery::currentAmps()` directly (R12: every configuration; NaN without an INA sensor or a valid sample, check `std::isnan`); prosdk adds no wrapper. NaN logged through telemetry or `DataLog` is stored as NaN | none |
| `RobotController::IsSystemTimeValid()` (`HAL_GetSystemTimeValid`) | ESP32: probot-core's `ds::systemTimeValid()` (R10): true once the Driver Station page has set the robot's clock to the browser's UTC time, so DataLogManager names its log `WPILIB_<UTC date>_<time>.wpilog` and writes `systemTime` entries as upstream. A core without R10: true once the clock is past 2024-01-01. Host: simulation (always true) | hand-written `src/prosdk/platform/hal/esp32/DriverStation.cpp`, `CoreOps.cpp` |
| Telemetry, tunables and alerts on the Driver Station (`prosdk::platform::RegisterDashboardBackends()`, `ProbotTelemetryBackend`, `ProbotTunableBackend`) | backends on probot-core's dashboard topic store instead of NetworkTables (`NetworkTablesTelemetryBackend` / `NetworkTablesTunableBackend` are not ported). Same topic layout (`/Telemetry/...`, `/Tunables/...`, `.type` children, struct/protobuf type strings and schemas, JSON properties) with these differences: a robust tunable is one topic with flag ROBUST instead of `<path>/value` + `<path>/tune`; robust means only the echo of the applied value, so its initial value is the code default (upstream re-applies the retained `<path>/tune` value; prosdk's own robust tunables, the scheduler's `Cancel` with raw command addresses, `Selectable`'s `selected` and the `/Preferences` mirrors, must not come back after a reboot); keeping a value across reboots is the opt-in `properties` `{"persistent": true}` (section 7); telemetry timestamps are ignored (the store keeps the latest value); `.type` topics are read-only (properties `{"mutable":false}`, DS writes refused); DS writes are queued by core, drained before each loop and applied by `TunableRegistry::Update()`; writes of the wrong type, to immutable tunables or to unknown topics are refused with core's `WriteStatus`; topics published before core's store exists wait and are sent on the next cycle. Core limits (`PROBOT_DASH_MAX_TOPICS` 128, `PROBOT_DASH_MAX_VALUE` 1024, `PROBOT_DASH_MAX_SCHEMAS` 32, `PROBOT_DASH_MAX_SCHEMA_BYTES` 4096 per schema, all counted in `PROBOT_DASH_POOL_BYTES` 32768): an overflow is reported once on the console per limit and the robot keeps running (a refused telemetry entry is discarded, a refused tunable makes `Publish` return false, a refused schema leaves its values undecodable on the DS). A value over `PROBOT_DASH_MAX_VALUE` (or one the pool cannot hold) is refused: with probot-core R16 (draft PR #42) the Driver Station keeps the topic's previous value but marks it "Güncel değil" with the reason on its row (a widget lists its topics that have no row, such as Mechanism2d parts or the scheduler's lists, under the widget) and logs it once, until a later value is stored; with an older core the DS gets no signal and shows the previous value as current. This limits `Field2d` objects (`struct:Pose2d[]`, 24 bytes per pose): 42 poses with the default 1024, so a trajectory with more poses stops updating on the DS (marked not current with R16). The limit is global (one value for every topic, 16-16384); before `#include <prosdk.h>`, define it as the power of two at or above 24 x the largest object's poses (4096: 170 poses, 8192: 341, 16384: 682). Three costs come with it. (1) Core keeps each value in a buffer rounded up to the next power of two (at most the limit) and counts it in `PROBOT_DASH_POOL_BYTES` (32768, which also holds names, types, properties and schemas); a `Field2d` that is both telemetry and tunable (`/Telemetry/Field/<object>` and `/Tunables/Field/<object>`) holds two such buffers per object, so a 342-682 pose object takes 2 x 16384 B, the whole default pool, and the second value is refused like an oversized one: raise `PROBOT_DASH_POOL_BYTES` by the rounded size of every large value (for example 65536). (2) The limit itself sizes two heap buffers outside the pool, live values or not: the DS frame buffer (limit + 519 B, at least 4.6 KB with the default schema limit) and the WebSocket receive buffer for DS writes (limit + 4 B), together about 5.6 KB at 1024, 8.7 KB at 4096, 16.9 KB at 8192 and 33 KB at 16384. (3) A DS write of an object (the whole array) also counts against `PROBOT_DASH_WRITE_QUEUE` (2048 B): the DS's typed pose list can write at most 85 poses unless that is raised to 24 x poses too (dragging works only on objects of up to 8 poses). Persistent tunables are stored only up to 2048 bytes whatever the limit (section 7). With core R13 the wpimath protobuf descriptors (`geometry2d.proto` 1360 B, most others under 2 KB) fit the default schema limit except `kinematics.proto` (4332 B): a kinematics type sent as protobuf (`TunableProtobuf`, or a protobuf-only log of `ChassisVelocities`, the wheel and swerve module types or the kinematics classes; telemetry prefers their struct) needs `#define PROBOT_DASH_MAX_SCHEMA_BYTES 8192` (or at least 4332) before `#include <prosdk.h>`. A refused schema is reported with its size and `PROBOT_DASH_MAX_SCHEMA_BYTES`. A core without R13 limits schemas by `PROBOT_DASH_MAX_VALUE`: there a protobuf-typed topic needs `PROBOT_DASH_MAX_VALUE` of at least the descriptor's size (2048 covers every descriptor but `kinematics.proto`), and the console message gives that size. Alerts: active alerts are logged to `/Telemetry/Alerts/<group>` (`.type` "Alerts", string[] `errors`/`warnings`/`infos`, newest first) after each loop, replacing the SystemCore `MrcLibAlert` transport | hand-written `src/prosdk/platform/dashboard` |
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
| Joysticks (`prosdk::DriverStation` joystick data, `GenericHID`, `Gamepad` and the other HID classes) | ESP32: ports 0-5 are probot-core's browser gamepads (core R8; an older core delivers port 0 only), remapped to WPILib's Gamepad (SDL) layout through core's mapping profile: 15 buttons, 6 axes (Y down positive; trigger axes analog 0..1 with core's deadzone), POV 0 = core's 8-way D-pad (the D-pad buttons follow it, diagonals press two). A gamepad whose frames stop for core's input timeout (`PROBOT_INPUT_TIMEOUT_MS`, 500 ms) reads as disconnected at once. `GetJoystickName()` = the browser's gamepad name (at most 63 bytes UTF-8); `GetJoystickIsGamepad()`/`GetJoystickGamepadType()`: a gamepad of type `STANDARD` when the browser reports the standard mapping, otherwise a generic HID (`UNKNOWN`). No rumble, LEDs or touchpads (`SetRumble`/`SetLeds` have no effect). Host: simulation as upstream (`design/hal-esp32.md`, Driver Station) | HAL backend (`src/prosdk/platform/hal/CoreJoystick.cpp`) |
| `prosdk::Preferences` | API unchanged; storage is `prosdk/platform/PreferencesStore.hpp` instead of the persistent NT table `/Preferences`. ESP32: one NVS blob (namespace `prosdk`, key `prefs`), rewritten on every change (a `Set` of the stored value writes nothing; do not set changing values every loop). Host: memory, or a JSON file named with `prosdk::platform::prefs::SetPreferencesFile()`. Getters follow the NT entry semantics (numeric getters convert between integer/float/double, other types give the default); a `Set` replaces value and type (in NT a `Set` of another type fails). No `.type` key. Name clash: Arduino-ESP32's Preferences library defines a global `::Preferences`; with `using namespace prosdk;` and `<Preferences.h>` in the same sketch, write `prosdk::Preferences` | `wpilibc/0005` |
| Preferences on the Driver Station | upstream's dashboards see the NT table `/Preferences`. Here the robot host mirrors the store as robust tunables `/Preferences/<key>` on the probot-core dashboard (flags TUNABLE, MUTABLE, ROBUST; store path `/Preferences/<key>` through a second `ProbotTunableBackend` registered for `/Preferences`). Keys added, changed, retyped or removed at run time appear at the next `TunableRegistry::Update()` (the mirror runs as the registry's pre-update callback, on the robot loop); a Driver Station edit is applied by that update and written back with the store's `Set` (persisted to NVS, survives a reboot). Persistence is the store's, not the dashboard's (core's `PERSISTENT` flag is not used) | hand-written `src/prosdk/platform/dashboard/DashboardPreferences.*` |
| `prosdk::filesystem::GetOperatingDirectory()` / `GetDeployDirectory()` | ESP32: `/littlefs` and `/littlefs/deploy` (LittleFS mounted by the sketch with `LittleFS.begin()`; the sketch's `data/deploy` folder after a LittleFS upload). Host: upstream simulation paths. Both configurable (`prosdk/platform/Storage.hpp`). `GetLaunchDirectory()` returns `/` if the current path is unavailable | `wpilibc/0006` |
| `prosdk::DataLogManager` | default directory `prosdk::platform::GetLogDirectory()`: ESP32 `/littlefs/logs` when LittleFS is mounted, otherwise the log discards its data (a writer whose output goes nowhere; `GetLogDir()` is `""`) and a console line says so; host `<operating directory>/logs`. `LogNetworkTables()` has no effect (no ntcore). Random file names come from `prosdk::platform::RandomEngine`. `Log()` echoes to the platform console | `wpilibc/0007` |
| `prosdk::DataLogManager` log download | upstream's logs are copied off the roboRIO/SystemCore over the network (SSH, the DS log viewer). Here, with probot-core R11 file sharing, the manager's thread offers the log directory on the Driver Station as share `"logs"` once the directory exists (looked for again once a second; `probot::files::share(name, vfsDir)`, so LittleFS `/littlefs/logs` or an SD card's `/sd/logs` alike): the Logs page's "Dosyalar" card lists and downloads the files directly in it while the robot is idle (not while an OpMode runs, not after an e-stop). A share named `"logs"` that the sketch made is replaced. `Stop()` takes the share back before it returns; core waits at most 1 s for a running download to stop, but a download stalled on a slow browser can keep a file open longer (each chunk can take 2 s or more), so a sketch that unmounts the file system after `Stop()` waits until `probot::files::busy()` is false (probot-core R17, in the same draft PR #38; with an older core it waits a few seconds instead; prosdk itself never unmounts). A share core refuses (relative, `.`/`..` segments, over 95 bytes, or 4 other shares) prints one warning and is tried again once a second, so the share is made once the sketch unshares one of its own folders. With an older core nothing is shared and nothing is printed | `wpilibc/0015`; hand-written `CoreOps.cpp`, `CoreBridge.hpp` |
| `prosdk::DataLogManager` old-log deletion | free space from `prosdk::platform::GetStorageSpace()` (ESP32: `esp_littlefs_partition_info` of the mounted LittleFS partition, as the toolchain has no `statvfs`; host: `fs::space`) instead of `fs::space`. ESP32 limits scaled to flash (`prosdk/platform/WpilibcConfig.hpp`): delete the oldest `WPILIB_*.wpilog` below `PROSDK_DATALOGMANAGER_FREE_SPACE_THRESHOLD` (128 KiB, at most half the partition) until that much is free or `PROSDK_DATALOGMANAGER_FILE_COUNT_THRESHOLD` (3) files remain, warning below twice the threshold, sizes in KiB; host keeps 50 MB / 10 files / MB. Equal modification times (LittleFS without mtime) are ordered by name. A log directory outside `/littlefs` (e.g. an SD card) has unknown space on ESP32: nothing is deleted (upstream's fallback) | `wpilibc/0012` |
| `DataLogManager::LogConsoleOutput()` | upstream tails the SystemCore console log file (`FileLogger`, not vendored). Here a console tee (`prosdk::platform::SetConsoleTee`) copies the prosdk console (`ConsoleWrite`: errors, warnings, library messages, `DataLogManager::Log`; not `Serial` or stdout) into a `PROSDK_DATALOGMANAGER_CONSOLE_BUFFER` (2048) byte buffer that the manager's thread drains into the `console` string entry every loop (whole lines, stamped when drained, up to 250 ms late); overflow is logged as a "bytes of console output dropped" line. On by default on the robot (`RobotBase::IsReal()`, as upstream); `LogConsoleOutput(true)` also enables it on the host (upstream: no effect there). `DataLogManager::Stop()` stops the writer before the last drain, so output after the last loop is lost | `wpilibc/0012` |
| `DataLogManager` thread | stack `PROSDK_DATALOGMANAGER_THREAD_STACK_SIZE` (8192; the SafeThread default 4096 is too small for the `std::format` of the date/time rename, about 3.2 KB on an x86-64 host estimate), thread name `datalogmgr` | `wpilibc/0012`, `wpiutil/0009` |
| `DataLogManager` log file name | none (same `WPILIB_<yyyymmdd>_<hhmmss>...wpilog` text, UTC): the time stamp comes from `gmtime_r` and `snprintf` instead of `std::format`'s chrono formatter, which linked 47 KB of flash (`design/size.md`) | `wpilibc/0013` |
| `prosdk::NetworkBooleanEvent` | over a boolean tunable instead of an NT boolean topic: the `(loop, name)` constructor publishes (or shares, for the same normalized name) a `Tunable<bool>` at `name`, initially `false`, editable on the Driver Station (`/Tunables/<name>`); `(loop, TunableTable, topic)` and `(loop, tableName, topic)` use `<table>/<topic>`; `(loop, name)` replaces the `BooleanTopic` constructor, `(loop, Tunable<bool>)` (the event takes the tunable over) the `BooleanSubscriber` one, `(loop, TunableTable, topic)` the `NetworkTable` one; the `NetworkTableInstance` overload is removed. The event is true while the Driver Station is attached and the tunable is true (upstream: while NT is connected and the topic is true). The value updates when the tunable registry updates (once per robot loop) | `wpilibc/0011` |
| `prosdk::SerialPort::SetReadBufferSize()` / `SetWriteBufferSize()` | no effect on ESP32: the UART driver buffers are fixed at 1 KiB each way when the port opens (`src/prosdk/platform/hal/esp32/SerialPort.cpp`); host: simulation as upstream | HAL backend |
| `prosdk::RobotController::GetComments()` | ESP32: always `""` (SystemCore reads the comments string of its robot image; the probot has none) | HAL backend |
| `prosdk::Notifier`, `prosdk::Watchdog`, `prosdk::internal::DriverStationModeThread` | threads started through `ScopedThreadConfig` (names `notifier`, `watchdog`, `dsmode`; stacks `PROSDK_NOTIFIER_THREAD_STACK_SIZE` 8192, `PROSDK_WATCHDOG_THREAD_STACK_SIZE` 6144, `PROSDK_DS_MODE_THREAD_STACK_SIZE` 6144 in `prosdk/platform/WpilibcConfig.hpp`) | `wpilibc/0008` |
| `prosdk::Tracer::ClearEpochs()` (also called by `Watchdog::Reset()` / `Enable()`) | keeps the entries added since the previous clear, marked unset, and erases an entry after a whole loop without it, so a loop that repeats the previous loop's epochs allocates nothing (upstream frees every entry and allocates a map node and a key string again per epoch per loop). The first new epoch of a loop erases the entries that loop has not added again before it allocates, so the epochs never hold more heap than upstream's at any time. `PrintEpochs()` prints the same epochs as upstream; an epoch that is not added again keeps its heap until the next new epoch or for one more loop | `wpilibc/0014` |
| Pneumatics (`Solenoid`, `DoubleSolenoid`, `Compressor`, `PneumaticsControlModule`, `PneumaticHub`, `PneumaticsBase`), `PowerDistribution`, `OnboardIMU` and their `*Sim` classes | API vendored unchanged and compiled. ESP32: the kit has no PCM/PH/PDP/PDH and no onboard IMU; the ESP32 HAL backend reports an error when one is constructed (`design/hal-esp32.md`), so they fail at run time, not at compile time. Host: the simulation backend implements them as upstream (the upstream tests run) | HAL stubs |
| `prosdk::CAN` and the HAL CAN API | ESP32: implemented on the TWAI peripheral (bus 0, external transceiver, classic CAN only: CAN FD frames are refused; pins and bit rate from `prosdk::platform::SetCanConfig(tx, rx, bitrate)`, without it CAN calls fail with a message; bus utilization reads 0). No FRC CAN heartbeat is sent, so FRC CAN devices (Talon, Spark, PCM/PH, PDP/PDH) stay disabled or fail (`design/hal-esp32.md`, CAN). Host: simulation as upstream | HAL backend |
| `GetWPILibVersion()` | returns `prosdk 2027.0.0-alpha.1 (WPILib b68effca)` (upstream: generated from the Gradle version) | hand-written `src/prosdk/platform/WPILibVersion.cpp` |
| `prosdk::DigitalOutput::EnablePWM()`, `UpdateDutyCycle()`, `SetPWMRate()` (digital PWM generators) | ESP32: implemented on the LEDC peripheral (systemcore stubs them): 6 generators, each on one DIO output, one rate for all (5 Hz to 40 kHz, default 500 Hz, at most 14 bits of duty). The duty is forced to 0 while the output gate is closed (the roboRIO does not gate them) and the emergency stop drives their pins low. `EnablePPS()` and `Pulse()` report `HAL_HANDLE_ERROR`. Host: simulation as upstream (`design/hal-esp32.md`, DIO) | HAL backend |
| Robot program entry point | `PROSDK_ROBOT_MAIN(Robot)` (`prosdk/platform/RobotMain.hpp`, via `<prosdk.h>`) replaces `int main() { return wpi::StartRobot<Robot>(); }`. Robot: the class is constructed on probot-core's user task at boot (not during static initialization, not on a new thread) and its loop runs from core's loop passes (design/PORT.md 8.1). Host: expands to `main()` with `StartRobot`. Only `TimedRobot`, `TimesliceRobot` and `OpModeRobot` run on the robot; a class deriving from `RobotBase` directly is reported and the program halts | hand-written `src/prosdk/platform/RobotMain.*` |
| `RobotBase::StartHosted()` (new, virtual), `TimedRobot` / `OpModeRobotBase::StartCompetition()` | `StartHosted()` does what `StartCompetition()` did before its loop and returns the callback queue; `StartCompetition()` calls it and then loops as upstream. `PeriodicPriorityQueue` gains `RunDue(now, lookahead)` (non-blocking `RunCallbacks`, same order and rescheduling), `Update(func)` and `GetVersion()` | `wpilibc/0010` |
| Robot loop timing on the robot | callbacks run only on core's loop passes. The pass period is the GCD (whole ms, 1..1000) of the callback periods and phases and follows `AddPeriodic`; a period below 1 ms runs once per pass, a phase that is not whole ms (e.g. `PeriodicOpMode::AddPeriodic`) moves to the nearest pass, a non-whole-ms period runs up to one pass late; a loop period outside 1..1000 ms is refused (reported, program halts). A late core pass restarts core's grid; the schedule is re-aligned (no catch-up, like upstream) and a warning is printed at most once per second | hand-written `src/prosdk/platform/RobotMain.cpp` |
| Robot program exceptions | constructor/loop exceptions are reported like upstream `StartRobot` (and a non-`std::exception` one too), then the program halts with outputs disabled instead of terminating (on the robot a terminate reboots). With probot-core R12 a halt (also an unsupported loop period or a robot class without a hosted loop) stops the Driver Station (`probot::requestStop()`) and every later INIT/START is stopped again with a console line, so the DS never shows the halted program as running; a core without R12 leaves the DS enableable (nothing runs). `platform::RobotProgramHalted()` | hand-written `src/prosdk/platform/RobotMain.cpp`, `hal/esp32/SmartIo.cpp` |
| `RobotBase::GetThreadId()` | ESP32: `std::thread::id{}` (the robot runs on core's user task, a FreeRTOS task without a pthread; ESP-IDF's `pthread_self()` aborts there). Host: upstream | `wpilibc/0010` |
| `DriverStationDisplay` (`HAL_WriteDisplayAnsi`) | ESP32: frames go to the Driver Station console with ANSI sequences removed; a frame equal to the previous one is not repeated. Host: simulation backend (upstream) | hand-written `src/prosdk/platform/hal/esp32/DriverStation.cpp` |
| Upstream test `TimedRobotTest` `ConstructorPublishesProgramStartTime`, `ConstructorRegistersTelemetryAndTunableBackends` | replaced by `TimedRobotTest RobotHostHooks` (they read NT topics; ntcore is not ported). The robot tests no longer reset the NT instance | `wpilibc/0004` |
| `prosdk::cmd::CommandHelper<Base, CRTP>` (command names) | built without RTTI: a command is named after its CRTP class when it is constructed (`prosdk::util::GetTypeName<CRTP>()`, for example `prosdk::cmd::InstantCommand` or a user's `DriveDistance`). Upstream C++ names it in the `Command` constructor with `GetTypeName(*this)`, which there yields the base type (`wpi::cmd::Command`). `SetName()` and `WithName()` work as upstream | `commandsv2/0001` |
| `prosdk::cmd::Subsystem::GetName()`, `SubsystemBase` default name | the static type name (`wpiutil/0004`): `prosdk::cmd::Subsystem` for a class that implements `Subsystem` directly (upstream: its dynamic type), `prosdk::cmd::SubsystemBase` for a `SubsystemBase` (as upstream, which names it in the base constructor). Name subsystems with `SubsystemBase(name)` or `SetName()` | none |
| `PrintCommand`, `CommandScheduler` warnings (loop overrun, subsystem registered twice, non-interruptible default command) | written to the platform console (`prosdk::platform::ConsoleWrite`) instead of stdout | `commandsv2/0002` |
| `CommandScheduler` watchdog epochs | same epoch names (`<command>.Execute()`, `.Initialize()`, `.End(...)`, `<subsystem>.Periodic()`), built in a buffer the scheduler reuses across loops instead of a new `std::string` per epoch per loop | `commandsv2/0005` |
| `prosdk::cmd::NetworkButton` | re-implemented over tunables instead of an NT boolean topic: `NetworkButton(path)` and `NetworkButton(table, field)` (path `table/field`) publish one `Tunable<bool>` per normalized path, shared by every button on it and removed with the last one (on the Driver Station a writable boolean under `/Tunables/<path>`). The trigger is true while the value is true and a Driver Station is attached (upstream: while NetworkTables is connected). The constructors taking `nt::BooleanTopic`, `nt::BooleanSubscriber`, `nt::NetworkTable` and `nt::NetworkTableInstance` are removed | `commandsv2/0003` |
| `CommandScheduler` and `Command` on the Driver Station | API unchanged: `telemetry::Log(name, scheduler)` and `tunables::Publish(name, scheduler)` go through the probot-core dashboard backends (`.type` `Scheduler`: `Names`, `Ids`, writable `Cancel`); commands publish `.type` `Command` (`name`, `running`). The Driver Station has a `Command` widget but no `Scheduler` widget yet (design/core-requests.md R5) | none |
| Upstream tests `CommandScheduleTest NetworkTablesTunableCancelClearsValue`, `CommandTunableButtonTest NetworkTablesLifecycleMatchesGlassRunCancel`, `NetworkButtonTest` | the first two read the NT tunable backend (ntcore is not ported) and are removed; the same flows on the Driver Station backend are tested in `test/prosdk/commands2/DashboardCommandsTest.cpp`. `NetworkButtonTest` is rewritten over a mock tunable backend | `commandsv2/0004`, `commandsv2/0003` |
| `AprilTagFieldLayout`, `AprilTagFields` | do not exist at the pinned upstream (b68effca): WPILib 2027 replaced them with the fields library, ported as `prosdk::fields` (`Field`, `FieldTag`, `FieldImage`, `GetField(FieldId)`, `GetFields()`, `GetFieldName()`, `FieldId::DEFAULT_FIELD` = 2026 Rebuilt Welded; `Field::GetTagPose(id)`, `GetTags()`, `GetOrigin()`, `Serialize(path)`) | none |
| Built-in field files (`prosdk::fields::GetResource_<field>_json()`, extern "C" `FIELDSGetResource_<field>_json(size_t*)`) | generated when vendoring (`wpilib.py` rule kind `resource`, `src/prosdk/impl/fields/resources/*.cpp`) instead of at build time by `cmake/scripts/GenResource.cmake`; same symbols and contents. Const arrays in flash rodata: the JSON and tag arrays of all 18 fields are 70,239 bytes in a sketch that calls `GetField` (it switches over every `FieldId`, so all of them link in) | `wpilib.py` |
| `prosdk::fields::Field(std::string_view path)` | reads the file with `prosdk::util::raw_fd_istream` instead of `MemoryBuffer::GetFile` (`MemoryBuffer` is not vendored, it maps files with mmap); same errors (`Cannot open file: ...`, JSON parse errors). On the robot the path is on a mounted file system (e.g. `/littlefs/deploy/...`) | `fields/0001` |
| `prosdk::fields::GetFieldImages()`, `GetFieldImage(path)` | always empty (the documented "not available" result): the field images are not vendored (section 4). Image metadata (`Field::GetImage()`: path, corners, size) is kept. The per-year headers `prosdk/fields/frc/*.hpp`, `ftc/*.hpp` still declare their `GetResource_*_png()` / `_jpg()`, which are defined nowhere (as upstream without its `fieldimages` library): calling one is a link error | hand-written `src/prosdk/platform/FieldImages.cpp` |
| `prosdk::apriltag::AprilTagDetector::Detect()` | with `Config::numThreads` > 1 the C library's worker threads (`pthread_create`) start through `ScopedThreadConfig` (name `apriltag`, stack `PROSDK_APRILTAG_THREAD_STACK_SIZE` = 8192, prosdk thread priority and core). The default `numThreads` is 1 (no threads), as upstream. Heap use during detection is not measured on the robot (no camera on the kit): the union-find alone takes 8 bytes per pixel of the decimated image (about 150 KB for 320x240 at the default `quadDecimate` 2, about 600 KB without decimation), so camera robots need PSRAM or decimation | `apriltag/0001` |
| AprilTag C library `pjpeg-idct.c` | none (build fix: the IDCT helpers take `int32_t*` instead of `int*`; `int32_t` is `long` on Xtensa) | `apriltag/0002` |
| AprilTag C library `common/pam.h` | none (header fix: includes `<stddef.h>` for `size_t`, so the header compiles on its own, as the host header check does) | `apriltag/0003` |
| AprilTag C library `tag36h11.c`, `tag16h5.c` | none (the code tables are `const`, so they stay in flash instead of 4,928 B of RAM; `design/size.md`) | `apriltag/0004` |
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
| wpilibc | `wpilibc/src/main/native/include/wpi/system/SystemServer.hpp`, `cpp/system/SystemServer.cpp` | returns the SystemCore system server's ntcore instance (HAL `SystemServer.h` is not vendored) |
| wpilibc | `wpilibc/src/main/native/include/wpi/hardware/expansionhub/*`, `cpp/hardware/expansionhub/*` | REV Expansion Hub through the SystemCore USB `rhsp` server, driven over NT topics; no rhsp server, ntcore is not ported |
| wpilibc (tests) | `wpilibc/src/test/native/cpp/NetworkTablesTelemetryBackendTest.cpp`, `NetworkTablesTunableBackendTest.cpp` | the NT backends are not vendored |
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
| `simulation/halsim_gui`, `halsim_ds_socket`, `halsim_ws_core`, `halsim_ws_server`, `halsim_ws_client`, `halsim_xrp` | dynamically loaded desktop simulation plugins (GUI, FRC Driver Station socket, WebSocket HAL state, XRP over UDP) built on wpinet; no dynamic loading on the robot, probot-core is the Driver Station |
| ExpansionHub (`wpilibc` `hardware/expansionhub`) | REV Expansion Hub through SystemCore's USB `rhsp` server over NetworkTables; no rhsp server, no hub, no ntcore |
| `aos_sdk`, `aosnt`, `ntcoreffi` | AOS packaging, AOS-to-NetworkTables bridge, ntcore C FFI (with a C DataLogManager): Linux/desktop libraries on ntcore/wpinet |
| DataLog reader, upb | desktop tool side |
| argparse | desktop command-line tools |
| Romi and XRP vendordeps | other robots |
| `glass`, `wpigui`, `thirdparty/imgui_suite`, `outlineviewer`, `datalogtool`, `tools/sysid`, `tools/wpical` | desktop GUI applications (Dear ImGui/OpenGL) that run on the driver or developer computer, not on the robot |
| `benchmark`, `developerRobot`, `shared/singlelib` | build infrastructure of the upstream repository (benchmark harness, developer test robot program, single-library packaging), not library code |
| `epilogue-runtime`, `epilogue-processor`, `wpiunits`, `wpiannotations` | Java-only (annotation-based logging, Java units library and annotations); the C++ units are `prosdk::units` |

## 6. Status of this file

Wave 1 (foundation) covers wpiutil, telemetry and tunables. Wave 2 adds wpimath (wpi/math, wpi/units,
struct, protobuf, json), Eigen, gcem and Sleipnir, the HAL (common code, the host simulation backend
and the hand-written ESP32-S3 backend, `design/hal-esp32.md`) and the DataLog writer. Wave 3 adds
wpilibc (framework, opmodes, Driver Station and the generated HID classes, events, hardware, drive,
LED, system, sysid, preferences, simulation, smartdashboard), with NetworkTables removed by the
`wpilibc/*` patches and the robot host hooks in `prosdk/platform/RobotHost.hpp`. The robot host
(`PROSDK_ROBOT_MAIN`, hosted loop on probot-core, `wpilibc/0010`) runs robot classes on the robot.
commandsv2 (the command-based framework, `commandsv2/*` patches) is vendored on top of wpilibc.
fields (field metadata and the built-in AprilTag layouts, `fields/0001`) and apriltag (detector,
pose estimator and the AprilTag C library, `apriltag/*`) are vendored on wpimath. drivers (the
generated PWM motor controllers and GoBildaPinpoint, unchanged) is vendored on wpilibc; the
hand-written devices are listed in section 7, including the Boardoza board drivers
(`design/boardoza.md`) with the SPI, timed DIO, pulse I/O and six-step platform extensions.

Wave 5 ports wpilibcExamples: 63 sketches in `examples/` and the five upstream example tests
except UnitTest in `test/examples` (decisions, pin map and deviations in `design/examples.md`).
They are ported by hand, not by `tools/upstream/wpilib.py`; `tools/upstream/examples.json` records
the upstream files at the pin and `tools/upstream/examples.py --check` (CI) reports upstream
changes.

The robot side builds on probot-core's requests R1-R15 (`design/core-requests.md`,
CoreOps ABI 8): gamepads on ports 0-5 with analog triggers and names (R8), leveled console lines
(R9), the Driver Station's wall clock (R10), the RSL state, comms-disable count, low-voltage flag
and the halt stop (R12 part 1) and the schema size limit (R13) are in section 3; R14 (the gzip DS
page) only saves flash; R12's dashboard persistence (ABI 6, core draft PR #39) is the opt-in
persistent tunables of section 7; R15's grid restarts (ABI 7, #41) only keep the robot loop's
callbacks in their passes; R11's file sharing (ABI 8, #38) offers DataLogManager's log folder on the
Driver Station (section 3). R16 (#42) has no prosdk API: the Driver Station marks a topic whose last value
was refused (section 3); R17 (`probot::files::busy()`, in #38) is what a sketch that unmounts after
`DataLogManager::Stop()` polls (section 3).

## 7. prosdk additions (not in WPILib 2027)

Hand-written classes in `src/prosdk/drivers/` (`design/drivers.md`), tested on the host simulation
HAL (`test/prosdk/drivers`). They use only prosdk classes (`PWM`, `DigitalOutput`, `DigitalInput`,
`AnalogInput`, `Encoder`, `I2C`, `SerialPort`, `SPI`) and the prosdk platform HAL extensions
(`prosdk/platform/hal/SPI.hpp`, `BitBang.hpp`, `PulseIO.hpp`, `SixStep.hpp`; `design/hal-esp32.md`),
no Arduino or probot-core API. The Boardoza rows below have no WPILib counterpart unless the row
says so; none has been tested on hardware (`design/boardoza.md`).

| Class (header) | What it is |
|---|---|
| `prosdk::Servo` (`prosdk/drivers/servo/Servo.hpp`) | the WPILib 2026 `frc::Servo` API (removed in 2027) on a GPIO (`prosdk::PWM`, 20 ms) or on a PCA9685 channel. `SetBounds()` takes the 2026 five-argument form and uses only max and min, as 2026 |
| `prosdk::BTS7960` (`prosdk/drivers/motor/BTS7960.hpp`) | BTS7960/IBT-2 H-bridge on digital PWM (PWM on RPWM/LPWM or on the bridged enable); `MotorController`, `MotorSafety` |
| `prosdk::VNH5019` (`prosdk/drivers/motor/VNH5019.hpp`) | VNH5019 H-bridge (INA/INB/PWM, optional EN/DIAG), brake or coast at 0; `MotorController`, `MotorSafety` |
| `prosdk::PCA9685` (`prosdk/drivers/pwm/PCA9685.hpp`) | 16-channel I2C PWM controller with buffered writes and its own output gate (all outputs off while disabled; emergency-stop limitation in `design/drivers.md`) |
| `prosdk::ProHubMotor` (`prosdk/drivers/motor/ProHubMotor.hpp`), `prosdk::prohub` (`prosdk/drivers/ProHub.hpp`) | ProHub motor port (two PCA9685 channels) and the board constants and pin setup |
| `prosdk::ProHubEncoders`, `prosdk::ProHubEncoder` (`prosdk/drivers/encoder/ProHubEncoders.hpp`) | ProHub encoder stream (108-byte frames over `SerialPort`) and an `Encoder`-like view of one channel |
| `prosdk::MPU6050` (`prosdk/drivers/imu/MPU6050.hpp`) | MPU-6050 gyro/accelerometer with the `OnboardIMU` getters, integrated yaw, bias calibration |
| `prosdk::SPI`, `prosdk::SpiSegment` (`prosdk/drivers/bus/SPI.hpp`) | the WPILib 2026 `frc::SPI` shape (removed in 2027): `SPI(Port, csGpio)` with `kPort0` = SPI2 and `kPort1` = SPI3 (unscoped `Port` and `Mode` enums), chip select any GPIO or -1 for a device alone on its bus, `SetClockRate` 78125 Hz to 40 MHz (default 1 MHz), `SetMode`, `Write`, `Read` (`initiate` is ignored: every read clocks out zeros), `Transaction`, and the new `TransactionSegments(span<const SpiSegment>)` that keeps CS active over several segments. Throws on a bad configuration; move-only. Not ported: auto-transfer and the accumulator. Pins: `prosdk::platform::SetSPIPins(bus, sck, mosi, miso)`. Not gated. Sim: `prosdk::sim::RegisterSpiDevice`, `CancelSpiDevice`, `GetSpiDeviceConfig` (`prosdk/simulation/SpiSim.hpp`) |
| `prosdk::DigitalShift`, `prosdk::WriteWaveform` (`prosdk/drivers/bus/DigitalShift.hpp`) | timed bit-banging on DIO pins: `DigitalShift(DigitalOutput& clock, DigitalInput& data, highTime 1 ns-20 us, lowTime 0-1 ms)`, `ShiftIn(bits 1-64)` returning `std::optional<uint64_t>`; `WriteWaveform(DigitalOutput&, spans)` writes 1-64 level spans of at most 20 us in total. Interrupts are masked per clock-high phase (at most 20 us) and for a whole waveform |
| `prosdk::PulseTransmitter`, `prosdk::Pulse` (`prosdk/drivers/bus/PulseTransmitter.hpp`) | pulse trains on an RMT TX channel: `PulseTransmitter(gpio, carrierHz 0 or 2 kHz-1 MHz, duty 0.1-0.9)`, `Send` of up to 1024 pulses of 1-32767 us. Shares the 3 RMT TX channels with `AddressableLED`. Not gated while disabled; the emergency stop drives the pin low. Sim: `prosdk/simulation/PulseIOSim.hpp` |
| `prosdk::PulseCapture` (`prosdk/drivers/bus/PulseCapture.hpp`) | pulse capture on an RMT RX channel (4): `PulseCapture(gpio, idleTime 10 us-32.767 ms, glitchFilter 0-3 us)`, a 256-pulse buffer read with `Read`; `GetDropped()` counts the overflow |
| `prosdk::LM75A`, `prosdk::HIH8000`, `prosdk::ENS210`, `prosdk::MS5611`, `prosdk::DPS310`, `prosdk::BME688` (`prosdk/drivers/environment/`) | I2C temperature, humidity, pressure and gas sensors, `(I2C::Port, address)` constructors (`MS5611` also takes the oversampling), no thread: a non-blocking `Update()` from a periodic method (`LM75A` reads on demand). `ENS210` takes an optional address although the chip's is fixed |
| `prosdk::OPT3001`, `prosdk::OPT4048` (`prosdk/drivers/light/`) | I2C ambient light and colour sensors, no thread |
| `prosdk::CaliPile` (`prosdk/drivers/thermal/CaliPile.hpp`) | Excelitas TPiS 1S/1T (`Variant`) on I2C; sends the general-call reload (0x00, 0x04) the chip needs after power-up |
| `prosdk::MLX90640`, `prosdk::MLX90641` (`prosdk/drivers/thermal/`) | Melexis thermal cameras on a reader thread, on the vendored Melexis API (`thermal/melexis/`, Apache-2.0, `NOTICE`). The plan's 1 MHz bus was dropped: the drivers expect 400 kHz, the datasheet maximum for EEPROM access |
| `prosdk::BNO055`, `prosdk::BMI088`, `prosdk::LSM303AGR` (`prosdk/drivers/imu/`) | I2C IMUs with the `OnboardIMU` getters and a reader thread (or `updatePeriod` 0 and `Update()`), as `MPU6050`. `BNO055` adds `updatePeriod` and `externalCrystal` constructor arguments and reports heading counter-clockwise positive; `LSM303AGR` adds `SetHardIronOffset` |
| `prosdk::VL53L7CX` (`prosdk/drivers/range/VL53L7CX.hpp`) | ST multizone ToF on I2C over the vendored ST ULD (`range/st/`, BSD-3-Clause, `NOTICE`): `VL53L7CX(Port, lpnChannel = -1, address = 0x29, Resolution = k4x4, units::hertz_t = 15)`; `GetDistances(span<int16_t> distances, span<uint8_t> status = {})` also returns each zone's target status (added to the plan's API), `GetTargetStatus`, `SetAddress`, `IsReady`. A thread uploads the 84 KiB firmware at the first call |
| `prosdk::DRV2605L` (`prosdk/drivers/haptic/`), `prosdk::PCF8563` (`rtc/`), `prosdk::GP8403` (`dac/`), `prosdk::I2CSwitch` (`bus/`), `prosdk::NT3H2111` (`memory/`) | haptic driver, real-time clock, 2-channel 0-5/0-10 V DAC, TPT29545/PCA9545-style I2C switch (the selection is global bus state: drivers with their own thread must not sit behind it), NTAG I2C plus memory (blocks 0 and 0x38-0x3A refused) with an optional field-detect input. The DRV2605L and GP8403 outputs are not gated |
| `prosdk::NAU7802`, `prosdk::HX711`, `prosdk::LoadCell` (`prosdk/drivers/adc/`) | 24-bit load-cell ADCs (`NAU7802` on I2C, `HX711(dataGpio, clockGpio, Gain)` through `DigitalShift`) and `LoadCell(std::function source)` for tare and scale |
| `prosdk::UbloxGnss`, `prosdk::NmeaGnss`, `prosdk::GnssFix`, `prosdk::nmea`, `prosdk::ubx` (`prosdk/drivers/gnss/`) | GNSS receivers: u-blox UBX NAV-PVT over `SerialPort` or I2C (0x42), NMEA 0183 over `SerialPort`; a reader thread (or `updatePeriod` 0); the NMEA and UBX parsers are usable on their own |
| `prosdk::TwoPinHBridge` (aliases `L9110S`, `BA6208`), `prosdk::VNH7070AS`, `prosdk::DRV8307` (`prosdk/drivers/motor/`) | `MotorController`s with `MotorSafety` on digital PWM, 20 kHz: two-input H-bridges, the VNH7070AS (VNH5019 semantics plus SEL0 and the CS output), the DRV8307 BLDC driver with internal Hall commutation. `DRV8307` adds an nFAULT input with `IsFault()`, and names the plan's `SetBrake` `SetBrakeMode` |
| `prosdk::DRV8317H` (`prosdk/drivers/motor/DRV8317H.hpp`) | 3-phase BLDC driver in 3x PWM mode with Hall six-step commutation in a GPIO interrupt (`prosdk/platform/hal/SixStep.hpp`: `OpenSixStep`, `SetSixStepTable`, `SetSixStepOutput`, `GetSixStepFeedback`, `CloseSixStep`; MCPWM, at most 2 motors). `MotorController`, `MotorSafety`; coasts while the output gate is closed |
| `prosdk::W25QFlash`, `prosdk::W25NFlash` (`prosdk/drivers/memory/`) | Winbond SPI NOR and SPI NAND flash, `(SPI::Port, cs, hz)`, raw read/program/erase |
| `prosdk::platform::MountSdCard` (`prosdk/platform/SdCard.hpp`) | platform addition: mounts a FAT16/FAT32 microSD card over SPI with ESP-IDF's `esp_vfs_fat_sdspi_mount` (`MountSdCard(spiBus, csGpio, mountPoint = "/sd", hz = 20 MHz)`), for example as the DataLogManager directory. Never unmounts; no exFAT; host stub returns false |
| `prosdk::TMAG5170` (`prosdk/drivers/magnetic/`) | TI 3D Hall sensor on SPI (32-bit frames, CRC4) |
| `prosdk::ClockedAddressableLED` (`prosdk/drivers/led/`) | clock-and-data (APA102-style) LED strips on a whole SPI bus without chip select; takes `AddressableLED::LEDData`, so `LEDPattern` works (WPILib's `AddressableLED` is one-wire only) |
| `prosdk::SX126x`, `prosdk::SX126xConfig` (`prosdk/drivers/radio/SX126x.hpp`) | Semtech SX1261/SX1262 radio, LoRa packets only (no FSK): `(SPI::Port, cs, busy, dio1, reset, rxEn = -1, txEn = -1, Config)`; a thread polls DIO1. `Config` adds `dio2AsRfSwitch` and `spiClockHz` to the plan's fields |
| `prosdk::IRTransceiver`, `prosdk::OokReceiver` (`prosdk/drivers/radio/`), `prosdk::PYD1598` (`prosdk/drivers/sensor/`) | 38 kHz IR (NEC, including extended NEC and `SendNecRepeat`), 433 MHz OOK remote codes, and the Excelitas DigiPyro motion sensor, on the pulse I/O classes. Decoding runs in `Poll()` on the caller's thread. Additions to the plan: `IRTransceiver` pins may be -1 (receive-only or transmit-only), `IRTransceiver::GetLastPulses` and `OokReceiver::GetRecentPulses`, and a second `PYD1598` constructor with a `Config` |
| `prosdk::FT8xx`, `prosdk::ft8xx` panel presets (`prosdk/drivers/display/FT8xx.hpp`, `FT8xxPanels.hpp`) | Bridgetek FT800/FT810 (EVE) display controller on SPI: `FT8xx(SPI::Port, cs, pd, const FT8xxPanel&, intGpio = -1, clockHz = 20 MHz)`, display lists and coprocessor commands, resistive touch. Unlike the plan there is no blocking `Begin()`: the `ft8xx` thread brings the chip up and reports it through `IsReady()` |
| `prosdk::AnalogCurrentSensor` (`prosdk/drivers/analog/AnalogCurrentSensor.hpp`) | linear analog current sensor on `AnalogInput` (`adcGpio, voltsPerAmp, zeroVolts, divider`), with the `INA250A1(...)` and `ACS758LCB050B(...)` presets and `CalibrateZero()` |

ESP32 HAL addition used by BTS7960, VNH5019, TwoPinHBridge, VNH7070AS and DRV8307: digital PWM
generators on LEDC (section 3). The platform HAL extensions for the Boardoza drivers (SPI, timed
DIO, pulse I/O, six-step) are prosdk-only C++ APIs in `prosdk::platform::hal`, not WPILib HAL C
functions; two differ from the implementation plan: the timed DIO functions take GPIO numbers
instead of `HAL_DigitalHandle`s (wpilibc's `DigitalInput`/`DigitalOutput` do not expose their
handles), and `OpenPulseRx` takes the glitch filter in microseconds.

Persistent tunables (platform addition, `src/prosdk/platform/dashboard/ProbotTunableBackend.*`): a
mutable tunable whose `TunableConfig::properties` has `{"persistent": true}` (the NetworkTables
topic property of that name; upstream's tunable backend only passes it on) keeps its value across
reboots in probot-core's dashboard store (R12 part 2, NVS namespace `pb_dash`, CoreOps ABI 6). It is
published with core's `PERSISTENT` flag; when the topic is created core queues its stored value, and
`Publish()` applies it before it returns, through the Driver Station write path (value set, tune
revision, `onRemoteSet`, `onTune` after the initial publish), as upstream applies a robust tunable's
retained value. A tunable published before core's store exists gets its stored value when it is
published (the pre-cycle flush, before the robot loop). A stored value that does not decode any more
is refused, which erases it. `ROBUST` alone never persists (section 3), immutable tunables and the
`.type` topics of complex tunables are never persistent. Core stores only Driver Station writes the
robot accepted, never a value the robot program sets (`Set()`, a getter): a write the backend refuses
(type mismatch, a payload that does not decode) is not stored, an applied one is, once no OpMode
runs (a flash write stalls both cores). After a reboot a persistent tunable's stored tune wins over
its code default, so changing the default in code has no effect on a robot that stored a tune; call
`ClearPersistedTunables()` once after changing a default. Stored values are limited to 2048 bytes
and to `PROBOT_DASH_MAX_VALUE` (larger: PB-E308 on the console, not stored); keep numbers and short
strings. Core drops an accepted tune not stored yet when its topic is removed: a tune made during an
OpMode on a tunable removed when the OpMode ends (an OpMode-scoped object) may not reach the next
reboot. Published again in the same run (`Remove` and `Publish`, or a new tunable object on the same
path), a persistent tunable gets the last value core accepted on that path in this run (a tune the
robot applied, or core's restore), not core's possibly older record. Restored or remembered, the value
is applied as a Driver Station write: it comes back with a tune revision, `onRemoteSet` and `onTune`,
and it overwrites a `Set()` made while the tunable was not published (between `Remove` and `Publish`,
or a new object's `Set()` before its `Publish()`) or since the tune, as after a reboot. A restored or
remembered value equal to the tunable's own is not applied (no tune revision, no `onTune`; core's
record stays), and a tunable of another value type or type string takes core's record.
`prosdk::platform::ClearPersistedTunables()` (`<prosdk.h>`) erases every stored value and the tunes
not stored yet (core's `clearPersisted()`; flash write: call it while the robot is stopped) and also
what the backend remembers for republishing, so a tunable published again afterwards keeps its own
value until it is tuned again (core stores the next Driver Station write). With a probot-core without
persistence the opt-in does nothing: the tunable starts from its code value and one console warning
per path says so (a republish does not repeat it); `ClearPersistedTunables()` returns false. The
bridge detects persistence with core's `PROBOT_HAS_DASH_PERSIST` macro and a requires-check on
`probot::dashboard::clearPersisted()` (earlier drafts of core's persistence stored the robot's own
values and had no macro; they are not wired).
