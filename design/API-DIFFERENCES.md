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
| HAL backend | host: the WPILib simulation backend (`host/hal/sim`, outside the Arduino tree; `HAL_GetRuntimeType()` is `HAL_RUNTIME_SIMULATION`). ESP32: the prosdk backend in `src/prosdk/platform/hal/esp32` (SystemCore's backend is its reference). The function contract is `design/hal-esp32-functions.txt` | layout |
| HAL handle version check (`getHandleTypedIndex`, `handles/HandlesInternal.hpp`) | `__FIRST_SYSTEMCORE__` replaced by `PROSDK_TARGET_HW`: like SystemCore, the ESP32 does not check the handle version byte; the host build checks it | `hal/0001` |
| `HAL_SetPrintErrorImpl` default (text printed by `HAL_SendError`) | written to the platform console instead of `fwrite(stderr)` | `hal/0002` |
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

The source of truth for these lists is the module data in `tools/upstream/modules/*.py`; this table is
updated with it.

Vendored but not compiled on their own by the header check (`tools/build/header_check.py`): Eigen's
internal headers (`Eigen/src/**`, IWYU-private, they error unless included through their module header)
and gcem's internal headers (`gcem_incl/*`, they need declarations that only `gcem.hpp` provides). Both
are checked through the module headers that include them.

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
struct, protobuf, json), Eigen, gcem and Sleipnir. Later waves (hal, wpilibc, commandsv2, datalog) add
their renames and exclusions here.
