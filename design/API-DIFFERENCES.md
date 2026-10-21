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

Kept unchanged (wire formats and C ABI): `wpi.proto.*`, `wpi_proto_*`, `struct:<Type>` and `proto:<Type>`
type strings, `WPI_*` and `HAL_*` C functions, `WPILIB_*` macros.

## 2. Identifiers renamed because Arduino.h defines them as macros

Every enumerator of an enum that has at least one colliding name is renamed (k prefix, WPILib C++ style;
the Java names are the same upper-case words, so there is no other spelling to use).

| WPILib | prosdk | Colliding Arduino macros |
|---|---|---|
| `wpi::util::Alert::Level::HIGH` / `MEDIUM` / `LOW` | `prosdk::util::Alert::Level::kHigh` / `kMedium` / `kLow` | `HIGH`, `LOW` |
| `wpi::tunables::TunableConfig::Polling::DEFAULT` / `ALWAYS_GET` / `GET_ON_CHANGE` | `prosdk::tunables::TunableConfig::Polling::kDefault` / `kAlwaysGet` / `kGetOnChange` | `DEFAULT` |

Other Arduino macro names used inside vendored headers (internal names, parameters, `F`, `likely`, ...)
need no rename: each public header is wrapped in `#pragma push_macro` / `#undef` / `#pragma pop_macro` for
exactly the macros it uses (computed by the script from the installed core headers).

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

The source of truth for these lists is the module data in `tools/upstream/modules/*.py`; this table is
updated with it.

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

Wave 1 (foundation) covers wpiutil, telemetry and tunables. Later waves (wpimath, units, hal, wpilibc,
commandsv2, datalog) add their renames and exclusions here.
