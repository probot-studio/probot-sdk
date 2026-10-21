# wpiutil (allwpilib 2027, b68effca) -> prosdk on ESP32-S3: inventory and port plan

Root: `/tmp/claude-0/-srv-probot-sdk/8fcba988-962f-4d98-ac5d-540d36df924f/scratchpad/wpi/allwpilib/wpiutil/src/main/native` (abbreviated `W/` below).
Method: static greps and reads only (no builds). The transitive closure was computed by a small include resolver
(`/tmp/claude-0/-srv-probot-sdk/8fcba988-962f-4d98-ac5d-540d36df924f/scratchpad/tools/closure2.py`). It walks the `#include`s of
wpimath, commandsv2, wpilibc, telemetry, tunables, hal and datalog (main sources only, without test/jni/python/examples),
then follows header -> header and header -> same-stem .cpp inside wpiutil and datalog.
I checked toolchain facts against the installed Arduino-ESP32 3.3.10 package in `/root/.arduino15/packages/esp32`.

---

## 0. Headline findings (read first)

1. **Effective C++ standard on Arduino-ESP32 3.3.10 is C++20, not C++23.**
   `tools/esp32s3-libs/3.3.10/flags/cpp_flags` contains `-std=gnu++2b` (token 25) **and later** `-std=gnu++2a` (token 28). GCC uses
   the last one, so sketches compile as gnu++20. wpiutil 2027 is `cxx_std_23`, and the needed set uses C++23 library features:
   `std::print/println` (`W/include/wpi/util/print.hpp`), `std::expected/unexpected` (`json.hpp`, llvm headers, wpimath and hal
   headers), `std::byteswap` and `std::to_underlying` (llvm `Endian.hpp`/`STLForwardCompat.hpp`), `std::invoke_r` (`FunctionExtras.hpp`)
   and `if consteval` (wpimath 13 uses). The toolchain (GCC 14.2, libstdc++ 14.2.0) has `<print>`, `<expected>`,
   `<stacktrace>` and `<format>`, so only the flag is wrong.
   **Fix:** have the flag come after `@cpp_flags`. Options: a `build_opt.h` in each sketch with `-std=gnu++2b` (platform.txt
   copies it, and `"@{build.opt.path}"` comes after `{compiler.cpp.flags}`), or `build.extra_flags`. `compiler.cpp.extra_flags` does
   NOT work because it comes *before* cpp_flags. In an ESP-IDF component build, set the standard per component.
   The other route is to back-port everything to C++20 (replace std::print and std::expected, shim byteswap/to_underlying/invoke_r).
   That needs far more changes and also reaches wpimath and hal.
2. **fmt is gone.** 2027 wpiutil has no `fmt` thirdparty. Everything uses `std::format` / `std::print` (`print.hpp`, `fmt/raw_ostream.hpp`,
   `Logger.cpp`, `Struct.hpp`, `Color*.hpp`). The libstdc++ for esp32s3 exports `std::to_chars(char*,char*,double,...)`, so floating-point
   `std::format` links. Flash cost is real, so measure it once with a probe build.
3. **Sendable / SendableRegistry / SendableBuilder no longer exist** anywhere in the 2027 C++ tree. A grep for `Sendable` matches only
   docs (`telemetry/doc/*.md`, `tunables/doc/*.md`). `tunables.md:829-830` says: `SendableChooser` -> `Selectable<T>`;
   `Sendable`/`SmartDashboard.putData` -> `ComplexTunable` + `Tunables.publish()`; telemetry uses `TelemetryLoggable`.
   Nothing needs porting for Sendable.
4. **DataLog moved out of wpiutil** into its own module `datalog/` (`wpi/datalog/*.hpp`, namespace `wpi::log`, 13 files / 4917 lines).
5. **JSON is no longer nlohmann.** It is Mozilla's jtjson (`W/thirdparty/json`, 424-line header + 1569-line .cpp), backed by
   double-conversion. It is small and portable, a good fit for an MCU.
6. **StringMap is no longer LLVM's.** `W/include/wpi/util/StringMap.hpp` is `std::map<std::string,T,std::less<>>` (header-only).
7. **Things that fail to compile on ESP32 as-is** (all confirmed missing in the toolchain/IDF include trees):
   - `W/cpp/StackTrace.cpp`: the `#elif !defined(_WIN32) && !defined(__EMSCRIPTEN__)` branch includes `<execinfo.h>`. There is **no
     execinfo.h**. `__cpp_lib_stacktrace` is not defined there because only `<string>` is included, not `<version>`/`<stacktrace>`.
   - `W/cpp/MappedFileRegion.cpp`: `<sys/mman.h>` is **absent**.
   - `W/thirdparty/debugging/src/*.cpp`: no ESP branch, so `is_debugger_present` stays undefined (unused by consumers).
   - `W/unix/Demangle.cpp` is fine, but `Demangle.hpp::GetTypeName(const T&)` uses `typeid(obj)`, which is a **hard error with
     `-fno-rtti`** (the Arduino default, `cpp_flags` token 6). It is used by commandsv2 `Command.cpp:20`, `Subsystem.cpp:24` and
     `SubsystemBase.cpp:16`. This is the only RTTI use in the whole needed set (apart from Eigen debug-only code).
8. **Threads:** Arduino sdkconfig sets `CONFIG_PTHREAD_TASK_STACK_SIZE_DEFAULT 2048`, `PRIO_DEFAULT 5` and `CORE_DEFAULT -1`.
   Every `std::thread` (SafeThread, FileLogger, DataLogBackgroundWriter, hal Notifier) gets a **2 KB stack** unless
   `esp_pthread_set_cfg()` is called first. Patch `SafeThreadOwnerBase::Start()` (`W/cpp/SafeThread.cpp`).
9. **Synchronization (events/semaphores/WaitForObject) is pure standard C++.** It needs no OS handles and ports unchanged.

---

## 1. Inventory by area

Counts: `include/` has 64 files / 12,864 lines. `cpp/` (excluding jni) has 22 files / 3,659 lines. `unix/` and `windows/` have 1 file each.

| thirdparty | files | lines | build role |
|---|---|---|---|
| argparse | 1 | 2,543 | header-only CLI arg parser |
| debugging | 6 | 209 | psnip debug trap + `is_debugger_present` (linux/macos/windows/emscripten .cpp) |
| double-conversion | 20 | 6,822 | 8 .cpp; already knows `__XTENSA__` (`utils.h:152`) |
| htrie | 7 | 7,423 | header-only hat-trie (`htrie_map/set`, array-hash) |
| json | 3 | 2,663 | jtjson (`json.hpp`, `json.cpp`, `jtckdint.h`) |
| llvm | 45 | 16,068 | 35 headers + 10 .cpp |
| mpack | 2 | 15,534 | MessagePack (amalgamated) |
| nanopb | 23 | 9,306 | 3 .cpp runtime + Python generator (generator not needed at build time; wpimath ships 9 pre-generated `*.npb.cpp`) |
| sigslot | 1 | 836 | `Signal.h` header-only |
| upb | 153 | 30,621 | 47 .c + 106 .h/.inc |

### 1.1 First-party (`W/include/wpi/util`, `W/cpp`)

| Area | Files | Needed by (direct) | Platform code | ESP32 verdict |
|---|---|---|---|---|
| **Synchronization** | `Synchronization.h/.hpp`, `cpp/Synchronization.cpp` (489 l), `EventVector.hpp`, `Handle.h/.hpp`, `UidVector.hpp` | wpilibc (DriverStationModeThread, Notifier, Watchdog, PeriodicPriorityQueue, DriverStationBackend, DriverStationSim), hal (HandlesInternal.hpp, DriverStation.h, Notifier sim/systemcore, FIRSTDriverStation, MrcLibDs) | one `#ifndef _WIN32` (destructor waits with `std::atomic_int::wait`) | **Port as-is.** Implemented with `wpi::util::mutex`, `condition_variable`, `DenseMap`, `UidVector` and `std::atomic`. No OS handles. `atomic::wait` falls back to the libstdc++ waiter pool (gthreads present: `_GLIBCXX_HAS_GTHREADS 1`). It only runs at static destruction, which never happens on ESP32. probot-core's "new DS packet" should call `WPI_SetEvent` on the HAL's registered handles (EventVector). |
| mutex / cv / spinlock | `mutex.hpp`, `priority_mutex.hpp`, `condition_variable.hpp`, `spinlock.hpp` | everywhere (mutex: 50+ files); spinlock: hal sim (6 files) | priority_mutex only for `__linux__ && WPI_USE_PRIORITY_MUTEX` (systemcore). spinlock picks `recursive_spinlock2` on `__arm__`, otherwise `1`. | mutex -> `std::mutex` (an ESP-IDF pthread mutex is a FreeRTOS mutex, which already has priority inheritance). cv -> `std::condition_variable`. **spinlock: patch** to alias `std::mutex`/`std::recursive_mutex` under `ESP_PLATFORM`. A busy-spin with `std::this_thread::yield()` (sched_yield) never lets a lower-priority holder on the same core run, so it can livelock. |
| **SafeThread** | `SafeThread.hpp`, `cpp/SafeThread.cpp` (139 l) | wpilibc DataLogManager, OpModeRobot, MotorSafety; hal Notifier (sim, systemcore) | none (std::thread) | **Port with patch:** call `esp_pthread_set_cfg()` (stack >= 4-8 KB, prio, core, name) inside `SafeThreadOwnerBase::Start()` before `std::thread(...)`. The default 2 KB stack is too small for std::format. `GetNativeThreadHandle()` returns a pthread_t. |
| **timestamp** | `timestamp.h/.hpp`, `cpp/timestamp.cpp` | wpimath MathShared, wpilibc RobotBase/DSBackend/DataLogManager/LEDPattern, hal sim+systemcore, datalog | none | **Port as-is.** Units are now **nanoseconds** (2026 used microseconds). Default is `steady_clock`, which is monotonic on this toolchain (`_GLIBCXX_USE_CLOCK_MONOTONIC 1` -> ESP-IDF esp_timer, 1 us resolution). Recommend `SetNowImpl([]{return esp_timer_get_time()*1000;})` at init. The hal sim already uses this hook (`hal/.../sim/MockHooks.cpp:28`). `std::atomic<int64_t>` needs `__atomic_*_8`, which ESP-IDF's newlib component provides. `GetSystemTime()` = `system_clock`: it stays at 1970 unless probot-core sets it with `settimeofday`, for example from the browser's `Date.now()` on DS connect, which gives sensible datalog filenames. |
| **StackTrace / Demangle** | `StackTrace.hpp`, `cpp/StackTrace.cpp`, `cpp/StackTraceWrap.cpp`, `Demangle.hpp`, `unix/Demangle.cpp`, `windows/Demangle.cpp` | StackTrace: wpimath `Rotation2d.hpp`; commandsv2 Command.cpp, CommandPtr.cpp; wpilibc Errors.cpp + 11 hardware classes. Demangle: commandsv2 (3 files) | StackTrace: `__cpp_lib_stacktrace` / execinfo / none. Demangle: unix (`abi::__cxa_demangle`) vs windows | **Patch StackTrace.cpp:** add an `ESP_PLATFORM` branch that walks frames with `esp_backtrace_get_start()`/`esp_backtrace_get_next_frame()` (`esp_system/include/esp_debug_helpers.h` exists) and prints PCs (decode with addr2line, or rely on the panic decoder). Alternatively register one through the existing hook `SetGetStackTraceImpl`. Do not use `std::stacktrace`: `libstdc++exp.a` exists and `_GLIBCXX_HAVE_STACKTRACE 1`, but libbacktrace has no ELF file to read on target. **Patch Demangle.hpp:** `#if defined(__GXX_RTTI)` -> typeid+`__cxa_demangle`, otherwise return a fallback ("Command"/"Subsystem", or let the caller pass a name). Alternatively compile prosdk with `-frtti` via build_opt.h; the toolchain ships both rtti and no-rtti libstdc++ variants, but the Arduino core libs are no-rtti. |
| fs | `fs.hpp` (208 l), `cpp/fs.cpp` (284 l) | wpilibc Filesystem.cpp (`current_path`), DataLogManager.cpp (`status`, `create_directory`, `space`, `directory_iterator`, `remove`), Errors.cpp (`path::filename`), hal sim Extensions.cpp; datalog DataLogBackgroundWriter (`space`, `exists`, `rename`, `OpenFileForWrite`) | `_WIN32` vs POSIX (`open/close`, `O_CLOEXEC` guarded) | **Mostly feasible.** The toolchain's `libstdc++.a` (esp32s3/no-rtti) *does* export `std::filesystem::{current_path, create_directory, directory_iterator,...}`. Config: `_GLIBCXX_HAVE_DIRENT_H 1`, `USE_MKDIR`, `USE_CHMOD`, `USE_ST_MTIM`, but **no `statvfs`** (`fs::space` returns an error) and **no realpath**. Runtime behaviour depends on the ESP-IDF VFS mount (LittleFS/FAT/SD). POSIX `fs::OpenFile` works over VFS. **Namespace clash risk:** wpiutil declares a top-level `namespace fs` and Arduino `libraries/FS/src/FS.h` also declares `namespace fs {class File; class FS; enum SeekMode}` plus `using fs::File`. Rename to `prosdk::fs`. |
| raw_ostream (llvm) | `raw_ostream.hpp`, `cpp/llvm/raw_ostream.cpp` (739 l), `fmt/raw_ostream.hpp` | wpilibc Tracer, ScopedTracer; datalog DataLogWriter; internally StackTrace, json, DynamicStruct, ErrorHandling | 18 ifdefs (Win console/UTF-16, `__MVS__`, `__linux__` EINTR) | **Port.** POSIX branch: `write/lseek/fstat/isatty` exist in ESP-IDF VFS. `<sys/uio.h>` exists (`esp32s3-libs/3.3.10/include/newlib/platform_include/sys/uio.h`). `outs()/errs()` map to STDOUT/STDERR fds. On the robot, most use is the in-memory `raw_svector_ostream`/`raw_string_ostream`. |
| MemoryBuffer / MappedFileRegion | `MemoryBuffer.hpp`, `cpp/llvm/MemoryBuffer.cpp`, `SmallVectorMemoryBuffer.hpp`, `MappedFileRegion.hpp/.cpp` | only `datalog/DataLogReader.hpp` (reading .wpilog) | mmap / Windows file mapping | **Drop**, together with DataLogReader. To keep it, stub `MappedFileRegion` to always fail so that MemoryBuffer uses its read() path (`MemoryBuffer.cpp:248`). |
| print / Logger | `print.hpp`, `Logger.hpp`, `cpp/Logger.cpp` | print: wpimath (10), wpilibc (6), hal (8), datalog (4), commandsv2 (1). Logger: datalog | none | **Port** (needs C++23 `<print>`). stdout is UART/USB-CDC on Arduino. To mirror into the browser DS console, give probot-core a `fopencookie`/`funopen` FILE* tee, or wrap `wpi::util::print` to also push into the core's log ring. |
| UsageReporting | `UsageReporting.h/.hpp`, `cpp/UsageReporting.cpp` | wpimath (25 headers), wpilibc (47), commandsv2 | none | **Port.** The default is a no-op; `RobotBase.cpp:50` installs `SetReportUsageImpl` -> HAL. On ESP32, keep the no-op or forward to a DS "about" page. |
| Alert | `Alert.h/.hpp`, `cpp/Alert.cpp` (450 l) | wpilibc RobotBase, OpModeRobot, DSBackend, AlertSim; hal MrcLibAlert (`WPI_SetAlertBackend`), sim Reset | none | **Port.** It has a pluggable C backend (`struct WPI_AlertBackend`, `WPI_SetAlertBackend/GetAlertBackend`). Route it to probot-core's DS alerts panel: install a backend, or poll `WPI_GetAlerts`. |
| RuntimeCheck | `RuntimeCheck.h`, `cpp/RuntimeCheck.cpp` | wpilibc RobotBase.hpp | `_WIN32` only (MSVC runtime check); otherwise returns 1 | Port (trivial) or stub. |
| string (C ABI) | `string.h/.hpp`, `cpp/string.cpp` | hal (21), wpilibc, datalog | none | Port. |
| Struct serialization | `struct/Struct.hpp` (726 l), `struct/DynamicStruct.hpp/.cpp`, `struct/SchemaParser.hpp/.cpp` | Struct.hpp: wpimath (46), wpilibc (5), telemetry (3), tunables (4), hal (1), datalog. Dynamic/SchemaParser: only datalog `DataLogReaderThread.hpp` | none | **Struct.hpp: port** (header-only; uses `std::format` for type names and a `wpi::util::mutex`). DynamicStruct and SchemaParser are portable but tool-side only: keep optional or drop. |
| Protobuf serialization | `protobuf/Protobuf.hpp` (396 l), `ProtobufCallbacks.hpp` (688 l), `cpp/protobuf/Protobuf.cpp`; `ProtobufMessageDatabase.hpp/.cpp` (upb) | Protobuf/Callbacks: wpimath (44+21), telemetry, tunables, wpilibc, datalog. MessageDatabase: only datalog `DataLogReaderThread.hpp` | none | **Protobuf via nanopb: port.** nanopb was designed for MCUs. pb.h config is untouched (no PB_ENABLE_MALLOC/PB_FIELD_32BIT defines in any build file). wpimath ships pre-generated `.npb.cpp/.h` (18 files), so protoc is not needed. **ProtobufMessageDatabase + upb: drop.** |
| Containers/util header-only | `Algorithm`, `array`, `circular_buffer`, `static_circular_buffer`, `ct_string`, `DecayedDerivedFrom`, `NullDeleter`, `priority_queue`, `type_name`, `SymbolExports`, `deprecated`, `nodiscard.h`, `interpolating_map` (unused in 2027), `rotated_span`, `SpanExtras`, `scope`, `FastQueue`, `CallbackManager` | mostly wpimath/wpilibc/hal (see 2.1) | `type_name.hpp`: MSVC vs `__PRETTY_FUNCTION__`; `FastQueue`: one ifdef | **Port all.** Pure standard C++, zero cost when unused. `CallbackManager` pulls in SafeThread and raw_ostream (ntcore only). |
| Color | `Color.hpp` (1103 l), `Color8Bit.hpp` | wpilibc (LEDPattern, AddressableLED, Mechanism2d) | none | Port (std::format). |
| Base64, sha1, future, raw_istream, MessagePack | `Base64.hpp/.cpp`, `sha1.hpp/.cpp`, `future.hpp/.cpp`, `raw_istream.hpp/.cpp`, `MessagePack.hpp/.cpp` | only ntcore/wpinet | raw_istream: 3 ifdefs | Portable. Drop with ntcore/wpinet, or keep as optional (probot-core has its own WebSocket). |
| Camera | `PixelFormat.h/.hpp`, `RawFrame.h/.hpp/.cpp` | nobody in the scope (cscore) | jni | **Drop.** |
| JNI | `jni_util.hpp` (1047 l), `cpp/jni/WPIUtilJNI.cpp` (656 l) | hal/wpimath jni dirs only | JNI | **Drop.** |

### 1.2 Thirdparty

| Lib | Needed by | ESP32 verdict |
|---|---|---|
| **llvm** headers: `SmallVector`, `SmallString`, `SmallSet`, `SmallPtrSet`, `DenseMap(+Info, InfoVariant)`, `StringExtras`, `MathExtras`, `FunctionExtras`, `function_ref`, `Endian`, `Compiler`, `MemAlloc`, `ErrorHandling`, `Casting`, `PointerIntPair/Union/LikeTypeTraits`, `iterator(_range)`, `STLForwardCompat`, `type_traits`, `ADL`, `EpochTracker`, `ReverseIteration`, `AlignOf`, `Errc/Errno`, `IOSandbox`, `WindowsError`, `ConvertUTF`, `raw_ostream`, `MemoryBuffer`, `SmallVectorMemoryBuffer` (35). cpp: `SmallVector`, `SmallPtrSet`, `StringExtras`, `MathExtras`, `MemAlloc`, `ErrorHandling`, `raw_ostream`, `MemoryBuffer`, `ConvertUTF`, `ConvertUTFWrapper` (10) | SmallVector: wpimath (32), commandsv2, wpilibc, telemetry, hal, datalog. DenseMap: commandsv2, wpilibc, tunables, hal, datalog. SmallSet/SmallPtrSet/FunctionExtras: commandsv2. StringExtras: wpilibc, telemetry, tunables, hal. Compiler/MemAlloc: hal | **Port** (except MemoryBuffer, and ConvertUTF is optional because only Windows raw_ostream and wpinet use it). No RTTI use (`Casting.hpp` uses `classof`). `LLVM_ENABLE_EXCEPTIONS` is not defined, so SmallVector overflow calls `report_fatal_error` -> `abort()` -> ESP panic/reboot. Install `install_fatal_error_handler` so the message reaches the DS first. SmallVector size type is 32-bit on a 32-bit target. |
| **json** (jtjson) | wpimath geometry/kinematics/trajectory `to_json/from_json` (13 cpp), wpilibc Preferences, DSBackend, Mechanism2d, DataLog/NT backends; tunables `TunableConfig.hpp` | **Port.** Needs C++23 `<expected>`. `jtckdint.h` uses `__int128` only when `__SIZEOF_INT128__` is defined (it is not on xtensa) and otherwise falls back to builtins. |
| **double-conversion** | json.cpp | **Port.** `__XTENSA__` is already in the CORRECT_DOUBLE_OPERATIONS list. Note: the S3 FPU is single-precision only, so double math is soft-float. |
| **nanopb** (pb_common/encode/decode) | Protobuf.hpp, so wpimath/telemetry/tunables/wpilibc | **Port.** |
| **htrie** (`htrie_map`, `array-hash`) | telemetry `TelemetryRegistry.cpp`, tunables `TunableRegistry.cpp` | **Port** (header-only, no platform code; heavy templates). |
| sigslot (`Signal.h`) | datalog `DataLogReaderThread.hpp` only (+26 wpinet) | Portable header-only; drop with DataLogReaderThread, or keep optional. |
| mpack | ntcore only (MessagePack.hpp) | Drop unless NT4 is ported. |
| upb | `ProtobufMessageDatabase` -> datalog `DataLogReaderThread` only | **Drop** (30k lines, tool-side dynamic protobuf). |
| argparse | none (tools) | Drop. |
| debugging | none | Drop (no ESP branch; `psnip_trap` could map to `__builtin_trap`). |

---

## 2. Needed-set (precise)

### 2.1 Direct `wpi/util/*` includes per consumer (main sources, without test/jni/python)

- **wpimath (17):** Algorithm, array, circular_buffer, ct_string, json, MathExtras, mutex, print, protobuf/ProtobufCallbacks, protobuf/Protobuf, SmallVector, StackTrace, struct/Struct, SymbolExports, timestamp, UsageReporting (+ `pb.h`).
- **commandsv2 (9):** DecayedDerivedFrom, Demangle, DenseMap, FunctionExtras, print, SmallSet, SmallVector, StackTrace, UsageReporting.
- **wpilibc (35):** Alert.h/.hpp, Color, Color8Bit, condition_variable, DenseMap, EventVector, fs, FunctionExtras, Handle.hpp, json, MathExtras, mutex, NullDeleter, print, priority_queue, raw_ostream, RuntimeCheck.h, SafeThread, SmallPtrSet, SmallString, SmallVector, StackTrace, StringExtras, string.h/.hpp, StringMap, struct/Struct, Synchronization.h/.hpp, timestamp, UsageReporting + datalog (DataLog, DataLogBackgroundWriter, FileLogger). (`wpi/util/SensorUtil.hpp` belongs to wpilibc, not wpiutil.)
- **telemetry (8):** htrie_map, mutex, protobuf/Protobuf, SmallVector, StringExtras, StringMap, struct/Struct, SymbolExports.
- **tunables (10):** DenseMap, htrie_map, json, mutex, protobuf/Protobuf, StringExtras, StringMap, struct/Struct, SymbolExports, type_name.
- **hal (25):** Alert.h, circular_buffer, Compiler, condition_variable, DenseMap, EventVector, fs, Handle.h, MemAlloc, mutex, print, priority_queue, SafeThread, SmallString, SmallVector, spinlock, StringExtras, string.h/.hpp, StringMap, struct/Struct, Synchronization.h/.hpp, timestamp, UidVector. By subdir: `include/` = string, mutex, Synchronization, spinlock, Compiler, UidVector, Struct, Handle. `sim/` adds spinlock (6), SafeThread, fs, priority_queue, EventVector, DenseMap. `systemcore/` adds circular_buffer, SafeThread. `cpp/` = jni_util (30, drop) + MrcLib.
- **datalog (internal):** condition_variable, DenseMap, Endian, fs, Logger, MemoryBuffer, mutex, print, protobuf/Protobuf, ProtobufMessageDatabase, raw_ostream, Signal.h, SmallString, SmallVector, StringExtras, string, StringMap, struct/DynamicStruct, struct/Struct, timestamp + `upb/mem/arena.h`, `upb/reflection/def.h`.

### 2.2 Transitive closure (headers + implementing .cpp)

The union is 143 headers, of which 70 are `upb/*`, all only via datalog's reader. With implementation files included it is 299 entries. Without upb and the datalog reader, the required wpiutil set is:

- **Headers (wpi/util):** ADL, Alert.h/.hpp, Algorithm, array, Casting, circular_buffer, Color, Color8Bit, Compiler, condition_variable, ConvertUTF (only via raw_ostream on Win), ct_string, DecayedDerivedFrom, Demangle, DenseMap, DenseMapInfo, Endian, EpochTracker, Errc, Errno, ErrorHandling, EventVector, fs, function_ref, FunctionExtras, Handle.h/.hpp, htrie_map (+htrie_hash, array-hash/*), IOSandbox, iterator, iterator_range, json, Logger, MathExtras, MemAlloc, mutex, NullDeleter, PointerIntPair, PointerLikeTypeTraits, PointerUnion, print, priority_mutex, priority_queue, protobuf/Protobuf, protobuf/ProtobufCallbacks, raw_ostream, ReverseIteration, RuntimeCheck.h, SafeThread, SmallPtrSet, SmallSet, SmallString, SmallVector, spinlock, StackTrace, STLForwardCompat, StringExtras, StringMap, string.h/.hpp, struct/Struct, SymbolExports, Synchronization.h/.hpp, timestamp.h/.hpp, type_name, type_traits, UidVector, UsageReporting.h/.hpp, WindowsError (header only); plus `pb.h`, `pb_common.h`, `pb_decode.h`, `pb_encode.h`, `wpi/double-conversion/*` (11).
- **Sources:** `cpp/{Alert, Logger, RuntimeCheck, SafeThread, StackTrace, StackTraceWrap, Synchronization, UsageReporting, fs, string, timestamp}.cpp`, `cpp/protobuf/Protobuf.cpp`, `unix/Demangle.cpp`; `thirdparty/llvm/cpp/llvm/{ErrorHandling, MathExtras, MemAlloc, SmallPtrSet, SmallVector, StringExtras, raw_ostream}.cpp`; `thirdparty/json/src/json.cpp`; `thirdparty/double-conversion/src/*.cpp` (8); `thirdparty/nanopb/src/pb_{common,encode,decode}.cpp`. That is **33 .cpp files**.
- **Pulled in only by the datalog reader side** (DataLogReader / DataLogReaderThread): MemoryBuffer(.cpp), MappedFileRegion(.cpp), Signal.h, DynamicStruct(.cpp), SchemaParser(.cpp), ProtobufMessageDatabase(.cpp), all of upb (47 .c).

### 2.3 Not needed by any target module
argparse, debugging, mpack/MessagePack, Base64, sha1, future, raw_istream, FastQueue, SpanExtras, scope, CallbackManager, rotated_span, static_circular_buffer, interpolating_map (no users in 2027), htrie_set, PixelFormat/RawFrame, jni_util/WPIUtilJNI, ConvertUTFWrapper, windows/Demangle.

---

## 3. Platform-specific code paths (wpiutil, without upb)

Ifdef counts per file (`_WIN32|_MSC_VER|__linux__|__APPLE__|__FRC_*|__EMSCRIPTEN__|__has_include|...`):
llvm `Compiler.hpp` 23, `raw_ostream.cpp` 18, `mpack.h` 17, `MemoryBuffer.cpp` 9, `MappedFileRegion.cpp` 7, `ErrorHandling.cpp` 4, double-conversion `utils.h` 4, `MappedFileRegion.hpp` 4, htrie 3+3, `string-to-double.cpp` 3, `raw_istream.cpp` 3, `fs.cpp` 3, `StackTrace.cpp` 2, `priority_mutex.hpp` 2, `SymbolExports.hpp` 2, `MathExtras.hpp/.cpp` 2+2, `MemAlloc.hpp` 2, `ConvertUTF.cpp` 2, `nanopb pb.h` 2, plus single ifdefs in `mutex.hpp`, `condition_variable.hpp`, `fs.hpp`, `type_name.hpp`, `FastQueue.hpp`, `Synchronization.cpp`, `RuntimeCheck.cpp`, `nodiscard.h`, `deprecated.hpp`, `jtckdint.h` and each `debugging/src/*.cpp`. Build split: `unix/` vs `windows/` (Demangle only).
Robot-target ifdefs: only `__FIRST_SYSTEMCORE__` in `priority_mutex.hpp:16`. There are **no** `__FRC_ROBORIO__` paths left in wpiutil.

Most of these resolve on ESP32 through the generic POSIX/GCC branch. The exceptions are execinfo (StackTrace), mman (MappedFileRegion) and `debugging/src`.

---

## 4. ESP32 (FreeRTOS + newlib, GCC 14.2) feasibility notes

- **Toolchain facts** (`esp-x32/2601/xtensa-esp-elf/include/c++/14.2.0/xtensa-esp-elf/esp32s3/bits/c++config.h`): `_GLIBCXX_HAS_GTHREADS 1`, `_GLIBCXX_USE_CLOCK_MONOTONIC 1`, `_GLIBCXX_USE_SCHED_YIELD 1`, `_GLIBCXX_HAVE_STACKTRACE 1`, `_GLIBCXX_HAVE_DIRENT_H 1`. Not defined: `_GLIBCXX_HAVE_SYS_STATVFS_H`, `_GLIBCXX_USE_PTHREAD_COND_CLOCKWAIT`, `_GLIBCXX_USE_REALPATH`. libstdc++ variants: `lib/esp32s3/{,no-rtti/}libstdc++{,exp}.a`.
- **Missing headers** (searched toolchain + `esp32s3-libs/3.3.10/include`): `execinfo.h`, `sys/mman.h`, `sys/statvfs.h`, `sys/inotify.h`. **Present:** `sys/uio.h`, `sys/poll.h`, `dirent.h`, `esp_debug_helpers.h`, `esp_pthread.h`.
- **condition_variable timed waits:** without `PTHREAD_COND_CLOCKWAIT`, libstdc++ turns steady_clock deadlines into system_clock deadlines. If probot-core calls `settimeofday` while threads are waiting, those waits shift once. Set the wall time early, or accept the one-off glitch. This affects `WaitForObjects(timeout)`, Notifier and Watchdog.
- **Memory:** Synchronization keeps one global DenseMap and allocates on the heap per handle, which is small. DataLog (datalog module) defaults `BLOCK_SIZE = 16 KiB`, `MAX_BUFFER_COUNT = 64` (1 MiB) and `MAX_FREE_COUNT = 16` (256 KiB) (`DataLog.hpp:383,490-491`). That does not fit in internal SRAM, so these must be made configurable (for example 2 KiB x 16), or the buffers moved to PSRAM (`CONFIG_SPIRAM 1` is in the qio_qspi sdkconfig, but the board may lack PSRAM). `DataLogBackgroundWriter` `MIN_FREE_SPACE = 5 MiB` assumes `fs::space` works, and it errors on ESP32.
- **DataLog module** (`datalog/src/main/native`): DataLog.cpp, DataLogWriter.cpp (raw_ostream) and DataLogBackgroundWriter.cpp (std::thread + fs + POSIX file I/O) can be ported with the patches above. FileLogger.cpp is `__linux__` inotify only, so drop it; wpilibc DataLogManager uses it to tail system logs. DataLogReader/ReaderThread (MemoryBuffer, sigslot, upb) are tool-side, so drop them. Recommendation: add a `DataLog` subclass that streams blocks to probot-core (WebSocket download of `.wpilog` in the browser DS), and optionally to LittleFS/SD through DataLogBackgroundWriter.
- **Exceptions:** `-fexceptions` is on. `print.hpp` catches `std::system_error`. `SafeThread.cpp` throws `runtime_error` in `SetSafeThreadNotifiers`. json has `parse_or_throw`. Struct/Color throw on invalid input.
- **RTTI:** only `Demangle.hpp:27` (`typeid(type)` on a polymorphic object). No `dynamic_cast`/`typeid` anywhere else in wpiutil, datalog, wpimath (non-Eigen), commandsv2, wpilibc, telemetry, tunables or hal. sigslot, htrie, argparse and mpack have no typeid either.
- **std::print on ESP32:** with C++23 enabled, libstdc++ 14 `std::print(FILE*)` uses `fwrite`. It goes to UART/USB-CDC stdout (no terminal/Unicode console path on non-Windows).
- **Static init:** `timestamp.cpp` reads the clock in a static initializer (`original_program_start_time`). This is fine because esp_timer is up before C++ constructors run.

---

## 5. Recommended prosdk port actions (wpiutil part)

1. **Build:** add `-std=gnu++2b` after cpp_flags (sketch `build_opt.h` shipped with every prosdk example, documented in the README; or the IDF component CMake). Without it, wpiutil, wpimath and hal headers do not compile.
2. **Rename:** `wpi::util` -> `prosdk::util`, include path `wpi/util/...` -> `prosdk/util/...`. Also rename the **top-level `fs` namespace** to `prosdk::fs` to avoid Arduino `fs::File`/`fs::FS`. Leave the `WPI_*` C symbols alone, or prefix them `PROSDK_*` consistently, because HAL C headers use them.
3. **Patches (small, `#ifdef ESP_PLATFORM`):**
   - `StackTrace.cpp`: ESP backtrace branch (or return "").
   - `Demangle.hpp`: no-RTTI fallback. commandsv2 should then default names via `SetName`, or use a CRTP/`type_name.hpp` helper.
   - `spinlock.hpp`: alias to std::mutex / std::recursive_mutex.
   - `SafeThread.cpp` `Start()`: `esp_pthread_set_cfg` (stack, prio, pin, name). Preferably expose `SafeThreadOwner::SetStackSize/Priority`.
   - `timestamp`: call `SetNowImpl(esp_timer)` at prosdk init.
   - `MappedFileRegion.cpp`: exclude, or stub to an error.
   - Exclude `debugging/`, `argparse`, `upb`, `mpack`/`MessagePack.cpp`, `RawFrame`/`PixelFormat`, `jni`, `windows/`.
4. **probot-core routing** (wpiutil hooks already exist): `WPI_SetAlertBackend` -> DS alerts. `SetReportUsageImpl` -> no-op or DS about page. stdout -> DS console tee (`fopencookie`). `install_fatal_error_handler` -> DS console before abort. DS packet arrival -> `WPI_SetEvent` (EventVector) so `DriverStation::WaitForData`-style code and `DriverStationModeThread` work. Set wall clock from the browser.
5. **Keep as optional (portable, zero-cost if unused):** DynamicStruct/SchemaParser, sigslot, Base64, sha1, future, raw_istream, FastQueue, CallbackManager, rotated_span, static_circular_buffer, interpolating_map, SpanExtras, scope, htrie_set, ConvertUTF.
6. **Verify with one probe build** (when the server is not loaded): a TU that includes print.hpp, json.hpp, Struct.hpp, Protobuf.hpp, Synchronization.hpp, SafeThread.hpp and htrie_map.hpp, built with `-std=gnu++2b -fno-rtti` for esp32s3. Measure the flash delta of `std::format`, and check `std::filesystem::directory_iterator` on a LittleFS mount.

## 6. Reference: per-file include lists of first-party headers
(from `grep '#include'`; condensed)
- Header-only, std only: Algorithm, array, Base64, circular_buffer, ct_string, DecayedDerivedFrom, FastQueue, Handle.h/.hpp, interpolating_map, Logger, NullDeleter, priority_queue, rotated_span, scope, sha1, SpanExtras, static_circular_buffer, StackTrace, StringMap, string.h/.hpp, SymbolExports, timestamp.h/.hpp, type_name, UidVector.
- With threading: mutex/condition_variable/priority_mutex (`<pthread.h>` only under `__linux__`), spinlock (`<thread>`), SafeThread (`<thread>` + Synchronization + cv + mutex), EventVector, future, CallbackManager (SafeThread, UidVector, raw_ostream).
- With OS: fs.hpp (`<filesystem>`, `<fstream>`), MappedFileRegion, jni_util (`<jni.h>`), print (`<print>`), Demangle (`<typeinfo>`).
