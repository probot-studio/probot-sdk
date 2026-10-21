# wpimath (allwpilib b68effca, 2027 dev) -> prosdk on ESP32-S3: port investigation

Root: `allwpilib/wpimath` (paths below are relative to `wpimath/` unless they start with another module name).
Method: read-only greps/reads plus a small include-closure script (no builds).

## 1. Size and layout

| Area | Files | Notes |
|---|---|---|
| `src/main/native/include/wpi/math/**` | 215 headers | 16 submodules (table in section 2) |
| `src/main/native/include/wpi/units/**` | 45 headers, about 6.9k lines | The C++ units library lives in wpimath. `base.hpp` alone is 3531 lines. The top-level `wpiunits/` module is Java only. |
| `src/main/native/cpp/**` | 147 files | 58 core `.cpp` (incl. `filter/internal`), 35 `struct/*.cpp`, 37 `proto/*.cpp`, 13 `jni/**` (11 cpp + 2 hpp) |
| `src/generated/main/native/cpp/wpimath/protobuf/` | 9 `.npb.h` + 9 `.npb.cpp`, 3290 lines | nanopb 0.4.9 output. **Committed to git**, so no generation is needed. |
| `src/main/proto/wpimath/protobuf/*.proto` | 9 | controller, geometry2d, geometry3d, kinematics, shape, spline, system, trajectory, wpimath |
| `thirdparty/eigen` | 306 files, 5.5 MB | Eigen **5.0.1**, header-only, includes `unsupported/Eigen/MatrixFunctions`. No .c/.cpp. |
| `thirdparty/gcem` | 68 files, 336 KB | gcem 1.x, header-only constexpr math. No .c/.cpp. |
| `thirdparty/sleipnir` | 63 files, about 14.7k lines | 10 tiny `src/**/*.cpp` (explicit template instantiations) and headers. `include/gch/small_vector.hpp` is a 12-line shim onto `wpi::util::SmallVector`. |
| `src/test/native` | 198 files | gtest. Useful for host-side validation of the port. |

Total non-JNI wpimath source: about 40.7k lines (headers + cpp, excluding thirdparty).

### Build definitions (`CMakeLists.txt`, `BUILD.bazel`)
- `wpimath` = all `cpp/**/*.cpp` minus `cpp/jni/**`, plus `src/generated/.../protobuf/*.cpp` (an OBJECT lib), plus Sleipnir `src/**/*.cpp`.
- Links against **telemetry, tunables, wpiutil**. Bazel: `//telemetry`, `//tunables`, `//wpiutil`, `@eigen`, gcem, sleipnir.
- Defines are only `WPILIB_EXPORTS`/`SLEIPNIR_EXPORTS` (DLL export, irrelevant here). `cxx_std_23`. **`SLEIPNIR_DISABLE_DIAGNOSTICS` is not defined upstream**, so std::print diagnostics are compiled in.
- Include roots: `src/main/native/include`, `src/generated/main/native/cpp`, `thirdparty/{eigen,gcem,sleipnir}/include`, and `thirdparty/sleipnir/src`.

### Generated files
- `generate_wpimath.py` calls three generators:
  - `generate_numbers.py`: Java `Nat`/`N1..N20` only.
  - `generate_quickbuf.py`: Java QuickBuffers only.
  - `generate_nanopb.py`: C++ `.npb.h/.npb.cpp`. Needs protoc and the nanopb generator.
- All C++ outputs are committed under `src/generated/main/native/cpp/wpimath/protobuf/`. No C++ generation step is needed for the port.
- Re-generation is only needed if `.proto` files change.
- The generated files embed serialized `FileDescriptorProto` byte arrays: geometry2d 1360 B, geometry3d 1979, kinematics 4332, controller 1999, trajectory 1858, system 1445, spline 1091, shape 887, wpimath 639. That is about 15.6 KB of flash rodata in total.
- The generated files use C identifiers like `wpi_proto_ProtobufPose2d` and package `wpi.proto`.
  - **Do not rename these** during the wpi->prosdk rename. The `.proto` package/type names (`proto:wpi.proto.ProtobufPose2d`) and struct type names (`struct:Pose2d`) are what AdvantageScope/Glass recognize.
  - The descriptor bytes are hex-encoded, so a text sed will not corrupt them. A sed on `wpi_proto_` identifiers would break the link between `.npb.h` and `*Proto.cpp` unless it is applied consistently.
- Units: `wpiunits/generate_units.py` is Java-only. The C++ units are hand-written.

## 2. Per-submodule inventory (core files exclude `struct/` and `proto/`)

| Submodule | core hdr | core cpp | struct/proto hdr | Notable deps (counts = include lines / call sites) | wpi/util headers used |
|---|---|---|---|---|---|
| controller | 16 | 9 | 9/9 | telemetry=6, tunables=11, sleipnir=4 (ArmFeedforward.cpp), ReportUsage=5, throw=9 | MathExtras, UsageReporting, array |
| estimator | 22 | 10 | 0/0 | ReportUsage=8, throw=18, std::format=3 | UsageReporting, array |
| filter | 6 | 7 (incl. internal/AnalogPrototypes, BilinearTransform, JacobiElliptic, Zpk) | 0/0 | throw=23 (BiquadFilterDesign.cpp 18), std::complex | Algorithm, UsageReporting, array, circular_buffer |
| fmt | 1 (Eigen.hpp std::formatter) | 0 | - | std::format, Eigen/SparseCore | - |
| geometry | 14 (+detail/RotationVectorToMatrix) | 7 | 22/22 | json (to_json/from_json in 7 cpp), StackTrace (Rotation2d.hpp), throw=9 | MathExtras, StackTrace, json |
| interpolation | 1 (TimeInterpolatableBuffer) | 0 | - | std::vector | MathExtras |
| kinematics | 25 | 12 | 27/27 | json (ChassisVelocities/Accelerations), ReportUsage=12 | MathExtras, UsageReporting, array, json |
| linalg | 3 (DARE.hpp header-only with std::expected, EigenCore, ct_matrix) | 0 | 2/2 (Matrix/Vector, templates) | Eigen LU/QR/Cholesky | - |
| optimization | 1 (SimulatedAnnealing) | 0 | - | `<random>` (random_device + mt19937 per call) | - |
| path | 1 (TravelingSalesman) | 0 | - | `<random>` (same pattern, called per iteration) | array |
| random | 1 (Normal) | 1 | - | `<random>` | Algorithm |
| shape | 2 (Ellipse2d, Rectangle2d) | 1 | 4/4 | sleipnir Problem (Ellipse2d::Nearest), throw=2 | array |
| spline | 5 | 0 | 4/4 | throw (MalformedSplineException) | array |
| system | 8 (DCMotor, Discretization, LinearSystem, LinearSystemLoop, LinearSystemUtil, Models, NumericalIntegration, NumericalJacobian) | 1 | 3/3 | throw=33 (Models.hpp 27), MatrixFunctions (exp), Eigenvalues | Algorithm, SmallVector, UsageReporting |
| trajectory | 22 (incl. 9 constraints, Trapezoid/Exponential profile, Differential/Holonomic trajectory) | 7 | 5/8 | json (DifferentialTrajectory, HolonomicTrajectory, *Sample), print(stderr), throw=5 | MathExtras, UsageReporting, json, print |
| util | 4 (MathShared, MathUtil, StateSpaceUtil, ComputerVisionUtil) | 3 | - | mutex, timestamp, std::format | Algorithm, mutex, timestamp |
| units (`wpi/units`) | 45 | 0 | - | gcem, `<chrono>`, `<format>`, `<locale>` (localeconv), optional telemetry/tunables via `__has_include` | ct_string (only when telemetry/tunables are present) |

Struct/proto counts: 41 struct headers + 35 struct cpp, 42 proto headers + 37 proto cpp. Template-only ones have no cpp: Matrix, Vector, LinearSystem, SimpleMotorFeedforward, TrapezoidProfile, SwerveDriveKinematics, and others.

JNI (drop entirely): `cpp/jni/` has DAREJNI, EigenJNI, Exceptions, Transform3dJNI, Twist3dJNI, SleipnirJNIUtil.hpp, `autodiff/{Gradient,Hessian,Jacobian,Variable,VariableMatrix}JNI`, `optimization/ProblemJNI`.

### Direct external include tally (non-JNI)
- `SymbolExports.hpp` 176
- `units/*`: length 67, time 61, base 61, angle 36, velocity 34, math 31, ...
- `Struct.hpp` 45, `Protobuf.hpp` 43, `ProtobufCallbacks.hpp` 21
- `Eigen/Core` 45, `gcem.hpp` 26
- `wpi/util/array.hpp` 32, `UsageReporting.hpp` 25, `MathExtras` 13, `json` 13, `ct_string` 6, `Algorithm` 4
- `telemetry` 7, `tunables` 13
- `sleipnir/*` 6 (only ArmFeedforward.cpp and Ellipse2d.cpp)
- `circular_buffer` 2, `timestamp` 1, `StackTrace` 1, `SmallVector` 1, `print` 1, `mutex` 1

## 3. Transitive external dependency closure (computed)

Starting from all non-JNI wpimath files and following quoted and angle includes, these files outside wpimath are needed:

- **wpiutil (own include)**: Algorithm, StackTrace, StringMap, SymbolExports, UsageReporting.h/.hpp, array, circular_buffer, ct_string, mutex, print, priority_mutex, string.h/.hpp, timestamp, `protobuf/Protobuf.hpp`, `protobuf/ProtobufCallbacks.hpp`, `struct/Struct.hpp`.
- **wpiutil/thirdparty/llvm**: ADL, Compiler, DenseMapInfo, Endian, MathExtras, STLForwardCompat, SmallVector, function_ref, type_traits.
- **wpiutil/thirdparty/json**: `wpi/util/json.hpp`, a 424-line header. It is Mozilla/jart "json.cpp", **not nlohmann**. Its impl `src/json.cpp` (1569 lines) + `jtckdint.h` needs double-conversion and `raw_ostream`.
- **wpiutil/thirdparty/nanopb**: `pb.h`, `pb_encode.h`, `pb_decode.h` + `src/pb_{common,encode,decode}.cpp` (3139 lines). WPILib's nanopb is **modified**: it adds `pb_filedesc_t` / `file_descriptor` to `pb_msgdesc_t`. Stock nanopb or the ESP-IDF one would not work.
- **telemetry** (headers): TelemetryEntry, TelemetryLoggable, TelemetryRegistry, TelemetryTable.
- **tunables** (headers): ComplexTunable, CustomTunable, Tunable, TunableConfig, TunableRegistry, TunableTable, `detail/{TunableBase,TunableDetail,TunableMember,TunableTypeTraits,TunableTypeValue}`.
- Unresolved names were only system headers: `avr/pgmspace.h`, `sal.h`, `sanitizer/*`, `pthread.h` (all in `#if` branches), plus the C standard headers.

wpiutil `.cpp` needed at link time for this closure:
- `SmallVector.cpp` (llvm), `StringMap.cpp` (llvm, via json), `json.cpp` + double-conversion sources, `raw_ostream.cpp`
- `StackTrace.cpp`, `timestamp.cpp`, `UsageReporting.cpp`, `string.cpp`, `protobuf/Protobuf.cpp`, nanopb `src/*`, `struct/*` if DynamicStruct is wanted
- Telemetry `.cpp` (7 files) and tunables `.cpp` (6 files)

Platform-sensitive pieces of that wpiutil closure (these belong to the wpiutil port, listed so the wpimath port does not miss them):
- `raw_ostream.cpp` includes `fs.hpp`, `IOSandbox.hpp`, `<unistd.h>`, `<sys/uio.h>`. The json impl only needs `raw_string_ostream`, so either split `raw_ostream.cpp` or stub the fd streams.
- `StackTrace.cpp` uses `<stacktrace>` or `<execinfo.h>`. It must be stubbed: return "" or an `esp_backtrace` string.
- `timestamp.cpp` (`NowDefault` via chrono) maps to `esp_timer_get_time()*1000`.
- `UsageReporting.cpp` uses `std::atomic<fnptr>`, `std::format`. It can stay as is (default no-op).
- `mutex.hpp` resolves to `std::mutex` unless `WPI_HAVE_PRIORITY_MUTEX`. That is OK on ESP-IDF (pthread-backed libstdc++).

## 4. Platform-specific constructs inside wpimath itself

| Construct | Where | Embedded action |
|---|---|---|
| Threads/locks | Only `cpp/util/MathShared.cpp` (`static wpi::util::mutex setLock` + `std::scoped_lock`) | Keep (std::mutex works) or replace with a FreeRTOS mutex. No std::thread or atomics anywhere else in wpimath. |
| Time | `MathShared.cpp` DefaultMathShared::GetTimestamp -> `wpi::util::Now()`. Units `base.hpp` has `std::chrono::duration` conversions (header, harmless). | Route `Now()` to `esp_timer_get_time()`. |
| Stack traces | `geometry/Rotation2d.hpp:16,64`: `GetStackTrace(1)` in the (x=0,y=0) error path (inside `if !consteval`) | Stub `GetStackTrace` to return "". |
| File I/O / console | None in wpimath except `DrivetrainSplineTrajectoryGenerator.cpp:41` `wpi::util::print(stderr, ...)` (malformed-spline error) | stderr goes to the UART console on ESP-IDF; better to route through MathShared::ReportError. |
| Error reporting | `MathSharedStore` (ReportError/ReportWarning/GetTimestamp), used by about 20 files (feedforwards, PID, LQR, Kalman filters, pose estimators, SlewRateLimiter, Debouncer, EdgeCounterFilter, Rotation2d, MathUtil) | Implement a `ProsdkMathShared` that forwards to the probot-core DS console/error log. This is the same pattern as wpilibc `RobotBase.cpp:74 WPILibMathShared`. |
| UsageReporting | 25 files call `wpi::util::ReportUsage(...)` (kinematics, odometry, Kalman, PID, filters, LinearSystemLoop, TrapezoidProfile) | Keep as a no-op, or forward to a probot-core "features used" list. |
| Exceptions | 102 `throw` sites: 49 `std::invalid_argument`, 43 `std::domain_error`, 5 `std::runtime_error`, 1 `std::logic_error`, 4 `MalformedSplineException`. **One try/catch used for control flow** in `DrivetrainSplineTrajectoryGenerator.cpp:62,107,140` (catches `SplineParameterizer::MalformedSplineException` and returns a do-nothing trajectory). | Arduino-ESP32 builds with `-fexceptions`, so keep exceptions. Do not convert to abort: trajectory generation relies on catch. An uncaught throw (bad Models args, uncontrollable LQR, singular rotation matrix, ...) means abort and reboot. Document this, and optionally wrap user-callback entry points in probot-core with try/catch -> DS error. |
| RTTI | **None**: no `typeid`, `dynamic_cast`, `type_info`, or `std::any` in wpimath, Sleipnir, or telemetry/tunables native code. Eigen mentions typeid only in debug paths (AssignEvaluator.h/Redux.h under EIGEN_DEBUG_*). | Compatible with `-fno-rtti`. |
| `<random>` | `random/Normal.hpp:30,59`, `Normal.cpp:16`, `optimization/SimulatedAnnealing.hpp:61`, `path/TravelingSalesman.hpp:162` (inside `Neighbor()`, called every iteration): each creates `std::random_device rd; std::mt19937 gen{rd()};` **on the stack per call** | Stack risk: mt19937 state is about 2.5 KB, and libstdc++ `random_device` also embeds an mt19937 union, so about 5 KB of stack per call. Arduino `loopTask` has an 8 KB stack. Also, whether `std::random_device` works on ESP-IDF libstdc++ (getentropy/urandom) is unverified. **Patch** to a single static engine seeded from `esp_random()`. |
| Locale | `units/base.hpp:106` `localeconv()` in `detail::to_string` | newlib provides it. OK. |
| iostream | Only with `UNIT_LIB_ENABLE_IOSTREAM` (off by default). | Leave off. |
| `std::format` | 10 wpimath files + `units/formatter.hpp` (std::formatter for units), `fmt/Eigen.hpp` | Works with GCC 14 libstdc++, but costs flash (tens of KB). Keep: MathShared's API is `std::format_args`. |
| Heap containers | `std::map` vision-update buffers in PoseEstimator(3d)/TwoDeadWheelPoseEstimator(3d). `TimeInterpolatableBuffer` (std::vector, 1.5 s history, about 75 entries at 50 Hz). Dynamic Eigen in Sleipnir. | Small. Fine. |

### C++23 / compiler features used (GCC 14 -std=gnu++2b covers all)
- wpimath: `if consteval` / `if !consteval` (12 files: ct_matrix, Rotation2d/3d, Transform3d, Twist3d, LinearSystem, LinearFilter, PID/ProfiledPID/BangBang, TrapezoidProfile, DifferentialDriveKinematics), `std::expected` (DARE.hpp), concepts/`requires` (21 files), `consteval` (13 files), constexpr Eigen via `ct_matrix` + gcem.
- Sleipnir: `std::print`/`std::println` (`util/print.hpp`, removed with `SLEIPNIR_DISABLE_DIAGNOSTICS`), `std::views::zip` (problem.hpp:843, diagnostics), `std::byteswap` (spy.hpp, diagnostics), `std::to_underlying`, `std::unreachable`, `std::views::reverse/drop`, `<source_location>`.
- Nothing needs a newer compiler than GCC 14.

## 5. Telemetry and tunables coupling (hard dependency)
- `PIDController`, `BangBangController` and `ProfiledPIDController` inherit `wpi::telemetry::TelemetryLoggable` and `wpi::tunables::ComplexTunable`.
  - They hold `wpi::tunables::Tunable<double>` members (m_Kp, m_Ki, m_Kd, m_iZone, ...).
  - They implement `LogTo(TelemetryTable&)`, `GetTelemetryType()`, `PublishTunable(TunableTable&)` and `GetTunableType()`.
  - Files: `cpp/controller/PIDController.cpp:13-46`, `BangBangController.cpp:12-29`, `ProfiledPIDController.hpp:453-487`.
  - These are unconditional. wpimath cannot compile without telemetry + tunables headers and libs, or a stubbed copy of them.
- `units/base.hpp` uses `#if __has_include(<wpi/telemetry/TelemetryTable.hpp>) && !defined(UNIT_LIB_DISABLE_TELEMETRY)` to add `LogValueTo(TelemetryTable&, name, unit_t)`. It logs the base-unit double and sets property `"unit"` to a compile-time abbreviation (`ComplexAbbrev<Units>()` built from `ct_string`).
  - `units/tunable.hpp` (`TunableUnit`, `TunableMemberValue`, `MakeTunableMember`) needs tunables.
  - **When renaming include paths, also rename the `__has_include(<wpi/telemetry/...>)` / `<wpi/tunables/...>` probes.** If you miss them, unit telemetry silently disappears.
- TelemetryTable logs any `StructSerializable` type as `struct:<Name>` raw bytes with a published schema (`TelemetryTable.hpp:254-262, 378-385`), and any `ProtobufSerializable` type as `proto:...` (`:283-291`).
  - So wpimath's struct/proto specializations are exactly what makes `table.Log("pose", pose)` work.
- Recommendation: port telemetry + tunables as real modules. They use only mutex, StringMap, DenseMap, htrie_map, json, Struct/Protobuf and type_name, with no RTTI or threads. Back them with a probot-core backend: a TelemetryBackend/TunableBackend over the DS WebSocket.

## 6. Struct and protobuf serialization: keep it?

**Recommendation: keep struct serialization (it is cheap and directly useful for telemetry). Keep protobuf too, because the core headers include it unconditionally.**

- Wiring: each core value-type header includes its own `proto/*Proto.hpp` and `struct/*Struct.hpp` **at the bottom**. Examples: `Pose2d.hpp:282-283`, `Rotation2d.hpp:267-268`, `Translation3d.hpp:363-364`, `ChassisVelocities.hpp:221-222`, `SwerveModuleVelocity.hpp:75-76`, `ArmFeedforward.hpp:295-296`, `LinearSystem.hpp:261`, about 30 headers in total.
  - So every user of `Pose2d` transitively includes `wpi/util/struct/Struct.hpp` and `wpi/util/protobuf/Protobuf.hpp` -> `pb.h`/`pb_encode.h`/`pb_decode.h`.
  - Dropping protobuf means editing those ~30 headers and deleting 42+37 files plus the 18 generated files. That is doable with an `#if PROSDK_ENABLE_PROTOBUF` guard, but not worth it: nanopb is MCU-native.
- Struct (`wpi/util/struct/Struct.hpp`, 726 lines):
  - Header-only, with fixed-size `Pack/Unpack` on `std::span<uint8_t>`, constexpr `GetSize()`, `GetSchema()` strings, `ForEachNested`.
  - Deps: `Endian`, `array`, `ct_string`, `function_ref`, `mutex` (only for `StructArrayBuffer`), `type_traits`, `<format>`.
  - Example: Pose2d is 24 bytes, schema `"Translation2d translation;Rotation2d rotation"`.
  - Ideal for a compact binary telemetry channel to the browser DS, which can decode with the schema exactly like AdvantageScope does.
- Protobuf (`Protobuf.hpp` 396 lines + `ProtobufCallbacks.hpp` 688 + `Protobuf.cpp`):
  - Uses WPILib-patched nanopb with `PB_PROTO_HEADER_VERSION 40` checks.
  - Messages use nanopb callbacks (`UnpackCallback`/`PackCallback`, with SmallVector storage).
  - Cost is about 3k lines of nanopb, about 15.6 KB of descriptor rodata, and the code of 37 proto cpp files. Acceptable.
  - `ProtobufMessageDatabase.hpp` / `DynamicStruct` / `SchemaParser` (upb-based) are **not** used by wpimath; skip them.
- Keep the wire names unchanged (`Pose2d`, `wpi.proto.ProtobufPose2d`) for AdvantageScope compatibility even after the C++ namespace becomes `prosdk::`.

## 7. Sleipnir

- Used by exactly two wpimath call sites (plus JNI, dropped):
  1. `cpp/controller/ArmFeedforward.cpp:27-110`, `ArmFeedforward::Calculate(currentAngle, currentVelocity, nextVelocity)`: when `kA >= 0.1` it builds an autodiff graph (`slp::VariableMatrix`, `slp::Variable`, RK4 over VarMat), then `slp::Gradient` + `slp::Hessian` (Eigen sparse) and a Newton loop of up to 50 iterations. It runs **every call**, i.e. per robot loop.
  2. `cpp/shape/Ellipse2d.cpp:15-55`, `Ellipse2d::Nearest`: `slp::Problem<double>` with 2 decision variables and 1 equality constraint, solved with the SQP solver.
- Sleipnir contents:
  - 10 `src/*.cpp` files (7-15 lines each): explicit instantiations for `double` of Gradient, Hessian, Jacobian, VariableMatrix/solve, OCP, Problem, ipm, newton, sqp, plus `util/pool.cpp`. Headers declare `extern template`, so these sources must be compiled.
  - About 14k header lines: autodiff expression graph, IPM/SQP/Newton solvers, regularized LDLT (Eigen SimplicialLDLT/SparseCholesky), filter, feasibility restoration, OCP, multistart.
- Deps: Eigen (Core, SparseCore, SparseCholesky, QR, LU, Cholesky), `wpi/util/SmallVector` (via the gch shim), `<chrono>` (profiler, timeouts: steady_clock works on ESP-IDF).
  - `<print>`, `<fstream>` (spy.hpp) and `<format>` are all only under `#ifndef SLEIPNIR_DISABLE_DIAGNOSTICS`.
  - `<future>` + `std::async` appears only in `optimization/multistart.hpp`, which wpimath does not include.
  - `slp_assert` is `assert`. Throws appear only in hessian/jacobian (bad input) and the Python-assert path. No RTTI.
- **Memory hazard (must patch):** `src/util/pool.cpp`: `thread_local PoolResource pool{16384};`.
  - `PoolResource::add_chunk` allocates `bytes_per_block * 16384` at once, plus a `std::vector<std::byte*>` free list of 16384 pointers.
  - On 32-bit Xtensa, `sizeof(Expression<double>)` is about 48 B (vptr, 2 doubles, 3 intrusive ptrs, int32, uint32; all 31 derived `*Expression final` types add no members). So the **first expression allocation grabs about 768 KB + 64 KB**.
  - ESP32-S3 has 512 KB SRAM. Without PSRAM this throws `std::bad_alloc` and aborts. With PSRAM it silently pins about 830 KB per FreeRTOS task that ever touches Sleipnir, because it is thread_local and never freed.
  - The pool is enabled for every OS except `_WIN32` (`autodiff/expression.hpp:22-29` `USE_POOL_ALLOCATOR`).
  - Fix: change the guard to `#if defined(_WIN32) || defined(ESP_PLATFORM)` to use plain `new` (intrusive_shared), or lower `blocks_per_chunk` to about 64-256.
- Other Sleipnir settings: define `SLEIPNIR_DISABLE_DIAGNOSTICS` (drops std::print, spy fstream, profiler printing, `views::zip` diagnostics). The `EXPORT_TEMPLATE_*` / `SLEIPNIR_DLLEXPORT` macros become empty on non-Windows.
- Flash: the instantiated IPM+SQP+Newton solvers and Eigen sparse LDLT are probably 100-300 KB. That is an estimate, not measured. They are all pulled in by the `Problem<double>` instantiation even though only SQP is used. That is acceptable, but a candidate for `-ffunction-sections`/`--gc-sections`, which Arduino-ESP32 already uses.
- CPU: the ArmFeedforward 3-arg Calculate is a per-loop autodiff + Newton solve in soft-float double (see section 8). Measure it. If it is too slow, keep the analytic fallback path (already used when kA < 0.1) as an option.

## 8. Embedded-specific concerns (not compile blockers)
1. **Double precision is software-emulated** on ESP32-S3 (the FPU is single-precision only).
   - All of wpimath is `double`: units default to `double`, and `Matrixd`/`Vectord` use double.
   - Plain geometry, kinematics, odometry, PID and feedforward are cheap enough.
   - Heavy per-loop paths:
     - `LTVUnicycleController::Calculate` / `LTVDifferentialDriveController::Calculate` call `DiscretizeAB` (matrix exponential) and `DARE<3,2>` / `DARE<5,2>` **on every call** (`cpp/controller/LTV*.cpp:71-76`).
     - The UKF variants (MerweUKF, S3UKF): sigma points plus Cholesky every step.
     - Sleipnir in ArmFeedforward.
   - My rough estimate is several ms to tens of ms per LTV call (unmeasured). Benchmark on target before advertising 50 Hz use.
   - Constructors that solve DARE (LQR, KalmanFilter, SteadyStateKalmanFilter) are a one-time cost.
2. **Eigen stack use:** `EIGEN_STACK_ALLOCATION_LIMIT` defaults to 131072 and `EIGEN_ALLOCA = alloca` when `alloca` is a macro (newlib `<alloca.h>` under gnu++). So dynamic-size temporaries up to 128 KB can be put on an 8 KB task stack.
   - Define `-DEIGEN_STACK_ALLOCATION_LIMIT=4096`. The value must stay ≥ the largest fixed-size matrix; the static assert in `DenseStorage.h:42` checks this. 4096 covers up to 22x22 doubles.
   - Also define `-DEIGEN_NO_IO`: wpimath never uses Eigen streams, and the macro avoids `<sstream>`/`<iosfwd>`.
   - Also `-DEIGEN_MPL2_ONLY` (optional) and `NDEBUG` / `EIGEN_NO_DEBUG`.
   - Raise the robot loop task stack: core owns `loop()`, and the Arduino default is 8 KB. 16-32 KB is advisable for the Kalman/LTV/trajectory code.
3. **`<random>` stack and entropy:** see section 4. Patch `Normal.hpp`, `Normal.cpp`, `SimulatedAnnealing.hpp` and `TravelingSalesman.hpp`.
4. **Exceptions:** keep `-fexceptions`. Note `MalformedSplineException` control flow.
5. **Sleipnir pool:** see section 7. Must patch.
6. **Flash size:** wpimath + Eigen templates + Sleipnir + nanopb + std::format will be large (likely more than 1 MB if everything is linked). Check the partition scheme. I could not find which ESP32-S3 module/PSRAM the kit uses: KIT.md and the core docs grep showed nothing.

## 9. What is needed to compile 100% of wpimath C++ (non-JNI) for ESP32-S3

Sources to compile:
- wpimath: the 58 core `.cpp`, 35 struct `.cpp`, 37 proto `.cpp` and 9 generated `.npb.cpp`. Drop the 11 JNI `.cpp` and 2 JNI headers.
- Sleipnir `src/**` (10 files).
- From wpiutil: nanopb (3 files), `Protobuf.cpp`, llvm `SmallVector.cpp`/`StringMap.cpp`, `json.cpp` + double-conversion, `raw_ostream.cpp` (stripped), `StackTrace.cpp` (stubbed), `timestamp.cpp` (ESP impl), `UsageReporting.cpp`, `string.cpp`.
- telemetry (7 cpp) and tunables (6 cpp).

Header-only pieces: Eigen, gcem, units, and the wpiutil headers listed in section 3.

Changes required (minimal set):

| # | Change | Kind |
|---|---|---|
| 1 | Namespace/include rename wpi:: -> prosdk::, `wpi/...` -> `prosdk/...`, including the `__has_include(<wpi/telemetry|tunables/...>)` probes in `units/base.hpp`. Keep proto/struct wire names and the `wpi_proto_*` C identifiers (or rename them consistently in `.npb.*` and `*Proto.cpp`). | mechanical |
| 2 | Flatten include roots for the Arduino library layout (only `src/` is on the include path): `src/Eigen/`, `src/unsupported/Eigen/`, `src/gcem.hpp` + `src/gcem_incl/`, `src/sleipnir/`, `src/gch/`, `src/pb*.h`, generated `wpimath/protobuf/*.npb.h`. Sleipnir also needs `thirdparty/sleipnir/src` on the path only for its own cpp. Check its relative includes. | build |
| 3 | Sleipnir: `USE_POOL_ALLOCATOR=false` on ESP (or a small pool). Define `SLEIPNIR_DISABLE_DIAGNOSTICS`. | patch (2 lines) |
| 4 | `<random>`: replace per-call `random_device` + `mt19937` in 4 places with a shared engine seeded by `esp_random()`. | patch |
| 5 | MathShared: provide a probot-core implementation (DS error/warning, `esp_timer` timestamp) and install it at startup. | new glue |
| 6 | wpiutil platform shims: `Now()`, `GetStackTrace()` (return ""), `raw_ostream` without fs/IOSandbox, `mutex` -> std::mutex. | stub (wpiutil port) |
| 7 | Eigen config: `EIGEN_STACK_ALLOCATION_LIMIT=4096`, `EIGEN_NO_IO`, `EIGEN_DONT_PARALLELIZE` (default), NDEBUG. | defines |
| 8 | telemetry/tunables: port them, with a probot-core backend. Alternatively, a stub with no-op tables, but the full port is preferred since the project goal is "no missing features". | port |
| 9 | json: keep it (small, MCU-friendly, needed for `to_json`/`from_json` on 14 types and trajectory JSON loading). Alternative: `#if PROSDK_ENABLE_JSON` around the 13 cpp definitions; the headers only forward-declare `json`. | keep |

Things that are truly impossible or pointless on the MCU: **only the JNI layer**. Everything else in wpimath is pure math and can run, with the CPU/memory caveats above.

## 10. Key file references
- Build: `wpimath/CMakeLists.txt`, `wpimath/BUILD.bazel:149-188`
- Generated: `wpimath/src/generated/main/native/cpp/wpimath/protobuf/*.npb.{h,cpp}`, `wpimath/generate_{wpimath,nanopb,numbers,quickbuf}.py`, `GeneratedFiles.md`
- Units telemetry/tunable hooks: `src/main/native/include/wpi/units/base.hpp:66-90, 228-240, 3451-3507`, `units/tunable.hpp`
- MathShared: `include/wpi/math/util/MathShared.hpp`, `cpp/util/MathShared.cpp`, wpilibc reference `wpilibc/src/main/native/cpp/framework/RobotBase.cpp:74-89`
- Sleipnir pool: `thirdparty/sleipnir/src/util/pool.cpp`, `include/sleipnir/util/pool.hpp:49-101`, `include/sleipnir/autodiff/expression.hpp:20-57`
- Sleipnir callers: `cpp/controller/ArmFeedforward.cpp`, `cpp/shape/Ellipse2d.cpp`
- Exceptions used as control flow: `cpp/trajectory/DrivetrainSplineTrajectoryGenerator.cpp:41,62-66,107-110,140`, `include/wpi/math/spline/SplineParameterizer.hpp:53`
- Struct/Protobuf infrastructure: `wpiutil/src/main/native/include/wpi/util/struct/Struct.hpp`, `.../protobuf/Protobuf.hpp`, `.../protobuf/ProtobufCallbacks.hpp`, `wpiutil/src/main/native/thirdparty/nanopb/`
- Telemetry struct logging: `telemetry/src/main/native/include/wpi/telemetry/TelemetryTable.hpp:206-291, 337-385, 653-681`
