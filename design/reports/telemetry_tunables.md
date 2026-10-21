# WPILib 2027 `telemetry` + `tunables` — architecture, usage, and the prosdk/probot-core backend design

Source: allwpilib @ b68effca (sparse checkout at
`/tmp/claude-0/-srv-probot-sdk/8fcba988-962f-4d98-ac5d-540d36df924f/scratchpad/wpi/allwpilib`, called `$W` below).
Target: prosdk (ESP32-S3, Arduino-ESP32 3.3.10, GCC 14.2, gnu++2b, `-fexceptions`, `-fno-rtti`) on top of
probot-core 0.4.0 (`/srv/probot/core`).

All paths, counts and names below come from reading the sources (read-only, no builds).

---

## 1. TL;DR

* `telemetry` and `tunables` are **two small modules that depend only on `wpiutil`** (CMake:
  `target_link_libraries(telemetry PUBLIC wpiutil)`, `target_link_libraries(tunables PUBLIC wpiutil)`).
  They do **not** depend on ntcore or datalog. The NetworkTables/DataLog code lives in **wpilibc**
  (`wpilibc/src/main/native/{include/wpi,cpp}/backend/*`, 2,177 lines). It is plugged in at runtime by
  `RobotBase::RobotBase()` (`wpilibc/src/main/native/cpp/framework/RobotBase.cpp:223-228`).
* Both are designed around a **pluggable backend interface plus a static, prefix-routed registry**:
  * telemetry: `TelemetryBackend` / `TelemetryEntry` / `TelemetryRegistry` (push model, no update loop,
    fully thread-safe);
  * tunables: `TunableBackend` / `TunableRegistry` (poll model: `TunableRegistry::Update()` runs once per
    robot loop on the main thread, under a global recursive mutex).
* **wpimath, commandsv2 and wpilibc depend on them at compile time.** `PIDController`, `Command`,
  `CommandScheduler`, `Field2d` and others inherit `TelemetryLoggable` and/or `ComplexTunable`, and
  `units/base.hpp` provides `LogValueTo` for every unit type. So both modules must be ported in full,
  even if no backend is registered (in that case values go to the built-in discard/noop backend).
* For prosdk we keep the frontends and registries almost verbatim (about 8.9k lines of C++ including
  mocks). We add **two new backends**, `ProbotTelemetryBackend` and `ProbotTunableBackend`, about
  600–900 lines in total, modelled on the NT backends. Both sit on top of **one new probot-core service,
  a "dashboard topic store" plus WebSocket protocol extensions**: announce, value batch, remove, schema,
  client write, resync. The core also needs a library-level periodic hook that runs in every phase,
  a larger user-task stack, and a DS page that renders a topic tree and edits tunables.

---

## 2. Module inventory

### 2.1 telemetry (`$W/telemetry`)

| File (src/main/native/...) | Lines | Role |
|---|---:|---|
| include/wpi/telemetry/Telemetry.hpp + cpp/Telemetry.cpp | 334+16 | Static facade (`wpi::telemetry::Log(name, v)`, `GetTable()`, `KeepDuplicates`, `SetProperty`) on the root table `/` |
| include/wpi/telemetry/TelemetryTable.hpp + cpp/TelemetryTable.cpp | 717+397 | Hierarchical table. Type-dispatching `Log<T>()`, per-table entry cache, `.type` handling |
| include/wpi/telemetry/TelemetryLoggable.hpp + cpp | 115+10 | `TelemetryLoggable` virtual interface plus the concepts `SupportsTelemetry`, `SupportsTelemetryValue`, `SupportsTelemetryWithTypeName` (ADL `LogTo` / `LogValueTo` / `GetTelemetryTypeName`) |
| include/wpi/telemetry/TelemetryEntry.hpp | 236 | **Backend sink interface** (one per path) |
| include/wpi/telemetry/TelemetryBackend.hpp | 108 | **Backend interface** (entry factory plus schema registry) |
| include/wpi/telemetry/TelemetryRegistry.hpp + cpp | 203+349 | Global registry: longest-prefix backend routing, table cache, metadata replay, warning reporter |
| DiscardTelemetryBackend.{hpp,cpp} | 70+83 | Drops everything (`IsDiscard()==true`) |
| MultiTelemetryBackend.{hpp,cpp} | 110+258 | Fan-out to N backends |
| MockTelemetryBackend.{hpp,cpp} | 208+219 | Unit-test backend |
| detail/PathUtil.hpp | 81 | `NormalizeName`, `NormalizeTableName`, `NormalizeBackendPrefix`, `IsPathOrDescendant` |
| **Total C++** | **3,514** | |

wpiutil headers used: `StringMap`, `htrie_map`, `SmallVector`, `mutex`, `StringExtras`, `SymbolExports`,
`struct/Struct.hpp`, `protobuf/Protobuf.hpp` (which pulls in nanopb `pb.h`/`pb_encode.h`/`pb_decode.h`).
It also uses `<format>` and `<print>`: `std::print(stderr, ...)` in `DefaultReportWarning`.

### 2.2 tunables (`$W/tunables`)

| File (src/main/native/...) | Lines | Role |
|---|---:|---|
| include/wpi/tunables/Tunable.hpp | 297 | `Tunable<T, I...>` front type plus aliases `TunableBool/Int32/Int64/Float/Double/String/Raw/*Vector` |
| detail/TunableDetail.hpp | 525 | Storage classes: `TunableValueBase<T>`, `TunableStructBase`/`TunableStruct`/`TunableStructVector`, `TunableProtobufBase`/`TunableProtobuf` |
| detail/TunableMember.hpp | 436 | Pointer-to-member wrappers: `TunableMemberValueBase<T>`, `TunableMemberStructBase`, `TunableMemberProtobufBase` (published via `table.Publish("x", this, &C::m_x)`) |
| detail/TunableBase.hpp | 131 | `TunableBase`: a 32-bit `m_uid` with TYPE_FLAG/CHANGE_FLAG bits. Move-tracking through the registry |
| detail/TunableTypeValue.hpp | 43 | `enum class TunableTypeValue` (33 values: BOOLEAN..STRING_ARRAY, STRUCT, PROTOBUF, COMPLEX and MEMBER_* twins) |
| detail/TunableTypeTraits.hpp | 77 | `GetTunableTypeValue<T>()`, `TunableValueType` concept |
| ComplexTunable.{hpp,cpp} | 130+30 | `ComplexTunable : detail::TunableBase` with `PublishTunable(TunableTable&)`, `UpdateTunable()`, `GetTunableType()`, child helpers |
| CustomTunable.hpp | 49 | Custom type adapter hook (`CustomTunable<T>` / ADL `GetCustomTunable`) |
| TunableTable.{hpp,cpp} | 151+53 | Hierarchical publish API |
| Tunables.{hpp,cpp} | 154+39 | Root facade: `Add<T>()`, `AddComplex<T>()`, `Publish()`, `Remove()`, `GetTable()` |
| TunableConfig.hpp | 121 | `properties` (`wpi::util::json`), `robust`, `typeString`, `isMutable`, `onTune`, `onRemoteSet`, `parent`, `polling` (DEFAULT/ALWAYS_GET/GET_ON_CHANGE) |
| TunableRegistry.{hpp,cpp} | 379+1306 | UID registry, backend routing, complex parent/child graph, tune revision, `Update()`, update mutex |
| **TunableBackend.hpp** | 119 | **Backend interface** |
| Selectable.{hpp,cpp} | 209+41 | `Selectable<T>` (replacement for SendableChooser): children `default`, `options`, `selected` |
| MockTunableBackend.{hpp,cpp} | 557+488 | Unit-test backend (a good template for a non-NT backend) |
| detail/PathUtil.hpp | 85 | Path normalization |
| **Total C++** | **5,420** | |

wpiutil headers used: `DenseMap`, `htrie_map`, `json` (the WPILib-specific, smaller `wpi/util/json.hpp`, which
needs `<expected>`), `mutex`/`recursive_mutex`, `StringExtras`, `StringMap`, `struct/Struct.hpp`,
`protobuf/Protobuf.hpp`, `SymbolExports`, `type_name` (mock only).

RTTI: no `dynamic_cast` or `typeid` anywhere in either module. Backends downcast with `static_cast`, keyed by
`TunableTypeValue`. This is compatible with `-fno-rtti`.
Exceptions: 57 `try`/`throw` sites across both modules. They guard struct/protobuf packing and user
callbacks. That is fine with `-fexceptions`.

### 2.3 The real backends live in wpilibc

| File | Lines | Notes |
|---|---:|---|
| wpilibc/.../backend/NetworkTablesTelemetryBackend.{hpp,cpp} | 101+382 | One `GenericPublisher` per path, lazily published on first `Log*` with the NT type string. `KeepDuplicates` re-publishes with `keepDuplicates`. `SetProperty` parses JSON into topic properties. Schemas go to `inst.AddSchema` |
| wpilibc/.../backend/DataLogTelemetryBackend.{hpp,cpp} | 103+276 | Same idea over `wpi::log::DataLog`. **Not registered by default** (no caller in wpilibc/main) |
| wpilibc/.../backend/NetworkTablesTunableBackend.{hpp,cpp} | 103+1212 | See section 4.4 |

Default wiring (`RobotBase.cpp:205-228`):

```cpp
TelemetryRegistry::SetReportWarning(... reporter->ReportTelemetry(path,msg) ...);
TunableRegistry::SetReportWarning(... reporter->ReportTunable(msg) ...);
TelemetryRegistry::RegisterBackend("", make_shared<backend::NetworkTablesTelemetryBackend>(inst, "/Telemetry"));
TunableRegistry::RegisterBackend("",  make_shared<backend::NetworkTablesTunableBackend>(inst, "/Tunables"));
```

`TunableRegistry::Update()` is called once per loop in `IterativeRobotBase.cpp:158` and `OpModeRobot.cpp:167`
(with a watchdog epoch "TunableRegistry::Update()"). Telemetry has **no** update call.

Answers to "default backends?": the production default is **NetworkTables** for both, under prefixes `/Telemetry`
and `/Tunables`. DataLog is available for telemetry only and must be opted into, for example by wrapping it in
`MultiTelemetryBackend`. Discard is the fallback when nothing matches. Mocks are for tests.

---

## 3. Telemetry architecture (output-only, push)

### 3.1 Frontend flow

```
wpi::telemetry::Log("drive/pose", pose)                 // Telemetry.hpp facade
  -> TelemetryRegistry::GetTable("/").Log(name, value)
     TelemetryTable::Log<T>() compile-time dispatch (TelemetryTable.hpp:172-293):
       1. SupportsTelemetryWithTypeName<T>  -> child table; SetType(GetTelemetryTypeName) logs ".type"; LogTo(child, v)
       2. SupportsTelemetry<T>              -> child table; LogTo(child, v)            (TelemetryLoggable & ADL LogTo)
       3. SupportsTelemetryValue<T>         -> LogValueTo(table, name, v)               (units: base.hpp:238/3498)
       4. string-like                       -> entry->LogString(v, "string"|custom, 0)
       5. span-convertible (vector/array)   -> Log(name, span<const V>)
       6. StructSerializable<T,I...>        -> PublishStructSchema (once per entry) + entry->LogRaw(bytes, "struct:Name")
       7. ProtobufSerializable<T>           -> PublishProtobufSchema + entry->LogRaw(bytes, "proto:Name")
       8. integral -> LogInt64 ; floating -> LogDouble ; std::formatter -> LogString(format("{}"))
     Span overload: bool/int16/int32/int64/float/double/string/string_view arrays, uint8 -> raw,
       other integrals -> int64[], struct arrays -> LogRaw(..., "struct:Name[]"), formatter -> string[]
```

Struct takes priority over protobuf. wpimath has 41 `*Struct.hpp` and 42 proto headers, so geometry and
kinematics go out as **struct**.

`TelemetryTable` caches `EntryHandle`s (a `shared_ptr<CachedEntry>` holding entry, backend, and a
published-schema set) in `m_entriesMap`. A "reset generation" counter invalidates caches when backends are
re-registered (`TelemetryRegistry::RegisterBackend` calls `table.Reset()` on every table and
`RemoveEntry()` on rerouted paths).

Table typing: `TelemetryTable::SetType(type)` writes a `".type"` string entry once. If a later call uses a
different type, it triggers `ReportWarning("table type mismatch...")` and the value is skipped.

Metadata: `KeepDuplicates(name)` and `SetProperty(name, key, jsonValue)` are recorded in the registry
(`RecordKeepDuplicates`/`RecordProperty`). They are re-applied to new entries after a backend switch
(`ApplyEntryMetadata`). Property values are **JSON text** (for example `"\"deg\""`).

### 3.2 Backend contract (what we implement)

```cpp
class TelemetryBackend {                                 // TelemetryBackend.hpp
  virtual std::shared_ptr<TelemetryEntry> GetEntry(std::string_view path) = 0;  // path normalized, e.g. "/drive/pose"
  virtual void RemoveEntry(std::string_view path) {}
  virtual bool HasSchema(std::string_view schemaName) const = 0;
  virtual void AddSchema(std::string_view name, std::string_view type, std::span<const uint8_t>) = 0;
  virtual void AddSchema(std::string_view name, std::string_view type, std::string_view) = 0;
};
class TelemetryEntry {                                   // TelemetryEntry.hpp
  virtual bool IsDiscard() const { return false; }
  virtual void KeepDuplicates() = 0;
  virtual void SetProperty(std::string_view key, std::string_view jsonValue) = 0;
  virtual void LogBoolean(bool, int64_t ts) = 0;
  virtual void LogInt8/16/32(...)  -> default LogInt64
  virtual void LogInt64(int64_t, int64_t) = 0;  LogFloat(float,..)=0;  LogDouble(double,..)=0;
  virtual void LogString(std::string_view v, std::string_view typeString, int64_t) = 0;
  virtual void LogBooleanArray(span<const bool>,..)=0;  LogBooleanArray(span<const int>,..)=0;
  virtual void LogInt16Array/Int32Array/Int64Array/FloatArray/DoubleArray(...)=0;
  virtual void LogStringArray(span<const std::string>,..)=0;  LogStringArray(span<const std::string_view>,..)=0;
  virtual void LogRaw(span<const uint8_t> v, std::string_view typeString, int64_t) = 0;
};
```

Rules from the headers: no method may throw. Failures are reported with `TelemetryRegistry::ReportWarning(path,msg)`.
Entries **must be thread-safe** and **must copy** spans and string_views. `timestamp==0` means "now".
Schema `type` strings are `"structschema"` for struct and `"proto:FileDescriptorProto"` for protobuf, and
`AddStructSchema<T>` walks nested structs (`ForEachStructSchema`). Struct schema names look like `"struct:Pose2d"`
and the schema text is like `"Translation2d translation;Rotation2d rotation"`.

Duplicate suppression is **the backend's job**. The NT backend relies on NT's own dedup and only forwards
`keepDuplicates`.

### 3.3 Threading

From `telemetry/doc/telemetry.md` §"Thread Safety": `Log` may be called from any thread. There is no `update()`.
The registry does not serialize calls into backends, so a backend needs fine-grained locks. In prosdk, logging
normally happens on the core-1 user task, but commands or worker tasks may log from elsewhere. Our entry must
therefore be safe from both cores, which the probot-core `Mux` provides.

### 3.4 Consumers in this repo (counts are files in src/main)

* **wpimath**: 6 files include `wpi/telemetry/`. `PIDController`, `ProfiledPIDController` and
  `BangBangController` are `TelemetryLoggable` (types "PIDController", "BangBangController").
  `units/base.hpp` gives `LogValueTo` for every unit (guarded by `__has_include(<wpi/telemetry/TelemetryTable.hpp>)
  && !UNIT_LIB_DISABLE_TELEMETRY`). Every struct type is loggable through the struct path.
* **commandsv2**: 14 files. 11 `LogTo` overrides: `Command` ("Command": name, running, isParented…),
  `SubsystemBase` ("Subsystem": .hasDefault/.default/.hasCommand/.command), `CommandScheduler`
  ("Scheduler": Names, Ids), and the composition commands.
* **wpilibc**: 59 files, 27 `LogTo` overrides. Hardware: PWMMotorController "Motor Controller", Encoder
  "Quadrature Encoder", DigitalInput/Output, AnalogInput, DutyCycle(Encoder), AnalogEncoder/Potentiometer,
  EdgeCounter, Tachometer, Solenoid/DoubleSolenoid/Compressor, PowerDistribution, ADXL345, SharpIR. Drives:
  DifferentialDrive and MecanumDrive. Dashboard objects: `Field2d` (Pose2d[] struct arrays per object),
  `Mechanism2d`/`MechanismLigament2d` (dims double[], color string, angle/weight/length), and
  `Gamepad` ("HID:Gamepad", per-axis and per-button values).
* romiVendordep: 1 (`Romi Gyro`).
* Full list of telemetry/tunable type strings in use: Scheduler, Command, Subsystem, PIDController,
  BangBangController, Motor Controller, Field2d, Mechanism2d, line, Analog Input, AbsoluteEncoder, Ultrasonic,
  Tachometer, Solenoid, Double Solenoid, Selectable, Quadrature Encoder, PWM, PowerDistribution, MecanumDrive,
  DifferentialDrive, HID:Gamepad, Edge Counter, Duty Cycle, Digital Input/Output, Compressor, Accelerometer,
  3AxisAccelerometer, Romi Gyro.

Nothing logs **automatically**. User code calls `wpi::telemetry::Log("Drive", m_drive)` and similar. No
framework class calls `Telemetry::Log` itself; a grep for `telemetry::Log(` and `tunables::Publish(` in
wpilibc/commandsv2/wpimath main returns nothing.

---

## 4. Tunables architecture (bidirectional, poll)

### 4.1 Object model

* `detail::TunableBase` holds only `uint32_t m_uid`. If the tunable is unregistered, the uid encodes the
  `TunableTypeValue` with `TYPE_FLAG`. Once registered, it is a registry uid (30-bit `UID_MASK`, type in
  bits 24+). `CHANGE_FLAG` marks a local change. Move constructors and assignment call
  `TunableRegistry::MoveTunable` so the registry's raw pointer follows the object. The destructor calls
  `UnregisterTunable`. **A plain `Tunable<double> x{0.1}` does not touch the registry until published**,
  or until it is constructed with a `TunableConfig`.
* `Tunable<T>` (Tunable.hpp) picks a storage class at compile time: `TunableValueBase<T>` for the 13 plain
  types, `TunableStruct`/`TunableStructVector` for struct types, `TunableProtobuf`, or `CustomTunable`
  adapters. `units/tunable.hpp` adds `TunableUnit<...>` (stores base-unit double with property
  `unit="m/s"`) and `TunableMemberValue` for units.
* `ComplexTunable` is itself a `TunableBase` (type COMPLEX). Backends receive it as an entry that only
  carries `.type` (`GetTunableType()`). Its children are published as separate paths by
  `PublishTunable(TunableTable&)`. The registry keeps a parent/child graph so that tune revisions
  propagate to parents and `PublishChildTunable`/`RemoveChildTunable` work on every alias path.
* Member tunables: `table.Publish("p", this, &C::m_p, cfg)` makes a registry-owned
  `TunableMember*` wrapper (type `MEMBER_*`) that reads and writes the parent object's field.

### 4.2 Registry

`TunableRegistry` (static, `Instance` in TunableRegistry.cpp:38) holds the following state:
* `DenseMap<uint32_t, unique_ptr<TunableInfoImpl>> tunables` (uid → tunable*, config copy, type, tuneRevision,
  parents, children, owned member wrapper);
* `htrie_map<char, shared_ptr<TunableBackend>> backends` with longest-prefix routing, plus `backendSnapshot`;
* `recursive_mutex updateMutex`, `recursive_mutex backendsMutex`, and `mutex tunablesMutex`.

`Update()` (TunableRegistry.cpp:1205):
1. lock `updateMutex`;
2. run the optional pre-update callback;
3. `UpdateComplexTunables()` (calls each complex's `UpdateTunable()`);
4. call `backend->Update()` on every backend;
5. `FinishUpdate()` applies queued `ResetChangedAfterUpdate` uids and runs `RunAfterUpdate` callbacks
   (this is where `onTune` runs).

Local `Set()` triggers `SetTunableChanged()`, which calls `TunableRegistry::NotifyChanged(uid)`, which calls
`backend->MarkDirty(uid)` on every backend.

Helpers for backends: `GetTunable(uid) -> TunableInfo{tunable*, config*, type}`, `RecordTuneApplied(uid)`,
`ResetChangedAfterUpdate(uid)`, `RunAfterUpdate(fn)`, `ReportWarning(msg)`.

### 4.3 Backend contract (what we implement)

```cpp
class TunableBackend {                                  // TunableBackend.hpp
  struct PublishedTunable { std::string path; uint32_t uid; };
  virtual void Retire() {}
  virtual bool Publish(std::string_view path, uint32_t uid, detail::TunableBase& t,
                       const TunableConfig* config, detail::TunableTypeValue type) = 0;
  virtual void MarkDirty(uint32_t uid) = 0;
  virtual void Remove(std::string_view path) = 0;
  virtual std::vector<PublishedTunable> RemovePrefix(std::string_view prefix) = 0;
  virtual void UnregisterTunable(uint32_t uid) = 0;
  virtual void Update() = 0;
};
```

None of these methods may throw. `Publish` returns false for a duplicate path. `RemovePrefix` must return
exactly what it removed, because the registry republishes those to the new backend.

### 4.4 How the NT tunable backend works (the reference to mirror)

`NetworkTablesTunableBackend.cpp`:
* `Publish` switches on `TunableTypeValue` (lines 864–1030). It creates a `ValueTunableEntry<T>` for the 13 value
  types (NT types "boolean","int","float","double","string","raw","boolean[]","int[]","float[]","double[]","string[]").
  It creates a `StructTunableEntry` ("struct:Name", schema published), a `ProtobufTunableEntry`, or a
  `ComplexTunableEntry`; the complex entry only publishes `<path>/.type`.
* Non-robust tunables use one topic for publish and subscribe. Robust tunables use `<path>/value` (robot→dash)
  and `<path>/tune` (dash→robot), and on publish they apply a retained remote value as the initial tune.
  Properties: `config->properties` plus `"robust":true`. If the tunable is immutable, no listener is attached.
* Value access is by static_cast on the type tag:
  `GetValue<T>(info)` = `static_cast<TunableValueBase<T>*>(info.tunable)->Get()` or, for MEMBER_* types,
  `static_cast<TunableMemberValueBase<T>*>(info.tunable)->Get(info.config->parent)`. `SetValue<T>` works the
  same way. Struct and protobuf use `PackStruct`/`UnpackStruct` and `PackProtobuf`/`UnpackProtobuf` on the
  `*StructBase`/`*ProtobufBase` (and member) virtuals.
* `Update()` (lines 1102–1212):
  1. drain NT listener events. For each event: `entry->UpdateTunable(value)` → `TypeMatches` →
     `DoUpdateTunable` (SetValue) → `TunableRegistry::RecordTuneApplied(uid)` → `onRemoteSet` → force
     republish;
  2. push complex, ALWAYS_GET and dirty entries with `UpdateNetwork()`, which compares against `m_lastValue`
     and publishes if changed, then calls `ResetChangedAfterUpdate(uid)`;
  3. for applied uids, `NotifyOnTune` → `RunAfterUpdate(onTune(tunable, parent))`.
  Re-entrancy handling uses insert epochs, deferred erases and `m_updateDepth`, because `onTune` may
  publish or remove.

### 4.5 Consumers

* **wpimath** (7 files): `PIDController` is `TelemetryLoggable, ComplexTunable`, with members
  `Tunable<double> m_Kp, m_Ki, m_Kd, m_iZone` and member-published `setpoint` (ALWAYS_GET plus onTune).
  `ProfiledPIDController` and `BangBangController` follow the same pattern. `units/tunable.hpp` provides
  `TunableUnit`. **Every PIDController object therefore contains `Tunable<double>` members, so the tunables
  library is a hard compile dependency of wpimath.** CMake confirms it:
  `target_link_libraries(wpimath telemetry tunables wpiutil)`.
* **commandsv2** (4 files): `Command : TelemetryLoggable, ComplexTunable` publishes `name` (immutable,
  GET_ON_CHANGE) and `running` (mutable bool; onTune schedules or cancels, so a dashboard can **run or cancel
  commands**). `CommandScheduler` publishes `Names` (string[]), `Ids` (int64[] of pointers) and `Cancel`
  (robust int64[]; onTune cancels those commands).
* **wpilibc** (8 files): `Field2d` is ComplexTunable, with per-object `Tunable<std::vector<Pose2d>>`
  (struct array) so that dashboard edits move field objects. `RobotBase`, `IterativeRobotBase` and
  `OpModeRobot` provide wiring and the update call. The NT backend is described above.
* `Selectable<T>` (tunables) replaces `SendableChooser` and is the auto chooser. Type "Selectable",
  children `default` (string, immutable), `options` (string[], immutable), `selected` (string, robust,
  mutable).

### 4.6 Threading

Tunables are **not thread-safe**. They are accessed from the main robot thread. `Update()` holds the global
`recursive_mutex` while it runs getters, setters and callbacks. Secondary threads must take
`TunableRegistry::GetUpdateMutex()`. In prosdk, therefore, `Update()` and every `onTune` **must run on the
core-1 user task** and never on the httpd/WS task on core 0. Remote writes must be **queued** from core 0 and
**drained** inside `Update()`.

### 4.7 Type support summary

| Category | Telemetry | Tunables |
|---|---|---|
| bool, int(8/16/32/64), float, double, string | yes (ints widen to int64) | bool, int32, int64, float, double, string |
| arrays | bool, int16/32/64, float, double, string, raw | vector<bool/int32/int64/float/double/string>, raw (vector<uint8_t>) |
| struct (`wpi::util::Struct<T>`) | yes, plus struct arrays, with schema publication | yes (`TunableStruct`, `TunableStructVector`) |
| protobuf (nanopb) | yes (used only when T is not struct-serializable) | yes |
| user objects | `TelemetryLoggable` / ADL `LogTo` / `LogValueTo` / `std::formatter` | `ComplexTunable`, member pointers, `CustomTunable` |
| metadata | `.type`, properties (JSON), keepDuplicates | `.type`, `TunableConfig::properties` (json), robust, mutable, typeString |

---

## 5. Porting notes (prosdk)

1. **Port both modules verbatim**: rename `wpi::telemetry` to `prosdk::telemetry`, `wpi::tunables` to
   `prosdk::tunables`, and `wpi/...` includes to `prosdk/...`. Drop `src/main/python`, `src/main/java` and the
   `robotpy_*` files. Keep `Mock*Backend` and `DiscardTelemetryBackend`; they are tiny, and the mocks are
   needed for the host tests. Keep `MultiTelemetryBackend` as well, for DS plus a future SD-card log.
2. wpiutil prerequisites: `StringMap`, `DenseMap`, `SmallVector`, `htrie_map`, `json` (`wpi/util/json.hpp`,
   424-line header plus its .cpp; needs `<expected>`, which GCC 14 has), `mutex.hpp` (maps to
   `std::mutex`/`std::recursive_mutex`, which work on Arduino-ESP32 through pthread-on-FreeRTOS; leave
   `WPI_HAVE_PRIORITY_MUTEX` undefined), `struct/Struct.hpp`, `protobuf/Protobuf.hpp` plus nanopb.
   **Option:** to save flash, drop protobuf. Make `ProtobufSerializable` always false, or stub `Protobuf.hpp`;
   struct already covers all wpimath types that dashboards need. Telemetry and tunables both reference
   protobuf only through concepts and `if constexpr` branches, so a stub concept that is always false
   compiles those branches out. Keep it if the wpiutil port already brings nanopb.
3. `std::print(stderr, ...)` in the two `DefaultReportWarning` functions: replace with `Serial`/`printf` or a
   forward to `probot::printf`. prosdk's robot glue overrides both through `SetReportWarning` anyway.
4. `<format>` (`std::format`) is used on hot paths: `TelemetryTable` builds the path string for struct
   warnings and in `GetTable`. It works on GCC 14 newlib but costs flash. Accept it, or swap to string
   concatenation where trivial.
5. **Stack**: the user task stack is 8 KB (`STACK_USER` in `core_config.hpp`). `TelemetryTable::Log` for a
   struct uses a `SmallVector<uint8_t,128>`. `TunableRegistry::Update` plus std::function plus json dumping
   plus `std::format` makes 8 KB tight once user code and command-based code are added. The core needs to
   expose a configurable stack (section 7, item F).
6. Memory: every logged path costs a `StringMap` node in `TelemetryTable::m_entriesMap`, a registry
   `entryBackends` node, our backend entry, and the core topic slot. That is roughly 150–250 B per path plus
   the value. With 100–200 paths this is 30–50 KB of heap, which is fine on the S3 (≈320 KB free heap
   typical) but must be documented. Hard caps on the core side (section 7) prevent runaway growth from
   `Log()` called with dynamic names.

---

## 6. prosdk backend design

Two new files, plus glue:

```
src/prosdk/backend/ProbotTelemetryBackend.hpp/.cpp   (~250 lines)
src/prosdk/backend/ProbotTunableBackend.hpp/.cpp     (~450-600 lines)
src/prosdk/framework/DashboardGlue.cpp               (registration + update hook, ~60 lines)
```

Both talk only to the new probot-core C API `probot::dashboard::*` (section 7.1). Neither knows about
WebSockets.

### 6.1 Shared value encoding (core and backend agree)

`probot::dashboard::ValueType` (u8), which maps 1:1 to the NT type strings:

| code | type string | payload (little-endian) |
|---:|---|---|
| 1 | `boolean` | u8 0/1 |
| 2 | `int` | i64 |
| 3 | `float` | f32 |
| 4 | `double` | f64 |
| 5 | `string` (or a custom string type, e.g. `json`) | UTF-8 bytes |
| 6 | `raw` / `struct:X` / `struct:X[]` / `proto:X` | bytes |
| 7 | `boolean[]` | u8 × n |
| 8 | `int[]` | i64 × n |
| 9 | `float[]` | f32 × n |
| 10 | `double[]` | f64 × n |
| 11 | `string[]` | repeated { u16 len, bytes } |

The type string travels separately in the announce. For example ValueType 6 with type string
`struct:Pose2d[]` tells the page to decode with the schema.

### 6.2 `prosdk::backend::ProbotTelemetryBackend`

```cpp
namespace prosdk::backend {
class ProbotTelemetryBackend final : public prosdk::telemetry::TelemetryBackend {
 public:
  explicit ProbotTelemetryBackend(std::string_view prefix = "/Telemetry");
  std::shared_ptr<telemetry::TelemetryEntry> GetEntry(std::string_view path) override;
  void RemoveEntry(std::string_view path) override;           // Entry::MarkRemoved + dashboard::unpublish
  bool HasSchema(std::string_view name) const override;       // dashboard::hasSchema
  void AddSchema(std::string_view name, std::string_view type, std::span<const uint8_t> s) override;
  void AddSchema(std::string_view name, std::string_view type, std::string_view s) override;  // dashboard::addSchema
 private:
  class Entry;                                                // see below
  std::string m_prefix;
  prosdk::util::mutex m_mutex;
  prosdk::util::StringMap<std::shared_ptr<Entry>> m_entries;
};

class ProbotTelemetryBackend::Entry final : public telemetry::TelemetryEntry {
  // state: full path ("/Telemetry/drive/pose"), dashboard::TopicId m_id = kInvalid,
  //        ValueType m_vt, std::string m_typeString, json-text properties, bool m_keepDup,
  //        std::atomic<bool> m_removed, prosdk::util::mutex m_mutex
  bool IsDiscard() const override { return m_removed; }
  void KeepDuplicates() override;      // m_keepDup=true; if published: dashboard::setFlags(id, KEEP_DUPLICATES)
  void SetProperty(k, jsonV) override; // merge into json object; if published: dashboard::setProperties(id, dump)
  void LogBoolean(bool v, int64_t) override  { Put(VT_BOOLEAN, "boolean", &u8, 1); }
  void LogInt64(int64_t v, int64_t) override { Put(VT_INT, "int", &v, 8); }
  void LogFloat / LogDouble / LogString(v, type) / LogRaw(v, type) / Log*Array -> Put(...)
  // Put(): on first call publishes the topic:
  //   m_id = dashboard::publish(path, vt, typeString, propsJson, FLAG_TELEMETRY | (keepDup?KEEP:0));
  //   on type change -> ReportWarning(path,"type mismatch") and return (same as NT behaviour);
  //   then dashboard::set(m_id, bytes, len)  (core copies, dedups, marks dirty).
  //   string[] / bool[] (span<const int>) are re-encoded into a small stack/SmallVector buffer first.
};
}
```

Notes:
* `timestamp` is ignored. The DS shows the latest value, so a timestamp column is optional. A later version
  can forward `esp_timer_get_time()`.
* If `dashboard::publish` fails because the store is full or the value is too large, call
  `TelemetryRegistry::ReportWarning(path, "dashboard topic limit reached")` once, and from then on behave as
  discard: `IsDiscard()` returns true, so `TelemetryTable` short-circuits further logging.
* Dedup happens in the core (`memcmp` with the last value), so the user can `Log` every 20 ms loop without
  flooding the WiFi link.
* Thread safety: a per-entry `prosdk::util::mutex` around lazy publish, and the core store is protected by
  its own `Mux`.

### 6.3 `prosdk::backend::ProbotTunableBackend`

This is a simplified clone of `NetworkTablesTunableBackend`. Use `MockTunableBackend.cpp` (488 lines) for the
non-NT bookkeeping and the NT backend for the per-type entries and update ordering.

```cpp
namespace prosdk::backend {
class ProbotTunableBackend final : public prosdk::tunables::TunableBackend {
 public:
  explicit ProbotTunableBackend(std::string_view prefix = "/Tunables");
  ~ProbotTunableBackend() override;                 // Retire()
  void Retire() override;                           // unpublish all topics, stop draining writes
  bool Publish(std::string_view path, uint32_t uid, tunables::detail::TunableBase& t,
               const tunables::TunableConfig* cfg, tunables::detail::TunableTypeValue type) override;
  void MarkDirty(uint32_t uid) override;            // enqueue entries of uid into m_dirty
  void Remove(std::string_view path) override;
  std::vector<PublishedTunable> RemovePrefix(std::string_view prefix) override;
  void UnregisterTunable(uint32_t uid) override;
  void Update() override;
 private:
  class Entry;                       // base: uid, path, TopicId, UpdateKind{DIRTY,ALWAYS_GET,COMPLEX}
  template <typename T> class ValueEntry;   // 13 plain types (+MEMBER_*): GetValue<T>/SetValue<T> via static_cast
  class StructEntry;                 // TunableStructBase / TunableMemberStructBase: Pack/UnpackStruct + schemas
  class ProtobufEntry;               // optional (compile out with protobuf)
  class ComplexEntry;                // publishes "<path>/.type" string topic only
  std::string m_prefix;
  prosdk::util::mutex m_mutex;
  prosdk::util::StringMap<std::shared_ptr<Entry>> m_entries;              // path -> entry
  prosdk::util::DenseMap<uint32_t, std::vector<std::string>> m_uids;       // uid  -> paths
  prosdk::util::DenseMap<uint16_t, std::shared_ptr<Entry>> m_byTopic;     // TopicId -> entry (for writes)
  std::vector<std::shared_ptr<Entry>> m_dirty, m_alwaysGet, m_complex;
  bool m_retired = false; int m_updateDepth = 0;  // + deferred erase list (copy NT logic)
};
}
```

`Publish`:
1. Pick the entry class by `type`, the same switch as NT lines 878–985.
2. Compute the type string (honour `config->typeString`).
3. Build the properties JSON: `config->properties.dump()`, plus `"robust":true` if robust, plus
   `"mutable":false` if immutable.
4. Call `dashboard::publish(prefix+path, vt, typeString, props, FLAG_TUNABLE | (isMutable?FLAG_MUTABLE:0) | (robust?FLAG_ROBUST:0))`.
5. Push the initial value: `UpdateNetwork(force=true)`.
6. Robust tunables: if the core holds a **persisted** value for that path (optional NVS feature, section 7,
   item E), apply it as an initial tune, as NT does with retained values.

Without persistence, robust behaves the same as non-robust, because the core always echoes the robot's
value back after a write, so the DS sees the accepted value.

`Update()` runs on core 1, inside `TunableRegistry::Update()`:
```
1. writes = dashboard::drainWrites(buf)            // FIFO filled by WS task on core 0
   for each {topicId, vt, bytes}:
     entry = m_byTopic[topicId]; info = TunableRegistry::GetTunable(entry->uid)
     if (!entry || !info || vt mismatch) -> dashboard::rejectWrite(topicId) ; continue
     if (info.config && !info.config->isMutable) -> reject ; continue
     entry->Apply(info, bytes)   // SetValue<T>/UnpackStruct ; on decode failure ReportWarning + reject
     TunableRegistry::RecordTuneApplied(uid); onRemoteSet(info); entry->forceRepublish = true
     appliedUids.push_back(uid)
2. for e in m_complex, m_alwaysGet, swap(m_dirty): e->UpdateNetwork(false)
     // GetValue<T> -> compare with m_lastValue -> dashboard::set(topicId, bytes) -> ResetChangedAfterUpdate(uid)
3. for uid in appliedUids: NotifyOnTune(uid)  -> ResetChangedAfterUpdate + RunAfterUpdate(onTune(...))
```
Copy the NT re-entrancy protections: insert epoch, deferred erase, and `m_updateDepth`. `onTune` can
publish or remove tunables, as `Command` and `CommandScheduler` do.

Selectable needs no special code. It is a ComplexTunable with three plain children, and the DS recognizes
`.type == "Selectable"` and renders a dropdown that writes `selected`.

### 6.4 Glue in the prosdk robot framework

```cpp
// prosdk/framework/DashboardGlue.cpp  (called once from prosdk's RobotBase/OpModeRobot init)
void prosdk::detail::InstallDashboardBackends() {
  telemetry::TelemetryRegistry::SetReportWarning([](auto path, auto msg){ RateLimitedWarn("T", path, msg); });
  tunables::TunableRegistry::SetReportWarning([](auto msg){ RateLimitedWarn("U", "", msg); });
  telemetry::TelemetryRegistry::RegisterBackend("", std::make_shared<backend::ProbotTelemetryBackend>("/Telemetry"));
  tunables::TunableRegistry::RegisterBackend("",  std::make_shared<backend::ProbotTunableBackend>("/Tunables"));
  probot::runtime::addPeriodicService(&prosdk::detail::DashboardPeriodic);   // new core API (7, item C)
}
static void prosdk::detail::DashboardPeriodic() { tunables::TunableRegistry::Update(); }
```

`RateLimitedWarn` forwards to `probot::printf` (the existing text console) and to a `/Warnings` string topic.
It is rate-limited so that a type-mismatch storm cannot flood the 256-B console.

`TunableRegistry::Update()` must also run while the robot is **STOPPED**. The auto chooser `Selectable`,
PID gains and "run command" buttons all need to work before INIT. That is why item C (section 7) is
mandatory: probot-core 0.4.0 calls user code only in the INIT and RUN stages (`lifecycle.hpp: step()`).

Host tests: register `MockTelemetryBackend`/`MockTunableBackend` exactly as upstream tests do. For the
backends themselves, compile probot-core's `probot::dashboard` store on host. It is header-only and uses the
host `Mux` fallback, so tests can assert announce/value/write flows without hardware.

---

## 7. What probot-core must add

probot-core today has only `probot::telemetry::{print,println,printf,clear}`, a 256-byte ring buffer pushed as
`'T'` frames every 250 ms (`driver_station_esp32.hpp: pushTaskLoop`). There is no structured data channel, no
inbound data except joystick frames, and `WsJoystick::wsHandler` **closes the connection on any binary frame
larger than `MAX_FRAME` (4+40+3 = 47 bytes)** (`ws_joystick.hpp:223`). The following features are needed.

### A. `probot::dashboard` topic store (new header `src/probot/dashboard/dashboard.hpp`)

This is thread-safe and header-only, like the rest of core. The writer is the user task on core 1 (or any
task). The readers are the push task and the WS handler on core 0.

```cpp
namespace probot::dashboard {
  using TopicId = uint16_t;  constexpr TopicId kInvalid = 0xFFFF;
  enum class ValueType : uint8_t { Boolean=1, Int=2, Float=3, Double=4, String=5, Raw=6,
                                   BooleanArray=7, IntArray=8, FloatArray=9, DoubleArray=10, StringArray=11 };
  enum Flags : uint8_t { TELEMETRY=1, TUNABLE=2, MUTABLE=4, ROBUST=8, KEEP_DUPLICATES=16, PERSISTENT=32 };

  // Idempotent per path: same path+type returns the same id; different type -> kInvalid.
  TopicId publish(const char* path, ValueType vt, const char* typeString,
                  const char* propertiesJson /*nullable*/, uint8_t flags);
  void    unpublish(TopicId id);
  void    setProperties(TopicId id, const char* propertiesJson);
  void    setFlags(TopicId id, uint8_t flags);
  // Copies value; drops if identical to last value unless KEEP_DUPLICATES; marks dirty.
  bool    set(TopicId id, const void* data, size_t len);
  bool    hasSchema(const char* name);
  void    addSchema(const char* name, const char* type, const void* data, size_t len);

  // Inbound tunable writes (filled by the WS task, drained on the user task)
  struct Write { TopicId id; ValueType vt; uint16_t len; const uint8_t* data; };
  // Calls fn for each queued write in FIFO order; returns count. Storage is valid only during fn.
  size_t  drainWrites(void (*fn)(const Write&, void* ctx), void* ctx);
  void    rejectWrite(TopicId id, uint8_t reason);  // optional: sends 'K' NACK to DS

  // stats for /health
  struct Stats { uint16_t topics, maxTopics; uint32_t bytesUsed, bytesMax, dropped; };
  Stats   stats();
}
```

Implementation constraints:
* **Allocation outside `Mux`.** `portENTER_CRITICAL` sections must only `memcpy` and swap pointers, as the
  `lock.hpp` comment says. Use a fixed slot table of `PROBOT_DASH_MAX_TOPICS` (default 128) entries:
  name, type string, props, value pointer and capacity, `seq`, `dirty`, `announced`. Value buffers are
  (re)allocated before taking the lock.
* Limits as macros: `PROBOT_DASH_MAX_TOPICS` (128), `PROBOT_DASH_MAX_VALUE` (1024 B, which covers a Pose2d[]
  of about 40 poses), `PROBOT_DASH_POOL_BYTES` (32 KB total), `PROBOT_DASH_WRITE_QUEUE` (2 KB byte ring).
  When a limit is exceeded, `publish`/`set` return failure. The backend then reports it once and goes to
  discard.
* The write queue is a lock-protected byte ring of `{id, vt, len, bytes}` records. When it overflows, drop
  the **oldest** record for the same id (last-writer-wins). If that is not possible, drop the new record and
  count it.

### B. WebSocket protocol extensions (`/joystick` socket, same owner model)

Server→client (push task). Add these to the existing `'S'` and `'T'` frames:

| Byte | Name | Payload (LE) |
|---|---|---|
| `'A'` 0x41 | announce | repeated { u16 id, u8 vt, u8 flags, u8 nameLen, name, u8 typeLen, type, u16 propsLen, propsJson } |
| `'V'` 0x56 | value batch | u32 serverMs, repeated { u16 id, u16 len, bytes } (only dirty topics; split across frames at ~1400 B) |
| `'R'` 0x52 | remove | repeated u16 id |
| `'C'` 0x43 | schema | u8 nameLen, name, u8 typeLen, type, u16 len, bytes (e.g. `struct:Pose2d`, `structschema`, `"Translation2d translation;Rotation2d rotation"`) |
| `'K'` 0x4B | write ack/nack | u16 id, u8 status (0 ok, 1 unknown id, 2 immutable, 3 type mismatch, 4 decode error, 5 queue full) |

Client→server (WS handler):

| Byte | Name | Payload |
|---|---|---|
| `'W'` 0x57 | tune write | u16 id, u8 vt, bytes (≤ `PROBOT_DASH_MAX_VALUE`) |
| `'Q'` 0x51 | resync request | none. On receipt the server re-sends all `'C'`, `'A'` and full `'V'` frames. The page sends it on every (re)connect |

Required code changes in core:
* `WsJoystick::wsHandler`: dispatch on the first byte **before** the size check. Keep the 47-byte cap for
  `'J'`/`'P'` only. Allow `'W'` up to `3 + PROBOT_DASH_MAX_VALUE` and `'Q'` at 1 byte. Reject `'W'` from
  non-owners; the existing `wsPeerFilter` and owner authorizer already gate this. `'W'` must **only enqueue**
  (`dashboard::detail::pushWrite`). User code must never run on core 0.
* `DriverStation::pushTaskLoop`: add a dashboard tick. Run it at **50 ms** (`PROBOT_DASH_PERIOD_MS`, default
  50 = 20 Hz), separate from the 250 ms state tick, because 250 ms is too coarse for plots. Send pending
  schemas, announces and removes first, then the dirty values as batched `'V'`. Bound the bytes per tick
  (`PROBOT_DASH_MAX_BYTES_PER_TICK`, about 8 KB) so `httpd_ws_send_frame_async` cannot starve `'S'`
  heartbeats. Raise the `ds_push` stack from 4096 if needed, since frame building uses a static or
  heap buffer rather than a large stack array.
* Resync on `'Q'`: set an "announce all" flag that the push task consumes.
* HTTP fallbacks, for when WS is down and for debugging:
  * `GET /dash` returns a JSON snapshot `{topics:[{id,path,type,flags,props,value}]}`, with base64 for raw
    values;
  * `GET /dashSet?id=N&v=<json>` (owner only) enqueues a write.
* `/health` and `'S'`: add `dashTopics`, `dashBytes` and `dashDropped`.

### C. A periodic service hook that runs in every phase (mandatory for tunables)

`probot::runtime::addPeriodicService(void (*fn)())` (max 4) registers a function that `userTask` calls once
per iteration **after** `g_machine.step(...)`. It runs in every stage (STOPPED, TRANSITION, INIT, RUN) on core 1,
under the same loop-deadline heartbeat. When e-stop is latched it is not called. prosdk uses it for
`TunableRegistry::Update()`, and later for `CommandScheduler::Run()` in disabled mode and simulation physics
`Update`. This replaces WPILib's `robotPeriodic`/`disabledPeriodic` without bringing back the removed
`robotInit` user hook (PB-E106).
Also add `probot::runtime::addInitService(void (*fn)())`, run once on the user task before the first `step`.
prosdk uses it to install backends, because `setup()` belongs to core.

### D. DS page (index_html.h) — new "Data" / "Tunables" widgets

* Topic tree, keyed on `/` segments. It shows `/Telemetry/...` (read-only) and `/Tunables/...` (editable),
  with a live value display for each ValueType.
* Editors per type: checkbox (boolean), number input or slider (int/float/double; use `min`/`max`/`step`
  properties and show the `unit` property), text input (string), comma list (arrays). Send `'W'` on
  commit (Enter or blur, not on every keystroke) and show the pending state until the echoed `'V'` or
  `'K'` arrives.
* Generic struct decoder in JS: parse `structschema` text (`type name[;...]`, `type name[N]`, nested
  structs, `enum{}`, bitfields are optional) and decode `struct:X` / `struct:X[]` raw values to fields.
  This is about 150 lines of JS and covers all 41 wpimath struct types without per-type code.
* Typed widgets selected by the sibling `.type` topic: `Selectable` (dropdown writing `selected`),
  `Command` (button toggling `running`), `PIDController` (gain editors), `Field2d` (canvas drawing
  Pose2d[] per object), `Mechanism2d` (canvas). Everything else falls back to the generic tree. Phase it:
  tree plus editors plus Selectable plus Command first, Field2d and Mechanism2d later.
* Optional: simple line plot for numeric topics (ring buffer in the browser).
* Page size: index_html.h is already 2,203 lines and is served chunked. The added JS should be minified,
  or served gzipped as a second asset `/dash.js` if the total grows a lot.

### E. Optional: persistence of tunables (`PERSISTENT` flag)

Store the last DS-written value of a tunable in NVS (`Preferences` namespace "probot-dash", keyed by a
hash of the path). On `publish` with `ROBUST|PERSISTENT`, expose it as an "initial remote value" that the
backend applies as the first tune (matching NT's robust retained-value semantics). Debounce flash writes
to at most one every few seconds. Not required for v1.

### F. Configurable user-task stack

Make `STACK_USER` (core_config.hpp: 8192) overridable (`#ifndef PROBOT_USER_STACK`). prosdk will
recommend 16–24 KB once telemetry, tunables and commands run in the user task.

### G. Keep `probot::printf` and `'T'`

Text telemetry stays as the console. prosdk routes the telemetry and tunable warning reporters there.

---

## 8. Minimal v1 scope (recommendation)

1. Port `telemetry` and `tunables` verbatim (namespace rename). Stub protobuf if nanopb is not in the wpiutil
   port. Keep the mocks and the upstream unit tests (`TelemetryTableTest.cpp`, `TunableTest.cpp`,
   `SelectableTest.cpp`) as host tests.
2. probot-core: items A, B (`'A'`, `'V'`, `'R'`, `'C'`, `'W'`, `'Q'`; `'K'` optional), C, D (tree,
   editors, struct decoder, Selectable, Command), F. Items E and the typed canvases come later.
3. prosdk: `ProbotTelemetryBackend` + `ProbotTunableBackend` + glue (`InstallDashboardBackends` via
   `addInitService`, `TunableRegistry::Update` via `addPeriodicService`).
4. Do **not** port `NetworkTablesTelemetryBackend`, `NetworkTablesTunableBackend` or ntcore; that is out of
   scope on this MCU. `DataLogTelemetryBackend` can come later as an SD/LittleFS log behind
   `MultiTelemetryBackend` if datalog is ported.

Estimated new code: prosdk backends about 700–900 lines. probot-core store plus protocol about 500–700 lines of
C++. DS JS about 500–800 lines.
