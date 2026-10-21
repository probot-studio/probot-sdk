# commandsv2 (C++) porting report — allwpilib b68effca -> prosdk (ESP32-S3)

Root: `/tmp/claude-0/-srv-probot-sdk/8fcba988-962f-4d98-ac5d-540d36df924f/scratchpad/wpi/allwpilib/commandsv2`
(all paths below are relative to that root unless absolute).

## 0. Headline facts

- **Namespace is `wpi::cmd`** (not `frc2`), headers under `wpi/commands2/...`. Port target: `prosdk::cmd`, `prosdk/commands2/...`.
- Size: hand-written C++ = 31 `.cpp` (2,935 lines) + 37 `.hpp` (4,935 lines); generated controller wrappers = 15 `.cpp` (2,666 lines) + 15 `.hpp` (5,759 lines). 36 Catch2 test files (~170 TEST_CASE/SECTION), useful as a host-side regression suite.
- Build: `CMakeLists.txt` globs `src/main/native/cpp/*.cpp` + `src/generated/main/native/cpp/*.cpp`, links only `wpilibc` (which pulls hal, ntcore, wpimath, wpiutil, telemetry, tunables, datalog transitively). C++23 (`cxx_std_23`).
- **commandsv3 is Java-only — confirmed.** `commandsv3/build.gradle`: `useJava = true`, `useCpp = false`. 83 `.java` files, zero C/C++ sources; non-Java files are only build scripts, `CommandsV3.json`, Java jinja templates (`src/generate/main/java/*.java.jinja`) and `src/main/proto/protobuf_commands.proto`. It is built on JDK continuations (`Continuation.java`, `ContinuationScope.java`, `Coroutine.java`) -> no C++ counterpart to port. Top-level `CMakeLists.txt` only adds `commandsv2` (line 375). Skip v3.
- **No `typeid`/`dynamic_cast` written directly in commandsv2**, BUT `wpi::util::GetTypeName()` (wpiutil `Demangle.hpp`: `Demangle(typeid(type).name())`) is called in 3 places -> hard compile error under `-fno-rtti`. Must be replaced.
- **Exceptions: 12 `throw WPILIB_MakeError(wpi::err::CommandIllegalUse, ...)` sites** (misuse detection only; never on the normal path). Works with `-fexceptions`.
- **Threads: none created by commandsv2 itself.** Indirect: `wpi::Watchdog` (wpilibc, global background thread + HAL notifier), `wpi::Notifier` (used by `NotifierCommand`, thread + HAL notifier), one `std::mutex` in `CommandGenericHID.cpp`.
- **The scheduler is not thread-safe**: everything (Run, Schedule, Trigger binding, Cancel) must happen on one task — the probot-core user task (core 1).

## 1. Class inventory

### 1.1 Core (`src/main/native/include/wpi/commands2/`, impl in `src/main/native/cpp/`)

| Class / file | Base | Notes / deps |
|---|---|---|
| `Command` (`Command.hpp/.cpp`) | `wpi::telemetry::TelemetryLoggable`, `wpi::tunables::ComplexTunable` | Virtuals `Initialize/Execute/End(bool)/IsFinished/GetRequirements/RunsWhenDisabled/GetInterruptionBehavior`, pure `ToPtr() &&`. Rvalue decorators (`WithTimeout, Until, OnlyWhile, BeforeStarting, AndThen, Repeatedly, AsProxy, Unless, OnlyIf, WithDeadline, DeadlineFor, AlongWith, RaceWith, IgnoringDisable, WithInterruptBehavior, FinallyDo x2, HandleInterrupt, WithName`) all forward to `CommandPtr`. `enum class InterruptionBehavior {CANCEL_SELF, CANCEL_INCOMING}`. Ctor: `m_name{wpi::util::GetTypeName(*this)}` (**RTTI**). Dtor calls `CommandScheduler::GetInstance().Cancel(this)`. `SetComposed(true)` stores `wpi::util::GetStackTrace(1)`. `LogTo` logs name/running/isParented/interruptBehavior/runsWhenDisabled. `PublishTunable`: "name" (read-only) and "running" (writable; onTune schedules/cancels). Free fn `RequirementsDisjoint(Command*, Command*)`. Members: `SmallSet<Subsystem*,4> m_requirements`, `std::string m_name`, `std::optional<std::string> m_previousComposition`. |
| `CommandHelper<Base, CRTP>` (`CommandHelper.hpp`) | `Base` | CRTP providing `ToPtr() &&` via `std::make_unique<CRTP>(std::move(*static_cast<CRTP*>(this)))`. No RTTI. **Ideal hook to inject a compile-time type name** (see §4). |
| `CommandPtr` (`CommandPtr.hpp/.cpp`) | — | Owning `unique_ptr<Command>` wrapper. All decorators (§above) + `Unwrap() &&`, `Cancel() const&`, `IsScheduled() const&`, `HasRequirement`, `GetName`, `UnwrapVector`. `AssertValid()` throws on moved-from use. Move ctor stores `rhs.m_moveOutSite = wpi::util::GetStackTrace(1)` **on every move** (string alloc + stack walk — expensive on MCU; stub). Private helper classes in .cpp: `RunsWhenDisabledCommand`, `InterruptBehaviorCommand`, `FinallyCommand` (all `: WrapperCommand`). |
| `CommandScheduler` (`CommandScheduler.hpp/.cpp`) | `TelemetryLoggable`, `ComplexTunable`, `final` | Singleton `GetInstance()` (function-local static). pImpl `Impl`: `SmallSet<Command*,12> scheduledCommands`, `DenseMap<Subsystem*,Command*> requirements`, `DenseMap<Subsystem*,unique_ptr<Command>> subsystems` (also default cmds), `wpi::EventLoop defaultButtonLoop` + `activeButtonLoop*`, `bool disabled`, action vectors (`initActions/executeActions/interruptActions/finishActions`), `DenseMap<Command*,CommandPtr> ownedCommands`. Member `wpi::Watchdog m_watchdog` (timeout `TimedRobot::DEFAULT_PERIOD` = 20 ms, callback `std::puts("CommandScheduler loop time overrun.")`). API: `SetPeriod`, `Get/SetActiveButtonLoop`, `GetDefaultButtonLoop`, `Schedule(Command*/span/init_list/const CommandPtr&/CommandPtr&&)`, `Run`, `Register/UnregisterSubsystem(...)`, `UnregisterAllSubsystems`, `SetDefaultCommand` (template + CommandPtr&&), `RemoveDefaultCommand`, `GetDefaultCommand`, `Cancel(...)`, `CancelAll`, `IsScheduled(...)`, `Requiring`, `Disable/Enable`, `PrintWatchdogEpochs`, `OnCommandInitialize/Execute/Interrupt(x2)/Finish`, `RequireUngrouped(...)`, `RequireUngroupedAndUnscheduled(...)`, `LogTo` ("Names", "Ids"), `PublishTunable` ("Names", "Ids" read-only; "Cancel" writable int64 list of command pointer ids). Ctor calls `wpi::util::ReportUsage("CommandScheduler","")`. |
| `Subsystem` (`Subsystem.hpp/.cpp`) | — (pure interface w/ virtual dtor) | `Periodic`, `SimulationPeriodic`, `GetName()` -> `wpi::util::GetTypeName(*this)` (**RTTI**, real dynamic type here), `SetDefaultCommand` (template + CommandPtr&&), `RemoveDefaultCommand`, `GetDefaultCommand`, `GetCurrentCommand`, `Register`, factories `Idle, RunOnce, Run, StartEnd, RunEnd, StartRun, Defer`. Dtor unregisters. |
| `SubsystemBase` (`SubsystemBase.hpp/.cpp`) | `Subsystem`, `TelemetryLoggable` | Ctor auto-registers with scheduler. Default ctor `m_name{GetTypeName(*this)}` (**RTTI**; note: evaluated in base ctor so upstream yields "wpi::cmd::SubsystemBase", not the derived name). `SetName`. `LogTo`: `.hasDefault/.default/.hasCommand/.command`. |
| `Requirements` | — | Implicit from `initializer_list<Subsystem*>` / `span`; stores `std::vector`. |
| `Commands.hpp/.cpp` (namespace `wpi::cmd`, called as `cmd::X`) | free functions | `None, Idle, RunOnce, Run, StartEnd, RunEnd, StartRun, Print, Wait, WaitUntil, Either, Select<Key>, Defer, DeferredProxy (x2), Sequence (vector + variadic), RepeatingSequence, Parallel, Race, Deadline`, `impl::MakeVector`. |
| `FunctionalCommand` | `CommandHelper<Command,…>` | 4 std::functions. |
| `InstantCommand` | `CommandHelper<FunctionalCommand,…>` | |
| `RunCommand` | `CommandHelper<FunctionalCommand,…>` | |
| `StartEndCommand` | `CommandHelper<FunctionalCommand,…>` | |
| `PrintCommand` | `CommandHelper<InstantCommand,…>` | `wpi::util::print("{}\n")` -> route to Serial/log. RunsWhenDisabled = true. |
| `WaitCommand` | `CommandHelper<Command,…>` | `wpi::Timer m_timer`; `SetName(std::format("{}: {}", GetName(), duration))` (needs units `std::formatter`). RunsWhenDisabled = true. LogTo "duration". |
| `WaitUntilCommand` | `CommandHelper<Command,…>` | Ctor(time) uses `wpi::Timer::GetMatchTime()` (-> `wpi::MatchState::GetMatchTime()`). RunsWhenDisabled = true. |
| `NotifierCommand` | `CommandHelper<Command,…>` | Owns `wpi::Notifier` (separate thread!). `Initialize` -> `StartPeriodic`, `End` -> `Stop`. |
| `ConditionalCommand` | `CommandHelper<Command,…>` | LogTo onTrue/onFalse/selected. |
| `SelectCommand<Key>` (header-only template) | `CommandHelper<Command, SelectCommand<Key>>` | `std::unordered_map<Key, unique_ptr<Command>>`; LogTo "selected". |
| `DeferredCommand` | `CommandHelper<Command,…>` | `unique_function<CommandPtr()>`; LogTo "deferred". |
| `ProxyCommand` | `CommandHelper<Command,…>` | Schedules the inner command on the scheduler; name `Proxy(...)` via std::format. |
| `ScheduleCommand` | `CommandHelper<Command,…>` | Fork-style scheduling. |
| `RepeatCommand` | `CommandHelper<Command,…>` | Name `Repeat(...)`. |
| `WrapperCommand` | `CommandHelper<Command,…>` | Base for decorator wrappers. |
| `SequentialCommandGroup`, `ParallelCommandGroup`, `ParallelRaceGroup`, `ParallelDeadlineGroup` | `CommandHelper<Command,…>` | `AddCommands` throws if running / if parallel requirements overlap. |

### 1.2 Buttons / HID (`include/wpi/commands2/button/`, `cpp/button/`)

| Class | Notes / deps |
|---|---|
| `Trigger` | Wraps `std::function<bool()>` + `wpi::EventLoop*` (defaults to scheduler's default loop). Bindings: `OnChange, OnTrue, IfTrue, OnFalse, WhileTrue, WhileFalse, ToggleOnTrue, ToggleOnFalse` (each `Command*` and `CommandPtr&&`), operators `&& || !`, `Debounce(time, wpi::math::Debouncer::DebounceType)`, `MultiPress(n, window)` (uses `wpi::math::EdgeCounterFilter`), `Get()`. `AddBinding` -> `EventLoop::Bind(unique_function)`. |
| `JoystickButton`, `POVButton` (header-only) | `: Trigger`; take `wpi::GenericHID*` or `wpi::HIDDevice*`; `wpi::POVDirection`. |
| `GamepadButton` (header-only) | `: Trigger`; `wpi::Gamepad*`, `wpi::Gamepad::Button`. |
| `NetworkButton` | `: Trigger`; **ntcore**: `wpi::nt::BooleanTopic/BooleanSubscriber/NetworkTable/NetworkTableInstance`; condition = `IsConnected() && sub->Get()`. |
| `RobotModeTriggers` (static) | `Autonomous()` -> `RobotState::IsAutonomousEnabled`, `Teleop()` -> `IsTeleopEnabled`, `Disabled()` -> `IsDisabled`, `Utility()` -> `IsUtilityEnabled`. (No "Test" in 2027; it is "Utility".) |
| `CommandGenericHID` (**final**, not a base class anymore) | Wraps `wpi::GenericHID*` from `wpi::DriverStation::GetGenericHID(port)`. `GetCommandGenericHID(port)` caches per port in a static `std::array<unique_ptr<CommandGenericHID>, DriverStationBackend::JOYSTICK_PORTS>` guarded by `std::mutex hidsMutex`. Triggers: `Button, POV (x2), POVUp/UpRight/Right/DownRight/Down/DownLeft/Left/UpLeft/Center, AxisLessThan, AxisGreaterThan, AxisMagnitudeGreaterThan`; `SetRumble(GenericHID::RumbleType, double)`, `IsConnected()`. |
| `CommandGamepad` | Composes `CommandGenericHID*` + `wpi::Gamepad`; `Button(int / Gamepad::Button)`, `FaceDown/Right/Left/Up, Back, Guide, Start, LeftStick, RightStick, LeftBumper, RightBumper, DpadUp/Down/Left/Right, Misc1..`, paddles, touchpad, trigger axes etc. (197-line .cpp). |
| `CommandJoystick` | Composes `CommandGenericHID*` + `wpi::Joystick`; `Trigger(...)` and `Top(...)`. |
| Generated (15): `CommandXboxController`, `CommandDualShock4Controller`, `CommandDualSenseController`, `CommandDualSenseEdgeController`, `CommandLogitechF310Controller`, `CommandSteamController`, `CommandGameCubeController`, `CommandSwitchProController`, `CommandSwitch2ProController`, `CommandSwitch2GCController`, `CommandSwitchN64Controller` (template `commandhid.*.jinja`, schema `//wpilibj:hid_schema`), and `CommandNiDsXboxController`, `CommandNiDsPS4Controller`, `CommandNiDsPS5Controller`, `CommandNiDsStadiaController` (template `first_ds_commandhid.*.jinja`). | Each composes `CommandGenericHID*` + the matching wpilibc `wpi/driverstation/<X>Controller.hpp` class; trigger-axis methods take `double threshold = 0.5`. There is **no `CommandPS4Controller`/`CommandPS5Controller`** any more: PS4 = `CommandDualShock4Controller` / `CommandNiDsPS4Controller`, PS5 = `CommandDualSenseController` / `CommandNiDsPS5Controller`. Generator: `generate_hids.py` (jinja2). Recommendation: copy the pre-generated files (sed-rename), don't run the generator on the device build; keep only the controllers the browser DS / probot-core mapping can actually present (`probot/io/joystick_mappings`). |

### 1.3 SysId (`include/wpi/commands2/sysid/SysIdRoutine.hpp`, `cpp/sysid/SysIdRoutine.cpp`)

`wpi::cmd::sysid::Config` (rampRate, stepVoltage, timeout, `recordState` callback), `Mechanism` (`drive(volt_t)`, `log(SysIdRoutineLog*)`, `Subsystem*`, name), `SysIdRoutine : wpi::sysid::SysIdRoutineLog` with `Quasistatic(Direction)` / `Dynamic(Direction)` returning `CommandPtr`; uses `wpi::Timer`. Base `wpi::sysid::SysIdRoutineLog` lives in **wpilibc** (`wpilibc/src/main/native/include/wpi/sysid/SysIdRoutineLog.hpp`) and depends on `wpi/datalog/DataLog.hpp` + `wpi/system/DataLogManager.hpp` (.cpp). Portable if prosdk provides a DataLog backend (SD/LittleFS file or streamed over WebSocket to the browser DS); otherwise make `recordState`/log go to telemetry.

## 2. External dependencies (by module)

### wpilibc
| Header | Used by | ESP32 action |
|---|---|---|
| `wpi/driverstation/RobotState.hpp` (`IsDisabled`, `IsAutonomousEnabled`, `IsTeleopEnabled`, `IsUtilityEnabled`) -> `internal::DriverStationBackend` | `CommandScheduler.cpp`, `RobotModeTriggers.cpp` | Back with probot-core `probot::robot::state()` phase (see §5). |
| `wpi/driverstation/DriverStation.hpp` (`GetGenericHID(port)`), `internal/DriverStationBackend.hpp` (`JOYSTICK_PORTS`) | `CommandGenericHID` | Back with probot-core WebSocket joystick (`probot/io/gamepad.hpp`, `joystick_api.hpp`). |
| `wpi/driverstation/GenericHID.hpp`, `HIDDevice.hpp`, `POVDirection.hpp`, `Gamepad.hpp`, `Joystick.hpp`, 15 controller headers (`XboxController.hpp` … `NiDsStadiaController.hpp`) | button/* | Port with the HID work package. |
| `wpi/framework/RobotBase.hpp` (`constexpr IsSimulation()`) | `CommandScheduler::Run` (`if constexpr` → `SimulationPeriodic`) | constexpr false on target; true on host-sim build. |
| `wpi/framework/TimedRobot.hpp` (`DEFAULT_PERIOD` = 20 ms) | scheduler ctor watchdog timeout | Match probot-core `USER_LOOP_PERIOD_MS` = 20. Avoid pulling TimedRobot in just for a constant. |
| `wpi/system/Watchdog.hpp` | scheduler | Upstream impl = global `std::thread` + `HAL_CreateNotifier`. Scheduler only uses `Reset/AddEpoch/Disable/IsExpired/PrintEpochs/SetTimeout`; provide a **synchronous, threadless** Watchdog (timestamp compare in `Disable()`/`IsExpired()`, fire callback lazily). |
| `wpi/system/Timer.hpp` (`Timer`, `GetMatchTime`) | `WaitCommand`, `WaitUntilCommand`, `SysIdRoutine` | Timer on `esp_timer_get_time()`; `GetMatchTime` from probot-core `autoPeriodSeconds/autoStartMs`. |
| `wpi/system/Notifier.hpp` | `NotifierCommand` | Map to `esp_timer` periodic callback or a FreeRTOS task; callback runs concurrently with scheduler (same as upstream). |
| `wpi/system/Errors.hpp` (`WPILIB_MakeError`, `wpi::err::CommandIllegalUse`, `wpi::RuntimeError : std::runtime_error`) | 12 throw sites | Keep (exceptions on) or macro to `log + abort()`. |
| `wpi/event/EventLoop.hpp` (`Bind`, `Poll`, `Clear`; throws if Bind/Clear while polling), `wpi/event/BooleanEvent.hpp` | scheduler, Trigger | Pure C++ — port as-is. |
| `wpi/sysid/SysIdRoutineLog.hpp` (+ DataLog) | sysid | See §1.3. |

### wpiutil
`SmallSet`, `SmallVector`, `DenseMap`, `FunctionExtras.hpp` (`unique_function`), `DecayedDerivedFrom.hpp`, `Demangle.hpp` (**typeid**), `StackTrace.hpp` (`GetStackTrace`; `cpp/StackTrace.cpp` includes `<execinfo.h>`/`backtrace()` — not in newlib → stub returning "" or esp_backtrace-based), `UsageReporting.hpp` (`ReportUsage` → no-op), `print.hpp` (`wpi::util::print` → Serial/printf), `mutex.hpp` (via telemetry).

### wpimath
`wpi/math/filter/Debouncer.hpp` (Trigger::Debounce), `wpi/math/filter/EdgeCounterFilter.hpp` (Trigger::MultiPress) — both timestamp-based (MathShared timestamp → esp_timer). `wpi/units/time.hpp`, `voltage` etc. (units library, header-only; `std::format` support needed for WaitCommand name).

### ntcore
Only `NetworkButton` (`BooleanTopic`, `NetworkTable`, `NetworkTableInstance`). No NT on ESP32 → either drop `NetworkButton` or re-implement it as a "dashboard button" over the prosdk tunables backend (a writable bool tunable from the browser DS). Recommend the latter, keeping the class name and the `(table, field)` ctor.

### telemetry / tunables (new 2027 modules, replace Sendable/SmartDashboard)
- `TelemetryLoggable` (`LogTo(TelemetryTable&)`, `GetTelemetryType()`): `Command`, `CommandScheduler`, `SubsystemBase`, overrides in `ConditionalCommand, DeferredCommand, ParallelDeadlineGroup, ProxyCommand, RepeatCommand, SequentialCommandGroup, SelectCommand, WaitCommand` (11 implementing files).
- `ComplexTunable` (`PublishTunable(TunableTable&)`, `UpdateTunable()`, `GetTunableType()`, `SetChildTunableChanged`), `TunableConfig` (`isMutable`, `parent`, `polling = ALWAYS_GET/GET_ON_CHANGE`, `robust`, `onTune`): `Command` (name/running), `CommandScheduler` (Names/Ids/Cancel).
- **No Sendable / SmartDashboard / Shuffleboard usage** in 2027 commandsv2 — fully replaced by telemetry+tunables. Nothing auto-publishes: the user calls e.g. `Telemetry::Log("Scheduler", CommandScheduler::GetInstance())`; tunables are serviced by `TunableRegistry::Update()` (called by `IterativeRobotBase::LoopFunc` after `RobotPeriodic`).
- telemetry has internal `wpi::util::mutex` (fine on FreeRTOS). No typeid in telemetry/tunables sources (grep).
- Scheduler "Cancel" tunable and `GetScheduledCommandIds` reinterpret `Command*` as `int64_t` (32-bit pointers on Xtensa — fine, widened).

## 3. RTTI (`-fno-rtti`)

Direct grep for `typeid|dynamic_cast|dynamic_pointer_cast`: **0 hits** in commandsv2 C++. All `static_cast` uses (`Command.cpp:216`, `CommandScheduler.cpp:528`, `CommandHelper.hpp`) are fine.

Indirect via `wpi::util::GetTypeName(const T&)` → `typeid(type).name()` on a polymorphic object (needs RTTI):
1. `cpp/Command.cpp:20` — `Command::Command() : m_name{GetTypeName(*this)}`. Evaluated inside the base ctor, so upstream already returns the base type name ("wpi::cmd::Command"), not the subclass.
2. `cpp/SubsystemBase.cpp:16` — same situation ("wpi::cmd::SubsystemBase").
3. `cpp/Subsystem.cpp:24` — `Subsystem::GetName()` at runtime → true dynamic type name.

Fix (no RTTI): (a) simplest: literals `"Command"` / `"SubsystemBase"` / `"Subsystem"` (matches upstream behaviour for 1 & 2); (b) better: a `constexpr std::string_view TypeName<T>()` built from `__PRETTY_FUNCTION__` and set from `CommandHelper<Base,CRTP>`'s constructor (CRTP knows the concrete type) — gives Java-like class names with zero RTTI; for subsystems add an optional `SubsystemBase` CRTP helper or require `SubsystemBase(name)`. Also `wpi::util::Demangle` (`abi::__cxa_demangle`) becomes unused → drop to save flash.

Also check transitive: whatever prosdk uses for wpiutil/telemetry must not use `typeid` (grep of telemetry/tunables: none). `std::function` and `unique_function` work without RTTI (`target_type()` unavailable, not used).

## 4. Exceptions

12 throw sites, all `throw WPILIB_MakeError(wpi::err::CommandIllegalUse, ...)` (type `wpi::RuntimeError : std::runtime_error`):
- `CommandScheduler.hpp:217` (template `SetDefaultCommand`: default command must require its subsystem), `CommandScheduler.cpp:283` (same, CommandPtr overload), `:433` (`RequireUngrouped`: already composed — message includes stack trace), `:458` (`RequireUngroupedAndUnscheduled`).
- `CommandPtr.cpp:41` (`AssertValid`: moved-from CommandPtr).
- `SequentialCommandGroup.cpp:71`, `ParallelCommandGroup.cpp:72,88`, `ParallelRaceGroup.cpp:59,75`, `ParallelDeadlineGroup.cpp:72,88` (adding while running / overlapping requirements in parallel groups).
- Indirect: `EventLoop::Bind/Clear` throw if called during `Poll()` (i.e. binding a Trigger from inside a binding).
No `try/catch` anywhere in commandsv2. With `-fexceptions` these become uncaught → `std::terminate` → ESP32 abort/reboot with message; acceptable as "programmer error" but recommend a prosdk error macro that prints a PB-Exxx style diagnostic to Serial + browser DS before `abort()`, so it also compiles if a user turns exceptions off. Keep `throw` semantics on the host-test build so the 36 upstream Catch2 tests (5 `REQUIRE_THROWS/CHECK_THROWS`) still pass.

## 5. Threads / concurrency

- commandsv2 itself spawns no threads. `std::mutex hidsMutex` in `cpp/button/CommandGenericHID.cpp:14` (works on IDF 5 libstdc++ gthreads/FreeRTOS; could be a portMUX or removed if construction is single-task).
- `wpi::Watchdog` (scheduler member): upstream = shared background `std::thread` + `HAL_CreateNotifier/HAL_SetNotifierAlarm` (`wpilibc/src/main/native/cpp/system/Watchdog.cpp`). Replace with threadless version (see §2).
- `wpi::Notifier` (`NotifierCommand`): thread + HAL notifier. Replace with `esp_timer` (callback in esp_timer task — must be short) or a dedicated FreeRTOS task with `vTaskDelayUntil`.
- Scheduler, EventLoop, Trigger bindings, DenseMaps are **unsynchronized**: all calls must come from the probot-core user task. Joystick/DS data arrives on core 0 (WebSocket) → prosdk's GenericHID/RobotState must read a per-cycle latched snapshot (equivalent of `DriverStationBackend::RefreshData()` at the top of `LoopFunc`).
- probot-core's E-stop (`runEmergencyStop`) **deletes the user task**; any scheduler state is abandoned (fine since latched), but the stop hook runs on a separate task — prosdk's stop path must not touch the scheduler concurrently.

## 6. How `CommandScheduler::Run()` is meant to be driven

Doc (`CommandScheduler.hpp:31`): "A Command-based robot should call Run() on the singleton instance in its periodic block". In C++ there is no `TimedCommandRobot`; the template calls `CommandScheduler::GetInstance().Run()` from `Robot::RobotPeriodic()`. (RobotPy has `TimedCommandRobot` = `TimedRobot.add_periodic(scheduler.run, period, 0.005)` — `src/main/python/commands2/timedcommandrobot.py`.) `IterativeRobotBase::LoopFunc()` order (wpilibc `cpp/framework/IterativeRobotBase.cpp:94-175`): `DriverStationBackend::RefreshData()` → mode transition Exit/Init → mode Periodic (Disabled/Autonomous/Teleop/Utility) → **`RobotPeriodic()` (scheduler Run here)** → `TunableRegistry::Update()` → `SimulationPeriodic()` (sim only) → watchdog → `NetworkTableInstance::FlushLocal()`. RobotPeriodic runs **every cycle in every mode, including disabled**.

`Run()` sequence (`CommandScheduler.cpp:157-226`):
1. return if scheduler `Disable()`d (scheduler-local flag, independent of DS).
2. `Periodic()` of every registered subsystem (+ `SimulationPeriodic()` if `RobotBase::IsSimulation()`), watchdog epoch each.
3. `activeButtonLoop->Poll()` — all Trigger bindings evaluated (they may Schedule/Cancel).
4. `isDisabled = RobotState::IsDisabled()`; for a copy of `scheduledCommands`: if disabled and `!RunsWhenDisabled()` → `Cancel` (End(true), interrupt actions); else `Execute()` → exec actions → if `IsFinished()` → `End(false)`, finish actions, release requirements, erase owned `CommandPtr`.
5. For each registered subsystem without a requiring command and with a default command → `Schedule(default)`.
6. Watchdog disable; print epochs on overrun.

DS enable interaction:
- `Schedule()` silently refuses commands when `RobotState::IsDisabled() && !command->RunsWhenDisabled()` (`CommandScheduler.cpp:98-101`) — this includes default commands, so they only start once enabled.
- On transition to disabled, the next `Run()` interrupts every non-`RunsWhenDisabled` command (End(true)). This relies on Run() being called while disabled.
- `WaitCommand`, `WaitUntilCommand`, `PrintCommand` report `RunsWhenDisabled() == true`; `IgnoringDisable(true)` decorator wraps any command.
- `RobotModeTriggers::Autonomous/Teleop/Utility/Disabled` let users bind commands to mode changes (auto routines are commonly started with `RobotModeTriggers::Autonomous().WhileTrue(autoCmd)` or from `AutonomousInit`).
- Subsystem `Periodic()` and Trigger polling continue while disabled.

### Mapping onto probot-core (important)
probot-core 0.4.0 (`/srv/probot/core/src/probot/core/lifecycle.hpp`, `runtime.hpp:120-131`) runs a user task on core 1 every `USER_LOOP_PERIOD_MS` (20 ms) and calls **only** the active mode's hook: `*Init` once, `*InitLoop` during INIT stage, `*Start`, `*Loop` during RUN, `*Stop` on leaving; during STOPPED/TRANSITION **no user hook runs**, and outputs are neutralized whenever stage != RUN. There is no `robotPeriodic`.

Consequence: if users call `Run()` from `teleopLoop()/autonomousLoop()`, the scheduler never sees the disabled state → commands are never interrupted on stop and resume `Execute()` without `Initialize()` on the next enable; subsystem `Periodic()` stops while disabled. Recommendations:
1. Add to probot-core a weak `robotPeriodic()` hook (and/or an internal prosdk hook pointer) invoked by `userTask` **every cycle in every stage**, after the mode hook — mirrors `LoopFunc`. prosdk's robot framework (TimedRobot/IterativeRobotBase/OpModeRobot port) drives `DriverStationBackend::RefreshData` → mode hooks → `RobotPeriodic` → `TunableRegistry::Update` → telemetry flush from there. Command-based users then call `CommandScheduler::GetInstance().Run()` in `RobotPeriodic()`, exactly like WPILib (optionally provide a `prosdk::TimedCommandRobot`/auto-run flag).
2. `RobotState` mapping from `probot::robot::Phase`: `AUTO_RUN` → enabled+AUTONOMOUS, `TELEOP_RUN` → enabled+TELEOPERATED, `STOPPED`/`TRANSITION`/`AUTO_INIT`/`TELEOP_INIT` → disabled (INIT stage is neutralized in probot-core, so it must count as disabled; the selected OpMode can still be reported as the robot mode so `GetRobotMode()` matches WPILib 2027 "mode selected while disabled"). E-stop latched → `IsEStopped()`.
3. Fallback if probot-core cannot change: in `teleopStop()/autonomousStop()` prosdk runs one `Run()` with disabled state (or `CancelAll()` of non-RunsWhenDisabled commands).
4. `SetPeriod()` / watchdog timeout = `USER_LOOP_PERIOD_MS`.
5. `Timer::GetMatchTime()` (WaitUntilCommand(time)) from `StateSnapshot.autoPeriodSeconds/autoStartMs`.

## 7. Other porting notes / risks

- `CommandScheduler::Schedule(CommandPtr&&)` stores into `ownedCommands`, erased only when the command *finishes* (`Run`), not in `Cancel()` → a scheduled-then-cancelled owned command is never freed (upstream behaviour as read; worth a deferred-erase fix on a heap-constrained MCU — verify against tests first).
- `GetStackTrace(1)` on every `CommandPtr` move and every `SetComposed(true)` → stub to cheap value (e.g. `""` or `__builtin_return_address` hex) — the "Originally composed at" message loses detail but `IsComposed()` only needs `has_value()`.
- `std::format` used in `WaitCommand.cpp`, `ProxyCommand.cpp` and the error macro (fmt-style). GCC 14 libstdc++ has `<format>`; units must supply `std::formatter`. Code size of `<format>` is notable (~tens of KB) — acceptable on S3 but measure.
- Heavy use of `std::function`, `unique_function`, `std::string` names, `SmallSet`/`DenseMap` → heap fragmentation; consider reserving; names are concatenated each `AddEpoch` call in Run (`command->GetName() + ".Execute()"` per command per cycle → allocations every 20 ms). Recommend making the scheduler's watchdog epochs compile-time optional (`PROSDK_CMD_WATCHDOG_EPOCHS`) or using a fixed buffer.
- `wpi::util::ReportUsage` → no-op.
- `RobotBase::IsSimulation()` is `constexpr`; host-sim build of prosdk can flip it to run `SimulationPeriodic`.
- `CommandGenericHID::GetCommandGenericHID(port)` array size `DriverStationBackend::JOYSTICK_PORTS` (6 upstream) → set to probot-core's gamepad count.
- Tests: `src/test/native/cpp/wpi/command/*` (36 files incl. `CommandTelemetryTest`, `CommandTunableButtonTest`, `NetworkButtonTest`, `RobotModeTriggersTest`, `RobotDisabledCommandTest`, `SchedulingRecursionTest`) use `wpi::sim::DriverStationSim` to toggle enable — port them to a host build with a fake RobotState backend.

## 8. Recommended port plan (commands)

1. Copy `src/main/native` + `src/generated/main/native` verbatim; rename `wpi::cmd`→`prosdk::cmd`, `wpi/commands2`→`prosdk/commands2` (namespaces of deps `wpi::`→`prosdk::`).
2. Replace 3 `GetTypeName` uses (CRTP `__PRETTY_FUNCTION__` name in `CommandHelper`, literals elsewhere).
3. Provide threadless `Watchdog`, esp_timer-based `Notifier`, esp_timer `Timer`, stub `GetStackTrace`, no-op `ReportUsage`, `print`→Serial.
4. Back `RobotState`/`DriverStation::GetGenericHID` with probot-core state + WebSocket joystick (latched per cycle).
5. Re-implement `NetworkButton` over prosdk tunables (or drop); keep `SysIdRoutine` if DataLog backend exists.
6. Get a per-cycle `robotPeriodic` hook into probot-core and call `Run()` there (the single most important behavioural requirement).
7. Keep exceptions (misuse-only) behind a prosdk error macro.
8. Skip commandsv3 entirely (Java-only, continuation-based).
