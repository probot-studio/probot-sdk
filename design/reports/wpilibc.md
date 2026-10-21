# wpilibc (allwpilib b68effca, 2027 dev): port inventory for prosdk on ESP32-S3 / probot-core 0.4.0

Source root: `.../scratchpad/wpi/allwpilib/wpilibc/src` (shortened below to `wpilibc/`).
Probot-core: `/srv/probot/core` (API.md, `src/probot/core/{lifecycle,runtime,core_config}.hpp`, `src/probot/robot/state.hpp`, `src/probot/io/gamepad.hpp`).

This was a read-only investigation using greps and file reads. Nothing was built.

---

## 0. Headline facts (2027 differs a lot from 2025/2026 WPILib)

1. **Size.** `src/main/native` has 130 `.hpp` and 113 `.cpp` files (35,752 lines). `src/generated/main` adds 30 `.hpp` and 30 `.cpp` files (23,085 lines): 15 HID controller classes and 15 `*ControllerSim` classes, generated from Jinja templates in `src/generate/main/...` by `generate_hids.py` and `generate_first_ds_hids.py`. There are about 160 top-level classes/structs/enum classes and 79 nested ones. Tests: 63 + 11 `.cpp` files under `src/test`, `src/generated/test`.
2. **Several older classes are gone from wpilibc:**
   - `Sendable`, `SendableBuilder`, `SmartDashboard`, `SendableChooser` and `Shuffleboard` do not exist anywhere in this checkout. A grep for `class SmartDashboard|SendableChooser|Shuffleboard|Sendable|SendableBuilder` matched nothing.
   - They are replaced by the separate **`telemetry/`** module (`wpi::telemetry::TelemetryLoggable` with `LogTo(TelemetryTable&)`, `TelemetryRegistry`, `TelemetryBackend`) and the **`tunables/`** module (`TunableRegistry`, `TunableBackend`, `Tunable`, `Selectable`). Both have pluggable backends. This helps the port a lot: no NT dependency is baked into devices.
   - **38 wpilibc classes implement `TelemetryLoggable`**: all sensors, PWM, motor controller, drives, Field2d, Mechanism2d, Gamepad and all 15 generated HIDs.
   - **No `Servo` class** exists anywhere in the tree. Only `ExpansionHubServo`, `XRPServo` and `RomiServo` exist.
   - **PWM motor controllers moved** to `allwpilib/drivers` (`wpi/drivers/motor/{PWMSparkMax,PWMSparkFlex,PWMTalonFX,PWMTalonSRX,PWMVenom,PWMVictorSPX,Spark,SparkMini,Koors40}.hpp`, generated from `drivers/src/generate/pwm_motor_controllers.json`). wpilibc keeps only the `PWMMotorController` base.
   - **`Alert` lives in wpiutil** (`wpiutil/.../wpi/util/Alert.hpp`), not wpilibc.
   - **`SysIdRoutine` lives in commandsv2** (`commandsv2/.../sysid/SysIdRoutine.hpp`). wpilibc only has `SysIdRoutineLog`.
   - **Test mode is now "Utility".** `RobotMode` = `UNKNOWN`, `AUTONOMOUS`, `TELEOPERATED`, `UTILITY`.
   - **No `RobotInit()`.** The constructor plays that role.
   - The DS control word carries a 56-bit **OpMode id/hash** (`HAL_ControlWord`, `wpi/hal/DriverStationTypes.h` lines 18-24 and 191-262).
3. **New FTC-like framework in WPILib itself.** `OpModeRobot` / `OpModeRobotBase` (`framework/OpModeRobot.hpp/.cpp`, 666 lines), `OpMode` and `PeriodicOpMode` (`opmode/`). This maps very naturally onto probot-core's FTC-style hooks (section 3).
4. **Real NetworkTables (ntcore) use is small.** The earlier count of 42 `wpi/nt` includes was inflated by false hits on `Event::`. Files with real `wpi::nt` / `NetworkTable` references:
   - `backend/NetworkTablesTelemetryBackend`
   - `backend/NetworkTablesTunableBackend` (1,315 lines)
   - `driverstation/internal/DriverStationBackend.cpp` (publishes the `/DriverStation` match-info table and the ControlWord struct)
   - `preferences/Preferences`
   - `event/NetworkBooleanEvent`
   - `framework/RobotBase.cpp` (starts the NT server, registers NT backends, `/Robot/ProgramStartTime`)
   - `IterativeRobotBase.cpp` and `OpModeRobot.cpp` (only `NetworkTableInstance::GetDefault().FlushLocal()`)
   - `system/DataLogManager` (NT entry/connection logging)
   - `system/SystemServer`
   - all 6 `hardware/expansionhub/*` files (they talk to the SystemCore "rhsp" server over NT topics `/rhsp/{usb}/motor{n}/...`)
5. **Threads in wpilibc:**
   - `Notifier` (one `std::thread` per instance)
   - `Watchdog::Impl` (one singleton thread)
   - `MotorSafety` manager (`SafeThread` waiting on HAL new-DS-data event)
   - `DriverStationModeThread` (no users in C++; Java-parity helper)
   - `StartRobot` (robot thread when `HAL_HasMain()`)
   - `DataLogManager` (background writer)

   All of these work on ESP-IDF pthreads, but they **escape probot-core's e-stop**. Core's e-stop kills only the core user task (`runtime.hpp: runEmergencyStop`). See section 4.4.
6. **Arduino macro collisions (compile blockers).** Arduino-ESP32 3.3.10 (`cores/esp32/esp32-hal-gpio.h`, `Arduino.h`) `#define`s `HIGH`, `LOW`, `INPUT`, `OUTPUT`, `ANALOG`, `RISING`, `FALLING`, `CHANGE`, `PI`, `DEFAULT`, `SERIAL`, `DISPLAY` and others. They collide with:
   - `wpi::math::Debouncer::DebounceType::RISING/FALLING` (wpimath; used by `event/BooleanEvent.hpp:127`)
   - `wpi::util::Alert::Level::HIGH` (`DriverStationBackend.cpp:266`)
   - `wpi::hal::SimDevice::Direction::INPUT/OUTPUT` (`PWMMotorController.cpp:86`, `ADXL345_I2C.cpp:36-43`, `SharpIR.cpp:40`, `AnalogEncoder.cpp:66`, `DutyCycleEncoder.cpp:67-69`)
   - `CompressorConfigType::ANALOG` (`pneumatic/CompressorConfigType.hpp:17`)

   **Recommendation:** add these enumerators to the mechanical rename table, for example `RISING` -> `RISING_EDGE`/`kRising` and `HIGH` -> `HIGH_SEVERITY`. Do this together with the `wpi::` -> `prosdk::` rename rather than relying on `#undef` or `push_macro`.
7. **No RTTI use** in wpilibc: no `dynamic_cast`, `typeid` or `std::any` hits, so `-fno-rtti` is fine. `std::print`/`<print>` appear only in `RobotBase.cpp` and `OpModeRobot.cpp`; GCC 14 supports them, or swap to `wpi::util::print`. `std::format` is used widely: it works on GCC 14 but adds flash size. `wpi/util/json.hpp` is used by `DriverStationBackend`, `Preferences`, both NT backends, the DataLog backend and `MechanismLigament2d`.

---

## 1. Dependency matrix (per class/group)

Legend:
- **HAL** = `wpi/hal/*` headers used.
- **NT** = ntcore.
- **TEL** = telemetry module (`TelemetryLoggable` / `LogTo`).
- **TUN** = tunables.
- **DL** = datalog.
- **SIM** = HAL SimDevice / `hal/simulation`.
- **THR** = owns a thread or mutex.
- Line counts are `.hpp` + `.cpp`.

### framework/ (lifecycle)
| Class | Lines | HAL | NT | TEL/TUN | Notes |
|---|---|---|---|---|---|
| `RobotBase` (+ `StartRobot<T>()`, `RunHALInitialization()`, `impl::RunRobot`) | 571 | DriverStation, HAL, Main, UsageReporting | yes (server start, ProgramStartTime, conn listener) | registers NT telemetry + tunable backends, Alert-based warning reporter | `std::thread`, mutex, condvar; `MathSharedStore` setup |
| `IterativeRobotBase` | 413 | DriverStation(Types) | `FlushLocal` only | `TunableRegistry::Update()` | `Watchdog` member |
| `TimedRobot` | 167 | Notifier | - | - | `PeriodicPriorityQueue`, `HAL_CreateNotifier` |
| `TimesliceRobot` | 147 | - | - | - | `TimedRobot::AddPeriodic` with offsets |
| `OpModeRobotBase` / `OpModeRobot<Derived>` | 666 | DriverStation(Types), HAL, Notifier | `FlushLocal` | `TunableRegistry::Update()`, `Alert` | `DenseMap`, `SafeThread` include, concepts |
| `opmode/OpMode`, `opmode/PeriodicOpMode` | 88/155 | - | - | - | pure |
| `internal/PeriodicPriorityQueue` | 348 | Notifier (`HAL_SetNotifierAlarm`, `WPI_WaitForObject`) | - | - | `wpi::util::priority_queue` |
| `internal/DriverStationModeThread` | 90 | DriverStation | - | - | `std::thread`; unused by C++ code |

### driverstation/
| Class | Lines | HAL | NT | Other |
|---|---|---|---|---|
| `internal/DriverStationBackend` | 2,050 | DriverStation, DriverStationTypes, HAL, Power. About 30 distinct HAL DS calls: `HAL_RefreshDSData`, `HAL_GetJoystick{Axes,Buttons,POVs,Touchpads,Descriptor}`, `HAL_GetMatchInfo`, `HAL_GetAllianceStation`, `HAL_GetGameData`, `HAL_GetMatchTime`, `HAL_GetVinVoltage`, `HAL_SetOpModeOptions`, `HAL_MakeOpModeId`, `HAL_ObserveUserProgramStarting`, `hal::GetControlWord` | yes (match data, ControlWord struct, opMode string) | DL (`StartDataLog`), json, Alerts (joystick resource warnings), mutexes |
| `DriverStation` (facade: `GetGenericHID`, `GetGamepad`, `StartDataLog`) | 158 | - | - | DL fwd |
| `RobotState` (static: `IsEnabled`, `IsAutonomous`, `IsTeleop`, `IsUtility`, `AddOpMode`, `GetOpModeId`, ...) | 258 | via backend | - | - |
| `MatchState` (`GetMatchTime`, `GetAlliance`, `GetLocation`, `GetMatchNumber`, `GetEventName`, `GetGameData`) | 124 | via backend | - | - |
| `GenericHID` (raw buttons/axes/POV, `BooleanEvent` factories, rumble, LEDs, touchpad) | 649 | `HAL_SetJoystickRumble`, `HAL_SetJoystickLeds` | - | - |
| `Gamepad` (2027 canonical SDL-style gamepad, `enum Button`/`Axis`, about 100 getters + `BooleanEvent` factories) | 1,951 | - | - | TEL |
| `Joystick` (flight stick) | 504 | - | - | - |
| `HIDDevice` (interface `GetHID()`) | 31 | - | - | - |
| `DriverStationDisplay` (FTC-style text: `AddData`, `AddLine`, `AddKeyedLine`, `UpdateLines`, `WriteRawAnsi`) | 263 | `HAL_WriteDisplayAnsi` | - | - |
| `DSGamepadChooser` (D-pad driven option chooser shown on the DS display) | 514 | - | - | uses `Gamepad` + `DriverStationDisplay` |
| `POVDirection`, `TouchpadFinger`, `Alliance`, `MatchType` | small | DriverStationTypes | - | - |
| **Generated:** `XboxController`, `NiDsXboxController`, `NiDsPS4Controller`, `NiDsPS5Controller`, `NiDsStadiaController`, `DualShock4Controller`, `DualSenseController`, `DualSenseEdgeController`, `LogitechF310Controller`, `SteamController`, `SwitchProController`, `Switch2ProController`, `Switch2GCController`, `SwitchN64Controller`, `GameCubeController` (all `: HIDDevice, TelemetryLoggable`, wrap a `GenericHID*`) | 12,309 (hpp) | - | - | TEL |

Note on old names:
- `PS4Controller`, `PS5Controller` and `StadiaController` no longer exist under those names. They are `NiDsPS4Controller`/`DualShock4Controller`, `NiDsPS5Controller`/`DualSenseController` and `NiDsStadiaController`.
- The 2027 `XboxController` layout is SDL-style and 0-based: buttons `A=0, B=1, X=2, Y=3, VIEW=4, XBOX=5, MENU=6, LEFT_STICK=7, RIGHT_STICK=8, LEFT_BUMPER=9, RIGHT_BUMPER=10, DPAD_UP..RIGHT=11..14`; axes `LEFT_X=0, LEFT_Y=1, RIGHT_X=2, RIGHT_Y=3, LEFT_TRIGGER=4, RIGHT_TRIGGER=5`.

### event/
| Class | Deps |
|---|---|
| `EventLoop` | pure |
| `BooleanEvent` | wpimath `Debouncer` (RISING macro clash) |
| `NetworkBooleanEvent` | NT topics |

### hardware/
| Class | Lines | HAL group | TEL | SIM | ESP32 mapping |
|---|---|---|---|---|---|
| `PWM` (`SetPulseTime` µs, `SetOutputPeriod`, `SetDisabled`) | 187 | PWM | yes | yes | LEDC (servo-type pulses) |
| `DigitalInput` / `DigitalOutput` (`Pulse`, `IsPulsing`, digital-PWM `EnablePWM`/`UpdateDutyCycle`) | 118/292 | DIO (`HAL_*DIO*`, `HAL_*DigitalPWM*`) | yes | yes | gpio + LEDC |
| `AnalogInput` (`GetValue`, `GetVoltage`) | 141 | AnalogInput | yes | yes | ADC1 oneshot (ADC2 unusable with WiFi) |
| `Encoder` (quadrature, `GetDistance`, `GetRate`, `SetDistancePerPulse`, `SetReverseDirection`, `SetRateWindow`) | 376 | Encoder | yes | yes | PCNT (S3: 4 units) |
| `CounterBase` (interface) | 35 | - | - | - | - |
| `EdgeCounter` / `Tachometer` + `EdgeConfiguration` | 116/198 | Counter | yes | - | PCNT / GPIO ISR + timestamps |
| `DutyCycle` / `DutyCycleEncoder` (absolute encoders) | 138/361 | DutyCycle, SimDevice | yes | yes | MCPWM capture or RMT RX |
| `AnalogEncoder` / `AnalogPotentiometer` | 278/155 | via AnalogInput, SimDevice | yes | yes | ADC |
| `AnalogAccelerometer` | 162 | via AnalogInput | yes | - | ADC |
| `ADXL345_I2C` | 292 | via I2C, SimDevice | yes | yes | Wire |
| `SharpIR` | 168 | via AnalogInput, SimDevice | yes | yes | ADC |
| `I2C` (`Transaction`, `Read`, `Write`, `AddressOnly`, `VerifySensor`) | 263 | I2C | - | - | Wire / i2c_master |
| `SerialPort` | 416 | SerialPort | - | - | HardwareSerial (UART1/2) |
| `CAN` / `CANPort` | 281/66 | CAN, CANAPI | - | - | TWAI (S3 has CAN 2.0B) |
| `AddressableLED` (`SetStart`, `SetLength`, `SetData`, `SetColorOrder`, `SetGlobalData`) | 256 | AddressableLED(Types) | - | - | RMT (WS2812) |
| `LEDPattern` (pure pattern combinators) | 742 | - | - | - | pure; pull to `Color`/units/wpimath |
| `MotorController` (interface: `SetThrottle`, `SetVoltage`, `GetThrottle`, `SetInverted`, `Disable`) | 84 | - | - | - | pure |
| `MotorSafety` (+ manager thread) | 307 | DriverStation(Types), new-data event | - | - | SafeThread -> see 4.4 |
| `PWMMotorController` (followers, deadband elimination, bounds) | 313 | via PWM, SimDevice | yes | yes | LEDC servo-pulse ESCs |
| `OnboardIMU` (SystemCore built-in IMU, `GetRotation3d`, `GetYaw`, ...) | 293 | IMU | - | - | none onboard |
| `PowerDistribution` (CTRE PDP / REV PDH over CAN) | 696 | PowerDistribution | yes | - | none |
| `Compressor`, `Solenoid`, `DoubleSolenoid`, `PneumaticsBase`, `PneumaticsControlModule`, `PneumaticHub`, enums | 2,476 total | CTREPCM, REVPH, Ports | partly | - | CAN pneumatics |
| `ExpansionHub`, `ExpansionHubMotor`, `ExpansionHubServo`, `ExpansionHubCRServo`, `ExpansionHub{Position,Velocity}Constants` | 1,460 total | - | **NT only** | - | REV hub via SystemCore USB |

### drive/
`RobotDriveBase` (139, `MotorSafety`), `DifferentialDrive` (401, TEL), `MecanumDrive` (316, TEL, wpimath geometry). All pure apart from `MotorSafety` and `MotorController`.

### smartdashboard/ (no NT any more; telemetry/tunables-based)
- `Field2d`: 251 lines, TEL + TUN (robot pose is tunable).
- `FieldObject2d`: 250 lines, TUN, mutex, wpimath trajectory.
- `Mechanism2d`: 129 lines, TEL.
- `MechanismObject2d`: 100 lines, TEL.
- `MechanismRoot2d`: 76 lines.
- `MechanismLigament2d`: 178 lines, TEL, json.

### backend/
- `NetworkTablesTelemetryBackend`: 483 lines, NT.
- `NetworkTablesTunableBackend`: 1,315 lines, NT.
- `DataLogTelemetryBackend`: 379 lines, DL, json.

### preferences/, sysid/
- `Preferences` (407): NT persistent topics plus json.
- `SysIdRoutineLog` (270): DL through `DataLogManager::GetLog()`.

### system/
- `Timer` (327): pure. Uses `RobotController::GetMonotonicTime` and `MatchState::GetMatchTime`.
- `Notifier` (292): HAL Notifier, Threads; owns a `std::thread`.
- `Watchdog` (382): HAL Notifier, `HAL_GetMonotonicTime`; singleton thread.
- `Tracer` (129) and `ScopedTracer` (62): pure.
- `RobotController` (378): `HAL_GetVinVoltage`, `HAL_GetBrownedOut`, `HAL_GetUser*` (3V3 rail), `HAL_GetCPUTemp`, `HAL_GetSerialNumber`, `HAL_GetTeamNumber`, `HAL_GetRSLState`, `HAL_CAN_GetCANStatus`, `HAL_SetBrownoutVoltages`, `HAL_GetMonotonicTime`.
- `Errors` (278) plus `WPIErrors.mac` / `WPIWarnings.mac`: `HAL_SendError`, `HAL_GetLastError`, `StackTrace`.
- `Threads` (107): `HAL_{Get,Set}{Current,}ThreadPriority`.
- `Filesystem` (69): SystemCore paths `/home/systemcore/deploy`.
- `DataLogManager` (493): DL, NT, SafeThread, fs.
- `SystemServer` (33): `HAL_GetSystemServerHandle` returns the NT instance.
- `RuntimeType`, `WPILibVersion` (generated from `src/generate/WPILibVersion.cpp.in`).

### util/
`SensorUtil` (55): `HAL_Check*Channel`, port counts.

### simulation/ (35 hand-written + 15 generated)
- **Physics, pure wpimath + `RobotController::GetBatteryVoltage()`:**
  - `LinearSystemSim<S,I,O>` (166, header-only template; Eigen, `random/Normal`)
  - `DCMotorSim` (230)
  - `FlywheelSim` (179)
  - `ElevatorSim` (308)
  - `SingleJointedArmSim` (313)
  - `DifferentialDrivetrainSim` (527)
  - `BatterySim` (93, header-only, pure units)
- **HAL-sim wrappers (need the HAL simulation data layer, `hal/simulation/*Data.h`):**
  - `AddressableLEDSim`, `AnalogInputSim`, `CTREPCMSim`, `DigitalPWMSim`, `DIOSim`, `DriverStationSim` (790 lines), `DutyCycleSim`, `EncoderSim`, `OnboardIMUSim`, `PowerDistributionSim`, `PWMSim`, `REVPHSim`, `RoboRioSim` (487), `SimDeviceSim`, `SimHooks`, `CallbackStore`
  - SimDevice-based: `ADXL345Sim`, `AnalogEncoderSim`, `DutyCycleEncoderSim`, `PWMMotorControllerSim`, `SharpIRSim`
- **Composite / pure helpers:** `AlertSim`, `DoubleSolenoidSim`, `SolenoidSim`, `PneumaticsBaseSim`, `GenericHIDSim`, `GamepadSim` (431), `JoystickSim`, plus the 15 generated `*ControllerSim` (4,564 lines). These drive `DriverStationSim`.

### Cross-library includes (what wpilibc pulls in)
- **wpiutil:** `UsageReporting` (47 includes), `mutex` (19), `Handle` (14), `StackTrace` (12), `SensorUtil`, `StringMap`, `NullDeleter`, `string`, `MathExtras`, `json` (6), `print`, `DenseMap`, `timestamp`, `Synchronization` (`WPI_WaitForObject`, `Event`), `Alert`, `SafeThread`, `fs`, `Color`/`Color8Bit`, `struct/Struct`, `SmallVector`, `raw_ostream`, `priority_queue`, `FunctionExtras`, `condition_variable`, `EventVector`, `RuntimeCheck`.
- **wpimath:** `util/MathUtil` (17), `geometry` (Rotation2d, Pose2d, Translation2d, Rotation3d, Quaternion), `system` (DCMotor, LinearSystem, NumericalIntegration, Models), `StateSpaceUtil`, `random/Normal`, `linalg/EigenCore`, `MathShared`, `trajectory/TrajectorySample`, `filter/Debouncer`.
- **units:** 99 includes.
- **HAL:** 218 distinct `HAL_*` functions referenced.

---

## 2. Classification

### MUST-PORT (core robot framework and generic hardware)

**Framework and timing**
- `RobotBase`, `IterativeRobotBase`, `TimedRobot`, `TimesliceRobot`, `OpModeRobotBase`/`OpModeRobot`, `OpMode`, `PeriodicOpMode`, `internal/PeriodicPriorityQueue`.
- `StartRobot` is reworked: see section 3. It becomes a hosted adapter and no longer owns `main`.
- `Timer`, `Notifier` (FreeRTOS backend), `Watchdog`, `Tracer`, `ScopedTracer`, `Errors` (route to `Serial` + probot telemetry), `Threads` (FreeRTOS priorities), `RobotController` (subset: battery from core `batteryVoltage`, monotonic time from `esp_timer_get_time()`, CPU temp from `temperatureRead()`; the rest return defaults), `RuntimeType`, `WPILibVersion`, `SensorUtil` (ESP32 pin rules).

**Driver Station and HID**
- `DriverStationBackend` (rewritten DS data source, section 4.1), `DriverStation`, `RobotState`, `MatchState` (defaults: no FMS).
- `GenericHID`, `Gamepad`, `Joystick`, `HIDDevice`, `POVDirection`, `TouchpadFinger`, `Alliance`, `MatchType`.
- All 15 generated HID classes (at minimum `XboxController`, `DualShock4Controller`, `DualSenseController`, `LogitechF310Controller`, `NiDs*` and `SwitchProController`). They are thin wrappers, so generate them all with the existing Jinja + JSON pipeline.
- `DriverStationDisplay` maps onto `probot::telemetry`. `DSGamepadChooser` follows.

**Events:** `EventLoop`, `BooleanEvent`.

**Actuators and sensors**
- `MotorController`, `MotorSafety`, `PWMMotorController`, `PWM`, `DigitalInput`, `DigitalOutput`, `AnalogInput`, `AnalogPotentiometer`, `AnalogEncoder`, `AnalogAccelerometer`, `Encoder`, `CounterBase`, `EdgeCounter`, `Tachometer`, `DutyCycle`, `DutyCycleEncoder`, `ADXL345_I2C`, `SharpIR`, `I2C`, `SerialPort`, `AddressableLED`, `LEDPattern`.
- The PWM motor controller drivers from `allwpilib/drivers` (Spark, PWMSparkMax and so on) are trivial `PWMMotorController` subclasses; port them via their JSON.
- **Add a `Servo` class** (re-port the 2026 `Servo` on top of `PWM`). WPILib 2027 has none, and probot-core API.md explicitly says core provides no servo class.

**Drives:** `RobotDriveBase`, `DifferentialDrive`, `MecanumDrive`.

**Misc**
- `Preferences` (backend change: NVS, section 4.3).
- `SysIdRoutineLog`. It needs a DataLog sink, which can be telemetry, an SD card, or a CSV stream to the DS. Pairs with commandsv2 `SysIdRoutine`.
- Alerts are wpiutil `Alert`. Their display needs a telemetry-side sink.

**Physics simulation (pure math; useful on host and on-device for model-based control and tests)**
`LinearSystemSim`, `DCMotorSim`, `FlywheelSim`, `ElevatorSim`, `SingleJointedArmSim`, `DifferentialDrivetrainSim`, `BatterySim`.

**HAL-sim layer (for host/sim builds)**
`RoboRioSim` (rename, for example `RobotControllerSim`), `DriverStationSim`, `SimHooks`, `CallbackStore`, `PWMSim`, `DIOSim`, `AnalogInputSim`, `EncoderSim`, `DutyCycleSim`, `DigitalPWMSim`, `AddressableLEDSim`, `SimDeviceSim`, `GenericHIDSim`, `GamepadSim`, `JoystickSim`, `*ControllerSim`, `PWMMotorControllerSim`, `DutyCycleEncoderSim`, `AnalogEncoderSim`, `ADXL345Sim`, `SharpIRSim`, `AlertSim`.

They are worth having **only if** the prosdk HAL gets a sim data layer (`hal/simulation/*Data`), for example for host unit tests with g++. Recommendation: port them in the host-test build. On device, compile them out, or make them header-only and unused.

### PORT-WITH-BACKEND-CHANGE (currently NT / DataLog / SystemCore-facing)
| wpilibc piece | Today | Map to |
|---|---|---|
| `NetworkTablesTelemetryBackend` (registered in `RobotBase()` for `""` -> `/Telemetry`) | NT server | New `prosdk::ProbotTelemetryBackend : TelemetryBackend`. It streams key/value (and struct-schema) entries to the browser DS over a **new WS frame type**. Core's current `'T'` frame is a 256-byte text ring (`probot::telemetry`), far too small for structured data, so core needs an extension point (4.5). Interim fallback: render selected entries as text lines into `probot::telemetry`. |
| `NetworkTablesTunableBackend` (`/Tunables`) | NT | `prosdk::ProbotTunableBackend : TunableBackend` (`Publish`, `MarkDirty`, `Remove`, `RemovePrefix`, `UnregisterTunable`, `Update`). Values are edited from the DS through a new WS frame or HTTP endpoint, and `Update()` runs once per loop in `LoopFunc`, as today. |
| `DataLogTelemetryBackend`, `DataLogManager` | `wpi::log` file on SystemCore disk | Optional: LittleFS or SD (SDMMC/SPI) file logger. The DataLog format is plain and portable, but the background-writer thread and flash wear matter. Default: disabled stub (`GetLog()` returns a discard log). |
| `SysIdRoutineLog` | DataLogManager | Same as above. Or stream CSV through the telemetry backend or a dedicated download endpoint. |
| `Preferences` | NT persistent topics, `networktables.json` | ESP32 NVS (`Preferences.h` from Arduino-ESP32 or `nvs_*`). The class name `Preferences` collides with Arduino's global `::Preferences` if a user does `using namespace prosdk;`. Keep it namespaced, or rename it. |
| `Field2d` / `FieldObject2d` | telemetry + tunables | Works unchanged once the backends exist. The DS needs a field widget, otherwise the data is just numbers. |
| `Mechanism2d` (+ Root, Ligament, Object) | telemetry | Same, plus a DS widget. |
| `NetworkBooleanEvent` | NT boolean topic | Re-implement over a tunable boolean (`Tunable<bool>`), or drop. |
| `DriverStationBackend` match-data NT publishing (`/DriverStation/*`, ControlWord struct, opMode) | NT | Drop the NT publishing. Optionally mirror via telemetry. |
| `RobotBase` NT server start + `/Robot/ProgramStartTime` + connection-listener usage reporting | NT | Remove. Register the probot backends instead. |
| `IterativeRobotBase` / `OpModeRobot` `FlushLocal()` | NT | Replace with a telemetry-backend flush hook (or nothing). |
| `DriverStationDisplay` (`HAL_WriteDisplayAnsi`) | DS display | `probot::telemetry::clear()` + `println()` (`UpdateLines` maps to a full redraw). |
| `GenericHID::SetRumble`/`SetLeds` | HAL to DS | No-op for now. Possible later through a WS frame using the browser Gamepad `vibrationActuator` (core change). |
| `OnboardIMU` | SystemCore IMU via HAL IMU | Keep the API (`GetRotation3d`, `GetYaw`, `ResetYaw`, mount orientation) with a pluggable backend, for example the existing sdk `probot/devices/sensors/imu/mpu6050.hpp` or a BNO08x. Otherwise drop it. |
| `CAN` | HAL CAN | ESP32-S3 TWAI. Optional, low priority. |
| `Filesystem` | `/home/systemcore/deploy` | LittleFS mount (for example `/littlefs/deploy`). |
| `RobotController` rails (3V3 current/faults, brownout, RSL, serial, team) | HAL Power | Brownout from core battery voltage. RSL state = core RSL blink. The rest return defaults. |
| `Threads` | HAL thread priority | FreeRTOS `vTaskPrioritySet`. Keep below core's `PRIO_CTRL` (4). |

### DROP (roboRIO/SystemCore/CAN-device or camera specific)
- `ExpansionHub*` (6 classes): REV Expansion Hub over SystemCore USB through the NT "rhsp" server. A REV hub over RS485 is out of scope.
- `PowerDistribution` + `PowerDistributionSim` (CTRE PDP / REV PDH over CAN).
- `PneumaticsControlModule`, `PneumaticHub`, `CTREPCMSim`, `REVPHSim`, `Compressor`, `CompressorConfigType`.
  - Keep `Solenoid`/`DoubleSolenoid`/`PneumaticsBase` only if a GPIO-valve `PneumaticsBase` implementation is wanted. Otherwise drop them as well.
- `SystemServer` (SystemCore NT instance handle).
- `DriverStationModeThread` (no C++ users).
- `StartRobot`'s `HAL_HasMain`/`HAL_RunMain` path, `WPI_IsRuntimeValid` check, `HAL_Shutdown`.
- Usage reporting (make `wpi::util::ReportUsage` a no-op).
- Camera server: not in wpilibc 2027 at all.
- `Utility` mode can stay in the API, but core has no third mode. `IsUtility()` is always false until core adds one.

---

## 3. Lifecycle today, and how to drive it from probot-core hooks

### 3.1 How WPILib 2027 runs a robot
1. **User `main`:** `int main() { return wpi::StartRobot<Robot>(); }`.
2. **`StartRobot<Robot>()`** (`RobotBase.hpp`):
   - checks `WPI_IsRuntimeValid`
   - calls `RunHALInitialization()`, which does `HAL_Initialize`, sets usage reporting, calls `DriverStationBackend::RefreshData()`, prints the banner and publishes the WPILib version
   - if `HAL_HasMain()`, spawns a `std::thread` running `impl::RunRobot`, then runs `HAL_RunMain()` on the main thread; otherwise calls `RunRobot` inline
   - `RunRobot` constructs `static Robot theRobot` (the robot constructor *is* robotInit) and calls `theRobot.StartCompetition()`, which blocks forever
   - on exit: `ResetMotorSafety()` and `HAL_Shutdown()`
3. **`RobotBase()` constructor:**
   - records the thread id
   - sets up MathShared (error reporting + timestamp for wpimath)
   - starts the NT server (`networktables.json`)
   - installs telemetry/tunable warning reporters (Alerts)
   - registers `NetworkTablesTelemetryBackend("/Telemetry")` and `NetworkTablesTunableBackend("/Tunables")`
   - waits up to 1 s for NT
   - publishes `/Robot/ProgramStartTime`, adds a connection listener
   - calls `RefreshData()`
4. **`TimedRobot(period=20 ms)`:**
   - registers `LoopFunc` in `PeriodicPriorityQueue` with absolute expiration times (start time + n*period; overruns skip whole periods)
   - creates a HAL Notifier
   - `StartCompetition()` calls `SimulationInit()` (sim only) and `ObserveUserProgramStarting()`, then loops `m_callbacks.RunCallbacks(m_notifier)`
   - `RunCallbacks` pops the earliest callback, sets the notifier alarm, blocks on `WPI_WaitForObject(notifier)`, runs it, re-queues it, and runs any other due callbacks
5. **`IterativeRobotBase::LoopFunc()`** decides mode transitions:
   - `RefreshData()`, `m_watchdog.Reset()`
   - reads `hal::GetControlWord()`; `mode = enabled ? word.GetRobotMode() : UNKNOWN`, so "disabled" is UNKNOWN
   - calls `DriverStationConnected()` once when DS-attached first goes true
   - if `mode != m_lastMode`: calls the old mode's `*Exit()` (`DisabledExit`, `AutonomousExit`, `TeleopExit`, `UtilityExit`), then the new mode's `*Init()`
   - `HAL_ObserveUserProgram(word)`
   - calls the current mode's `*Periodic()`, then `RobotPeriodic()`, `TunableRegistry::Update()`, sim hooks
   - `m_watchdog.Disable()`, NT `FlushLocal`, prints epochs if the period overran
6. **`OpModeRobotBase::LoopFunc()`** (2027 FTC-like):
   - reads `modeId = word.GetOpModeId()` (0 = none)
   - on change: ends and destroys the current `OpMode`
   - if a new id is selected (even while disabled): instantiates the opmode via its factory, registers its extra periodic callbacks, calls `opmode->DisabledPeriodic()`
   - on the enabled edge: `DisabledExit()`, then `StartCurrentOpMode()` (`opmode->Start()` + queue `opmode->Periodic()` at the robot period)
   - on the disabled edge: `EndCurrentOpMode()` (`End()` + destroy), `DisabledInit()`
   - while disabled: `DisabledPeriodic()` + `opmode->DisabledPeriodic()`
   - no opmode selected: `NonePeriodic()`
   - always: `RobotPeriodic()`, `TunableRegistry::Update()`
   - opmodes are registered with `AddOpMode<T>(RobotMode, name, group, desc[, colors])`, which calls `RobotState::AddOpMode`, then `HAL_SetOpModeOptions` to publish them to the DS

### 3.2 How probot-core runs code
Sources: `runtime.hpp`, `lifecycle.hpp`, API.md.

- **Ownership.** Core defines `setup()`, which calls `runtime_setup()`, and `loop()`, which is just `delay(100)`.
- **Tasks:**
  - **one persistent user task** on core 1 (`PRIO_USER=1`, 8 KB stack)
  - a `sysloop` supervisor on core 0 (`PRIO_CTRL=4`)
  - WiFi/httpd on core 0
- **User task loop:** `g_machine.step(hooks, requested, ...)` then `vTaskDelay(USER_LOOP_PERIOD_MS=20)`. This is relative delay, not drift-free.
- **PhaseMachine:** `STOPPED -> INIT -> RUN -> STOPPED`, plus `TRANSITION`.
  - The hooks are `autonomousInit`/`teleopInit` (INIT entry), `*InitLoop` (each tick in INIT), `*Start` (START), `*Loop` (each tick in RUN) and `*Stop` (leaving INIT or RUN).
  - **No hook runs in STOPPED or TRANSITION.**
  - Outside RUN, inputs are zeroed every tick (`zeroInputs`).
- **Modes and auto timer.** Only `AUTO` and `TELEOP` exist. The auto timer starts after `autonomousStart()` returns. When `autoPeriodSeconds` expires, the supervisor forces STOP, then TRANSITION with TELEOP preselected (not running).
- **Stall watchdog.** A loop tick longer than 2 s zeroes inputs and raises `deadlineMiss`. There is no kill.
- **DS timeout.** 10 s of silence leads to STOP (or neutral inputs, depending on config).
- **E-stop.** Latch, enable pin LOW, `vTaskDelete(user task)`, then the active `*Stop()` runs in a fresh task with a 500 ms budget (otherwise `ESP.restart()`). The robot stays locked until reboot.
- **State snapshot.** `probot::robot::state().read()` gives `status`, `phase`, `selectedMode`, `batteryVoltage`, `autoPeriodSeconds`, `autoStartMs`, `clientCount` and `deadlineMiss`. Globals: `g_ds_last_activity_ms` and `g_estop_latched`.
- **Joystick.** One port: `probot::io::gamepad().read()`, a `GamepadSnapshot` with up to 20 axes and 20 bools, raw W3C "standard gamepad" order, 500 ms failsafe.

### 3.3 Recommended model: "hosted" robot (prosdk owns the 10 hooks, user owns a WPILib Robot class)

prosdk ships one translation unit (or a macro, e.g. `PROSDK_ROBOT(MyRobot)`) that **defines all ten core hooks** and forwards them into a single robot instance. The robot is created lazily on the first hook, or better from a boot hook (4.5). Core keeps sole ownership of `setup()`/`loop()`, the task, deadlines and the e-stop.

**Control word synthesis** (the prosdk HAL DS layer; `HAL_RefreshDSData`/`hal::GetControlWord` read the core snapshot):

| core phase | enabled | RobotMode | opModeId | dsAttached |
|---|---|---|---|---|
| STOPPED / TRANSITION | no | UNKNOWN (selectedMode kept for info) | 0 | `g_ds_last_activity_ms != 0` |
| AUTO_INIT | no | AUTONOMOUS | id of selected auto OpMode | yes |
| AUTO_RUN | **yes** | AUTONOMOUS | same | yes |
| TELEOP_INIT | no | TELEOPERATED | id of selected teleop OpMode | yes |
| TELEOP_RUN | **yes** | TELEOPERATED | same | yes |
| e-stop latched | no, eStop bit set | - | 0 | - |

The ControlWord must be **updated by the hook adapter itself**, not read live from `state()`. Core publishes the phase before calling `init` and after `start`, so the adapter should set its own "effective" word at each hook boundary. This keeps WPILib's view consistent with the hook being executed.

**Hook mapping, TimedRobot / IterativeRobotBase**

| core hook | prosdk action |
|---|---|
| `autonomousInit` / `teleopInit` | set word = disabled + mode; run `LoopFunc()` once (gives `DisabledInit` on the first call and keeps `RobotPeriodic`) |
| `*InitLoop` | service the callback queue (`LoopFunc` at 20 ms: `DisabledPeriodic` + `RobotPeriodic`) |
| `*Start` | set word = enabled + mode; run `LoopFunc()` once (gives `DisabledExit` + `AutonomousInit`/`TeleopInit` + first periodic) |
| `*Loop` | service the callback queue (`AutonomousPeriodic`/`TeleopPeriodic` + `RobotPeriodic`) |
| `*Stop` | if `g_estop_latched`: only `MotorSafety`/HAL output kill (see 4.4), **never** full `LoopFunc` (fresh task, killed user task may have orphaned locks). Otherwise: set word = disabled/UNKNOWN; run `LoopFunc()` once (gives `*Exit` + `DisabledInit`). |

**Hook mapping, OpModeRobot.** This is the closest match: core INIT corresponds to "OpMode selected while disabled".

| core hook | prosdk action |
|---|---|
| `*Init` | opModeId = the chosen opmode for that RobotMode; `LoopFunc` creates the `OpMode` and calls `opmode->DisabledPeriodic()` |
| `*InitLoop` | `OpMode::DisabledPeriodic()` + `RobotPeriodic()` |
| `*Start` | enabled edge: `OpMode::Start()` |
| `*Loop` | `OpMode::Periodic()` + `RobotPeriodic()` |
| `*Stop` | `OpMode::End()` + destroy + `DisabledInit()` |

Core has only one auto and one teleop selection. With several opmodes per mode, prosdk needs a selector:
- (a) a core protocol extension where the DS lists `HAL_SetOpModeOptions` entries and sends an id with `cmd=mode`;
- (b) a tunable `Selectable` or `DSGamepadChooser` in INIT;
- (c) default to the first registered opmode of each mode.

**Periodic scheduling inside a core tick.** Replace `RunCallbacks(notifier)`, which blocks on a HAL notifier, with a non-blocking `PeriodicPriorityQueue::RunDue(now)`. It pops and runs every callback with `expirationTime <= now`, using the same catch-up arithmetic, and returns.
- `TimedRobot::StartCompetition()` is not used in hosted mode: there is no blocking loop.
- `AddPeriodic`, `TimesliceRobot` offsets and `OpMode::GetCallbacks()` then work at the core tick granularity.
- Users needing sub-20 ms callbacks set `#define USER_LOOP_PERIOD_MS 5` before `probot.h` (core supports this macro).
- Request core to use `vTaskDelayUntil` for drift-free 50 Hz.

**Gaps that need probot-core changes** (ask probot-core via `bp msg probot-core`; Tuna approves architecture):
1. **Always-on periodic.** Core calls nothing in STOPPED/TRANSITION, so `RobotPeriodic`, `DisabledPeriodic`, `CommandScheduler::Run`, telemetry and tunables freeze while stopped. Request an optional weak hook, e.g. `robotLoop()`, called every user-task tick in every stage, in the same task (so e-stop semantics are unchanged). Without it, prosdk would need a second task, which escapes e-stop: not acceptable.
2. **Boot hook.** WPILib constructs the robot (`RobotInit` equivalent) at program start. Core 0.4.0 deleted `robotInit`/`robotEnd` (PB-E106). A new optional weak `robotBoot()` (any name other than `robotInit`), run once in the user task before the first step, avoids constructing the robot lazily inside the first `autonomousInit` (which counts against `PROBOT_HOOK_DEADLINE_MS`). Interim: construct lazily on the first hook. The first-hook-only limitation is acceptable while gap 1 is unsolved.
3. **Extension points for telemetry/tunables/opmode list.** Core's `DriverStation::_server` (`driver_station_esp32.hpp:1035`) is private, with `max_uri_handlers=24`, so prosdk cannot add endpoints or WS frame types today. Request either a registration API for extra URI handlers / WS frame types, or core-owned frames for key/value telemetry, tunable edits and the opmode list.
4. **E-stop / enable notification.** There is no callback. prosdk can poll `probot::robot::g_estop_latched` from the HAL output layer (4.4). A small `onEmergencyStop()` weak hook run from sysloop *before* the task kill would be cleaner.
5. **Match time / DS mode.** `autoPeriodSeconds` and `autoStartMs` exist, so prosdk can compute `MatchState::GetMatchTime()` for auto; teleop has no duration, so it returns -1. Utility mode has no core equivalent.

**What to keep from `RobotBase()`:**
- MathShared setup
- telemetry/tunable warning reporters (route them to `probot::telemetry`)
- backend registration (probot backends)

Drop the NT server, the NT wait loop and ProgramStartTime. `StartRobot<T>` becomes a compile-time robot binding with no thread and no `HAL_RunMain`. `EndCompetition` is never called (the device never "exits").

---

## 4. Backend / HAL notes specific to wpilibc consumers

### 4.1 DriverStationBackend data source (HAL DS layer)
- `JOYSTICK_PORTS = 6`. Port 0 comes from `probot::io::gamepad().read()`; ports 1-5 report not connected.
- **Use the raw snapshot, not `joystick_api`.** `joystick_api` applies a deadzone and inverts Y by default. WPILib expects raw values, with forward stick negative, which matches the W3C raw value.
- **W3C to 2027 SDL/HAL gamepad layout remap** (W3C indices first):

  | W3C index | Meaning | 2027 target |
  |---|---|---|
  | buttons 0-3 | face | A/B/X/Y = 0-3 |
  | 4 / 5 | LB / RB | 9 / 10 |
  | 6 / 7 | LT / RT (bool only in core protocol) | trigger axes 4 / 5 as 0 or 1 (or -1/+1 per HAL convention; check `Gamepad`) |
  | 8 / 9 | Back / Start | VIEW 4 / MENU 6 |
  | 10 / 11 | stick clicks | 7 / 8 |
  | 12-15 | D-pad | DPAD 11-14 and the POV value |
  | 16 | Home | guide 5 |

  Axes 0-3 map 1:1.
- Fill `HAL_JoystickDescriptor`: `isGamepad = true`, `gamepadType` = standard, name "Probot DS", `supportedOutputs = 0`.
- `GetStickButtonPressed`/`Released` edge latching stays in the backend's `RefreshData()`. It must be called once per tick (`LoopFunc` already does it).
- Core already zeroes inputs outside RUN, which matches WPILib's "joysticks are still read while disabled". The values are simply neutral.
- Match info: FMS not attached, alliance `std::nullopt`, `MatchType::NONE`, empty event name. Battery: `state().batteryVoltage`.

### 4.2 Telemetry/tunables registration
In the hosted `RobotBase` constructor:
- `TelemetryRegistry::RegisterBackend("", ProbotTelemetryBackend)`
- `TunableRegistry::RegisterBackend("", ProbotTunableBackend)`

All 38 `TelemetryLoggable` classes then work unchanged. Details are in the separate `telemetry_tunables.md` report.

### 4.3 Preferences
Reimplement the static API (`Get`/`Set`/`Init` for String, Int, Double, Float, Boolean, Long; `ContainsKey`, `Remove`, `RemoveAll`, `GetKeys`) over NVS. NVS keys are limited to 15 characters, so hash or truncate long keys and store a key index.

### 4.4 Safety: threads vs. core e-stop
- Core's e-stop kills **only** the user task. Anything driven from a `Notifier` thread, the `Watchdog` thread or the `MotorSafety` thread keeps running. For example, a `Notifier` running a 200 Hz PID writing PWM would keep driving motors after e-stop.
- **Recommendation:** gate every actuator write in the prosdk HAL (PWM pulse, DIO out, digital PWM, AddressableLED optional) on "enabled":
  - enabled = core RUN phase && `!g_estop_latched` && `!deadlineMiss`
  - this is what the roboRIO/SystemCore FPGA does: outputs are disabled when the robot is disabled
  - in addition, `MotorSafety` stops all registered motors on disable
- Implement `Notifier` on `esp_timer` or a FreeRTOS task with priority below `PRIO_CTRL` (4) and pinned to core 1. The HAL gate keeps it safe.
- Make `Watchdog` thread-less for `IterativeRobotBase`: it only needs `IsExpired`/`PrintEpochs` checked synchronously at the end of `LoopFunc`. The standalone timeout callback can use `esp_timer`.
- ESP-IDF `std::thread` uses pthread with a small default stack (`esp_pthread_set_cfg`). Prefer explicit FreeRTOS tasks.

### 4.5 Motor drivers beyond FRC PWM ESCs
`PWMMotorController` produces 1000-2000 µs servo pulses. Typical probot hardware uses H-bridges with `analogWrite` at about 1 kHz plus direction pins. The existing sdk already has `bts7960b_motor_controller.hpp`, `boardoza_vnh5019_motor_controller.hpp` and `prohub/pca_motor.hpp`. Port these as `prosdk::MotorController + MotorSafety (+ TelemetryLoggable)` subclasses. Their simulation counterpart can reuse `PWMMotorControllerSim`-style SimDevice "Throttle".

---

## 5. Suggested port order (wpilibc part)
1. **System basics.**
   - HAL subset: time, DS-from-core, Notifier, DIO/PWM/ADC/PCNT/LEDC/RMT.
   - System classes: `Timer`, `Errors`, `RobotController` (subset), `Tracer`.
   - Hosted `RobotBase`/`IterativeRobotBase`/`TimedRobot` with the non-blocking queue.
   - `RobotState`, `MatchState`, `DriverStation`, `DriverStationBackend` (NT removed).
   - `GenericHID`, `Gamepad`, generated HIDs, `EventLoop`, `BooleanEvent`.
2. **Actuators and sensors.**
   - `MotorController`, `MotorSafety`, `PWM`, `PWMMotorController`, drivers' PWM ESCs, new `Servo`, H-bridge drivers.
   - `Encoder`, `DigitalInput`/`DigitalOutput`, `AnalogInput`, `AnalogPotentiometer`, `DutyCycle`/`DutyCycleEncoder`, `AnalogEncoder`, `EdgeCounter`/`Tachometer`, `I2C`, `SerialPort`, `ADXL345_I2C`, `SharpIR`, `AddressableLED`, `LEDPattern`, drive classes.
3. **Opmodes and display.** `OpModeRobot`/`OpMode`/`PeriodicOpMode`, `TimesliceRobot`, `DriverStationDisplay` (to `probot::telemetry`), `DSGamepadChooser`.
4. **Backends.** Telemetry and tunable probot backends, `Preferences` (NVS), `Field2d`, `Mechanism2d`, `SysIdRoutineLog` (DataLog stub, file or stream).
5. **Simulation.**
   - Physics sims: pure, so host and device are both possible.
   - HAL-sim wrappers plus `DriverStationSim`/`SimHooks` for host tests.
   - Port the relevant wpilibc tests: `TimedRobotTest`, `OpModeRobotTest`, `OpModeLifecycleTest`, `TimerTest`, `WatchdogTest`, `NotifierTest`, `BooleanEventTest`, `EventLoopTest`, `GenericHIDTest`, `JoystickTest`, `LEDPatternTest`, `DifferentialDriveTest`, `MecanumDriveTest`, and the `simulation/*SimTest` files.
6. **Optional / later.** `OnboardIMU` backend, `CAN` (TWAI), `DataLogManager` to SD, GPIO `PneumaticsBase`, `NetworkBooleanEvent` via tunables.
