# WPILib 2027 HAL: investigation and ESP32-S3 backend design for prosdk

Source: `allwpilib@b68effca`, `hal/` (paths below are relative to `allwpilib/hal/src/main/native/` unless they say otherwise).
Scope: what the HAL is, what wpilibc actually uses from it, and a concrete ESP32-S3 backend that sits on top of probot-core 0.4.0.

---

## 0. TL;DR

* The 2027 HAL is much smaller than the roboRIO-era HAL. There is **no SPI, Relay, AnalogOutput, AnalogTrigger, AnalogGyro, Interrupts, Accelerometer (built-in) or `HAL_GetFPGATime`**. There are **317 public `HAL_*` C functions** across 32 public headers, plus **430 `HALSIM_*` functions** in 20 `simulation/*Data.h` headers.
* Time comes from `HAL_GetMonotonicTime()`, in **nanoseconds**, using the same time base as `wpi::util::Now()`. A Notifier is a **wpiutil signal object**: users call `WPI_WaitForObject(notifierHandle)`. There is no `HAL_WaitForNotifierAlarm`.
* The Driver Station sits behind an **abstract C++ interface, `wpi::hal::MrcLibDs`** (`include/wpi/hal/cpp/MrcLibDs.hpp`, 27 virtual methods). All `HAL_*` DS functions in `cpp/FIRSTDriverStation.cpp` forward to it. **A single `ProbotCoreDs : MrcLibDs` class is the plug-in point for probot-core.** The sim's `MrcLibDsSimImpl` (`sim/DriverStation.cpp`, 559 lines) is a ready template: it has the triple-buffered joystick cache and the new-data event vector.
* The best template for real IO is **the systemcore backend, not the sim one**. Systemcore treats every IO pin as a "SmartIO" channel that is switched into one mode (DIO, PWM, encoder, counter, duty cycle, analog or ALED) through one `DigitalHandleResource<…, SmartIo, N>`. The 7 IO files (DIO/PWM/Encoder/Counter/DutyCycle/AnalogInput/AddressableLED, about 1,380 lines) touch vendor code at only **15 call sites**. The hardware lives in `systemcore/SmartIo.cpp` (176 lines over `mrclib`). **Port plan: copy those 7 files and rewrite `SmartIo.cpp` on ESP-IDF drivers.**
* Reuse the **CAN device stack from systemcore unchanged**: `CANAPI.cpp`, `REVPH.cpp`, `REVPDH.cpp`, `CTREPCM.cpp`, `CTREPDP.cpp`, `PowerDistribution.cpp` and `rev/*Frames` (about 15k lines, which are protocol code, not hardware code). Rewrite only `systemcore/CAN.cpp` (652 lines, SocketCAN plus libuv) on TWAI.
* Take the shared code from `cpp/` (handles, ErrorHandling, Main, FIRSTDriverStation, UsageReporting) as-is. `sim/` supplies the Notifier, Extensions (static-only), Threads, the HAL.cpp skeleton, and the whole mock-data layer, which the host-test build needs.
* Build **two backends**. `hal/esp32` is the target. `hal/sim` is the unmodified WPILib sim backend, for host `g++` tests and physics simulation. Both sit behind the same `prosdk/hal/*.h` headers.

---

## 1. Layout of `hal/`

| Dir | Files | Lines | Role |
|---|---:|---:|---|
| `include/wpi/hal/*.h,*.hpp` | 32 public headers + 7 `handles/` + 2 `cpp/` | 6,197 | C API (+ thin C++ helpers) |
| `include/wpi/hal/simulation/*.h` | 25 | – | `HALSIM_*` data/callback API (430 fns) |
| `cpp/` (shared by all backends) | 7 + `handles/HandlesInternal.cpp` + `mrclib/` (2) | 1,644 | DS front-end (`FIRSTDriverStation.cpp`), DashboardOpMode (NT), ErrorHandling, Main, UsageReporting, MrcLib DS/Alert (systemcore only) |
| `cpp/jni/` | ~50 | – | Java JNI. **Drop.** |
| `sim/` | 37 + `mockdata/` 37 | 4,537 + 3,618 | Desktop simulation backend |
| `systemcore/` | 40 + `mockdata/` 19 stubs + `rev/` 4 | 7,288 + 806 + 11,519 | Real hardware (SystemCore: Linux, SocketCAN, mrclib SmartIO, NT system server) |

There is **no athena/roboRIO backend** in 2027; systemcore is the only real-hardware backend.

### Handle machinery (`include/wpi/hal/handles/`, 976 lines total)
* `HandlesInternal.hpp`: a handle is 32 bits laid out as `[type:8][version:8][index:16]`. `HAL_HandleEnum` has 23 types (DIO=1 (`HANDLE_TYPE_HAL_BASE`), PORT, NOTIFIER, ANALOG_INPUT, PWM, DIGITAL_PWM, COUNTER, FPGA_ENCODER, ENCODER, COMPRESSOR, SOLENOID, VENDOR, SIMULATION_JNI, CAN, SERIAL_PORT, DUTY_CYCLE, ADDRESSABLE_LED, CTRE_PCM, CTRE_PDP, REV_PDH, REV_PH, CAN_STREAM, ALERT). The version check is skipped when `__FIRST_SYSTEMCORE__` is defined; keep it enabled on ESP32 because it is cheap.
* Resource templates:
  * `DigitalHandleResource<THandle,TStruct,size>`: fixed array, **one mutex per slot**, `Allocate(index, enum, name)` returns `std::expected<pair<handle, shared_ptr>, HAL_Status>`, with error strings for "previously allocated".
  * `IndexedHandleResource` / `IndexedClassedHandleResource`: fixed array indexed by channel.
  * `LimitedHandleResource` / `LimitedClassedHandleResource`: N slots, first free one is used.
  * `UnlimitedHandleResource`: a vector of `shared_ptr`, used for Notifier, CAN streams and SerialPort.
* Dependencies: `std::expected` (C++23; GCC 14 with `gnu++2b` is fine), `std::shared_ptr`, and `wpi::util::mutex` (`std::mutex`). **No RTTI is needed.** On ESP-IDF a `std::mutex` lazily creates a FreeRTOS mutex of about 100 B on first lock, so even a 49-slot table costs at most about 5 KB.
* `ErrorHandling.cpp` uses `thread_local` storage for `HAL_GetLastError`. ESP-IDF supports TLS. A `thread_local` with a non-trivial destructor (a `std::string`) inside FreeRTOS tasks still needs to be verified on target; a static fallback is acceptable.

---

## 2. HAL API groups (public C functions; counts are unique `HAL_*` names per header)

| Group (header) | # fns | Semantics / notes | wpilibc user |
|---|---:|---|---|
| **HAL core** (`HAL.h`) | 16 | `HAL_Initialize/Shutdown`, `GetLastError/GetErrorMessage`, `GetSerialNumber`, `GetComments`, `GetTeamNumber`, `GetRuntimeType` (enum `SYSTEMCORE`/`SIMULATION` only), `GetSystemActive`, `GetBrownedOut`, `GetCommsDisableCount`, **`GetMonotonicTime` (ns)**, `GetRSLState`, `GetSystemTimeValid`, `SimPeriodicBefore/After` | RobotBase, RobotController, OpModeRobot |
| **Ports** (`Ports.h`) | 19 | `HAL_GetNum*`: CanBuses, SmartIo, AnalogInputs, Counters, DigitalChannels, PWMChannels, DigitalPWMOutputs, Encoders, Interrupts, CTREPCM modules/solenoid channels, CTREPDP modules/channels, REVPDH modules/channels, REVPH modules/channels, DutyCycles, AddressableLEDs | SensorUtil, CAN |
| **DIO** (`DIO.h`) | 18 | Init/Free/Check/SetSimDevice, Set/Get, Set/GetDirection, `Pulse`, `PulseMultiple`, `IsPulsing`, `IsAnyPulsing`, **DigitalPWM** (`Allocate/Free`, global `SetDigitalPWMRate`, per-generator `DutyCycle`, `PPS`, `OutputChannel`). Systemcore stubs DigitalPWM and Pulse with `HAL_HANDLE_ERROR`. | DigitalInput/Output |
| **PWM** (`PWM.h`) | 7 | Init/Free/Check/SetSimDevice, `Set/GetPWMPulseTimeMicroseconds`, `SetPWMOutputPeriod(0=5 ms, 1\|2=10 ms, 3=20 ms)`. Pulse time 0 means output disabled. Servo/ESC-style only; there is no duty-cycle API. | PWM, PWMMotorController |
| **Encoder** (`Encoder.h`) | 18 (+2 enums) | `InitializeEncoder(a,b,reverse,1x/2x/4x)`, Get/GetRaw/EncodingScale/Reset/Stopped/Direction/Distance/Rate, **`SetEncoderRateWindow(ms)`**, DistancePerPulse, ReverseDirection, FPGAIndex, DecodingScaleFactor, EncodingType. There is no index-pulse or min-rate API in 2027; rate is window-based. | Encoder |
| **Counter** (`Counter.h`) | 8 | Single-channel edge counter: Init(channel, risingEdge), SetEdgeConfiguration, SetRateWindow, Reset, Get, GetRate, GetStopped | EdgeCounter, Tachometer |
| **AnalogInput** (`AnalogInput.h`) | 9 | Init/Free/CheckModule/CheckChannel/SetSimDevice, `GetAnalogValue` (12-bit raw), `GetAnalogVoltage`, VoltsToValue/ValueToVolts (systemcore: `raw/4095*3.3`). No averaging, oversampling or accumulator. | AnalogInput, AnalogEncoder, AnalogPotentiometer, SharpIR |
| AnalogOutput / AnalogTrigger | **0** | Removed in 2027 | – |
| **DutyCycle** (`DutyCycle.h`) | 6 | Init/Free/SetSimDevice, `GetDutyCycleFrequency`, `GetDutyCycleOutput` (0..1), `GetDutyCycleHighTime` (ns) | DutyCycle, DutyCycleEncoder |
| **Notifier** (`Notifier.h` + `.hpp`) | 7 | Create/SetName/Destroy, **`SetNotifierAlarm(alarmTime ns, interval ns, absolute, ack)`**, `CancelNotifierAlarm(ack)`, `AcknowledgeNotifierAlarm`, `GetNotifierOverrun`. The handle is a wpiutil signal object; firing it means `wpi::util::SetSignalObject(handle)`. | TimedRobot, OpModeRobot, Notifier, Watchdog, PeriodicPriorityQueue |
| **DriverStation** (`DriverStation.h` + `Types`) | 30 + 13 inline helpers | SendError, SetPrintErrorImpl, SendConsoleLine, SendProgramCrash, **Get(Uncached)ControlWord** (64-bit: 56-bit opmode hash, 2-bit robot mode, enabled, estop, FMS, DS-attached), **SetOpModeOptions**, AllianceStation, Joystick Axes/POVs/Buttons/Touchpads/AllData/Descriptor/IsGamepad/GamepadType/SupportedOutputs/Name, SetJoystickRumble/Leds, GameData, MatchTime, MatchInfo, GetOutputsEnabled, RefreshDSData, Provide/RemoveNewDataEventHandle, ObserveUserProgram(Starting), WriteDisplayAnsi. Limits: 6 joysticks, 12 axes, 8 POVs, 64 buttons, 2 touchpads × 2 fingers. | DriverStationBackend, GenericHID, OpModeRobot, DriverStationModeThread, DriverStationDisplay |
| **AddressableLED** (`AddressableLED.h`) | 5 | Init(channel), Free, `SetAddressableLEDStart`, `Length`, and **global** `SetAddressableLEDData(start, len, colorOrder, data)`: one shared buffer of `HAL_ADDRESSABLE_LED_MAX_LEN = 1024` LEDs. Each channel is a (start, length) window into it; 6 colour orders. | AddressableLED |
| **CAN** (`CAN.h`) | 6 | `HAL_CAN_SendMessage(bus, id, msg, periodMs)` (periodic repeat with 0 = once and -1 = stop), ReceiveMessage (latest by ID), GetCANStatus, Open/Close/ReadStreamSession. Message holds 64 data bytes plus FD flags. | CAN, RobotController |
| **CANAPI** (`CANAPI.h`) | 9 | Device-oriented: InitializeCAN(bus, mfr, devId, devType), CleanCAN, WriteCANPacket(+Repeating, RTR), StopCANPacketRepeating, ReadCANPacketNew/Latest/Timeout. Implemented generically on top of `CAN.h`. | CAN (wpilibc class) |
| **I2C** (`I2C.h`) | 5 | Initialize(port), Transaction, Write, Read, Close. Ports `HAL_I2C_PORT_0/1`. | I2C (+ADXL345_I2C) |
| SPI | **0** | Removed in 2027 | – |
| **SerialPort** (`SerialPort.h`) | 20 | Init(ONBOARD/MXP/USB_1/USB_2), InitDirect(name), GetFD, baud/data/parity/stop/write-mode/flow/timeout, Enable/DisableTermination, read/write buffer size, BytesReceived, Read, Write, Flush, Clear, Close | SerialPort |
| **Power** (`Power.h`) | 9 | `GetVinVoltage`, 3V3 user rail (voltage, current, active, faults, enable), ResetUserCurrentFaults, `SetBrownoutVoltages`, `GetCPUTemp` | RobotController |
| **PowerDistribution** (`PowerDistribution.h`) | 26 | CTRE PDP / REV PDH over CAN: init(bus, module, type), channel currents, totals, temperature, voltage, energy, switchable channel, version, faults/sticky faults, stream API | PowerDistribution |
| **CTREPCM** (`CTREPCM.h`) | 22 | Compressor, closed loop, pressure switch, current, faults, solenoid mask, one-shot | PneumaticsControlModule |
| **REVPH** (`REVPH.h`) | 26 | Compressor config (disabled/digital/analog/hybrid), analog pressure, voltages, solenoids, one-shot, faults, version | PneumaticHub |
| **IMU** (`IMU.h`) | 9 | `GetIMUAcceleration`, `GyroRates`, Euler angles (Flat/Landscape/Portrait), Quaternion, Yaw (3 mountings). Systemcore has an onboard IMU. | OnboardIMU |
| **Threads** (`Threads.h`) | 4 | Get/Set (current) thread priority; the handle is `pthread_t*`. Sim backend: no-op. | Threads, Notifier |
| **SimDevice** (`SimDevice.h/.hpp`) | 27 (+C++ wrappers) | Create/Free device, typed values (bool/double/enum/int/long). **Systemcore returns 0 handles.** wpilibc devices treat a non-zero handle as "simulated", so the hardware backend must return 0. | 9 wpilibc files |
| **Main** (`Main.h`) | 4 | SetMain/HasMain/RunMain/ExitMain: lets a host GUI own the main thread | RobotBase::StartRobot |
| **Extensions** (`Extensions.h`) | 6 | `LoadOneExtension`/`LoadExtensions` (dlopen of `HALSIM_EXTENSIONS`), RegisterExtension/Listener, OnShutdown | RobotBase (via Initialize) |
| **UsageReporting** (`UsageReporting.h/.hpp`) | 4 | ReportUsage(resource, data), PublishCanVersion, PublishVersion, PublishWpilibVersion | RobotBase |
| **SystemServer** (`SystemServer.h`) | 1 | `HAL_GetSystemServerHandle()` returns an `NT_Inst` connected to the localhost system daemon on :6810. **ExpansionHub classes use it** (`/rhsp/<usb>/…` NT topics). | SystemServer, ExpansionHub* |
| **Value** (`Value.h`) | 5 inline | `HAL_Value` tagged union | SimDevice |
| **Alerts** | – | Not a HAL API. Alerts are `WPI_CreateAlert…` in wpiutil (`wpi/util/Alert.h`) with a pluggable `WPI_AlertBackend` (11 function pointers). systemcore installs `MrcLibAlert`. | wpilibc Alert |
| **DashboardOpMode** (`DashboardOpMode.hpp`, `cpp/DashboardOpMode.cpp`) | 5 C++ | When the DS cannot select opmodes, publishes NT "String Chooser" topics for Auto/Teleop/Utility and returns the selected opmode id. **Needs ntcore.** | via MrcLibDs |

Totals: **317 `HAL_*`** names in public headers (including inline helpers and the `HAL_ENUM` macro) and **430 `HALSIM_*`**: DriverStationData 96, RoboRioData 46, EncoderData 36, CTREPCMData 29, REVPHData 29, DIOData 23, SimDeviceData 21, PowerDistributionData 20, MockHooks 20, AddressableLEDData 18, PWMData 15, DigitalPWMData 15, DutyCycleData 15, CanData 13, AnalogInData 11, IMUData 10, I2CData 9, NotifierData 3, Reset 1.

Errors (`Errors.h`): about 22 codes, for example `HAL_RESOURCE_IS_ALLOCATED -1029`, `HAL_RESOURCE_OUT_OF_RANGE -1030`, `HAL_HANDLE_ERROR -1098`, `HAL_CAN_TIMEOUT -1154`, `HAL_SIM_NOT_SUPPORTED -1155`, `HAL_USE_LAST_ERROR -1156`, plus `HAL_ERR_CANSessionMux_*`.

### Port counts in the existing backends (`*/PortsInternal.hpp`)
| Constant | sim | systemcore |
|---|---:|---:|
| CAN buses | 5 | 25 |
| SmartIO | 6 | 6 |
| Digital channels | 31 | 6 (= SmartIO) |
| PWM channels | 20 | 6 |
| Analog in | 8 | 6 |
| Encoders | 8 | 3 (pairs 0/1, 2/3, 4/5) |
| Counters | 8 | 0 (uses SmartIO) |
| DigitalPWM | 6 | 0 |
| DutyCycles | 6 | 0 (uses SmartIO) |
| AddressableLEDs | 6 | 6 |
| I2C | 2 | 2 |
| Interrupts | 8 | 0 |
| CTRE PCM/PDP, REV PDH/PH modules | 63 each | 63 each |

---

## 3. What wpilibc calls

`grep -rhoE '\bHAL_[A-Z]\w*\s*\(' wpilibc/src/main/native` finds **222 distinct `HAL_*` functions**; the full list is in `scratchpad/hal_used.txt`. Some `Free*` functions are used only as template arguments to `wpi::hal::Handle<…, &HAL_FreeX>` (for example `HAL_FreePWMPort`, `HAL_FreeEncoder`, `HAL_FreeDIOPort`), so the grep under-counts. wpilibc also calls **259 distinct `HALSIM_*`** functions, all from `wpilibc/.../simulation/*Sim.cpp`.

Files that call the HAL (37): `driverstation/{DriverStationDisplay,GenericHID,internal/DriverStationBackend}`, `framework/{IterativeRobotBase,OpModeRobot,RobotBase,TimedRobot}`, `hardware/bus/{CAN,I2C,SerialPort}`, `hardware/counter/{EdgeCounter,Tachometer}`, `hardware/discrete/{AnalogInput,DigitalInput,DigitalOutput,PWM}`, `hardware/imu/OnboardIMU`, `hardware/led/AddressableLED`, `hardware/motor/MotorSafety`, `hardware/pneumatic/{PneumaticHub,PneumaticsBase,PneumaticsControlModule}`, `hardware/power/PowerDistribution`, `hardware/rotation/{DutyCycle,Encoder}`, `internal/{DriverStationModeThread,PeriodicPriorityQueue}`, `simulation/{AddressableLEDSim,CallbackStore,DriverStationSim,REVPHSim,SimDeviceSim,SimHooks}`, `system/{Errors,Notifier,RobotController,SystemServer,Threads,Watchdog}`, `util/SensorUtil`.

**Every HAL group is used by wpilibc.** The ones used most are Joystick/DS (buttons 10×, axes 6×, POV 5×, MatchInfo 5×), Notifier (create/alarm/destroy 4–5×), CTREPCM closed loop, and IMU.

Other modules: commandsv2 only in tests; `romiVendordep` and `xrpVendordep` 4 files each (`HAL_SimDevice`); `simulation/halsim_*` 41 files (desktop extensions: ws_server/ws_client/ds_socket/gui/xrp; not for the target). `commandsv3`, `wpimath`, `ntcore`, `telemetry`, `tunables`, `datalog`, `epilogue-runtime` and `drivers` do **not** call the HAL directly.

Physics simulation classes (`DCMotorSim`, `ElevatorSim`, `FlywheelSim`, `SingleJointedArmSim`, `DifferentialDrivetrainSim`, `ADXL345Sim`, `AnalogEncoderSim`, `DutyCycleEncoderSim`, `GamepadSim`, `JoystickSim`, `GenericHIDSim`, `SolenoidSim`, `DoubleSolenoidSim`, `PneumaticsBaseSim`, `SharpIRSim`, `PWMMotorControllerSim`) make **0 HAL calls** and port with wpimath alone. The device Sim wrappers (`PWMSim`, `EncoderSim`, `DIOSim`, `DriverStationSim` with 66 calls, `RoboRioSim` with 41, `REVPHSim`, `CTREPCMSim`, and others) need the `HALSIM_*` symbols.

---

## 4. Backend reuse analysis

| Piece | Source | Reuse on ESP32 | Notes |
|---|---|---|---|
| Handles | `include/.../handles/*`, `cpp/handles/HandlesInternal.cpp` | **As-is** | Rename namespace only |
| ErrorHandling, Main, UsageReporting, FIRSTDriverStation | `cpp/*.cpp` | **As-is** | `DefaultSendErrorImpl` uses `std::format`; route print output to probot telemetry or Serial |
| DashboardOpMode | `cpp/DashboardOpMode.cpp` | As-is **only if ntcore is ported**; otherwise replace with probot-core opmode chooser (see section 6) | |
| HAL.cpp | `sim/HAL.cpp` (366) as skeleton | Edit | Remove the Windows timer code and `setlinebuf`; real values for serial number, brownout, RSL and so on (section 5) |
| HALInitializer | `sim/HALInitializer.*` | As-is (list edited) | |
| Notifier | `sim/Notifier.cpp` (363) | **Almost as-is** | SafeThread plus `condition_variable::wait_for`. On ESP-IDF pthread it has 1-tick resolution (1 ms at Arduino's `CONFIG_FREERTOS_HZ=1000`), which is enough for 20 ms loops. Phase 2: replace with `esp_timer` one-shot per alarm, whose callback runs in the esp_timer task and calls `SetSignalObject`. Drop Pause/Step timing (sim only). |
| Extensions | `sim/Extensions.cpp` | Edit | No `dlopen`. `HAL_LoadExtensions()` returns 0 and `LoadOneExtension` returns -1. Keep `RegisterExtension` and listeners for statically linked add-ons. |
| Threads | `systemcore/Threads.cpp` (pthread `sched_param`) | Rewrite (~60 lines) | See section 5 |
| DS | `sim/DriverStation.cpp` `MrcLibDsSimImpl` | **Template for `ProbotCoreDs`** | Keep `JoystickDataCache` triple buffering and `EventVector newDataEvents`; source data from probot-core |
| IO: DIO, PWM, Encoder, Counter, DutyCycle, AnalogInput, AddressableLED | `systemcore/*.cpp` (194+158+445+152+137+131+167) | **Copy and edit 15 mrclib touchpoints** | Same SmartIO "one pin, one mode" model; change `NUM_SMART_IO`, pair validation and mode enum |
| SmartIo hardware layer | `systemcore/SmartIo.{hpp,cpp}` (176) | **Rewrite on ESP-IDF** | This is where LEDC, MCPWM, PCNT, ADC and RMT live |
| CAN core | `systemcore/CAN.cpp` (652: SocketCAN, CAN FD, libuv timers, stream sessions) | Rewrite on TWAI (~400 lines) | Keep periodic-send map, latest-by-ID receive cache, stream sessions and NotAllowed/NoToken semantics; FreeRTOS RX task plus esp_timer for periodics |
| CANAPI, REVPH, REVPDH, CTREPCM, CTREPDP, PowerDistribution, rev/*Frames | `systemcore/` | **As-is** (behind `PROSDK_HAL_CAN`) | Pure protocol on top of `CAN.h`. Note `CANAPI.cpp` includes `wpi/util/DenseMap.hpp`. |
| I2C | `systemcore/I2C.cpp` (Linux i2c-dev) | Rewrite (~80 lines) | Arduino `TwoWire` |
| SerialPort | `systemcore/SerialPort.cpp` (termios) | Rewrite (~200 lines) | Arduino `HardwareSerial` |
| Power | `systemcore/Power.cpp` (mrclib) | Rewrite (~80 lines) | probot battery + temperature sensor |
| IMU | `systemcore/IMU.cpp` (mrclib) | Rewrite | External I2C IMU driver |
| SimDevice | `systemcore/SimDevice.cpp` (53, returns 0) | **As-is** | Required for real-hardware semantics |
| HALSIM mock data | `systemcore/mockdata/*` (19 stub files, 806 lines, `HAL_SIMDATAVALUE_STUB_CAPI`) | **As-is** for target | Device `*Sim` classes link and do nothing. Optional "mirror" mode: link the real `sim/mockdata` (3,618 lines) so HAL state can be streamed to the browser DS for debugging. |
| SystemServer | `sim/SystemServer.cpp` (returns 0) | As-is | ExpansionHub cannot work and should be dropped. Optionally map to the default NT instance later. |
| Host test backend | entire `sim/` + `sim/mockdata` | **As-is** | Lets prosdk unit tests and simulation run under g++ on the PC |

Rough effort for the ESP32 backend: about 4.5k lines reused verbatim, about 1.4k lines of systemcore IO copied with small edits, about 15k lines of CAN protocol reused verbatim, and **about 1.5–2k lines newly written** (SmartIo for ESP32, TWAI CAN, I2C, Serial, Power, IMU, Threads, ProbotCoreDs, HAL.cpp edits).

External dependencies the HAL brings in from wpiutil: `mutex`, `condition_variable`, `spinlock`, `Synchronization` (signal objects and events), `SafeThread`, `priority_queue`, `SmallVector`, `DenseMap`, `StringMap`, `UidVector`, `EventVector`, `timestamp`, `print`, `string` (`WPI_String`), and `Alert`. All of these must be in the wpiutil port. Not needed: `fs.hpp` (Extensions) and `wpinet` uv (systemcore CAN, being rewritten).

---

## 5. ESP32-S3 backend design (`prosdk/hal/esp32/`)

### 5.1 Channel model
Use **channel number = GPIO number** for every pin-based API (DIO, PWM, Encoder A/B, Counter, DutyCycle, AnalogInput, AddressableLED). This matches what Arduino users already expect (`DigitalInput(4)` means GPIO4). It also matches systemcore's model: one `DigitalHandleResource<HAL_DigitalHandle, SmartIo, 49>` (`NUM_SMART_IO = 49`, GPIO0–48), where each pin has exactly one mode.

* Validity mask: `GPIO_IS_VALID_GPIO(n)`, minus pins that are always reserved: 26–32 (SPI flash), 33–37 on octal-PSRAM modules (R8/R16; Kconfig `CONFIG_SPIRAM_MODE_OCT`), 19/20 (USB D-/D+ when USB-CDC console is used) and 43/44 (UART0).
* Pre-allocated with `previousAllocation = "probot-core"` so users get the standard "Resource already allocated" message: `NEOPIXEL_PIN` (default 3), `PROBOT_RSL_PIN`, `PROBOT_ESTOP_ENABLE_PIN`, `PROBOT_BATTERY_ADC_PIN`, and the INA SDA/SCL pins when defined.
* Strapping pins 0/3/45/46 are allowed, with a warning. (Built differently: 0/45/46 are allowed without a runtime warning, 3 is refused as `NEOPIXEL_PIN`; the user docs warn. See design/hal-esp32.md, Channel model.)
* Optional *virtual channel ranges* for off-chip IO through a "port provider" interface. The legacy sdk already has ProHub: PCA9685 I2C PWM on 0x40/0x41 and an encoder co-processor over UART at 921600 baud (`src/probot/prohub/*`). For example, PWM 100–115 would go to PCA9685 #0 and encoders 100–107 to ProHub frames. The provider decides per range, so the HAL front-end files stay the same.

### 5.2 Group → peripheral mapping

| HAL group | ESP32-S3 peripheral / API | Capacity to expose | Details |
|---|---|---|---|
| **Time** `HAL_GetMonotonicTime` | `esp_timer_get_time()` (µs, 64-bit SYSTIMER) × 1000 | – | Also call `wpi::util::SetNowImpl` so `Now()` uses the same base |
| **Notifier** | sim `Notifier.cpp` on std::thread/condvar (phase 1), then `esp_timer` (phase 2) | unlimited (`UnlimitedHandleResource`) | Overrun accounting is unchanged. Notifier thread priority must be above the robot thread. |
| **DIO** | `gpio_config` / `gpio_set_level` / `gpio_get_level` | any valid GPIO | `HAL_Pulse` via `esp_timer` one-shot (µs) or RMT; `PulseMultiple` loops over the mask |
| **DigitalPWM** (6) | **LEDC** low-speed, one timer shared by all generators (matches the HAL's global `SetDigitalPWMRate`) | 6 of the 8 LEDC channels; leave 2 for user `analogWrite`/servo patterns | 14-bit resolution at ≤4.8 kHz, 10-bit at ≤78 kHz. `PPS` mode uses a duty with a 1 Hz rate (or RMT). |
| **PWM** (servo/ESC µs pulses) | **MCPWM**: 2 groups × 3 timers/operators × 2 generators = **12 outputs**; 1 MHz resolution gives 1 µs steps; period per operator 5/10/20 ms | 12 simultaneous | Two generators on one operator share a period, so allocate operators per period value. Fallback: LEDC with a separate timer per period. |
| H-bridge motor PWM (BTS7960, VNH5019, TB6612) | LEDC or MCPWM duty | – | **There is no HAL API for this** (2027 PWM is pulse-µs only). Add `prosdk` motor-controller classes on DigitalPWM plus DIO, or add a small prosdk HAL extension `HAL_SetPWMDutyCycle` (MCPWM). Decide together with the motors report. |
| **Encoder** (quadrature) | **PCNT**: the S3 has **4 units** × 2 channels, giving x4 decode in hardware, a glitch filter, and 16-bit counts extended to 32-bit by the ±limit watch-point ISR | **4 hardware**, plus up to about 4 software (GPIO any-edge ISR, ≤ about 20 kHz edges) | Rate: a 1 kHz `esp_timer` sampler stores (t, count) in a ring and computes rate as Δcount/Δt over `SetEncoderRateWindow` (the 2027 API is window-based, so this fits). `Stopped` means no change within the window. Pair rule: any two GPIOs, replacing systemcore's `IsValidChannelPair`. |
| **Counter** | PCNT unit (shared pool with encoders) or GPIO ISR counter | pooled, 4 hardware total | Edge selection through PCNT channel edge actions |
| **DutyCycle** input | **MCPWM capture**: 3 channels per group, so **6**, with 80 MHz timestamps on both edges | 6 | High time in ns, period, output = high/period. Fallback: RMT RX (4 channels) or GPIO ISR plus `esp_timer`. |
| **AnalogInput** | **ADC1 oneshot** (`adc_oneshot_*`) plus `adc_cali` curve fitting; 12 dB attenuation (about 0–3.1 V) | **GPIO1–10 (10 channels)**. **ADC2 (GPIO11–20) is not allowed** because the WiFi AP is always on. | `GetAnalogValue` is 12-bit raw and `GetAnalogVoltage` is calibrated mV/1000, matching the systemcore conversion. Share the ADC unit handle with probot-core's battery `analogReadMilliVolts` by using Arduino's `analogRead*` underneath, or a single mutex-guarded oneshot handle. |
| **AddressableLED** | **RMT TX** (4 channels on S3), WS2812 encoder (`led_strip` component or a custom bytes encoder); DMA is available on one TX channel | **3 channels**, because probot-core's builtin NeoPixel (Adafruit, RMT) takes one | Keep the HAL's single 1024×3 B global buffer; colour order is converted the systemcore way |
| **CAN** | **TWAI** (one controller, CAN 2.0B up to 1 Mbit/s) plus an external 3.3 V transceiver (SN65HVD230 or similar) | `NUM_CAN_BUSES = 1` (0 when disabled) | Reject `HAL_CAN_FD_*` flags. **Must also send the FRC "universal heartbeat" frame** (roboRIO/systemcore normally do this outside the HAL) with the enabled bit from the DS state, or REV/CTRE devices stay disabled. Verify the exact frame layout against frc-docs or vendor docs before shipping. |
| **I2C** | Arduino `TwoWire` `Wire` (port 0) and `Wire1` (port 1); Transaction = `write` + `endTransmission(false)` + `requestFrom` | 2 | Sharing Wire with probot-core's INA battery reader is required: same pins and `setTimeOut`. Default pins come from the kit config (ProHub uses SDA1/SCL2). |
| **SPI** | – | – | Not in the 2027 HAL. Offer a separate `prosdk::SPI` over Arduino `SPIClass` (SPI2/SPI3) only if a driver needs it. |
| **SerialPort** | Arduino `HardwareSerial` | ONBOARD to `Serial1`, MXP to `Serial2`, USB_1 to USB-CDC `Serial` (console, read-only warning); `InitializeSerialPortDirect("UART1:rx=…,tx=…")` | Termination, timeout, write mode (flush on write) and buffer sizes in software; `HAL_GetSerialFD` returns -1 |
| **Power** | `GetVinVoltage` reads `probot::robot::state().batteryVoltage` (or `probot::io::battery`); 3V3 rail: nominal 3.3, current 0, always active, enable is a no-op (or GPIO-switched); `GetCPUTemp` uses `temperature_sensor` driver | – | `SetBrownoutVoltages` and `GetBrownedOut` become a **software brownout**: Vin below threshold disables outputs (section 5.4) |
| **PowerDistribution** | CTRE PDP / REV PDH via TWAI (as-is). Optional new type `PROBOT_INA`: a "virtual PD" with total current and voltage from the INA219/226 | – | `HAL_POWER_DISTRIBUTION_AUTOMATIC` probes CAN first |
| **CTREPCM / REVPH** | via TWAI (as-is) | 63 modules | Compile-time optional |
| **IMU** | External I2C IMU (BNO055 or BNO08x for fused output; MPU6050/ICM-42688 with Mahony/Madgwick in a 200 Hz task); fills `HAL_Quaternion`/Euler/rates with ns timestamps | 1 | Flat/Landscape/Portrait come from a fixed rotation of the quaternion |
| **Threads** | FreeRTOS. `SetCurrentThreadPriority(p)` calls `vTaskPrioritySet(NULL, map(p))`. For another thread, keep a `pthread_t → TaskHandle_t` registry filled by a prosdk thread helper, or return `HAL_THREAD_PRIORITY_ERROR`. | – | Map WPILib RT 1..99 onto FreeRTOS 5..18, staying below `esp_timer` (22) and WiFi (23). `std::thread` stack size and core come from `esp_pthread_set_cfg`. |
| **HAL core** | `GetSerialNumber` = eFuse MAC; `GetTeamNumber` = `#define`/NVS (default 0); `GetComments` = sketch name; `GetRuntimeType` = SYSTEMCORE (so `RobotBase::IsReal()` is true); `GetRSLState` = probot RSL blink state; `GetSystemActive` = not e-stopped; `GetCommsDisableCount` = DS-timeout count; `GetSystemTimeValid` = false until the browser DS sends wall-clock time (core feature request) | – | Alternative to SYSTEMCORE: add `HAL_RUNTIME_ESP32` and patch `RobotBase` (2 lines) |
| **UsageReporting** | no-op, or a one-time telemetry line | – | |
| **Extensions / Main / SimDevice / SystemServer** | static / as-is / returns 0 / returns 0 | – | |
| **Alerts** | Install a `WPI_AlertBackend` that forwards to the probot browser DS (core feature request), or keep the default wpiutil backend (NT publish, if ntcore is present) | – | |

### 5.3 DriverStation through probot-core (`ProbotCoreDs : wpi::hal::MrcLibDs`)
Install it with `wpi::hal::ForceDsInstance()`, or provide `GetDefaultDriverStationImpl()` as systemcore's `DriverStation.cpp` does (12 lines). Copy the cache logic from `MrcLibDsSimImpl`.

**DS pump task** (core 0, prio about 5, every 20 ms and whenever `gamepad().read().seq` or `state().read().seq` changes):
1. `s = probot::robot::state().read()` and `g = probot::io::gamepad().read()`. The gamepad read already applies the 500 ms input-timeout zeroing.
2. **Control word** via `HAL_MakeControlWord(opModeId, robotMode, enabled, eStop, fms=false, dsAttached)`:
   * `robotMode`: `AUTO_*` maps to AUTONOMOUS and `TELEOP_*` to TELEOPERATED. In `STOPPED`/`TRANSITION`, use `selectedMode`.
   * `enabled = core::inputsEnabled(s)`, i.e. RUN phase and `Status::START` and not `deadlineMiss`.
   * `eStop = probot::robot::g_estop_latched`.
   * `dsAttached = s.clientCount > 0`, or recent `g_ds_last_activity_ms`.
   * The `INIT` phase maps to "opmode selected, disabled", which is WPILib 2027 OpModeRobot's pre-start state. `RUN` maps to enabled.
   * `opModeId`: the id of the opmode the user picked in the probot DS (needs a core change, see section 6). Until then, use the **first registered option for that robot mode** from `HAL_SetOpModeOptions`, or the NT `DashboardOpMode` if ntcore is present.
3. **Match time**: in `AUTO_RUN`, `autoPeriodSeconds - (millis()-autoStartMs)/1000`. Otherwise -1 (unknown), or a teleop countdown if core adds one.
4. **Joystick 0** = the probot gamepad, remapped from browser W3C "standard" layout to the WPILib 2027 **SDL layout** (`Gamepad::Button` FACE_DOWN=0, FACE_RIGHT=1, FACE_LEFT=2, FACE_UP=3, BACK=4, GUIDE=5, START=6, LEFT_STICK=7, RIGHT_STICK=8, LEFT_BUMPER=9, RIGHT_BUMPER=10, DPAD_UP/DOWN/LEFT/RIGHT=11–14; `Axis` LX=0, LY=1, RX=2, RY=3, LT=4, RT=5). Use probot-core's active `joystick_mapping` to find the source indices.
   * Axes go to `float` plus `raw = int16(v*32767)`; mark them in `available`.
   * Triggers come from the mapped LT/RT. They are **0/1 only today** because `GamepadSnapshot.buttons` is `bool[20]`; this is a core limitation.
   * The POV bitmask comes from the D-pad buttons or axis, using `joystick_api::getPOV()` logic converted to `HAL_JOYSTICK_POV_*` bits.
   * Descriptor: `isGamepad=1`, `gamepadType` from the mapping, `name` = mapping name.
   * Joysticks 1–5 are empty; core supports one gamepad.
5. Alliance = UNKNOWN (or a configurable define). GameData and MatchInfo are empty.
6. `NewDriverStationData()`: swap the caches and `newDataEvents.Wakeup()`. This wakes `DriverStationModeThread` and `WaitForInitialPacket()`.

**Outputs from WPILib into probot-core:**
* `sendError`, `sendConsoleLine`, `sendProgramCrash` and `writeDisplayAnsi` go to `probot::telemetry::print` and Serial. The 256 B ring is small, so use rate limiting (`DefaultSendErrorImpl` already de-duplicates).
* `setJoystickRumble` and `setJoystickLeds` are no-ops until core supports them; the browser Gamepad API has `vibrationActuator`.
* `observeUserProgram(word)` is the per-loop heartbeat from WPILib. Use it to feed the HAL output watchdog (section 5.4).
* `getOutputsEnabled()` = enabled and dsAttached and not e-stopped and not browned out.
* `setOpModeOptions` forwards to core so the browser DS can list opmodes (feature request).

### 5.4 Output safety
On systemcore and the roboRIO, disabling outputs is done below the HAL. On ESP32 the backend must do it:
* A global `std::atomic<bool> gOutputsEnabled` is set by the DS pump (and by e-stop and brownout).
* When it becomes false: every MCPWM servo/ESC output is driven to "no pulse" (PWM_DISABLED=0). DigitalPWM/LEDC duty goes to 0 (configurable, because FRC leaves DIO-PWM running). The CAN heartbeat goes to its disabled state. Stored setpoints are kept, but the hardware ignores them until enabled again.
* E-stop: core kills its own user task. Any WPILib task must notice through the control word (estop bit) **and** the HAL gate, so motor outputs stop even if user code is stuck. `PROBOT_ESTOP_ENABLE_PIN` remains the hardware path.

### 5.5 Who runs the robot loop (integration decision, flag for orchestrator)
probot-core owns `setup()`/`loop()` and runs the FTC hooks in a persistent task on core 1 every `USER_LOOP_PERIOD_MS = 20`. WPILib's `StartRobot<Robot>()` blocks in `StartCompetition()` and paces itself with the HAL Notifier. There are two options:
* **(A) Run WPILib in its own task.** prosdk provides the 4 required core hooks as no-ops, and `HAL_ObserveUserProgram` feeds a prosdk watchdog. This is the cleanest WPILib behaviour (TimedRobot, OpModeRobot and `addPeriodic` all work). Core's stall detection does not see the WPILib loop, so ask core for an "external loop" mode or a heartbeat hook. Stack: 16–24 KB for the robot task (`esp_pthread_set_cfg`).
* **(B) Step WPILib from core hooks**: `teleopLoop()` calls `robot.LoopFunc()`. This preserves core's cooperative-stop guarantees but needs a "steppable" variant of `IterativeRobotBase` and bypasses the Notifier.
The HAL design above works with both, because only the DS pump and the output gate touch core state.

### 5.6 Proposed `PortsInternal.hpp` for ESP32
```
NUM_SMART_IO              = 49   // GPIO0..48, validity mask applied
NUM_DIGITAL_CHANNELS      = 49
NUM_PWM_CHANNELS          = 49   // any GPIO; concurrency: 12 (MCPWM) -> HAL_NO_AVAILABLE_RESOURCES
NUM_DIGITAL_PWM_OUTPUTS   = 6    // LEDC ch0..5, one shared timer
NUM_ANALOG_INPUTS         = 10   // GPIO1..10 (ADC1); HAL_CheckAnalogInputChannel(1..10)
NUM_ENCODERS              = 8    // 4 PCNT + 4 software
NUM_COUNTERS              = 8    // shares PCNT/software pool with encoders
NUM_DUTY_CYCLES           = 6    // MCPWM capture
NUM_ADDRESSABLE_LEDS      = 3    // RMT TX minus probot NeoPixel; 1024-LED shared buffer
NUM_INTERRUPTS            = 0
NUM_CAN_BUSES             = 1    // TWAI (0 if PROSDK_HAL_CAN off)
I2C ports                 = 2    // Wire, Wire1
Serial ports              = ONBOARD->UART1, MXP->UART2, USB_1->USB-CDC
CTREPCM/CTREPDP/REVPDH/REVPH modules = 63 (CAN), channels 8/16/24/16 (unchanged)
Joysticks                 = HAL_MAX_JOYSTICKS 6 (only port 0 populated)
```
Before any of these numbers go into docs, check them against the real kit board (KIT.md says the electronics are not final; ProHub, BTS7960 and VNH5019 drivers exist in the legacy sdk).

### 5.7 Proposed file layout
```
src/prosdk/hal/                 public headers (renamed wpi/hal -> prosdk/hal), incl. simulation/*.h, handles/*
src/prosdk/hal/impl/common/     cpp/{ErrorHandling,Main,FIRSTDriverStation,UsageReporting,handles/HandlesInternal}.cpp
src/prosdk/hal/impl/esp32/      HAL.cpp, HALInitializer, Ports, SmartIo(ESP-IDF), DIO, PWM, Encoder, Counter, DutyCycle,
                                AnalogInput, AddressableLED (from systemcore), Notifier, Extensions (from sim),
                                CAN_twai.cpp, CANAPI/REVPH/REVPDH/CTREPCM/CTREPDP/PowerDistribution/rev/* (systemcore),
                                I2C, SerialPort, Power, IMU, Threads, SimDevice(stub), SystemServer(stub),
                                ProbotCoreDs.cpp, mockdata/* (systemcore stubs)
src/prosdk/hal/impl/sim/        unmodified sim/ + sim/mockdata (host tests only, not compiled for ESP32)
```
For Arduino, select between the two with `#if defined(ARDUINO_ARCH_ESP32)` guards in each .cpp, because Arduino compiles every file under `src/`. Alternatively, keep `impl/sim` out of the Arduino library tree and add it only through CMake/Makefile for host tests.

---

## 6. Feature requests for probot-core (send to `probot-core` via bp; not done here)
1. An OpMode list and selection in the browser DS (`HAL_SetOpModeOptions` → DS UI → selected id).
2. Analog trigger values (float button values) in `GamepadSnapshot`; multiple gamepads; rumble and LED output.
3. A hook or flag for an external robot loop (option A), plus a heartbeat API.
4. A teleop match-time countdown, alliance selection and game-data string (optional).
5. A sink for the alert list and error/warning lines in the DS UI (`WPI_AlertBackend`, `HAL_SendError`), larger than the 256 B ring.
6. Wall-clock time from the browser at connect, for `HAL_GetSystemTimeValid` and DataLog timestamps.
7. An exported RSL blink state and a DS-timeout counter, used by `HAL_GetRSLState` and `HAL_GetCommsDisableCount`.

## 7. Items to drop or stub
* Drop: JNI, `simulation/halsim_*` desktop extensions, `dlopen` extension loading, Windows timer code, `cpp/mrclib/*` (systemcore DS/Alert transport), `SystemServer` NT client and therefore **ExpansionHub\*** (USB RHSP daemon), CAN FD, `HAL_GetSerialFD`.
* Stub with defined results: 3V3 rail current/faults, `HAL_PulseMultiple` (software loop), `SetJoystickRumble/Leds`, `GetComments`.
