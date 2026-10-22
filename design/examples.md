# Examples

prosdk ports of allwpilib's C++ examples, snippets and templates (`wpilibcExamples/src/main/cpp`,
pinned allwpilib `b68effc`) as Arduino sketches in `examples/`. This file records one decision per
upstream item, the pin map the ports use, what differs from upstream, and the compiled sizes.

`tools/upstream/examples.json` records, for each ported sketch and test, its upstream folders or files
and their git blob ids at the pin, and the blob ids of the four upstream lists.
`tools/upstream/examples.py --check` (CI, host job) reports every upstream change to review when the
pin moves, and any sketch folder that is neither in the manifest nor hand-written; `--write`
refreshes the ids once the ports follow upstream (design/PORT.md section 2).

The hand-written prosdk examples that existed before this port (HalSmoke, TimedRobotDifferential,
OpModeRobotExample, TimesliceRobotExample, CommandBasedRobot, DriversShowcase, AprilTagFieldPose)
are unchanged. The nine Boardoza examples were added later (section "Boardoza examples"). All
hand-written examples are listed under `"handwritten"` in `tools/upstream/examples.json`.

## Porting rules

- **Structure.** Same structure, logic and comments as upstream; only `wpi::` -> `prosdk::`
  and `wpi/...` -> `prosdk/...` change.
- **Sketch layout.** Each item is a sketch folder `examples/<Name>/<Name>.ino`.
  - Upstream `include/` and `cpp/` files become flat tabs (`.hpp`/`.cpp`) next to the `.ino`.
  - Local includes lose their directory (`"subsystems/Drive.hpp"` -> `"Drive.hpp"`).
  - When upstream defines the robot class in `Robot.cpp` without a `Robot.hpp`, that file
    becomes the `.ino`.
- **The `.ino` file.** It holds, in order:
  - upstream's copyright lines;
  - a header comment: what the example is, its upstream path, its wiring as GPIO numbers, its
    controls and every deviation;
  - `PROBOT_WIFI_AP_PASSWORD` / `PROBOT_WIFI_AP_CHANNEL`;
  - `#include <prosdk.h>`;
  - `PROSDK_ROBOT_MAIN(Robot)` in place of upstream's
    `#ifndef RUNNING_WPILIB_TESTS int main() ... #endif`.
- **Names.**
  - Snippets are named `Snippet<Name>`, because three snippet names collide with examples
    (Encoder, DutyCycleEncoder, MecanumDrive).
  - Templates are named `Template<Name>`.
  - Examples keep upstream's folder name.
- **Simulation code** (`SimulationPeriodic`, `*Sim` classes) stays. It only runs in the host
  simulator; on the robot it is inert.

## Pin map

The channel number is the GPIO number (design/hal-esp32.md). The ports avoid the pins the HAL
refuses:
- 19/20 (USB) and 43/44 (UART0);
- 26-32 (flash) and 33-37 (octal PSRAM);
- 22-25, which do not exist;
- GPIO 3 (probot-core's NeoPixel).

They also stay inside the hardware limits:
- 12 PWM outputs;
- 4 PCNT units shared by encoders and counters;
- 6 duty-cycle inputs;
- analog inputs only on GPIO 1-10;
- 3 LED strips.

| Use | GPIO |
|---|---|
| PWM motor controllers | 4, 5, 12, 13 (more: 14, 21, 47, 48) |
| Quadrature encoders A/B | 6/7, 15/16, 38/39, 40/41 |
| Digital inputs/outputs, duty cycle | 17, 18, 10, 11 |
| Analog inputs | 1, 2 |
| Addressable LED strip | 21 |
| I2C port 0 (HAL default) | SDA 8, SCL 9 |

The same pins also work in the host simulator, which limits PWM to 0-19, DIO to 0-30 and analog
to 0-7. The exceptions are SwerveBot and SwerveDrivePoseEstimator, which need more pins than
those ranges allow.

## Deviations common to several ports

- **No onboard IMU.** The Probot board has no IMU, so upstream's `OnboardIMU` becomes the kit IMU
  `prosdk::MPU6050` on I2C port 0.
  - It has the same getters (GetYaw, ResetYaw, GetRotation2d, GetAngleX/Y/Z, GetGyroRate*,
    GetAccel*).
  - Affected: Gyro, MecanumDrive, DifferentialDriveBot, MecanumBot, SwerveBot, the three pose
    estimators, SimpleDifferentialDriveSimulation, SnippetOnboardIMU, SnippetMecanumDrive,
    SnippetAccelerometerCollision and SnippetAccelerometerFilter.
  - The MPU6050 has no sim class. Where upstream wrote a simulated heading (`OnboardIMUSim`,
    DifferentialDrivePoseEstimator), that line is commented out, as upstream
    SimpleDifferentialDriveSimulation already does with its gyro.
- **No NetworkTables.**
  - DifferentialDrivePoseEstimator passed its simulated camera-to-object transform through an NT
    topic. It now passes a `std::array<double, 7>` member.
  - Telemetry, Field2d, Mechanism2d and choosers use prosdk telemetry and tunables. Field2d and
    Mechanism2d show as a topic tree on the Driver Station (core R5).
- **Driver Station.**
  - The Probot Driver Station sends no alliance, station, match info or game data
    (design/hal-esp32.md), so SnippetDigitalCommunication and SnippetI2CCommunication always see
    "no alliance".
  - `DriverStationDisplay` frames go to the Driver Station console with ANSI sequences removed,
    which affects DriverStationDisplayAnsi.
  - Preferences appear as `/Preferences/<key>` tunables and are stored in NVS (ArmSimulation).
- **Console output.** `prosdk::util::print` (snippets) writes to stdout, which is the USB serial
  on the robot, not the Driver Station console. It was kept as upstream.
- **Smart motor controller placeholder.** `ExampleSmartMotorController` (ElevatorTrapezoidProfile,
  ElevatorExponentialProfile, DriveDistanceOffboard) is upstream's API stub and does nothing; the
  kit has no smart motor controllers. Those examples show the structure only.
- **Swerve.** SwerveBot and SwerveDrivePoseEstimator use 8 quadrature encoders, but the ESP32-S3
  has 4 PCNT units.
  - On the robot the fifth `Encoder` fails, and the program halts with the error on the Driver
    Station console.
  - They run in the host simulator.
  - On hardware, use absolute turning encoders instead (DutyCycleEncoder or AnalogEncoder). The
    header comment of both sketches says so.
- **SnippetEncoder.** Upstream puts both encoders on DIO 0/1, which cannot be allocated twice.
  The port uses 6/7 and 15/16.

## Decisions

Status values:
- **ported**: a sketch with the given name.
- **merged**: covered by another prosdk example.
- **not ported**: with the reason.

### Examples (examples.json, 38)

| Upstream | Decision | prosdk example |
|---|---|---|
| Mechanism2d | ported | Mechanism2d |
| Encoder | ported | Encoder |
| Gyro | ported (MPU6050) | Gyro |
| MecanumDrive | ported (MPU6050) | MecanumDrive |
| ElevatorExponentialProfile | ported (smart controller placeholder) | ElevatorExponentialProfile |
| ElevatorTrapezoidProfile | ported (smart controller placeholder) | ElevatorTrapezoidProfile |
| ElevatorProfiledPID | ported | ElevatorProfiledPID |
| GettingStarted | ported | GettingStarted |
| HAL | merged: a C program with its own blocking `main()` loop; a sketch has no `main()` on the robot (the robot host runs inside probot-core's loop). The HAL C API on hardware is shown by HalSmoke | HalSmoke |
| HatchbotTraditional | not ported: pneumatics (DoubleSolenoid; the pneumatics HAL refuses on the ESP32) | |
| HatchbotInlined | not ported: pneumatics (DoubleSolenoid) | |
| RapidReactCommandBot | not ported: pneumatics (intake DoubleSolenoid) | |
| SwerveBot | ported (simulation only on hardware: 8 encoders) | SwerveBot |
| MecanumBot | ported | MecanumBot |
| DifferentialDriveBot | ported | DifferentialDriveBot |
| ArcadeDriveGamepad | ported | ArcadeDriveGamepad |
| TankDriveGamepad | ported | TankDriveGamepad |
| DutyCycleEncoder | ported | DutyCycleEncoder |
| DriverStationDisplayLines | ported | DriverStationDisplayLines |
| DriverStationDisplayAnsi | ported (ANSI stripped on the DS console) | DriverStationDisplayAnsi |
| DSGamepadChooser | ported | DSGamepadChooser |
| DriveDistanceOffboard | ported (smart controller placeholder) | DriveDistanceOffboard |
| RomiReference | not ported: Romi | |
| XRPReference | not ported: XRP | |
| Xrptimed | not ported: XRP | |
| StateSpaceFlywheel | ported | StateSpaceFlywheel |
| StateSpaceFlywheelSysId | ported | StateSpaceFlywheelSysId |
| StateSpaceElevator | ported | StateSpaceElevator |
| StateSpaceArm | ported | StateSpaceArm |
| ElevatorSimulation | ported, with its host test | ElevatorSimulation |
| ElevatorExponentialSimulation | ported | ElevatorExponentialSimulation |
| DifferentialDrivePoseEstimator | ported (MPU6050, NT entry replaced by an array) | DifferentialDrivePoseEstimator |
| MecanumDrivePoseEstimator | ported (MPU6050) | MecanumDrivePoseEstimator |
| ArmSimulation | ported, with its host test | ArmSimulation |
| UnitTest | not ported: pneumatics (the intake is a DoubleSolenoid; the example and its test are about that piston) | |
| SimpleDifferentialDriveSimulation | ported (MPU6050) | SimpleDifferentialDriveSimulation |
| SwerveDrivePoseEstimator | ported (simulation only on hardware: 8 encoders) | SwerveDrivePoseEstimator |
| SysIdRoutine | ported | SysIdRoutine |

### Snippets (snippets.json, 31)

| Upstream | Decision | prosdk example |
|---|---|---|
| Encoder | ported (second encoder moved to 15/16) | SnippetEncoder |
| DutyCycleEncoder | ported | SnippetDutyCycleEncoder |
| AnalogEncoder | ported | SnippetAnalogEncoder |
| EncoderDrive | ported | SnippetEncoderDrive |
| EncoderHoming | ported | SnippetEncoderHoming |
| LimitSwitch | ported | SnippetLimitSwitch |
| DigitalInput | ported | SnippetDigitalInput |
| AnalogInput | ported | SnippetAnalogInput |
| AnalogPotentiometer | ported | SnippetAnalogPotentiometer |
| AnalogAccelerometer | ported | SnippetAnalogAccelerometer |
| ADXLAccelerometers | ported (ADXL345 on I2C port 0) | SnippetADXLAccelerometers |
| AccelerometerCollision | ported (MPU6050) | SnippetAccelerometerCollision |
| AccelerometerFilter | ported (MPU6050) | SnippetAccelerometerFilter |
| OnboardIMU | ported (MPU6050) | SnippetOnboardIMU |
| ProfiledPIDFeedforward | ported | SnippetProfiledPIDFeedforward |
| AddressableLED | ported | SnippetAddressableLED |
| AprilTagsVision | not ported: camera/vision (cscore/CameraServer). AprilTag field poses are shown by AprilTagFieldPose | AprilTagFieldPose |
| CANPDP | not ported: CAN Power Distribution (the HAL's PowerDistribution is a stub) | |
| DigitalCommunication | ported (no alliance data) | SnippetDigitalCommunication |
| DutyCycleInput | ported | SnippetDutyCycleInput |
| EventLoop | ported | SnippetEventLoop |
| FlywheelBangBangController | ported | SnippetFlywheelBangBangController |
| HttpCamera | not ported: camera/vision | |
| I2CCommunication | ported (no alliance data) | SnippetI2CCommunication |
| IntermediateVision | not ported: camera/vision | |
| MotorControl | ported | SnippetMotorControl |
| SelectCommand | ported | SnippetSelectCommand |
| QuickVision | not ported: camera/vision | |
| Solenoid | not ported: pneumatics | |
| MecanumDrive | ported (MPU6050) | SnippetMecanumDrive |
| DifferentialDrive | ported | SnippetDifferentialDrive |

### Templates (templates.json, 12)

| Upstream | Decision | prosdk example |
|---|---|---|
| commandv2 (Command v2 Robot) | ported | TemplateCommandV2 |
| commandv2skeleton | ported | TemplateCommandV2Skeleton |
| opmode | ported | TemplateOpMode |
| timed | ported | TemplateTimed |
| timedskeleton | ported | TemplateTimedSkeleton |
| timeslice | ported | TemplateTimeslice |
| timesliceskeleton | ported | TemplateTimesliceSkeleton |
| robotbaseskeleton | not ported: a class derived from RobotBase directly needs a blocking `StartCompetition()`, which cannot run inside probot-core's loop; the robot host reports it and halts (prosdk/platform/RobotMain.hpp) | |
| Romi - Command Robot | not ported: Romi | |
| Romi - Timed Robot | not ported: Romi | |
| XRP - Command Robot | not ported: XRP | |
| XRP - Timed Robot | not ported: XRP | |

### Command class templates (commands.json, 8)

All eight are merged into one sketch, **TemplateCommandItems**.
- Each `ReplaceMe*` class is a tab with upstream's text.
- A small TimedRobot constructs one of each, runs the scheduler in `RobotPeriodic()` and schedules
  the commands in `TeleopInit()`, so every template is compiled and linked.

| Upstream | Decision | prosdk example |
|---|---|---|
| emptyclass | merged | TemplateCommandItems (ReplaceMeEmptyClass) |
| command2 | merged | TemplateCommandItems (ReplaceMeCommand2) |
| instantcommand | merged | TemplateCommandItems (ReplaceMeInstantCommand2) |
| parallelcommandgroup | merged | TemplateCommandItems (ReplaceMeParallelCommandGroup) |
| paralleldeadlinegroup | merged | TemplateCommandItems (ReplaceMeParallelDeadlineGroup) |
| parallelracegroup | merged | TemplateCommandItems (ReplaceMeParallelRaceGroup) |
| sequentialcommandgroup | merged | TemplateCommandItems (ReplaceMeSequentialCommandGroup) |
| subsystem2 | merged | TemplateCommandItems (ReplaceMeSubsystem2) |

### Totals

| Kind | Upstream | Ported | Merged | Not ported |
|---|---|---|---|---|
| Examples | 38 | 30 | 1 (HAL) | 7 |
| Snippets | 31 | 25 | 0 | 6 |
| Templates | 12 | 7 | 0 | 5 |
| Command templates | 8 | 0 | 8 (TemplateCommandItems) | 0 |

## Example tests

Upstream has five example tests: three in `wpilibcExamples/src/test/cpp/examples` and two in
`wpilibcExamples/src/test/cpp/snippets`.

- **ArmSimulation** and **ElevatorSimulation** are ported to `test/examples/<Name>/`, and the
  **DigitalCommunication** and **I2CCommunication** snippet tests to
  `test/examples/Snippet<Name>/` (file name kept as upstream, `<Name>Test.cpp`).
  - They share upstream's `main.cpp` (`test/examples/main.cpp`).
  - Each is its own host executable, `example_<Name>_tests` (`tools/build/modules/examples.cmake`).
  - Each executable is built from the example's `.cpp` tabs with `RUNNING_WPILIB_TESTS=1`. The
    `.ino` is not compiled, since it only holds `PROSDK_ROBOT_MAIN`.
  - The tests run the robot in the host simulator with paused timing, as upstream does. The
    simulator has alliance and match data, so the snippet tests check them although the probot
    Driver Station sends none.
- **UnitTest** is not ported, because the example is not ported (pneumatics).

## Boardoza examples

Hand-written `TimedRobot` sketches for the Boardoza boards (`design/boardoza.md`), with no upstream
counterpart. Each `.ino` header gives the boards, the power rule (the ESP32-S3 pins are not 5 V
tolerant), the wiring as GPIO numbers and the package document. Readings go to the Driver Station
(`/Telemetry`). The pins differ from the upstream pin map above, because I2C port 1 and SPI need
pins of their own. None of them has run on hardware.

| Example | Boards and classes | Bus and pins |
|---|---|---|
| BoardozaClimate | LM75ADP, HIH8131, TH09C, MS5611, HP303B, BME688 (`LM75A`, `HIH8000`, `ENS210`, `MS5611`, `DPS310`, `BME688`) | I2C port 1, SDA 17 / SCL 18, 100 kHz; BME688 on port 0 (`Wire`'s bus), since only two barometers fit on one port |
| BoardozaThermalCamera | MLX90640 (`MLX90640`; the MLX90641 works the same way) | I2C port 1, SDA 17 / SCL 18, 400 kHz |
| BoardozaMultizoneToF | VL53L7CX-1..4 (`VL53L7CX`, 8x8 at 15 Hz) | I2C port 1, SDA 17 / SCL 18, 400 kHz |
| BoardozaLoadCell | NAU7802, HX711 (`NAU7802`, `HX711`, `LoadCell`) | I2C port 1, SDA 17 / SCL 18, 400 kHz; HX711 DAT 4, CLK 5 |
| BoardozaGnss | SAM-M8Q or SAM-M10Q (`UbloxGnss` on I2C), GP-02 (`NmeaGnss`) | I2C port 1, SDA 17 / SCL 18, 100 kHz; serial `kOnboard`, RX 15 / TX 16 |
| BoardozaSdLog | MicroSD (`platform::MountSdCard`, DataLogManager to `/sd/logs`) | SPI bus 0: SCK 12, MOSI 11, MISO 13, CS 10 |
| BoardozaRotaryArgb | Rotary Encoder ARGB (`ClockedAddressableLED`, `Encoder`, `DigitalInput`) | SPI bus 1 alone: SCK 39, MOSI 40; encoder A 41 / B 42; button 38 |
| BoardozaLoRa | E22-900M22S (`SX126x`). Regulatory: 868.1 MHz, 14 dBm, at most 1 % duty cycle (EU 868.0-868.6 MHz sub-band); check local rules before changing them | SPI bus 0: SCK 12, MOSI 11, MISO 13, CS 10; BUSY 4, DIO1 5, NRST 6, RXEN 7, TXEN 15 |
| BoardozaEveDisplay | FT810 or FT800 board and bundles (`FT8xx`, `ft8xx::kBoardozaFT810Bundle`) | SPI bus 0: SCK 12, MOSI 11, MISO 13, CS 10; PD 4, INT 5 |

All nine build with arduino-cli; their sizes are in the Sizes table below.

## Sizes

ESP32-S3 (`esp32:esp32:esp32s3`, huge_app), compiled with `tools/build/build.sh examples <name>`.
Flash is "Sketch uses", RAM is "Global variables use" (static RAM; the robot also allocates heap
at run time).

Measured on 2026-10-22 (after R9/R10, CoreOps ABI 4) against the `src/esp32s3/libprosdk.a` of
that day and probot-core `feat/wall-clock` (b46069e); the numbers move with the archive and core.
The huge_app partition gives the sketch 3,145,728 bytes of flash; static RAM is out of 327,680
bytes. Every sketch also gets probot-core's WiFi AP and Driver Station.
The same archive against core `feat/multi-gamepad` (no R9/R10) gives TemplateTimed 1,864,712 /
69,764: core's R9 console and R10 clock add about 7 KB flash and 1.5 KB static RAM to every sketch.
The nine Boardoza rows were measured later the same day, against the archive of `bd0d149` (with the
Boardoza drivers) and the same core. That archive gives TemplateTimed 1,871,876 / 71,220 (+132 B
flash) and DriversShowcase 1,998,728 / 72,828 (+160 B; 1,998,568 in design/PORT.md): no Boardoza
driver is linked into them, the growth is the six-step e-stop hooks in `SmartIo.cpp` and the
unwind tables of the SPI pin functions in `HalConfig.cpp` (design/size.md section 8).

Rows marked "(small-items)" were re-measured later on 2026-10-22 with CoreOps ABI 5 (prosdk 3450336)
against probot-core `feat/small-items` (09a6fe4, R12 part 1, R13 and R14, the core CI builds against);
every other row was last measured against `feat/wall-clock` as above. Core R14 (the gzipped Driver
Station page) makes every sketch about 100 KB smaller: 99,856-100,940 B less flash and 16-96 B more
static RAM in the eight sketches re-measured (design/size.md section 9, which also lists
TimedRobotDifferential, CommandBasedRobot, AprilTagFieldPose, DriversShowcase and HalSmoke; their
numbers are in design/PORT.md 8.1). CI builds all examples on every push.

| Example | Flash (bytes) | RAM (bytes) |
|---|---|---|
| ArcadeDriveGamepad | 1,906,064 | 71,556 |
| ArmSimulation | 2,000,648 | 74,756 |
| BoardozaClimate | 1,918,108 | 72,340 |
| BoardozaEveDisplay | 1,921,260 | 71,872 |
| BoardozaGnss | 1,923,512 | 72,428 |
| BoardozaLoadCell | 1,906,372 | 72,364 |
| BoardozaLoRa | 1,920,648 | 71,872 |
| BoardozaMultizoneToF | 1,995,000 | 72,372 |
| BoardozaRotaryArgb | 1,926,612 | 72,544 |
| BoardozaSdLog (small-items) | 1,905,237 | 74,680 |
| BoardozaThermalCamera | 1,905,060 | 72,364 |
| DifferentialDriveBot | 1,971,912 | 73,348 |
| DifferentialDrivePoseEstimator | 2,093,956 | 75,844 |
| DriveDistanceOffboard | 1,952,336 | 71,660 |
| DriverStationDisplayAnsi | 1,874,692 | 71,284 |
| DriverStationDisplayLines | 1,895,472 | 71,284 |
| DSGamepadChooser | 1,914,608 | 71,356 |
| DutyCycleEncoder | 1,880,816 | 71,484 |
| ElevatorExponentialProfile | 1,873,664 | 71,276 |
| ElevatorExponentialSimulation | 1,996,048 | 74,756 |
| ElevatorProfiledPID | 1,942,120 | 72,196 |
| ElevatorSimulation | 2,006,000 | 74,756 |
| ElevatorTrapezoidProfile | 1,871,756 | 71,276 |
| Encoder | 1,885,172 | 71,876 |
| GettingStarted | 1,906,568 | 71,556 |
| Gyro | 1,940,216 | 72,700 |
| MecanumBot | 2,057,652 | 73,412 |
| MecanumDrive | 1,940,100 | 72,700 |
| MecanumDrivePoseEstimator | 2,066,344 | 75,908 |
| Mechanism2d | 1,946,940 | 72,484 |
| SimpleDifferentialDriveSimulation | 2,288,304 | 75,908 |
| SnippetAccelerometerCollision | 1,895,996 | 72,372 |
| SnippetAccelerometerFilter | 1,897,684 | 72,356 |
| SnippetAddressableLED | 1,877,444 | 74,292 |
| SnippetADXLAccelerometers | 1,890,544 | 72,340 |
| SnippetAnalogAccelerometer | 1,877,956 | 71,444 |
| SnippetAnalogEncoder | 1,879,064 | 71,444 |
| SnippetAnalogInput | 1,877,148 | 71,444 |
| SnippetAnalogPotentiometer | 1,878,152 | 71,444 |
| SnippetDifferentialDrive | 1,907,452 | 71,556 |
| SnippetDigitalCommunication | 1,868,700 | 71,236 |
| SnippetDigitalInput | 1,867,032 | 71,244 |
| SnippetDutyCycleEncoder | 1,880,332 | 71,484 |
| SnippetDutyCycleInput | 1,878,180 | 71,468 |
| SnippetEncoder | 1,885,424 | 71,876 |
| SnippetEncoderDrive | 1,919,952 | 72,148 |
| SnippetEncoderHoming | 1,916,336 | 72,172 |
| SnippetEventLoop | 1,935,860 | 72,196 |
| SnippetFlywheelBangBangController | 1,958,640 | 74,764 |
| SnippetI2CCommunication | 1,887,840 | 72,348 |
| SnippetLimitSwitch | 1,902,080 | 71,572 |
| SnippetMecanumDrive | 1,940,468 | 72,700 |
| SnippetMotorControl | 1,920,940 | 72,204 |
| SnippetOnboardIMU | 1,898,756 | 72,372 |
| SnippetProfiledPIDFeedforward | 1,934,640 | 72,140 |
| SnippetSelectCommand | 1,913,744 | 71,348 |
| StateSpaceArm | 2,251,272 | 72,260 |
| StateSpaceElevator | 2,251,280 | 72,260 |
| StateSpaceFlywheel | 2,089,356 | 72,260 |
| StateSpaceFlywheelSysId | 2,089,344 | 72,260 |
| SwerveBot | 2,116,600 | 73,348 |
| SwerveDrivePoseEstimator | 2,127,292 | 75,908 |
| SysIdRoutine | 2,080,340 | 75,084 |
| TankDriveGamepad (small-items) | 1,805,856 | 71,636 |
| TemplateCommandItems | 1,923,316 | 71,348 |
| TemplateCommandV2 | 1,930,184 | 71,468 |
| TemplateCommandV2Skeleton | 1,904,380 | 71,348 |
| TemplateOpMode | 1,871,184 | 71,220 |
| TemplateTimed (small-items) | 1,771,768 | 71,252 |
| TemplateTimedSkeleton | 1,859,744 | 71,220 |
| TemplateTimeslice | 1,874,968 | 71,220 |
| TemplateTimesliceSkeleton | 1,863,092 | 71,220 |
