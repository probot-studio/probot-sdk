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
- **Simulation code** (`SimulationPeriodic`, `*Sim` classes) stays. On the robot it is inert. On a
  PC it runs only in the host tests (`test/examples`: ArmSimulation, ElevatorSimulation and two
  snippets); there is no desktop simulation of a sketch (design/API-DIFFERENCES.md section 5).

## Pin map

The channel number is the GPIO number (design/hal-esp32.md). The ports avoid the pins the HAL
refuses:
- 19/20 (USB) and 43/44 (UART0);
- 26-32 (flash) and 33-37 (octal PSRAM);
- 22-25, which do not exist;
- GPIO 3 (probot-core's NeoPixel).

They also avoid the strapping pins 0, 45 and 46, which the HAL allows (hal-esp32.md). Only
SwerveBot and SwerveDrivePoseEstimator use one, for lack of pins: GPIO 46 for the back-right
turning encoder B, which a normal boot ignores (it counts only while GPIO 0 is held low). They say
so in their wiring. GPIO 45 and GPIO 0 are not used by any example.

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

The same pins also fit the host simulation HAL that the example tests use, which limits PWM to
0-19, DIO to 0-30 and analog to 0-7. The exceptions are SwerveBot and SwerveDrivePoseEstimator,
which need more pins than those ranges allow.

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
  - Telemetry, Field2d, Mechanism2d and choosers use prosdk telemetry and tunables. The Driver
    Station draws Field2d and Mechanism2d as widgets (probot-core #40, layouts in
    design/core-requests.md).
- **Driver Station.**
  - The alliance station, match info and game data come from the Driver Station's "Maç Bilgisi"
    card (design/hal-esp32.md; core request R18), set while the robot is disabled. Until it is
    set (and with a probot-core without R18) SnippetDigitalCommunication and
    SnippetI2CCommunication see "no alliance" on the robot.
  - `DriverStationDisplay` shows as a text panel on the Driver Station (core request R19, #44:
    the Dashboard's "Robot Ekranı" card and the Veri page; an older core shows plain text in the
    topic tree), updated in place; DriverStationDisplayAnsi keeps its in-place counter but loses
    its colors. The examples' screens fit core's default `PROBOT_DASH_MAX_VALUE` (1024 bytes); a
    full 48 x 160 screen needs 8192 (design/API-DIFFERENCES.md).
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
  - On hardware, use absolute turning encoders instead (DutyCycleEncoder or AnalogEncoder). The
    header comment of both sketches says so.
- **SnippetEncoder.** Upstream puts both encoders on DIO 0/1, which cannot be allocated twice.
  The port uses 6/7 and 15/16.
- **SysIdRoutine.**
  - Drive and Shooter name themselves (`SubsystemBase{"Drive"}`, `SubsystemBase{"Shooter"}`), so
    their SysId log entries (`sysid-test-state-Drive`, `-Shooter`) and command names differ.
    Upstream leaves both with the base name, and without RTTI both would be
    `prosdk::cmd::SubsystemBase` (API-DIFFERENCES.md, SubsystemBase default name). It has to be the
    base initializer: the `sysIdRoutine` member reads `GetName()` before the constructor body.
  - `Robot`'s first member mounts LittleFS (`bool m_fs = LittleFS.begin(true);`), so the log
    reaches /littlefs/logs; prosdk does not mount it. BoardozaSdLog mounts it as its no-card
    fallback.
  - The `Robot` constructor calls `DataLogManager::Start()` (upstream starts the log lazily at the
    first SysId test). The start shares the "logs" folder with the Driver Station, so logs of an
    earlier boot can be downloaded after a reboot or e-stop restart without running a test first.

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
| GettingStarted | ported (teleop steering negated: upstream's `+GetRightX()` turns left on stick right) | GettingStarted |
| HAL | merged: a C program with its own blocking `main()` loop; a sketch has no `main()` on the robot (the robot host runs inside probot-core's loop). The HAL C API on hardware is shown by HalSmoke | HalSmoke |
| HatchbotTraditional | not ported: pneumatics (DoubleSolenoid; the pneumatics HAL refuses on the ESP32) | |
| HatchbotInlined | not ported: pneumatics (DoubleSolenoid) | |
| RapidReactCommandBot | not ported: pneumatics (intake DoubleSolenoid) | |
| SwerveBot | ported (does not run on the ESP32-S3 as written: 8 quadrature encoders, 4 PCNT units) | SwerveBot |
| MecanumBot | ported | MecanumBot |
| DifferentialDriveBot | ported | DifferentialDriveBot |
| ArcadeDriveGamepad | ported | ArcadeDriveGamepad |
| TankDriveGamepad | ported | TankDriveGamepad |
| DutyCycleEncoder | ported | DutyCycleEncoder |
| DriverStationDisplayLines | ported | DriverStationDisplayLines |
| DriverStationDisplayAnsi | ported (plain text without colors in the DS topic tree) | DriverStationDisplayAnsi |
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
| SwerveDrivePoseEstimator | ported (does not run on the ESP32-S3 as written: 8 quadrature encoders, 4 PCNT units) | SwerveDrivePoseEstimator |
| SysIdRoutine | ported | SysIdRoutine |

### Snippets (snippets.json, 31)

| Upstream | Decision | prosdk example |
|---|---|---|
| Encoder | ported (second encoder moved to 15/16) | SnippetEncoder |
| DutyCycleEncoder | ported | SnippetDutyCycleEncoder |
| AnalogEncoder | ported | SnippetAnalogEncoder |
| EncoderDrive | ported | SnippetEncoderDrive |
| EncoderHoming | ported (switch read inverted: active-low switch to GND with the pull-up) | SnippetEncoderHoming |
| LimitSwitch | ported (switch reads inverted: active-low switches to GND with the pull-up) | SnippetLimitSwitch |
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
| DigitalCommunication | ported | SnippetDigitalCommunication |
| DutyCycleInput | ported | SnippetDutyCycleInput |
| EventLoop | ported | SnippetEventLoop |
| FlywheelBangBangController | ported | SnippetFlywheelBangBangController |
| HttpCamera | not ported: camera/vision | |
| I2CCommunication | ported | SnippetI2CCommunication |
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
    simulator sets alliance and match data, as the Driver Station's "Maç Bilgisi" card does on the
    robot.
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
| BoardozaGnss | SAM-M8Q or SAM-M10Q (`UbloxGnss` on I2C), GP-02 (`NmeaGnss`) | I2C port 1, SDA 17 / SCL 18, 100 kHz; serial `ONBOARD`, RX 15 / TX 16 |
| BoardozaSdLog | MicroSD (`platform::MountSdCard`, DataLogManager to `/sd/logs`) | SPI bus 0: SCK 12, MOSI 11, MISO 13, CS 10 |
| BoardozaRotaryArgb | Rotary Encoder ARGB (`ClockedAddressableLED`, `Encoder`, `DigitalInput`) | SPI bus 1 alone: SCK 39, MOSI 40; encoder A 41 / B 42; button 38 |
| BoardozaLoRa | E22-900M22S (`SX126x`). Regulatory: 868.1 MHz, 14 dBm, at most 1 % duty cycle (EU 868.0-868.6 MHz sub-band); check local rules before changing them | SPI bus 0: SCK 12, MOSI 11, MISO 13, CS 10; BUSY 4, DIO1 5, NRST 6, RXEN 7, TXEN 15 |
| BoardozaEveDisplay | FT810 or FT800 board and bundles (`FT8xx`, `ft8xx::kBoardozaFT810Bundle`) | SPI bus 0: SCK 12, MOSI 11, MISO 13, CS 10; PD 4, INT 5 |

All nine build with arduino-cli; their sizes are in the Sizes table below.

## Sizes

ESP32-S3 (`esp32:esp32:esp32s3`, huge_app), compiled with `tools/build/build.sh examples <name>`.
Flash is "Sketch uses", RAM is "Global variables use" (static RAM; the robot also allocates heap
at run time).

Measured by CI on 2026-10-23 (run 39890517789, prosdk `85ed58e`, CoreOps ABI 8) against probot-core
`feat/dash-set-refused` (#42, then the top of core's draft stack: R11 files, R12 persistence, R5 widgets, R15,
R16). The numbers move with the archive and core; design/size.md keeps the history (R14's gzipped
Driver Station page saved about 100 KB per sketch, core's #38-#42 added about 16-24 KB).
The huge_app partition gives the sketch 3,145,728 bytes of flash; static RAM is out of 327,680
bytes. Every sketch also gets probot-core's WiFi AP and Driver Station. The same CI run links the HAL
link test sketch at 1,709,957 / 75,996 against this core and at 1,745,453 / 68,956 against the oldest
supported core (`feat/dashboard-store`). CI builds all examples on pushes to `dev` and `stable`, on pull requests and
on manual runs; to refresh this table, take the "Sketch uses" / "Global variables use" lines after each
"== <name>" line of the examples job log.

The table predates the DataLog telemetry copy and the display terminal (749df1e, 6940946 and their
review fixes): every robot sketch now links the display terminal and the default-backend wrapper,
about 4 KB of flash and 64-128 B of RAM more, and a sketch that uses DataLogManager also links the log
copy (about 9 KB in all). Local builds (`tools/build/build.sh examples`, same archive and core) after
the review fixes: TemplateTimed 1,802,508 / 72,372, DriverStationDisplayLines 1,826,176 / 72,484,
DriverStationDisplayAnsi 1,805,464 / 72,484, DSGamepadChooser 1,845,324 / 72,556, SysIdRoutine
2,017,108 / 76,284. Since then CI pins core #43 (`feat/ds-match-info` 73c7e10, R18 match info, about
5 KB of core flash): TemplateTimed 1,812,056 / 73,164 and SnippetDigitalCommunication 1,809,196 /
73,172 locally (design/size.md section 13). CI now pins core #45 (`feat/gamepad-guide` e0cb17f, R19
display panel and R20 Home/Guide button on #44): DSGamepadChooser 1,854,452 / 73,292 and
DriverStationDisplayLines 1,835,176 / 73,228 locally (design/size.md section 14). The next CI run gives
the figures for all 79 sketches.

| Example | Flash (bytes) | RAM (bytes) |
|---|---|---|
| AprilTagFieldPose | 2,025,492 | 72,372 |
| ArcadeDriveGamepad | 1,833,196 | 72,692 |
| ArmSimulation | 1,928,352 | 75,892 |
| BoardozaClimate | 1,844,948 | 73,476 |
| BoardozaEveDisplay | 1,848,156 | 72,976 |
| BoardozaGnss | 1,850,324 | 73,500 |
| BoardozaLoRa | 1,847,568 | 72,976 |
| BoardozaLoadCell | 1,833,356 | 73,436 |
| BoardozaMultizoneToF | 1,921,908 | 73,444 |
| BoardozaRotaryArgb | 1,853,524 | 73,648 |
| BoardozaSdLog | 1,932,817 | 75,752 |
| BoardozaThermalCamera | 1,831,948 | 73,500 |
| CommandBasedRobot | 2,010,312 | 76,420 |
| DSGamepadChooser | 1,841,596 | 72,428 |
| DifferentialDriveBot | 1,899,176 | 74,484 |
| DifferentialDrivePoseEstimator | 2,021,248 | 76,980 |
| DriveDistanceOffboard | 1,879,752 | 72,732 |
| DriverStationDisplayAnsi | 1,801,624 | 72,356 |
| DriverStationDisplayLines | 1,822,384 | 72,420 |
| DriversShowcase | 1,925,972 | 73,900 |
| DutyCycleEncoder | 1,807,844 | 72,556 |
| ElevatorExponentialProfile | 1,800,704 | 72,348 |
| ElevatorExponentialSimulation | 1,923,640 | 75,828 |
| ElevatorProfiledPID | 1,869,244 | 73,332 |
| ElevatorSimulation | 1,933,564 | 75,828 |
| ElevatorTrapezoidProfile | 1,798,780 | 72,348 |
| Encoder | 1,812,196 | 72,948 |
| GettingStarted | 1,833,700 | 72,692 |
| Gyro | 1,867,452 | 73,836 |
| HalSmoke | 1,613,941 | 74,380 |
| MecanumBot | 1,985,652 | 74,484 |
| MecanumDrive | 1,867,396 | 73,836 |
| MecanumDrivePoseEstimator | 1,994,340 | 76,980 |
| Mechanism2d | 1,874,232 | 73,572 |
| OpModeRobotExample | 1,867,980 | 73,276 |
| SimpleDifferentialDriveSimulation | 2,216,612 | 76,980 |
| SnippetADXLAccelerometers | 1,817,624 | 73,476 |
| SnippetAccelerometerCollision | 1,823,032 | 73,444 |
| SnippetAccelerometerFilter | 1,824,740 | 73,428 |
| SnippetAddressableLED | 1,804,648 | 75,428 |
| SnippetAnalogAccelerometer | 1,804,984 | 72,596 |
| SnippetAnalogEncoder | 1,806,124 | 72,596 |
| SnippetAnalogInput | 1,804,172 | 72,596 |
| SnippetAnalogPotentiometer | 1,805,196 | 72,596 |
| SnippetDifferentialDrive | 1,834,560 | 72,692 |
| SnippetDigitalCommunication | 1,795,660 | 72,308 |
| SnippetDigitalInput | 1,794,080 | 72,316 |
| SnippetDutyCycleEncoder | 1,807,348 | 72,556 |
| SnippetDutyCycleInput | 1,805,156 | 72,540 |
| SnippetEncoder | 1,812,436 | 72,948 |
| SnippetEncoderDrive | 1,847,048 | 73,284 |
| SnippetEncoderHoming | 1,843,388 | 73,244 |
| SnippetEventLoop | 1,863,316 | 73,332 |
| SnippetFlywheelBangBangController | 1,886,168 | 75,836 |
| SnippetI2CCommunication | 1,814,776 | 73,420 |
| SnippetLimitSwitch | 1,829,276 | 72,644 |
| SnippetMecanumDrive | 1,867,772 | 73,836 |
| SnippetMotorControl | 1,848,056 | 73,340 |
| SnippetOnboardIMU | 1,825,808 | 73,444 |
| SnippetProfiledPIDFeedforward | 1,861,664 | 73,276 |
| SnippetSelectCommand | 1,840,848 | 72,484 |
| StateSpaceArm | 2,179,660 | 73,332 |
| StateSpaceElevator | 2,179,668 | 73,332 |
| StateSpaceFlywheel | 2,017,596 | 73,332 |
| StateSpaceFlywheelSysId | 2,017,584 | 73,332 |
| SwerveBot | 2,044,648 | 74,484 |
| SwerveDrivePoseEstimator | 2,055,412 | 76,980 |
| SysIdRoutine | 2,008,384 | 76,220 |
| TankDriveGamepad | 1,833,004 | 72,692 |
| TemplateCommandItems | 1,850,556 | 72,484 |
| TemplateCommandV2 | 1,857,364 | 72,540 |
| TemplateCommandV2Skeleton | 1,831,536 | 72,484 |
| TemplateOpMode | 1,798,108 | 72,292 |
| TemplateTimed | 1,798,684 | 72,308 |
| TemplateTimedSkeleton | 1,786,708 | 72,292 |
| TemplateTimeslice | 1,801,948 | 72,308 |
| TemplateTimesliceSkeleton | 1,790,092 | 72,292 |
| TimedRobotDifferential | 1,873,636 | 73,332 |
| TimesliceRobotExample | 1,849,924 | 73,276 |
