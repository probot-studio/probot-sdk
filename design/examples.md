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

The channel number is the GPIO number (design/hal-esp32.md). The pins are the header pins of the
Probot main board, the Boardoza Pulse S32-S3 (design/boardoza/P14-existing.md section 3.1;
decision of 2026-10-23): J3 (IO1, IO2), J4 (IO10, IO11), J5 (IO7, IO15, IO16, IO17), J6 (IO35-38),
J7 (IO39-42) and J8 (IO45-48). The ports avoid:
- the pins the HAL refuses: 19/20 (USB), 43/44 (UART0, the console, on J4), 26-32 (flash and
  PSRAM), 22-25 (do not exist) and GPIO 3 (probot-core's NeoPixel, the Pulse's RGB LED);
- the strapping pins 0 (BOOT), 45 and 46 (J8), which the HAL allows;
- every GPIO that is not on a Pulse header (4-6, 8, 9, 12-14, 18, 21, 33, 34).

The Pulse module (ESP32-S3-WROOM-1-N4R2) has quad PSRAM, so GPIO 33-37 are free. An
octal-PSRAM devkit (N8R8, N16R8) wires 33-37 to its PSRAM and the HAL does not refuse them there:
move the motor pins 35-37 (for example to 4, 5, 12), see design/hal-esp32.md "Other ESP32-S3
boards".

SwerveBot and SwerveDrivePoseEstimator do not fit (24 signals, 20 header pins): they keep pins off
the headers and use GPIO 46 for the back-right turning encoder B, which a normal boot ignores (it
counts only while GPIO 0 is held low). They say so in their wiring. GPIO 45 and GPIO 0 are not used
by any example.

They also stay inside the hardware limits:
- 12 PWM outputs;
- 4 PCNT units shared by encoders and counters;
- 6 duty-cycle inputs;
- analog inputs only on GPIO 1-10 (on the headers: 1, 2, 7, 10);
- 3 LED strips.

| Use | GPIO (Pulse header) |
|---|---|
| PWM motor controllers | 38, 37, 36, 35 (J6); more: 47, 48 (J8) |
| Quadrature encoders A/B | 39/40, 41/42 (J7); 47/48 (J8); 15/16 (J5, when SPI is unused) |
| Digital inputs/outputs, duty cycle | 17, 16 (J5), 10, 11 (J4) |
| Analog inputs | 1, 2 (J3) without I2C; 7 (J5), 10 (J4) when J3 is I2C |
| Addressable LED strip | 48 (J8) |
| I2C port 0 (HAL default) | SDA 1, SCL 2 (J3) |
| I2C port 1 (Boardoza sensor examples) | SDA 17, SCL 16 (J5): not together with SPI bus 0 |
| SPI bus 0 (HAL default) | SCK 17, MOSI 15, MISO 16, CS 7 (J5): not together with I2C port 1 on J5 |
| Serial `ONBOARD` (HAL default) | RX 10, TX 11 (J4 F1/F2) |

No example uses analog inputs and I2C port 0 together. The same pins also fit the host simulation
HAL that the example tests use, which follows the ESP32-S3 HAL: DIO and PWM 0-48, analog 0-10
(`tools/upstream/patches/hal/0005`; the robot accepts analog 1-10). The exceptions are SwerveBot and SwerveDrivePoseEstimator.

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
  The port uses 39/40 and 41/42.
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
| Encoder | ported (second encoder moved to 41/42) | SnippetEncoder |
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
(`/Telemetry`). I2C port 1 is SDA 17 / SCL 16 and SPI bus 0 is the Pulse's J5 header (the HAL
default), so the two do not go together in one sketch. None of them has run on hardware.

| Example | Boards and classes | Bus and pins |
|---|---|---|
| BoardozaClimate | LM75ADP, HIH8131, TH09C, MS5611, HP303B, BME688 (`LM75A`, `HIH8000`, `ENS210`, `MS5611`, `DPS310`, `BME688`) | I2C port 1, SDA 17 / SCL 16, 100 kHz; BME688 on port 0 (SDA 1 / SCL 2) (`Wire`'s bus), since only two barometers fit on one port |
| BoardozaThermalCamera | MLX90640 (`MLX90640`; the MLX90641 works the same way) | I2C port 1, SDA 17 / SCL 16, 400 kHz |
| BoardozaMultizoneToF | VL53L7CX-1..4 (`VL53L7CX`, 8x8 at 15 Hz) | I2C port 1, SDA 17 / SCL 16, 400 kHz |
| BoardozaLoadCell | NAU7802, HX711 (`NAU7802`, `HX711`, `LoadCell`) | I2C port 1, SDA 17 / SCL 16, 400 kHz; HX711 DAT 38, CLK 37 |
| BoardozaGnss | SAM-M8Q or SAM-M10Q (`UbloxGnss` on I2C), GP-02 (`NmeaGnss`) | I2C port 1, SDA 17 / SCL 16, 100 kHz; serial `ONBOARD`, RX 10 / TX 11 |
| BoardozaSdLog | MicroSD (`platform::MountSdCard`, DataLogManager to `/sd/logs`) | SPI bus 0: SCK 17, MOSI 15, MISO 16, CS 7 |
| BoardozaRotaryArgb | Rotary Encoder ARGB (`ClockedAddressableLED`, `Encoder`, `DigitalInput`) | SPI bus 1 alone: SCK 39, MOSI 40; encoder A 41 / B 42; button 38 |
| BoardozaLoRa | E22-900M22S (`SX126x`). Regulatory: 868.1 MHz, 14 dBm, at most 1 % duty cycle (EU 868.0-868.6 MHz sub-band); check local rules before changing them | SPI bus 0: SCK 17, MOSI 15, MISO 16, CS 7; BUSY 38, DIO1 37, NRST 36, RXEN 35, TXEN 39 |
| BoardozaEveDisplay | FT810 or FT800 board and bundles (`FT8xx`, `ft8xx::kBoardozaFT810Bundle`) | SPI bus 0: SCK 17, MOSI 15, MISO 16, CS 7; PD 38, INT 37 |

All nine build with arduino-cli; their sizes are in the Sizes table below.

## Sizes

ESP32-S3 with the sketch FQBN of the Probot main board, the Boardoza Pulse S32-S3
(`esp32:esp32:esp32s3:FlashSize=4M,PartitionScheme=huge_app,PSRAM=enabled,CDCOnBoot=cdc`,
`tools/build/arduino_recipe.py --fqbn`), compiled with `tools/build/build.sh examples`. Flash is
"Sketch uses", RAM is "Global variables use" (static RAM; the robot also allocates heap at run time,
and on the Pulse malloc > 4 KiB goes to the 2 MB PSRAM).

Measured locally on 2026-10-23 (all 79 sketches in one run, prosdk `ceaa8c4`: the Pulse pin
defaults, CoreOps ABI 10) against probot-core `feat/gamepad-guide` e0cb17f (#45); not re-measured
against `feat/pulse-board` 3792357 (#46, CI's pin now), whose code changes are a few bytes (the USB
serial timeout call, the DS timeout log order, the INA pins; the HAL link test is 1,738,699 bytes on
e0cb17f and 1,738,683 on 3792357). The numbers move with the archive and
core; design/size.md keeps the history. The huge_app partition gives the sketch 3,145,728 bytes of
flash; static RAM is out of 327,680 bytes. The largest sketch is SimpleDifferentialDriveSimulation
at 2,229,814 bytes (71 %). Four sketches share the most static RAM, 78,128 bytes:
DifferentialDrivePoseEstimator, MecanumDrivePoseEstimator, SimpleDifferentialDriveSimulation and
SwerveDrivePoseEstimator. Every sketch also gets probot-core's WiFi AP and Driver Station. CI builds
all examples on pushes to `dev` and `stable`, on pull requests and on manual runs; to refresh this
table, take the "Sketch uses" / "Global variables use" lines after each "== <name>" line of the
examples log.

| Example | Flash (bytes) | RAM (bytes) |
|---|---|---|
| AprilTagFieldPose | 2,038,974 | 73,448 |
| ArcadeDriveGamepad | 1,847,390 | 73,776 |
| ArmSimulation | 1,941,966 | 76,976 |
| BoardozaClimate | 1,859,286 | 74,568 |
| BoardozaEveDisplay | 1,862,690 | 74,116 |
| BoardozaGnss | 1,871,018 | 74,656 |
| BoardozaLoadCell | 1,847,766 | 74,592 |
| BoardozaLoRa | 1,861,942 | 74,116 |
| BoardozaMultizoneToF | 1,936,246 | 74,600 |
| BoardozaRotaryArgb | 1,868,166 | 74,780 |
| BoardozaSdLog | 1,985,302 | 77,020 |
| BoardozaThermalCamera | 1,846,226 | 74,592 |
| CommandBasedRobot | 2,029,690 | 77,552 |
| DifferentialDriveBot | 1,913,518 | 75,568 |
| DifferentialDrivePoseEstimator | 2,035,450 | 78,128 |
| DriveDistanceOffboard | 1,893,746 | 73,880 |
| DriversShowcase | 1,946,210 | 75,048 |
| DriverStationDisplayAnsi | 1,812,762 | 73,512 |
| DriverStationDisplayLines | 1,833,482 | 73,512 |
| DSGamepadChooser | 1,852,762 | 73,576 |
| DutyCycleEncoder | 1,822,134 | 73,712 |
| ElevatorExponentialProfile | 1,814,950 | 73,504 |
| ElevatorExponentialSimulation | 1,937,382 | 76,976 |
| ElevatorProfiledPID | 1,883,706 | 74,416 |
| ElevatorSimulation | 1,947,138 | 76,976 |
| ElevatorTrapezoidProfile | 1,813,054 | 73,504 |
| Encoder | 1,826,506 | 74,104 |
| GettingStarted | 1,847,894 | 73,776 |
| Gyro | 1,881,774 | 74,920 |
| HalSmoke | 1,636,479 | 75,600 |
| MecanumBot | 1,999,154 | 75,632 |
| MecanumDrive | 1,881,670 | 74,920 |
| MecanumDrivePoseEstimator | 2,007,918 | 78,128 |
| Mechanism2d | 1,888,374 | 74,728 |
| OpModeRobotExample | 1,878,966 | 74,424 |
| SimpleDifferentialDriveSimulation | 2,229,814 | 78,128 |
| SnippetAccelerometerCollision | 1,837,478 | 74,600 |
| SnippetAccelerometerFilter | 1,839,214 | 74,584 |
| SnippetAddressableLED | 1,818,858 | 76,520 |
| SnippetADXLAccelerometers | 1,831,990 | 74,568 |
| SnippetAnalogAccelerometer | 1,819,334 | 73,688 |
| SnippetAnalogEncoder | 1,820,454 | 73,688 |
| SnippetAnalogInput | 1,818,534 | 73,688 |
| SnippetAnalogPotentiometer | 1,819,518 | 73,688 |
| SnippetDifferentialDrive | 1,848,774 | 73,776 |
| SnippetDigitalCommunication | 1,810,154 | 73,456 |
| SnippetDigitalInput | 1,808,514 | 73,472 |
| SnippetDutyCycleEncoder | 1,821,674 | 73,712 |
| SnippetDutyCycleInput | 1,819,478 | 73,696 |
| SnippetEncoder | 1,826,726 | 74,104 |
| SnippetEncoderDrive | 1,861,482 | 74,376 |
| SnippetEncoderHoming | 1,857,806 | 74,400 |
| SnippetEventLoop | 1,877,434 | 74,416 |
| SnippetFlywheelBangBangController | 1,899,954 | 76,984 |
| SnippetI2CCommunication | 1,829,642 | 74,568 |
| SnippetLimitSwitch | 1,843,398 | 73,800 |
| SnippetMecanumDrive | 1,882,022 | 74,920 |
| SnippetMotorControl | 1,862,214 | 74,424 |
| SnippetOnboardIMU | 1,840,250 | 74,600 |
| SnippetProfiledPIDFeedforward | 1,876,266 | 74,368 |
| SnippetSelectCommand | 1,855,050 | 73,576 |
| StateSpaceArm | 2,192,730 | 74,480 |
| StateSpaceElevator | 2,192,738 | 74,480 |
| StateSpaceFlywheel | 2,030,774 | 74,480 |
| StateSpaceFlywheelSysId | 2,030,766 | 74,480 |
| SwerveBot | 2,058,002 | 75,632 |
| SwerveDrivePoseEstimator | 2,068,646 | 78,128 |
| SysIdRoutine | 2,060,086 | 77,488 |
| TankDriveGamepad | 1,847,198 | 73,776 |
| TemplateCommandItems | 1,864,642 | 73,576 |
| TemplateCommandV2 | 1,871,546 | 73,696 |
| TemplateCommandV2Skeleton | 1,845,610 | 73,576 |
| TemplateOpMode | 1,809,238 | 73,448 |
| TemplateTimed | 1,812,994 | 73,448 |
| TemplateTimedSkeleton | 1,801,082 | 73,448 |
| TemplateTimeslice | 1,816,258 | 73,448 |
| TemplateTimesliceSkeleton | 1,804,354 | 73,448 |
| TimedRobotDifferential | 1,887,930 | 74,416 |
| TimesliceRobotExample | 1,864,470 | 74,368 |
