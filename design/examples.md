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
are unchanged.

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

## Sizes

ESP32-S3 (`esp32:esp32:esp32s3`, huge_app), compiled with `tools/build/build.sh examples <name>`.
Flash is "Sketch uses", RAM is "Global variables use" (static RAM; the robot also allocates heap
at run time).

Measured on 2026-10-22 (after wave 5) against the `src/esp32s3/libprosdk.a` of that day; the
numbers move with the archive. The huge_app partition gives the sketch 3,145,728 bytes of flash;
static RAM is out of 327,680 bytes. Every sketch also gets probot-core's WiFi AP and Driver Station.

| Example | Flash (bytes) | RAM (bytes) |
|---|---|---|
| ArcadeDriveGamepad | 1,898,560 | 70,100 |
| ArmSimulation | 1,993,184 | 73,284 |
| DifferentialDriveBot | 1,964,412 | 71,876 |
| DifferentialDrivePoseEstimator | 2,086,476 | 74,372 |
| DriveDistanceOffboard | 1,944,832 | 70,188 |
| DriverStationDisplayAnsi | 1,867,168 | 69,812 |
| DriverStationDisplayLines | 1,887,972 | 69,828 |
| DSGamepadChooser | 1,907,152 | 69,900 |
| DutyCycleEncoder | 1,873,284 | 70,012 |
| ElevatorExponentialProfile | 1,866,204 | 69,820 |
| ElevatorExponentialSimulation | 1,988,544 | 73,284 |
| ElevatorProfiledPID | 1,934,628 | 70,724 |
| ElevatorSimulation | 1,998,500 | 73,284 |
| ElevatorTrapezoidProfile | 1,864,260 | 69,804 |
| Encoder | 1,877,688 | 70,356 |
| GettingStarted | 1,899,068 | 70,100 |
| Gyro | 1,932,692 | 71,228 |
| MecanumBot | 2,050,120 | 71,876 |
| MecanumDrive | 1,932,572 | 71,228 |
| MecanumDrivePoseEstimator | 2,058,840 | 74,436 |
| Mechanism2d | 1,939,424 | 71,012 |
| SimpleDifferentialDriveSimulation | 2,280,768 | 74,436 |
| SnippetAccelerometerCollision | 1,888,492 | 70,900 |
| SnippetAccelerometerFilter | 1,890,168 | 70,900 |
| SnippetAddressableLED | 1,869,936 | 72,836 |
| SnippetADXLAccelerometers | 1,883,064 | 70,868 |
| SnippetAnalogAccelerometer | 1,870,432 | 69,972 |
| SnippetAnalogEncoder | 1,871,556 | 69,972 |
| SnippetAnalogInput | 1,869,680 | 69,972 |
| SnippetAnalogPotentiometer | 1,870,640 | 69,972 |
| SnippetDifferentialDrive | 1,899,956 | 70,100 |
| SnippetDigitalCommunication | 1,861,176 | 69,716 |
| SnippetDigitalInput | 1,859,576 | 69,724 |
| SnippetDutyCycleEncoder | 1,872,844 | 70,012 |
| SnippetDutyCycleInput | 1,870,688 | 69,996 |
| SnippetEncoder | 1,877,904 | 70,356 |
| SnippetEncoderDrive | 1,912,480 | 70,692 |
| SnippetEncoderHoming | 1,908,832 | 70,716 |
| SnippetEventLoop | 1,928,320 | 70,724 |
| SnippetFlywheelBangBangController | 1,951,152 | 73,292 |
| SnippetI2CCommunication | 1,880,396 | 70,876 |
| SnippetLimitSwitch | 1,894,608 | 70,116 |
| SnippetMecanumDrive | 1,932,932 | 71,228 |
| SnippetMotorControl | 1,913,468 | 70,748 |
| SnippetOnboardIMU | 1,891,248 | 70,900 |
| SnippetProfiledPIDFeedforward | 1,927,152 | 70,668 |
| SnippetSelectCommand | 1,906,256 | 69,892 |
| StateSpaceArm | 2,243,780 | 70,724 |
| StateSpaceElevator | 2,243,788 | 70,724 |
| StateSpaceFlywheel | 2,081,896 | 70,740 |
| StateSpaceFlywheelSysId | 2,081,884 | 70,740 |
| SwerveBot | 2,109,116 | 71,876 |
| SwerveDrivePoseEstimator | 2,119,788 | 74,372 |
| SysIdRoutine | 2,072,780 | 73,612 |
| TankDriveGamepad | 1,898,380 | 70,100 |
| TemplateCommandItems | 1,915,856 | 69,892 |
| TemplateCommandV2 | 1,922,720 | 70,012 |
| TemplateCommandV2Skeleton | 1,896,868 | 69,892 |
| TemplateOpMode | 1,863,692 | 69,748 |
| TemplateTimed | 1,864,256 | 69,764 |
| TemplateTimedSkeleton | 1,852,276 | 69,764 |
| TemplateTimeslice | 1,867,484 | 69,764 |
| TemplateTimesliceSkeleton | 1,855,620 | 69,764 |
