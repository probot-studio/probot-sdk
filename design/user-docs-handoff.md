# User docs handoff: what probot-docs must document for prosdk

For probot-docs (probotstudio.com/docs). Release checklist item 18 (`design/audit-final.md` section
6.4). Written 2026-10-23 at prosdk `79cdeb7` (branch `dev`) against probot-core's draft stack up to
#42 (`feat/dash-set-refused` 940d300, `library.properties` version 0.4.0). Today the docs mention
probot-sdk only as a future library (`docs/ornekler.md:917`).

Every item below names the source in this repository. The source is the reference; this file only
lists what users need and what to warn them about. Nothing in prosdk has run on an ESP32-S3 board
yet (`audit-final.md` section 6.1); pages that describe hardware behaviour should say so until the
hardware validation is done.

## 1. Installation

- Library name `prosdk`, depends on probot-core (`probot`); `library.properties`. Library Manager
  installs probot-core with it. Not published yet (no tag, no release, no Library Manager entry):
  the docs go live together with the release.
- Precompiled: `precompiled=full`, the archive `src/esp32s3/libprosdk.a` (about 63 MB,
  debug-stripped; zip about 21 MB, `tools/release/package.sh`). Only the ESP32-S3 is supported.
- Board package: Arduino-ESP32 **3.3.10** (`esp32:esp32`), the version the archive is built with
  (`tools/build/esp32s3.cmake`, CI `ARDUINO_ESP32_VERSION`). Tell users to install exactly this
  version; another one is untested and the library has no compile-time check for it.
- Board settings: ESP32S3 Dev Module (`esp32:esp32:esp32s3`), **Partition Scheme: Huge APP (3MB No
  OTA/1MB SPIFFS)**. probot-core alone is about 1.08 MB; prosdk sketches are 1.6-2.3 MB
  (`design/PORT.md` 7.1, `design/examples.md` "Sizes").
- probot-core version: prosdk needs probot-core 0.4.0 or later (the framework hooks R1-R4). Older
  core features are detected at compile time; what each core feature adds is in
  `design/core-requests.md` "Status". Until core 0.4.0 is released this is a draft stack.

## 2. First robot

- Source: `README.md` "How to write a robot", `design/PORT.md` 8.1, `examples/TemplateTimed`.
- `#include <prosdk.h>`, a WPILib robot class, `PROSDK_ROBOT_MAIN(Robot)` at the end. No
  `setup()`/`loop()`: probot-core owns them. Core configuration macros (`PROBOT_WIFI_AP_PASSWORD`,
  ...) go before the include (core docs).
- Names: `wpi::` -> `prosdk::`, `<wpi/...>` -> `<prosdk/...>`, `frc2::` command-based ->
  `prosdk::cmd` (`design/API-DIFFERENCES.md` section 1); identifiers renamed because Arduino.h
  defines them as macros (section 2).
- Robot classes: `TimedRobot`, `TimesliceRobot`, `OpModeRobot` (`examples/TimedRobotDifferential`,
  `TimesliceRobotExample`, `OpModeRobotExample`, `TemplateOpMode`). Not supported: a class that
  derives from `RobotBase` directly with a blocking `StartCompetition()` (`examples.md`,
  robotbaseskeleton).
- Loop timing: the robot runs inside core's loop; the pass period is the GCD of the robot's periods,
  1-1000 ms (`API-DIFFERENCES.md` section 3, "Robot loop timing on the robot"). No real-time
  guarantee is measured yet.
- Exceptions: an exception from the constructor or the loop is shown on the Driver Station and the
  program halts with outputs off until reboot (`PORT.md` 8.1 "Exceptions").
- Command-based: `examples/CommandBasedRobot`, `TemplateCommandV2`, `TemplateCommandItems`
  (`prosdk::cmd`); command names come from the CRTP class (no RTTI; `API-DIFFERENCES.md` section 3,
  `CommandHelper`), subsystems are named with `SubsystemBase(name)` or `SetName()`.

## 3. Pins and hardware

- Source: `design/hal-esp32.md` "Channel model", "Capacities", "Output safety", "Not supported".
- Channel = GPIO number for every pin API (no roboRIO-style DIO 0-9). Pins refused and why (USB,
  UART0, flash/PSRAM, core's pins, pins already used by Arduino code).
- Capacities: 12 PWM outputs, 4 counters/encoders, 6 duty-cycle inputs, 10 analog inputs (ADC1),
  3 LED/pulse outputs, 2 I2C, 2 serial, 1 CAN (external transceiver), 2 SPI buses.
- Output safety: outputs are gated while disabled and on Driver Station loss; e-stop drives outputs
  low. PCA9685 limitation: `design/drivers.md` "PCA9685 and ProHub".
- Not on the kit (API compiles, does nothing or reports an error): pneumatics, PowerDistribution,
  OnboardIMU, DIO pulses, interrupts (`API-DIFFERENCES.md` section 3).
- The kit electronics and pin map are not final (KIT.md section 3); the examples' pin map is in
  `design/examples.md` "Pin map". Check before publishing wiring pages.

## 4. Driver Station

- Modes and opmodes come from core's browser Driver Station; `OpModeRobot` lists its opmodes after
  `PublishOpModes()` (`README.md`).
- Gamepads: ports 0-5, analog triggers, 8-way POV, names; mapped to WPILib's Gamepad layout. No
  rumble, LEDs or touchpads (`API-DIFFERENCES.md` section 3, "Joysticks").
- No alliance, match number or game data from the probot DS (core request R18); `DriverStationSim`
  has them on the host only (`examples.md` "Example tests").
- Console: lines carry a level (error/warning/info), shown on the DS Logs page
  (`API-DIFFERENCES.md` section 3, "Console levels"). `DriverStationDisplay` shows as plain text in the topic tree (`/Telemetry/DriverStationDisplay/text`), not the console.

## 5. Telemetry, tunables and widgets

- Source: `API-DIFFERENCES.md` section 3 ("Telemetry, tunables and alerts on the Driver Station",
  "Preferences on the Driver Station", `NetworkButton`, `NetworkBooleanEvent`, `CommandScheduler`
  rows), `design/core-requests.md` "P1 R5 delivered" and "Widget layouts".
- `prosdk::telemetry::Log(...)` shows under `/Telemetry`, `prosdk::tunables::Publish(...)` under
  `/Tunables` (editable) on the DS data page. NetworkTables do not exist on the robot.
- DS widgets chosen by `.type`: `Selectable` (auto chooser), `Command` (run/cancel), `PIDController`,
  `Alerts`, `Scheduler`, `Field2d`, `Mechanism2d` (the last three from core #40). Other structs show
  as a decoded tree.
- Preferences: stored in NVS, mirrored on the DS as `/Preferences/<key>` (editable).
- Persistent tunables: opt-in with `TunableConfig::properties` `{"persistent": true}`; only Driver
  Station writes the robot accepted are stored; a stored tune wins over a changed code default until
  `prosdk::platform::ClearPersistedTunables()`; stored values at most 2048 bytes and
  `PROBOT_DASH_MAX_VALUE` (`API-DIFFERENCES.md` section 7, "Persistent tunables"; the full rule is
  long, the docs need the user-facing summary and the "call `ClearPersistedTunables()` once after
  changing a default" advice).
- Limits (core macros, set in the sketch before the include): `PROBOT_DASH_MAX_TOPICS` 128,
  `PROBOT_DASH_MAX_VALUE` 1024 bytes, `PROBOT_DASH_POOL_BYTES` 32768, `PROBOT_DASH_MAX_SCHEMAS`,
  `PROBOT_DASH_MAX_SCHEMA_BYTES` (`core-requests.md` "P1 R5 delivered", R13; core `API.md`). A value
  larger than the limit is not sent: the console reports it once and the DS marks the topic
  "Güncel değil" (core #42, R16). `Field2d` with many poses is the usual case; the sizing advice is
  in `API-DIFFERENCES.md` section 3 and `src/prosdk/platform/dashboard/DashboardStore.hpp`.
- Not available: NT4 server, AdvantageScope and Elastic live connection (deferred out of the first
  stable release); the dashboard opmode chooser API is a no-op (`API-DIFFERENCES.md` section 3).

## 6. DataLog and files

- Source: `API-DIFFERENCES.md` section 3 (`DataLogManager` rows, `DataLog` buffers),
  `examples/SysIdRoutine`, `examples/BoardozaSdLog`, `core-requests.md` "P1 R11 wired", R17.
- `DataLogManager::Start()` logs to `/littlefs/logs` when the sketch mounted LittleFS
  (`LittleFS.begin()`), else nothing is written (console line says so). SD card:
  `prosdk::platform::MountSdCard()` and `SetLogDirectory("/sd/logs")`.
- File names get the real UTC date once the DS has set the robot clock (core R10).
- Telemetry can be copied into the log as `NT:/Telemetry/<path>` (the keys AdvantageScope knows
  from upstream logs). On the robot the copy is off by default: call
  `DataLogManager::LogNetworkTables(true)` after `Start()`. Say how fast it fills LittleFS: a double
  record is about 15 B, 20 doubles changing every 20 ms loop write about 15 KB/s, and the 896 KiB
  partition is full in about a minute (log few or rarely changing values, or log to an SD card).
  Tunables, Preferences and the DriverStationDisplay screen are not logged; alerts are.
- Download: the DS "Dosyalar" card on the Logs page lists and downloads the log folder (share
  `"logs"`), only while the robot is idle. A sketch that unmounts a card must call `Stop()` and then
  wait until `probot::files::busy()` is false.
- ESP32 buffer sizes and old-log deletion limits differ from upstream (`API-DIFFERENCES.md`
  section 3).
- `.wpilog` files open in AdvantageScope and WPILib's DataLogTool as usual.

## 7. Drivers

- Source: `design/drivers.md`, `API-DIFFERENCES.md` section 7, `examples/DriversShowcase`.
- Vendored WPILib drivers: PWM motor controllers (Spark, ...) and GoBildaPinpoint.
- Probot drivers: `Servo` (WPILib 2026 API), BTS7960, VNH5019, PCA9685 with `ProHubMotor`, ProHub
  encoder stream, MPU6050, the extra H-bridges and BLDC drivers.
- Boardoza boards: `design/boardoza.md` (board table, 5 V hazards: the ESP32-S3 pins are not 5 V
  tolerant, I2C addresses, threads), `design/drivers.md` "Boardoza boards", and the nine
  `examples/Boardoza*` sketches with their wiring in the `.ino` headers. None has run on hardware.

## 8. Differences from WPILib and what is not ported

- Source: `design/API-DIFFERENCES.md` (sections 1-5), `design/PORT.md` section 10.
- User-facing summary: no NetworkTables/NT4, no cameras (cscore, cameraserver), no pneumatics or
  PDP/PDH hardware, no Romi/XRP, no desktop tools (Glass, SysId app; SysId logs work and open in
  the desktop SysId tool), no Java/Python. Field images are not included.
- Examples: `design/examples.md` lists all upstream examples, which are ported and why the others
  are not; `examples/Snippet*` and `examples/Template*` are the snippet and template ports.

## 9. Status statements for the docs

- Version `2027.0.0-alpha.1`; based on WPILib 2027 at allwpilib `b68effca`.
- Verified: host unit tests (WPILib's own tests plus prosdk's), all 79 example sketches compile in
  CI, a QEMU smoke test of the ESP32-S3 archive. Not verified: anything on a real board (peripherals,
  WiFi/DS end to end, timing, drivers).
