# prosdk

prosdk is the robot programming library for the Probot ESP32-S3 kit. Its C++ API is compatible with
WPILib 2027: the same class and method names, with the root namespace `prosdk::` instead of `wpi::` and
the include root `<prosdk/...>` instead of `<wpi/...>`. It runs on top of the probot core library
(`depends=probot`), which owns WiFi, the browser Driver Station and the robot state machine.

**Status: alpha (2027.0.0-alpha.1), not released.** A port of WPILib 2027 (allwpilib `b68effca`)
on the probot-core 0.4.0 draft stack (core PRs #26-#45, not released yet).

- Works and is checked: the vendored libraries with WPILib's own host tests, the ESP32-S3 archive,
  all example sketches compile in CI, and a QEMU smoke test runs the archive on the emulated chip
  (math, threads, storage, scheduler, the hosted `TimedRobot`).
- Needs hardware validation: nothing has run on an ESP32-S3 board yet, so the HAL peripherals, the
  Driver Station end to end, real-time loop timing and every driver are unverified
  (design/audit-final.md section 6.1).
- Not in the first release: an NT4 server for AdvantageScope/Elastic live connection (deferred;
  `.wpilog` files from DataLogManager work with both).

The previous library (Probot Lib 0.2.x) stays on the `stable` branch (commit `ea2ea48`).

- Engineering contract: [design/PORT.md](design/PORT.md)
- Differences from the upstream API: [design/API-DIFFERENCES.md](design/API-DIFFERENCES.md)
- Attribution of vendored code: [NOTICE](NOTICE) and [licenses/](licenses/)

## How to write a robot

Write a WPILib robot class as usual and end the sketch with `PROSDK_ROBOT_MAIN(Robot)` instead of a
`main()` function. Put core's configuration macros before `#include <prosdk.h>`; the sketch has no
`setup()`/`loop()` (probot-core owns them).

```cpp
#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  void TeleopPeriodic() override { /* drive */ }
};

PROSDK_ROBOT_MAIN(Robot)
```

- Board: the Boardoza ESP32-S3 Pulse (the Probot main board for now; ESP32-S3-WROOM-1-N4R2,
  4 MB flash, 2 MB quad PSRAM, USB-C). Arduino IDE settings: ESP32S3 Dev Module, Arduino-ESP32
  3.3.10 (the version the precompiled archive is built with), Flash Size 4MB, Partition Scheme
  Huge APP (3MB No OTA/1MB SPIFFS), PSRAM QSPI PSRAM, USB Mode Hardware CDC and JTAG, USB CDC On
  Boot Enabled, Upload Mode UART0 / Hardware CDC. FQBN:
  `esp32:esp32:esp32s3:FlashSize=4M,PartitionScheme=huge_app,PSRAM=enabled,CDCOnBoot=cdc`.
  probot-core alone needs about 1 MB of flash. The bus default pins are the Pulse headers: I2C
  port 0 SDA 1 / SCL 2 (J3), serial `ONBOARD` RX 10 / TX 11 (J4), SPI bus 0 SCK 17 / MOSI 15 /
  MISO 16 (J5); the examples use J6-J8 for motors and encoders (design/examples.md "Pin map").
- Other ESP32-S3 boards: the robot's first member moves the buses before any device is built,
  for example `bool m_pinsConfigured = prosdk::platform::SetI2CPins(0, 8, 9) &&
  prosdk::platform::SetSerialPins(0, 15, 16) && prosdk::platform::SetSPIPins(0, 12, 11, 13);`
  (the same pattern as `prosdk::prohub::ConfigurePins()`). Set USB CDC On Boot for the USB port
  in use (Disabled when a USB-UART bridge port carries `Serial`). On octal-PSRAM modules (N8R8,
  N16R8) GPIO 33-37 are refused, so the examples' motor pins 35-37 must move (for example to 4, 5,
  12). Details: design/hal-esp32.md "Boards".
- `TimedRobot`, `TimesliceRobot` and `OpModeRobot` are supported. The robot is constructed when
  probot-core boots and its loop runs inside core's loop; core's loop period follows the robot's
  periods (`TimedRobot(10_ms)`, `AddPeriodic`), 1 ms to 1000 ms.
- Modes and opmodes come from the browser Driver Station (autonomous, teleop, utility; the opmodes
  of an `OpModeRobot` are listed there after `PublishOpModes()`).
- Telemetry and tunables appear on the Driver Station page; text and errors in its console.
- If the constructor or the loop throws, the error is shown and the program stops with outputs
  disabled until reboot. An e-stop always turns the outputs off.
- Examples (79 sketches; sizes in [design/examples.md](design/examples.md) "Sizes"):
  - 63 ports of WPILib's C++ examples, snippets (`examples/Snippet*`) and project templates
    (`examples/Template*`); start with `examples/TemplateTimed`. Decisions for every upstream
    example: [design/examples.md](design/examples.md).
  - prosdk examples: `examples/TimedRobotDifferential`, `examples/OpModeRobotExample`,
    `examples/TimesliceRobotExample`, `examples/CommandBasedRobot` (command-based, `prosdk::cmd`),
    `examples/DriversShowcase` (every driver in `prosdk/drivers`, design/drivers.md),
    `examples/AprilTagFieldPose` (robot pose from an AprilTag on the built-in field layout),
    `examples/HalSmoke` (HAL peripheral test). Details: [design/PORT.md](design/PORT.md) section 8.1.
  - 9 Boardoza board examples (`examples/Boardoza*`, design/boardoza.md).

## Developer quick start

```sh
python3 tools/upstream/wpilib.py              # regenerate vendored sources (pinned upstream)
python3 tools/upstream/wpilib.py --check      # verify the tree matches the script output
tools/build/build.sh esp32s3                  # build src/esp32s3/libprosdk.a
tools/build/build.sh esp32s3 header_check     # compile every public header after <Arduino.h>
tools/build/build.sh host --test              # build and run host unit tests
tools/build/build.sh examples                 # compile examples/* (Pulse FQBN) against the archive
python3 tools/upstream/examples.py --check    # ported examples still match the upstream pin
tools/release/package.sh                      # release zip in .build/package, smoke compile
```

## License

prosdk's own code is under [LICENSE](LICENSE) and [LICENSE-commercial](LICENSE-commercial).
Vendored third-party code keeps its original license; see [NOTICE](NOTICE).
