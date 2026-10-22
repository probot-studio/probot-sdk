# prosdk

prosdk is the robot programming library for the Probot ESP32-S3 kit. Its C++ API is compatible with
WPILib 2027: the same class and method names, with the root namespace `prosdk::` instead of `wpi::` and
the include root `<prosdk/...>` instead of `<wpi/...>`. It runs on top of the probot core library
(`depends=probot`), which owns WiFi, the browser Driver Station and the robot state machine.

**Status: in development (2027.0.0-alpha).** Not yet usable for robots. The previous library
(Probot Lib 0.2.x) stays on the `main` branch.

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

- `TimedRobot`, `TimesliceRobot` and `OpModeRobot` are supported. The robot is constructed when
  probot-core boots and its loop runs inside core's loop; core's loop period follows the robot's
  periods (`TimedRobot(10_ms)`, `AddPeriodic`), 1 ms to 1000 ms.
- Modes and opmodes come from the browser Driver Station (autonomous, teleop, utility; the opmodes
  of an `OpModeRobot` are listed there after `PublishOpModes()`).
- Telemetry and tunables appear on the Driver Station page; text and errors in its console.
- If the constructor or the loop throws, the error is shown and the program stops with outputs
  disabled until reboot. An e-stop always turns the outputs off.
- Examples: `examples/TimedRobotDifferential`, `examples/OpModeRobotExample`,
  `examples/TimesliceRobotExample`, `examples/CommandBasedRobot` (command-based, `prosdk::cmd`),
  `examples/DriversShowcase` (every driver in `prosdk/drivers`, design/drivers.md). Details: [design/PORT.md](design/PORT.md) section 8.1.

## Developer quick start

```sh
python3 tools/upstream/wpilib.py              # regenerate vendored sources (pinned upstream)
python3 tools/upstream/wpilib.py --check      # verify the tree matches the script output
tools/build/build.sh esp32s3                  # build src/esp32s3/libprosdk.a
tools/build/build.sh esp32s3 header_check     # compile every public header after <Arduino.h>
tools/build/build.sh host --test              # build and run host unit tests
```

## License

prosdk's own code is under [LICENSE](LICENSE) and [LICENSE-commercial](LICENSE-commercial).
Vendored third-party code keeps its original license; see [NOTICE](NOTICE).
