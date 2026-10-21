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

## Developer quick start

```sh
python3 tools/upstream/wpilib.py              # regenerate vendored sources (pinned upstream)
python3 tools/upstream/wpilib.py --check      # verify the tree matches the script output
tools/build/build.sh esp32s3                  # build src/esp32s3/libprosdk.a
tools/build/build.sh host --test              # build and run host unit tests
```

## License

prosdk's own code is under [LICENSE](LICENSE) and [LICENSE-commercial](LICENSE-commercial).
Vendored third-party code keeps its original license; see [NOTICE](NOTICE).
