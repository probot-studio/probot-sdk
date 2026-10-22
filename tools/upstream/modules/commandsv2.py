"""commandsv2: the WPILib 2027 command-based framework (design/reports/commands.md), namespace
prosdk::cmd: Command, CommandPtr, CommandHelper, Subsystem, SubsystemBase, CommandScheduler, every
command type and the Commands factories, buttons (Trigger, JoystickButton, POVButton,
GamepadButton, RobotModeTriggers, CommandGenericHID, CommandGamepad, CommandJoystick and the 15
generated Command*Controller wrappers, committed upstream under commandsv2/src/generated) and
sysid/SysIdRoutine.

- Public headers go to src/prosdk/commands2/ (main and generated include roots).
- Sources go to src/prosdk/impl/commandsv2/ (cpp/ and generated/) and into libprosdk.a.
- Host unit tests (Catch2, against the HAL sim backend) go to test/commandsv2/.

Patches (tools/upstream/patches/commandsv2):
- RTTI: commands take their name from CommandHelper<Base, CRTP> (the CRTP type, without RTTI);
  Subsystem/SubsystemBase keep the static type name of wpiutil/0004 (design/PORT.md section 7).
- NetworkButton is re-implemented over tunables (ntcore is not ported, PORT.md section 10).
- Console output (PrintCommand, scheduler warnings) goes to the platform console.
- The scheduler builds its watchdog epoch names in a reused buffer (no allocation per command per
  Run(); the Tracer side is wpilibc/0014).
Every behavior change is listed in design/API-DIFFERENCES.md.

commandsv3 is Java-only (JVM continuations) and has no C++ to vendor.
"""

NAME = "commandsv2"
SPARSE = ["commandsv2"]

_S = "commandsv2/src"
_N = f"{_S}/main/native"
_G = f"{_S}/generated/main/native"

RULES = [
    # public headers
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"]),
    # generated HID command wrappers (committed upstream)
    dict(kind="header", src=f"{_G}/include", dst="src", include_base="src", include=["*"]),
    # sources
    dict(kind="source", src=_N, dst="src/prosdk/impl/commandsv2", include=["cpp/*.cpp"]),
    dict(kind="source", src=f"{_G}/cpp/wpi/commands2", dst="src/prosdk/impl/commandsv2/generated",
         include=["*.cpp"]),
    # host unit tests (Catch2, against the HAL sim backend)
    dict(kind="test", src=f"{_S}/test/native/cpp", dst="test/commandsv2/cpp", include=["main.cpp"]),
    dict(kind="test", src=f"{_S}/test/native/cpp/wpi/command", dst="test/commandsv2/cpp",
         include=["*"]),
]

# Not vendored at all, with reasons.
NOT_VENDORED = {
    f"{_S}/generate": "Jinja templates of the Command*Controller wrappers; their output "
                      "(commandsv2/src/generated) is committed upstream and vendored instead",
    f"{_S}/main/java": "Java API (no JVM on the MCU)",
    f"{_S}/test/java": "Java tests",
    f"{_S}/main/python": "Python bindings (RobotPy commands2)",
    f"{_S}/test/python": "Python tests",
    f"{_S}/test/resources": "JUnit service registration (Java tests)",
    f"{_S}/test/native/BUILD.bazel": "Bazel build file",
    f"{_S}/dev": "upstream developer scratch executable",
    "commandsv2/{BUILD.bazel,CMakeLists.txt,build.gradle,commandsv2-config.cmake.in,generate.bzl}":
        "upstream build files (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with tools/build",
    "commandsv2/CommandsV2.json": "vendordep descriptor for the desktop/roboRIO build system",
    "commandsv2/generate_hids.py": "generator of the Command*Controller wrappers (its C++ output is committed upstream and vendored)",
    f"{_S}/generated/main/java": "generated Java Command*Controller wrappers (no JVM on the MCU)",
}
