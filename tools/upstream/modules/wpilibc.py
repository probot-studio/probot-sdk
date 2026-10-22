"""wpilibc: the WPILib 2027 robot library (design/reports/wpilibc.md): framework (RobotBase,
IterativeRobotBase, TimedRobot, TimesliceRobot, OpModeRobot), opmodes, Driver Station and every
HID class (incl. the 15 generated controllers and their *Sim classes, committed upstream under
wpilibc/src/generated), events, hardware (incl. pneumatics, PowerDistribution, OnboardIMU and CAN,
which compile and fail at runtime through the ESP32 HAL stubs), drive, LED, system, sysid,
preferences, simulation, smartdashboard (Field2d, Mechanism2d) and the DataLog telemetry backend.

- Public headers go to src/prosdk/ (main and generated include roots).
- Sources go to src/prosdk/impl/wpilibc/ (cpp/ and generated/) and into libprosdk.a.
- Host unit tests (Catch2, against the HAL sim backend) go to test/wpilibc/.

ntcore is not ported (design/PORT.md section 10): the NetworkTables backends, SystemServer and the
ExpansionHub classes (SystemCore "rhsp" server over NT topics) are excluded; the NT uses in
RobotBase, IterativeRobotBase, OpModeRobot, DriverStationBackend, Preferences and DataLogManager are
removed by patches (tools/upstream/patches/wpilibc), and NetworkBooleanEvent (with its test) is
re-implemented over tunables by a patch. Every exclusion is listed in design/API-DIFFERENCES.md.
"""

NAME = "wpilibc"
SPARSE = ["wpilibc"]
TRANSFORMS = {"fs": True}

_S = "wpilibc/src"
_N = f"{_S}/main/native"
_G = f"{_S}/generated/main/native"

_NT_TELEMETRY = ("NetworkTables telemetry backend (ntcore is not ported); replaced by the "
                 "probot-core dashboard backend (src/prosdk/platform/dashboard)")
_NT_TUNABLE = ("NetworkTables tunable backend (ntcore is not ported); replaced by the "
               "probot-core dashboard backend (src/prosdk/platform/dashboard)")
_SYSTEM_SERVER = ("returns the SystemCore system server's ntcore instance (HAL SystemServer.h is "
                  "not vendored; ntcore is not ported)")
_EXPANSION_HUB = ("REV Expansion Hub through the SystemCore USB 'rhsp' server, driven over NT "
                  "topics (/rhsp/...); there is no rhsp server and ntcore is not ported")

RULES = [
    # public headers
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"],
         exclude={
             "wpi/backend/NetworkTablesTelemetryBackend.hpp": _NT_TELEMETRY,
             "wpi/backend/NetworkTablesTunableBackend.hpp": _NT_TUNABLE,
             "wpi/system/SystemServer.hpp": _SYSTEM_SERVER,
             "wpi/hardware/expansionhub/*": _EXPANSION_HUB,
         }),
    # generated HID controllers and their *Sim classes (committed upstream)
    dict(kind="header", src=f"{_G}/include", dst="src", include_base="src", include=["*"]),
    # sources
    dict(kind="source", src=_N, dst="src/prosdk/impl/wpilibc", include=["cpp/*.cpp"],
         exclude={
             "cpp/backend/NetworkTablesTelemetryBackend.cpp": _NT_TELEMETRY,
             "cpp/backend/NetworkTablesTunableBackend.cpp": _NT_TUNABLE,
             "cpp/system/SystemServer.cpp": _SYSTEM_SERVER,
             "cpp/hardware/expansionhub/*": _EXPANSION_HUB,
         }),
    dict(kind="source", src=f"{_G}/cpp", dst="src/prosdk/impl/wpilibc/generated", include=["*"]),
    # host unit tests (Catch2, against the HAL sim backend)
    dict(kind="test", src=f"{_S}/test/native/cpp", dst="test/wpilibc/cpp", include=["*"],
         exclude={
             "NetworkTablesTelemetryBackendTest.cpp": _NT_TELEMETRY,
             "NetworkTablesTunableBackendTest.cpp": _NT_TUNABLE,
             "UnitNetworkTablesTest.cpp": "unit-typed NT topics (ntcore is not ported)",
             "hardware/ExpansionHubMotorTest.cpp": _EXPANSION_HUB,
         }),
    dict(kind="test-header", src=f"{_S}/test/native/include", dst="test/wpilibc/include",
         include_base="test/wpilibc/include", include=["*"]),
    dict(kind="test", src=f"{_S}/generated/test/native/cpp", dst="test/wpilibc/generated",
         include=["*"]),
]

_UPSTREAM_BUILD = ("upstream build file (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with "
                   "tools/build")

# Not vendored at all, with reasons.
NOT_VENDORED = {
    f"{_S}/test/native/c": "C compile check of HAL.h together with ntcore.h (ntcore is not ported)",
    f"{_S}/generate": "Jinja templates of the HID classes; their output (wpilibc/src/generated) "
                      "is committed upstream and vendored instead. WPILibVersion.cpp.in: "
                      "hand-written src/prosdk/platform/WPILibVersion.cpp",
    f"{_S}/main/python": "Python bindings",
    f"{_S}/test/python": "Python tests",
    f"{_S}/dev": "upstream developer scratch executable",
    # upstream build files, generators, documentation and bindings (no rule needed)
    "wpilibc/BUILD.bazel": _UPSTREAM_BUILD,
    "wpilibc/CMakeLists.txt": _UPSTREAM_BUILD,
    "wpilibc/build.gradle": _UPSTREAM_BUILD,
    "wpilibc/publish.gradle": _UPSTREAM_BUILD,
    "wpilibc/wpilibc-config.cmake.in": _UPSTREAM_BUILD,
    "wpilibc/robotpy_native_build_info.bzl": _UPSTREAM_BUILD,
    "wpilibc/robotpy_pybind_build_info.bzl": _UPSTREAM_BUILD,
    "wpilibc/generate.bzl": _UPSTREAM_BUILD,
    "wpilibc/src/test/native/BUILD.bazel": _UPSTREAM_BUILD,
    "wpilibc/generate_hids.py":
        "generator of the HID classes (Jinja templates in wpilibc/src/generate); its output is "
        "committed upstream and vendored",
    "wpilibc/generate_first_ds_hids.py":
        "generator of the FIRST Driver Station HID classes; its output is committed upstream and "
        "vendored",
    "wpilibc/generate_wpilibc.py":
        "upstream code generator driver; its output is committed upstream and vendored",
}
