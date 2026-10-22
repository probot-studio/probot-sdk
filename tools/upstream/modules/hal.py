"""hal: the 2027 HAL C API (every public header), the backend-independent sources and the WPILib
simulation backend (design/reports/hal.md).

- Public headers go to src/prosdk/hal/ (the include root of every backend).
- Backend-independent sources (handles, error handling, Main, UsageReporting, the DS front end
  that forwards to MrcLibDs) go to src/prosdk/impl/hal/ and into libprosdk.a.
- The sim backend goes to host/hal/sim/, outside the Arduino library tree (src/), and is compiled
  only by the host build (unit tests, desktop simulation). The ESP32 backend is hand-written in
  src/prosdk/platform/hal/esp32/ (systemcore is its reference, never vendored into src/).
Everything else is excluded with a reason (listed in design/API-DIFFERENCES.md).
"""

NAME = "hal"
SPARSE = ["hal"]
TRANSFORMS = {"fs": True}

_N = "hal/src/main/native"

RULES = [
    # public headers
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"],
         exclude={
             "wpi/hal/.clang-tidy": "lint configuration, not source",
             "wpi/hal/SystemServer.h": "returns an ntcore instance (NT_Inst) for the SystemCore "
                                       "system server; ntcore is not ported and there is no "
                                       "system server (its only users are the ExpansionHub classes)",
             "wpi/hal/cpp/MrcLibAlert.hpp": "SystemCore mrclib alert transport; its implementation "
                                           "(cpp/mrclib) is not ported",
         }),
    # backend-independent sources (archive and host)
    dict(kind="source", src=_N, dst="src/prosdk/impl/hal",
         include=["cpp/*.cpp"],
         exclude={
             "cpp/jni/*": "JNI (no JVM on the MCU)",
             "cpp/mrclib/*": "SystemCore mrclib Driver Station and alert transport; the probot-core "
                             "Driver Station backend replaces it",
             "cpp/DashboardOpMode.cpp": "publishes the opmode choosers over ntcore (not ported); "
                                        "replaced by a hand-written no-op "
                                        "(src/prosdk/platform/hal/DashboardOpMode.cpp)",
         }),
    # simulation backend (host build only)
    dict(kind="source", src=_N, dst="host/hal", include=["sim/*"],
         exclude={
             "sim/SystemServer.cpp": "implements HAL_GetSystemServerHandle (SystemServer.h, "
                                     "not vendored)",
         }),
    # host unit tests (Catch2, against the sim backend)
    dict(kind="test", src="hal/src/test/native/cpp", dst="test/hal/cpp", include=["*"]),
]

# Not vendored at all, with reasons.
NOT_VENDORED = {
    f"{_N}/systemcore": "SystemCore (Linux) hardware backend; reference for the hand-written ESP32 "
                        "backend (src/prosdk/platform/hal/esp32), never compiled as is",
    "hal/src/main/java": "Java bindings (no JVM on the MCU)",
    "hal/src/main/python": "Python bindings",
    "hal/src/dev": "desktop development executable",
    "hal/src/test/java": "Java tests",
    "hal/src/test/python": "Python tests",
}
