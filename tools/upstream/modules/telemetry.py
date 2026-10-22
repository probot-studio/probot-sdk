"""telemetry: the full C++ module including the Discard/Multi/Mock backends
(design/reports/telemetry_tunables.md). Python/Java bindings are not vendored."""

NAME = "telemetry"
SPARSE = ["telemetry"]

_N = "telemetry/src/main/native"

RULES = [
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"]),
    dict(kind="source", src=_N, dst="src/prosdk/impl/telemetry", include=["cpp/*.cpp"]),
    dict(kind="test", src="telemetry/src/test/native/cpp", dst="test/telemetry/cpp",
         include=["*"]),
]

_UPSTREAM_BUILD = ("upstream build file (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with "
                   "tools/build")

# upstream build files, documentation and bindings (no rule needed)
NOT_VENDORED = {
    "telemetry/BUILD.bazel": _UPSTREAM_BUILD,
    "telemetry/CMakeLists.txt": _UPSTREAM_BUILD,
    "telemetry/build.gradle": _UPSTREAM_BUILD,
    "telemetry/telemetry-config.cmake.in": _UPSTREAM_BUILD,
    "telemetry/robotpy_native_build_info.bzl": _UPSTREAM_BUILD,
    "telemetry/robotpy_pybind_build_info.bzl": _UPSTREAM_BUILD,
    "telemetry/src/test/native/BUILD.bazel": _UPSTREAM_BUILD,
    "telemetry/README.md": "documentation, not code",
    "telemetry/doc": "documentation, not code",
    "telemetry/src/main/java": "Java API (no JVM on the MCU)",
    "telemetry/src/main/python": "Python bindings",
    "telemetry/src/test/java": "Java tests",
    "telemetry/src/test/python": "Python tests",
}
