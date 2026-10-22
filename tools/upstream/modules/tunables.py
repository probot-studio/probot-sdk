"""tunables: the full C++ module including MockTunableBackend and Selectable
(design/reports/telemetry_tunables.md). Python/Java bindings are not vendored."""

NAME = "tunables"
SPARSE = ["tunables"]

_N = "tunables/src/main/native"

RULES = [
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"]),
    dict(kind="source", src=_N, dst="src/prosdk/impl/tunables", include=["cpp/*.cpp"]),
    dict(kind="test", src="tunables/src/test/native/cpp", dst="test/tunables/cpp",
         include=["*"]),
]

_UPSTREAM_BUILD = ("upstream build file (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with "
                   "tools/build")

# upstream build files, documentation and bindings (no rule needed)
NOT_VENDORED = {
    "tunables/BUILD.bazel": _UPSTREAM_BUILD,
    "tunables/CMakeLists.txt": _UPSTREAM_BUILD,
    "tunables/build.gradle": _UPSTREAM_BUILD,
    "tunables/tunables-config.cmake.in": _UPSTREAM_BUILD,
    "tunables/robotpy_native_build_info.bzl": _UPSTREAM_BUILD,
    "tunables/robotpy_pybind_build_info.bzl": _UPSTREAM_BUILD,
    "tunables/src/test/native/BUILD.bazel": _UPSTREAM_BUILD,
    "tunables/README.md": "documentation, not code",
    "tunables/doc": "documentation, not code",
    "tunables/src/main/java": "Java API (no JVM on the MCU)",
    "tunables/src/main/python": "Python bindings",
    "tunables/src/test/java": "Java tests",
    "tunables/src/test/python": "Python tests",
}
