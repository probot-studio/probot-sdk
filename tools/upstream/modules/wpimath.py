"""wpimath: all non-JNI C++ of wpimath, i.e. wpi/math (16 submodules incl. struct/ and proto/),
wpi/units and the committed nanopb output for wpimath/protobuf (design/reports/wpimath.md).
Eigen, gcem and Sleipnir are separate modules (eigen.py, gcem.py, sleipnir.py).

The generated .npb.h/.npb.cpp keep their wpi_proto_* C identifiers, the wpi.proto package names
and the serialized descriptor bytes (section 4.5 protected patterns); they live under
src/prosdk/generated/wpimath/protobuf (headers, included by the public proto headers) and
src/prosdk/impl/wpimath/generated (sources)."""

NAME = "wpimath"
SPARSE = ["wpimath"]
# multi_subscript: wpimath indexes Eigen and Sleipnir matrices with C++23 `m[i, j]` (PORT.md 7.1).
# guard_pasted: the units macros paste metric prefixes onto unit symbols (P ## S -> PS, an Xtensa
# special-register macro); the Arduino macro guard covers those formed names too.
TRANSFORMS = {"multi_subscript": True, "guard_pasted": True}

_W = "wpimath/src"
_N = f"{_W}/main/native"
_G = f"{_W}/generated/main/native/cpp"

RULES = [
    # public headers: wpi/math/**, wpi/units/**
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"]),
    # nanopb output (committed upstream; no generation step)
    dict(kind="header", src=_G, dst="src/prosdk/generated", include_base="src",
         include=["wpimath/protobuf/*.npb.h"]),
    # sources
    dict(kind="source", src=_N, dst="src/prosdk/impl/wpimath", include=["cpp/*.cpp", "cpp/*.hpp"],
         exclude={"cpp/jni/*": "JNI (no JVM on the MCU)"}),
    dict(kind="source", src=_G, dst="src/prosdk/impl/wpimath/generated",
         include=["wpimath/protobuf/*.npb.cpp"]),
    # host unit tests (Catch2), including the units tests
    dict(kind="test", src=f"{_W}/test/native/cpp", dst="test/wpimath/cpp", include=["*"]),
    dict(kind="test-header", src=f"{_W}/test/native/include", dst="test/wpimath/include",
         include_base="test/wpimath/include", include=["*"]),
]

_UPSTREAM_BUILD = ("upstream build file (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with "
                   "tools/build")

# Not vendored at all, with reasons (design/API-DIFFERENCES.md).
NOT_VENDORED = {
    f"{_N}/cpp/jni": "JNI (no JVM on the MCU)",
    f"{_W}/main/java": "Java API",
    f"{_W}/generated/main/java": "Java API (generated)",
    f"{_W}/generate": "generators for the Java sources",
    f"{_W}/main/python": "Python bindings",
    f"{_W}/test/java": "Java tests",
    f"{_W}/test/python": "Python tests",
    f"{_W}/dev": "upstream developer scratch executable",
    f"{_W}/main/proto": ".proto sources; the committed nanopb output is vendored instead",
    "wpimath/docs": "design notes (not code); see upstream",
    # upstream build files, generators, documentation and bindings (no rule needed)
    "wpimath/BUILD.bazel": _UPSTREAM_BUILD,
    "wpimath/CMakeLists.txt": _UPSTREAM_BUILD,
    "wpimath/build.gradle": _UPSTREAM_BUILD,
    "wpimath/wpimath-config.cmake.in": _UPSTREAM_BUILD,
    "wpimath/robotpy_native_build_info.bzl": _UPSTREAM_BUILD,
    "wpimath/robotpy_pybind_build_info.bzl": _UPSTREAM_BUILD,
    "wpimath/generate.bzl": _UPSTREAM_BUILD,
    "wpimath/src/test/native/BUILD.bazel": _UPSTREAM_BUILD,
    "wpimath/README.md": "documentation, not code",
    "wpimath/generate_nanopb.py":
        "nanopb generator script; its output is committed upstream and vendored",
    "wpimath/generate_numbers.py":
        "generator of the Nat/number headers; its output is committed upstream and vendored",
    "wpimath/generate_quickbuf.py": "Java protobuf (QuickBuffers) generator",
    "wpimath/generate_wpimath.py":
        "upstream code generator driver; its output is committed upstream and vendored",
    "wpimath/src/test/generate_biquad_fixtures.py":
        "scipy script that generated test fixture data committed upstream",
}
