"""catch2: the test framework the upstream C++ unit tests use (WPILib's copy in
thirdparty/catch2). Host tests only; never part of src/ or the archive."""

NAME = "catch2"
SPARSE = ["thirdparty/catch2"]
# Third-party code with no wpi names; only the include map applies.
TRANSFORMS = {"rename": False, "cxx23": False, "arduino_renames": False, "guard": False}

_N = "thirdparty/catch2/src/main/native"

RULES = [
    dict(kind="test-header", src=f"{_N}/include", dst="test/thirdparty/catch2/include",
         include_base="test/thirdparty/catch2/include", include=["*"]),
    dict(kind="test", src=f"{_N}/cpp", dst="test/thirdparty/catch2/cpp", include=["*"]),
]

# Not vendored at all, with reasons.
NOT_VENDORED = {
    "thirdparty/catch2/{BUILD.bazel,CMakeLists.txt,build.gradle,publish.gradle}":
        "upstream build files (Bazel/CMake/Gradle packaging); prosdk builds the host tests with tools/build",
    "thirdparty/catch2/extras": "Catch2 CMake test-discovery scripts; prosdk registers its test binaries in tools/build",
}
