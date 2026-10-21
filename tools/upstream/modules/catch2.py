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
