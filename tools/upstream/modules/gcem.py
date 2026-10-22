"""gcem: compile-time math (wpimath/src/main/native/thirdparty/gcem), header-only. Lives under
src/prosdk/thirdparty/gcem; `<gcem.hpp>` becomes `<prosdk/thirdparty/gcem/gcem.hpp>`."""

NAME = "gcem"
SPARSE = ["wpimath/src/main/native/thirdparty/gcem"]
TRANSFORMS = {"rename": False, "cxx23": False, "arduino_renames": False}

RULES = [
    dict(kind="header", src="wpimath/src/main/native/thirdparty/gcem/include",
         dst="src/prosdk/thirdparty/gcem", include_base="src", include=["*"]),
]
