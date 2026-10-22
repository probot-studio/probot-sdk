"""sleipnir: the autodiff and nonlinear optimization library WPILib ships in
wpimath/src/main/native/thirdparty/sleipnir (headers and the 10 src/*.cpp explicit
instantiations). Lives under src/prosdk/thirdparty/sleipnir; `<sleipnir/...>` and
`<gch/small_vector.hpp>` become `<prosdk/thirdparty/sleipnir/...>` (design/PORT.md section 3).

Every header includes prosdk/platform/config.hpp first (SLEIPNIR_DISABLE_DIAGNOSTICS, PROSDK_TARGET_HW
for the pool patch). ESP32 patches: tools/upstream/patches/sleipnir."""

NAME = "sleipnir"
SPARSE = ["wpimath/src/main/native/thirdparty/sleipnir"]
# multi_subscript: Sleipnir is written against C++23 `m[i, j]`; gnu++20 gets `m(i, j)` (PORT.md 7.1).
TRANSFORMS = {"config_include": [r"^src/prosdk/thirdparty/sleipnir/.*\.hpp$"], "multi_subscript": True}

_S = "wpimath/src/main/native/thirdparty/sleipnir"

RULES = [
    dict(kind="header", src=f"{_S}/include", dst="src/prosdk/thirdparty/sleipnir",
         include_base="src", include=["*"]),
    dict(kind="source", src=f"{_S}/src", dst="src/prosdk/impl/sleipnir", include=["*.cpp"]),
]
