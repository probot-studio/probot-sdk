"""eigen: Eigen 5.0.1 as patched and shipped by WPILib (wpimath/src/main/native/thirdparty/eigen),
including unsupported/Eigen/MatrixFunctions (design/reports/wpimath.md section 1). Header-only.
Lives under src/prosdk/thirdparty/eigen; `<Eigen/...>` includes become
`<prosdk/thirdparty/eigen/Eigen/...>` (design/PORT.md section 3)."""

NAME = "eigen"
SPARSE = ["wpimath/src/main/native/thirdparty/eigen"]
# Third-party code with no wpi names. Every module header (Eigen/<Module>,
# unsupported/Eigen/<Module>) includes prosdk/platform/config.hpp first, which sets
# EIGEN_STACK_ALLOCATION_LIMIT, EIGEN_NO_IO and EIGEN_MPL2_ONLY (design/PORT.md section 7).
TRANSFORMS = {
    "rename": False, "cxx23": False, "arduino_renames": False,
    "config_include": [r"^src/prosdk/thirdparty/eigen/(unsupported/)?Eigen/[^/]+$"],
}

_E = "wpimath/src/main/native/thirdparty/eigen/include"

RULES = [
    dict(kind="header", src=_E, dst="src/prosdk/thirdparty/eigen", include_base="src",
         include=["Eigen/*", "unsupported/Eigen/*"]),
    dict(kind="license", src=_E, dst="licenses/eigen",
         include=["COPYING.APACHE", "COPYING.BSD", "COPYING.MINPACK", "COPYING.MPL2"]),
]

# Not vendored, with reasons (design/API-DIFFERENCES.md).
NOT_VENDORED = {
    f"{_E}/BUILD.bazel": "Bazel build file",
    f"{_E}/MODULE.bazel": "Bazel build file",
}
