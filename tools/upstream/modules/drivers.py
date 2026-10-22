"""drivers: the WPILib 2027 device drivers library (allwpilib/drivers): the nine PWM motor
controllers generated from src/generate/pwm_motor_controllers.json (Koors40, PWMSparkFlex,
PWMSparkMax, PWMTalonFX, PWMTalonSRX, PWMVenom, PWMVictorSPX, Spark, SparkMini; committed upstream
under drivers/src/generated) and the goBILDA Pinpoint odometry computer (I2C).

- Public headers go to src/prosdk/drivers/ (include <prosdk/drivers/motor/Spark.hpp>).
- Sources go to src/prosdk/impl/drivers/ (cpp/ and generated/) and into libprosdk.a.
- Host unit tests (Catch2, against the HAL sim backend) go to test/drivers/.

The hand-written prosdk drivers (Servo, BTS7960, VNH5019, MPU6050, PCA9685/ProHub) sit next to
the vendored headers in src/prosdk/drivers/ (design/drivers.md); they are not listed in
generated.txt, so the vendoring script leaves them alone.
"""

NAME = "drivers"
SPARSE = ["drivers"]

_S = "drivers/src"

RULES = [
    # public headers (GoBildaPinpoint + the generated PWM motor controllers)
    dict(kind="header", src=f"{_S}/main/native/include", dst="src", include_base="src",
         include=["*"]),
    dict(kind="header", src=f"{_S}/generated/main/native/include", dst="src", include_base="src",
         include=["*"]),
    # sources
    dict(kind="source", src=f"{_S}/main/native", dst="src/prosdk/impl/drivers",
         include=["cpp/*.cpp"],
         exclude={"cpp/empty.cpp": "empty translation unit (upstream build placeholder)"}),
    dict(kind="source", src=f"{_S}/generated/main/native/cpp", dst="src/prosdk/impl/drivers/generated",
         include=["*"]),
    # host unit tests (Catch2, against the HAL sim backend)
    dict(kind="test", src=f"{_S}/test/native/cpp", dst="test/drivers/cpp", include=["*"]),
]

# Not vendored at all, with reasons.
NOT_VENDORED = {
    f"{_S}/generate": "Jinja templates and pwm_motor_controllers.json; their output "
                      "(drivers/src/generated) is committed upstream and vendored instead",
    "drivers/generate_pwm_motor_controllers.py": "generator of src/generated (output vendored)",
    f"{_S}/dev": "upstream developer scratch executables (C++ and Java)",
    f"{_S}/main/java": "Java API (no JVM on the MCU)",
    f"{_S}/generated/main/java": "generated Java motor controllers (no JVM on the MCU)",
    f"{_S}/test/java": "Java tests",
    f"{_S}/main/python": "Python bindings (robotpy semiwrap)",
    f"{_S}/test/python": "Python tests",
    "drivers/{BUILD.bazel,build.gradle,CMakeLists.txt,drivers-config.cmake.in,generate.bzl,"
    "robotpy_*.bzl}": "upstream build files (prosdk builds with tools/build)",
    f"{_S}/test/native/BUILD.bazel": "upstream build file (prosdk builds with tools/build)",
}
