"""apriltag: `prosdk::apriltag` (AprilTagDetector, AprilTagDetection, AprilTagPoseEstimator,
AprilTagPoseEstimate) and the University of Michigan AprilTag C library WPILib vendors in
apriltag/src/main/native/thirdparty/apriltag (with WPILib's patches already applied upstream).

The detector ships on the ESP32 (decision by measurement, design/PORT.md section 10): the kit has
no camera, but frames from a camera board can be passed to Detect(). The C library lives under
src/prosdk/thirdparty/apriltag (`#include <prosdk/thirdparty/apriltag/apriltag.h>`); its sources
include their siblings in include/common by bare name, which `include_search` resolves.
Field layouts (the former AprilTagFieldLayout) are in the fields module."""

NAME = "apriltag"
SPARSE = ["apriltag"]
TRANSFORMS = {"include_search": ["common"]}

_N = "apriltag/src/main/native"
_T = f"{_N}/thirdparty/apriltag"

_IMAGE_GEN = "returns tag images in a wpiutil RawFrame, the cscore camera frame type, which is " \
             "not vendored (cscore is not ported); generating printable tags is a desktop task"

RULES = [
    # public headers
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"],
         exclude={
             "wpi/apriltag/AprilTagDetector_cv.hpp": "OpenCV cv::Mat adapter; OpenCV is not "
                                                     "available on the MCU",
             "wpi/apriltag/AprilTagImageGenerator.hpp": _IMAGE_GEN,
         }),
    # the C library headers (installed publicly upstream as well)
    dict(kind="header", src=f"{_T}/include", dst="src/prosdk/thirdparty/apriltag",
         include_base="src", include=["*"]),
    # sources (built into libprosdk.a)
    dict(kind="source", src=_N, dst="src/prosdk/impl/apriltag", include=["cpp/*.cpp"],
         exclude={
             "cpp/jni/*": "JNI (no JVM on the MCU)",
             "cpp/AprilTag.cpp": _IMAGE_GEN,
         }),
    dict(kind="source", src=f"{_T}/src", dst="src/prosdk/impl/apriltag/thirdparty",
         include=["*.c", "common/*.c"]),
    # host unit tests (Catch2)
    dict(kind="test", src="apriltag/src/test/native/cpp", dst="test/apriltag/cpp", include=["*"]),
]

# Not vendored at all, with reasons.
NOT_VENDORED = {
    "apriltag/src/test/resources": "tag images used only by the Java tests",
    "apriltag/src/dev": "desktop development executable",
    "apriltag/AprilTag.json": "vendordep descriptor for the desktop/roboRIO build system",
    "apriltag/src/main/java": "Java API (no JVM on the MCU)",
    "apriltag/src/main/python": "Python bindings",
    "apriltag/{BUILD.bazel,CMakeLists.txt,apriltag-config.cmake.in,build.gradle,robotpy_*.bzl}":
        "upstream build files (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with tools/build",
    "apriltag/src/test/native/BUILD.bazel": "upstream build files (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with tools/build",
    "apriltag/README.md": "upstream readme (what the wrapper is, where its C library patches live); provenance is in NOTICE",
    "apriltag/src/test/java": "Java tests (no JVM on the MCU)",
    "apriltag/src/test/python": "Python tests and their tag images",
}
