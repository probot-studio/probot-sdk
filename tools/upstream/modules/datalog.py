"""datalog: the DataLog writer side (DataLog, DataLogWriter, DataLogBackgroundWriter)
(design/reports/wpiutil.md section 4). The reader side is a desktop tool; it is vendored only as
host test support (test/datalog/support) so the upstream writer tests can read back what they
wrote. Everything else is excluded with a reason (listed in design/API-DIFFERENCES.md).
"""

NAME = "datalog"
SPARSE = ["datalog", "wpiutil"]
TRANSFORMS = {"fs": True}

_N = "datalog/src/main/native"
_U = "wpiutil/src/main/native"
_SUPPORT = "test/datalog/support"

_READER = "reader side (desktop log tools); needs MemoryBuffer (mmap). Vendored only as host " \
          "test support in test/datalog/support"

RULES = [
    # public headers
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"],
         exclude={
             "wpi/datalog/DataLogReader.hpp": _READER,
             "wpi/datalog/DataLogReaderThread.hpp": "reader side; needs upb, sigslot, "
                                                    "DynamicStruct (desktop log viewer)",
             "wpi/datalog/FileLogger.hpp": "tails a Linux log file with inotify (no inotify on "
                                           "ESP32); DataLogManager uses it for system logs",
         }),
    # sources (built into libprosdk.a)
    dict(kind="source", src=_N, dst="src/prosdk/impl/datalog", include=["cpp/*.cpp"],
         exclude={
             "cpp/jni/*": "JNI (no JVM on the MCU)",
             "cpp/DataLogReader.cpp": _READER,
             "cpp/DataLogReaderThread.cpp": "reader side; needs upb, sigslot, DynamicStruct",
             "cpp/FileLogger.cpp": "Linux inotify (no inotify on ESP32)",
         }),
    # host unit tests (Catch2)
    dict(kind="test", src="datalog/src/test/native/cpp", dst="test/datalog/cpp", include=["*"],
         exclude={
             "DataLogReaderThreadTest.cpp": "DataLogReaderThread is not vendored (upb)",
             "FileLoggerTest.cpp": "FileLogger is not vendored (inotify)",
         }),
    # host test support: the reader and the wpiutil pieces it needs (Linux host has mmap)
    dict(kind="test-header", src=f"{_N}/include", dst=f"{_SUPPORT}/include",
         include_base=f"{_SUPPORT}/include", include=["wpi/datalog/DataLogReader.hpp"]),
    dict(kind="test-header", src=f"{_U}/thirdparty/llvm/include", dst=f"{_SUPPORT}/include",
         include_base=f"{_SUPPORT}/include",
         include=["wpi/util/MemoryBuffer.hpp", "wpi/util/SmallVectorMemoryBuffer.hpp"]),
    dict(kind="test-header", src=f"{_U}/include", dst=f"{_SUPPORT}/include",
         include_base=f"{_SUPPORT}/include", include=["wpi/util/MappedFileRegion.hpp"]),
    dict(kind="test", src=_N, dst=_SUPPORT, include=["cpp/DataLogReader.cpp"]),
    dict(kind="test", src=f"{_U}/thirdparty/llvm/cpp/llvm", dst=f"{_SUPPORT}/cpp",
         include=["MemoryBuffer.cpp"]),
    dict(kind="test", src=f"{_U}/cpp", dst=f"{_SUPPORT}/cpp", include=["MappedFileRegion.cpp"]),
]

_UPSTREAM_BUILD = ("upstream build file (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with "
                   "tools/build")

# Not vendored at all, with reasons.
NOT_VENDORED = {
    "datalog/src/main/java": "Java bindings (no JVM on the MCU)",
    "datalog/src/main/python": "Python bindings",
    "datalog/examples": "desktop example programs (printlog, writelog)",
    "datalog/src/dev": "desktop development executable",
    # upstream build files, generators, documentation and bindings (no rule needed)
    "datalog/BUILD.bazel": _UPSTREAM_BUILD,
    "datalog/CMakeLists.txt": _UPSTREAM_BUILD,
    "datalog/build.gradle": _UPSTREAM_BUILD,
    "datalog/datalog-config.cmake.in": _UPSTREAM_BUILD,
    "datalog/robotpy_native_build_info.bzl": _UPSTREAM_BUILD,
    "datalog/robotpy_pybind_build_info.bzl": _UPSTREAM_BUILD,
    "datalog/src/test/native/BUILD.bazel": _UPSTREAM_BUILD,
    "datalog/README.md": "documentation, not code",
    "datalog/doc": "documentation, not code (incl. the wpilog format spec wpilog.ksy)",
    "datalog/src/printlog": "Java printlog example (no JVM on the MCU)",
    "datalog/src/test/java": "Java tests",
    "datalog/src/test/python": "Python tests",
}
