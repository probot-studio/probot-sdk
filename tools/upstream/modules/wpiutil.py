"""wpiutil: the needed set (design/reports/wpiutil.md section 2.2) plus the "keep optional"
pieces of section 5 item 5. Everything else is excluded with a reason (listed in
design/API-DIFFERENCES.md)."""

NAME = "wpiutil"
SPARSE = ["wpiutil"]
TRANSFORMS = {"fs": True}

_N = "wpiutil/src/main/native"

# Shared by headers and sources.
_EXCLUDED_HEADERS = {
    "wpi/util/jni_util.hpp": "JNI (no JVM on the MCU)",
    "wpi/util/MappedFileRegion.hpp": "mmap is not available on ESP32; only the datalog reader used it",
    "wpi/util/MessagePack.hpp": "only ntcore uses it (mpack not vendored)",
    "wpi/util/PixelFormat.h": "camera support (cscore) is not ported",
    "wpi/util/PixelFormat.hpp": "camera support (cscore) is not ported",
    "wpi/util/RawFrame.h": "camera support (cscore) is not ported",
    "wpi/util/RawFrame.hpp": "camera support (cscore) is not ported",
    "wpi/util/protobuf/ProtobufMessageDatabase.hpp": "dynamic protobuf (upb) for the desktop log reader",
}

RULES = [
    # public headers
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src",
         include=["*"], exclude=_EXCLUDED_HEADERS),
    dict(kind="header", src=f"{_N}/thirdparty/llvm/include", dst="src", include_base="src",
         include=["*"],
         exclude={
             "wpi/util/MemoryBuffer.hpp": "needs MappedFileRegion (mmap); desktop log reader only",
             "wpi/util/SmallVectorMemoryBuffer.hpp": "needs MemoryBuffer",
         }),
    dict(kind="header", src=f"{_N}/thirdparty/htrie/include", dst="src", include_base="src",
         include=["*"]),
    dict(kind="header", src=f"{_N}/thirdparty/json/include", dst="src", include_base="src",
         include=["*"]),
    dict(kind="header", src=f"{_N}/thirdparty/sigslot/include", dst="src", include_base="src",
         include=["*"]),
    dict(kind="header", src=f"{_N}/thirdparty/double-conversion/include", dst="src",
         include_base="src", include=["*"]),
    dict(kind="header", src=f"{_N}/thirdparty/nanopb/include", dst="src/prosdk/thirdparty/nanopb",
         include_base="src", include=["*.h"]),
    # sources (built into libprosdk.a; also compiled by arduino-cli when no archive is present)
    dict(kind="source", src=_N, dst="src/prosdk/impl/wpiutil",
         include=[
             "cpp/*.cpp",
             "unix/*.cpp",
             "thirdparty/llvm/cpp/llvm/*.cpp",
             "thirdparty/json/src/*",
             "thirdparty/double-conversion/src/*.cpp",
             "thirdparty/nanopb/src/*.cpp",
         ],
         exclude={
             "cpp/jni/*": "JNI (no JVM on the MCU)",
             "cpp/MappedFileRegion.cpp": "mmap is not available on ESP32",
             "cpp/MessagePack.cpp": "only ntcore uses it (mpack not vendored)",
             "cpp/RawFrame.cpp": "camera support (cscore) is not ported",
             "cpp/protobuf/ProtobufMessageDatabase.cpp": "dynamic protobuf (upb), desktop only",
             "thirdparty/llvm/cpp/llvm/MemoryBuffer.cpp": "needs MappedFileRegion (mmap)",
         }),
    # host unit tests (Catch2)
    dict(kind="test", src="wpiutil/src/test/native/cpp", dst="test/wpiutil/cpp",
         include=["*"],
         exclude={
             "argparse/*": "argparse is not vendored (CLI parser)",
             "debugging/*": "debugging thirdparty is not vendored",
             "JniUtilTest.cpp": "JNI",
         }),
    dict(kind="test-header", src="wpiutil/src/test/native/include", dst="test/wpiutil/include",
         include_base="test/wpiutil/include", include=["*"]),
    dict(kind="test", src="wpiutil/src/generated/test/native/cpp", dst="test/wpiutil/generated",
         include_base="test/wpiutil/generated", include=["*"]),
]

# Not vendored at all (whole thirdparty trees), with reasons.
NOT_VENDORED = {
    f"{_N}/thirdparty/argparse": "command-line parser for desktop tools; uses std::any/RTTI",
    f"{_N}/thirdparty/debugging": "debugger detection has no ESP32 branch; unused by ported code",
    f"{_N}/thirdparty/mpack": "MessagePack for ntcore (NT4 is not ported yet)",
    f"{_N}/thirdparty/upb": "dynamic protobuf for the desktop DataLog reader",
    f"{_N}/thirdparty/nanopb/generator": "protoc plugin; generated .npb files are vendored instead",
    f"{_N}/windows": "Windows only",
    f"{_N}/resources": "desktop GUI icon",
}
