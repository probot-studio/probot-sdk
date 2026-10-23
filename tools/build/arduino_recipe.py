#!/usr/bin/env python3
"""Single source of truth for the Arduino-ESP32 3.3.10 esp32s3 C/C++ compile recipe.

Reproduces platform.txt `recipe.cpp.o.pattern` / `recipe.c.o.pattern` with the generic
`esp32s3` board defaults (design/reports/toolchain.md sections 1.2-1.8). Used by:
  - tools/build/esp32s3.cmake (CMake toolchain file, via `--out-dir`: writes cxx.rsp, c.rsp
    and recipe.cmake; the response files keep the exact flag order and quoting)
  - tools/upstream/wpilib.py (Arduino macro set computation)

The effective C++ standard is the one cpp_flags ends with (gnu++2a = gnu++20). Nothing here
appends another -std flag.

Sketches (tools/build/build.sh examples, tools/build/hal_link_test.py, tools/release/package.sh)
compile with SKETCH_FQBN, the Boardoza Pulse S32-S3 settings (`--fqbn` prints it). The
archive does not depend on its USB, PSRAM or flash-size options: CDCOnBoot only changes the
sketch's `Serial` macro, PSRAM=enabled only adds -DBOARD_HAS_PSRAM (psram_type stays qspi, so
the qio_qspi include dir is the same) and FlashSize/PartitionScheme only change the partition
table and image header. So the archive is built once with the generic defaults below
(ARDUINO_USB_CDC_ON_BOOT=0, no BOARD_HAS_PSRAM) and linked into SKETCH_FQBN sketches.
PSRAM=opi (octal-PSRAM modules) is not supported: it selects the qio_opi ESP-IDF libraries and
sdkconfig, which the archive is not built against (design/hal-esp32.md "Other ESP32-S3 boards").
tools/build/hal_link_test.py checks that no archive source or undefined symbol depends on those
options.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ARDUINO_ESP32_VERSION = "3.3.10"
TOOLCHAIN_VERSION = "2601"
# arduino-cli 1.x passes runtime.ide.version=10607
ARDUINO_IDE_VERSION = "10607"
MEMORY_TYPE = "qio_qspi"  # {build.boot}_{build.psram_type} defaults for esp32s3
# Boardoza Pulse S32-S3 (ESP32-S3-WROOM-1-N4R2: 4 MB flash, 2 MB quad PSRAM, native USB-C).
# USBMode=hwcdc and UploadMode=default are the esp32s3 defaults.
SKETCH_FQBN = ("esp32:esp32:esp32s3:FlashSize=4M,PartitionScheme=huge_app,PSRAM=enabled,"
               "CDCOnBoot=cdc")


def paths(home: str | None = None) -> dict[str, Path]:
    home = home or os.environ.get("ARDUINO15") or str(Path.home() / ".arduino15")
    pk = Path(home) / "packages" / "esp32"
    if not (Path(home) / "packages").is_dir():
        pk = Path(home) / ".arduino15" / "packages" / "esp32"
    return {
        "toolchain": pk / "tools" / "esp-x32" / TOOLCHAIN_VERSION / "bin",
        "sdk": pk / "tools" / "esp32s3-libs" / ARDUINO_ESP32_VERSION,
        "core": pk / "hardware" / "esp32" / ARDUINO_ESP32_VERSION,
    }


def board_defines() -> list[str]:
    """-D flags from recipe.cpp.o.pattern and build.extra_flags (generic esp32s3 defaults)."""
    return [
        "-DF_CPU=240000000L",
        f"-DARDUINO={ARDUINO_IDE_VERSION}",
        "-DARDUINO_ESP32S3_DEV",
        "-DARDUINO_ARCH_ESP32",
        '-DARDUINO_BOARD="ESP32S3_DEV"',
        '-DARDUINO_VARIANT="esp32s3"',
        "-DARDUINO_PARTITION_default",
        '-DARDUINO_HOST_OS="linux"',
        '-DARDUINO_FQBN="esp32:esp32:esp32s3"',
        "-DESP32=ESP32",
        "-DCORE_DEBUG_LEVEL=0",
        "-DARDUINO_RUNNING_CORE=1",
        "-DARDUINO_EVENT_RUNNING_CORE=1",
        "-DARDUINO_USB_MODE=1",
        "-DARDUINO_USB_CDC_ON_BOOT=0",
        "-DARDUINO_USB_MSC_ON_BOOT=0",
        "-DARDUINO_USB_DFU_ON_BOOT=0",
    ]


def preprocessor_flags(p: dict[str, Path]) -> list[str]:
    """compiler.cpreprocessor.flags + {includes} (core and variant)."""
    sdk, core = p["sdk"], p["core"]
    return [
        f"@{sdk}/flags/defines",
        "-iprefix", f"{sdk}/include/",
        f"@{sdk}/flags/includes",
        f"-I{sdk}/{MEMORY_TYPE}/include",
        f"-I{core}/cores/esp32",
        f"-I{core}/variants/esp32s3",
    ]


def cxx_flags(p: dict[str, Path], warnings: str = "-w") -> list[str]:
    """Full C++ compile flags in Arduino order (without -MMD -c, source and output)."""
    sdk = p["sdk"]
    return ([f"@{sdk}/flags/cpp_flags"] + warnings.split()
            + ["-Os", "-Werror=return-type"] + board_defines() + preprocessor_flags(p))


def c_flags(p: dict[str, Path], warnings: str = "-w") -> list[str]:
    sdk = p["sdk"]
    return ([f"@{sdk}/flags/c_flags"] + warnings.split()
            + ["-Os", "-Werror=return-type"] + board_defines() + preprocessor_flags(p))


def _cmake_quote(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"')


def rsp_text(args: list[str]) -> str:
    """GCC response file (libiberty buildargv quoting), one argument per line."""
    out = []
    for a in args:
        if any(c in a for c in " \t\n'\"\\"):
            a = "'" + a.replace("\\", "\\\\").replace("'", "\\'") + "'"
        out.append(a)
    return "\n".join(out) + "\n"


def write_if_changed(path: Path, text: str) -> None:
    if not path.exists() or path.read_text() != text:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)


def write_out_dir(p: dict[str, Path], out: Path, warnings: str) -> None:
    write_if_changed(out / "cxx.rsp", rsp_text(cxx_flags(p, warnings)))
    write_if_changed(out / "c.rsp", rsp_text(c_flags(p, warnings)))
    write_if_changed(out / "recipe.cmake", "".join([
        "# Generated by tools/build/arduino_recipe.py; do not edit.\n",
        f'set(PROSDK_ARDUINO_TOOLCHAIN_BIN "{_cmake_quote(str(p["toolchain"]))}")\n',
        f'set(PROSDK_ARDUINO_SDK "{_cmake_quote(str(p["sdk"]))}")\n',
        f'set(PROSDK_ARDUINO_CORE "{_cmake_quote(str(p["core"]))}")\n',
        f'set(PROSDK_ARDUINO_CXX_RSP "{_cmake_quote(str(out / "cxx.rsp"))}")\n',
        f'set(PROSDK_ARDUINO_C_RSP "{_cmake_quote(str(out / "c.rsp"))}")\n',
    ]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cmake", action="store_true", help="print CMake set() statements")
    ap.add_argument("--warnings", default="-w")
    ap.add_argument("--home", default=None)
    ap.add_argument("--out-dir", type=Path, help="write cxx.rsp, c.rsp and recipe.cmake here")
    ap.add_argument("--fqbn", action="store_true", help="print the sketch FQBN (SKETCH_FQBN)")
    a = ap.parse_args()
    if a.fqbn:
        print(SKETCH_FQBN)
        return 0
    p = paths(a.home)
    if a.out_dir:
        write_out_dir(p, a.out_dir, a.warnings)
    elif a.cmake:
        print(f'set(PROSDK_ARDUINO_TOOLCHAIN_BIN "{p["toolchain"]}")')
        print(f'set(PROSDK_ARDUINO_SDK "{p["sdk"]}")')
        print(f'set(PROSDK_ARDUINO_CORE "{p["core"]}")')
        # One string each, so CMake keeps the order (-iprefix must precede @includes).
        print('set(PROSDK_ARDUINO_CXX_FLAGS "'
              + _cmake_quote(" ".join(_shell(x) for x in cxx_flags(p, a.warnings))) + '")')
        print('set(PROSDK_ARDUINO_C_FLAGS "'
              + _cmake_quote(" ".join(_shell(x) for x in c_flags(p, a.warnings))) + '")')
    else:
        print(" ".join(_shell(x) for x in cxx_flags(p, a.warnings)))
    return 0


def _shell(s: str) -> str:
    # -DX="y" must reach the compiler as -DX="y" (string literal macro)
    if '"' in s:
        return "'" + s + "'"
    return s


if __name__ == "__main__":
    sys.exit(main())
