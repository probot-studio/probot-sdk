#!/usr/bin/env python3
"""HAL completeness link test for the ESP32 backend (design/hal-esp32.md, "Completeness").

Generates an Arduino sketch that takes the address of every function in
design/hal-esp32-functions.txt (the HAL_*/HALSIM_* functions a backend must define, cast to the
listed signature, plus the ones the common code defines) and uses the C++ hooks
(prosdk::hal::GetDefaultDriverStationImpl, the prosdk::hal::SimDevice constructors). All
references are reached from a probot-core hook (setup()/loop() belong to core), so the linker has
to resolve every one of them from libprosdk.a: a missing definition is an "undefined reference"
link error.

  python3 tools/build/hal_link_test.py --out DIR            # write DIR/HalLinkTest/HalLinkTest.ino
  python3 tools/build/hal_link_test.py --out DIR --compile  # and link it with arduino-cli

--compile links against probot-core from .upstream/probot-core (or --core PATH) through a
libraries folder of symlinks in DIR, with the sketch FQBN of tools/build/arduino_recipe.py (the
Boardoza Pulse S32-S3 settings), under the shared build lock at the lowest priority (2 jobs).
Needs src/esp32s3/libprosdk.a (tools/build/build.sh esp32s3). The build folder DIR/build is kept
between runs.

After the link, --compile also checks that the archive does not depend on the board options the
sketch FQBN sets and the archive recipe does not (USB CDC on boot, PSRAM): no undefined symbol of
libprosdk.a names an Arduino Serial object or String, and no archive source under src/prosdk
uses BOARD_HAS_PSRAM, ARDUINO_USB_* or the Arduino PSRAM API (prosdk.h and
prosdk/probot/CoreBridge.hpp compile in the sketch and may).

The sketch also includes <LittleFS.h> and <SPIFFS.h> and calls GetLogDirectory() behind a false
condition, so the link holds both Arduino libraries next to prosdk's LittleFS auto-mount. --compile
then checks the ELF's symbol table: the names the auto-mount's weak references bind to (the
`LittleFS` object, `esp_vfs_spiffs_register`) must be defined globals. If an Arduino-ESP32 update
renames, mangles or drops one of them, the weak reference stays null, prosdk would auto-mount the
partition under such a sketch, and this check fails.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import arduino_recipe  # noqa: E402

FUNCTIONS = ROOT / "design" / "hal-esp32-functions.txt"
SKETCH = "HalLinkTest"
FQBN = arduino_recipe.SKETCH_FQBN
ARCHIVE = ROOT / "src" / "esp32s3" / "libprosdk.a"
# Undefined symbols that would tie the archive to the sketch's CDCOnBoot/USBMode choice
BOARD_SYMBOLS = re.compile(r"Serial0|HWCDCSerial|USBSerial|_ZN6String")
# Names that src/prosdk/platform/Storage.cpp references weakly to see a sketch's file system library
FS_WEAK_SYMBOLS = ("LittleFS", "esp_vfs_spiffs_register")
BOARD_MACROS = (r"BOARD_HAS_PSRAM|ARDUINO_USB_(MODE|CDC_ON_BOOT|MSC_ON_BOOT|DFU_ON_BOOT)"
                r"|psramFound|ps_malloc")
# Headers that compile in the sketch (with the user's board options), not into the archive
SKETCH_SIDE = {"src/prosdk.h", "src/prosdk/probot/CoreBridge.hpp"}

SECTION = re.compile(r"^## (prosdk/hal/\S+\.h) \(\d+\)$")
SIGNATURE = re.compile(r"^(.*?)\b((?:HAL|HALSIM)_\w+)\s*\((.*)\)\s*$")
NAME_LIST = re.compile(r"^((?:HAL|HALSIM)_\w+)(\s|$)")


def parse() -> tuple[list[str], list[tuple[str, str | None]]]:
    """Returns (headers, [(function name, pointer type or None)]) to reference."""
    headers: list[str] = []
    names: list[tuple[str, str | None]] = []
    section = None  # "header" | "common" | "stream" | None
    for line in FUNCTIONS.read_text().splitlines():
        if line.startswith("## "):
            m = SECTION.match(line)
            if m:
                headers.append(m.group(1))
                section = "header"
            elif line.startswith("## already defined by the backend-independent code"):
                section = "common"
            elif line.startswith("## declared, but the sim backend does not define them"):
                section = "stream"
            else:
                section = None
            continue
        if not line or line.startswith("#") or section is None:
            continue
        if section == "header":
            m = SIGNATURE.match(line)
            if m:
                ret, name, params = m.group(1).strip(), m.group(2), m.group(3).strip()
                names.append((name, f"{ret} (*)({params})"))
        elif section == "common":
            m = NAME_LIST.match(line)
            if m:
                names.append((m.group(1), None))
        elif section == "stream":
            names.extend((w, None) for w in line.split() if re.fullmatch(r"HAL_\w+", w))
    # stream functions are also listed under PowerDistribution.h: keep each name once
    seen: set[str] = set()
    unique = [n for n in names if not (n[0] in seen or seen.add(n[0]))]
    return headers, unique


def all_hal_headers() -> list[str]:
    """Every top-level HAL C header (the common-code functions are declared in several)."""
    return sorted(f"prosdk/hal/{p.name}" for p in (ROOT / "src" / "prosdk" / "hal").glob("*.h"))


def sketch_source(headers: list[str], names: list[tuple[str, str | None]]) -> str:
    out = [
        "// Generated by tools/build/hal_link_test.py; do not edit.",
        "// Links a reference to every HAL function an ESP32 backend must provide",
        f"// ({len(names)} functions from design/hal-esp32-functions.txt plus the C++ hooks).",
        '#define PROBOT_WIFI_AP_PASSWORD "linktest1"',
        "#define PROBOT_WIFI_AP_CHANNEL 6",
        "#include <prosdk.h>",
        "// Arduino's file system libraries, for the auto-mount's weak references (FS check)",
        "#include <LittleFS.h>",
        "#include <SPIFFS.h>",
        "#include <prosdk/platform/Storage.hpp>",
        "",
    ]
    out += [f"#include <{h}>" for h in sorted(set(headers) | set(all_hal_headers()))]
    out += [
        "#include <prosdk/hal/SimDevice.hpp>",
        "#include <prosdk/hal/cpp/MrcLibDs.hpp>",
        "",
        "namespace {",
        "// Function addresses; the volatile reads in setup() keep every entry (and so every",
        "// definition) in the link.",
        "const void* const kRefs[] = {",
    ]
    for name, ptr in names:
        ref = f"static_cast<{ptr}>(&{name})" if ptr else f"&{name}"
        out.append(f"    reinterpret_cast<const void*>({ref}),")
    out += [
        "};",
        "volatile bool gNever = false;",
        "}  // namespace",
        "",
        "// setup()/loop() belong to probot-core: reference everything from a hook.",
        "void teleopInit() {",
        "  uintptr_t sum = 0;",
        "  for (const void* const& p : kRefs) {",
        "    sum ^= reinterpret_cast<uintptr_t>(*static_cast<const void* const volatile*>(&p));",
        "  }",
        "  if (gNever) {",
        "    prosdk::hal::SimDevice a{\"link\", 0};",
        "    prosdk::hal::SimDevice b{\"link\", 0, 1};",
        "    sum ^= reinterpret_cast<uintptr_t>(prosdk::hal::GetDefaultDriverStationImpl());",
        "    sum ^= prosdk::platform::GetLogDirectory().size();",
        "    sum ^= static_cast<uintptr_t>(LittleFS.begin(true)) ^",
        "           static_cast<uintptr_t>(SPIFFS.begin(true));",
        "    sum ^= static_cast<uintptr_t>(static_cast<HAL_SimDeviceHandle>(a)) ^",
        "           static_cast<uintptr_t>(static_cast<HAL_SimDeviceHandle>(b));",
        "  }",
        "  gNever = sum == 0;",
        "}",
        "",
    ]
    return "\n".join(out)


def write_if_changed(path: Path, text: str) -> None:
    if not path.exists() or path.read_text() != text:
        path.write_text(text)


def check_archive_abi() -> int:
    """Fails (1) when the archive depends on a board option the sketch FQBN changes."""
    bad = 0
    nm = arduino_recipe.paths()["toolchain"] / "xtensa-esp32s3-elf-nm"
    res = subprocess.run([str(nm), "-u", str(ARCHIVE)], capture_output=True, text=True)
    if res.returncode != 0:
        print(f"ABI check: {nm.name} -u failed: {res.stderr.strip()}")
        return 1
    undefined = {f[1] for f in (ln.split() for ln in res.stdout.splitlines())
                 if len(f) == 2 and f[0] == "U"}
    syms = sorted(s for s in undefined if BOARD_SYMBOLS.search(s))
    for sym in syms:
        print(f"ABI check: libprosdk.a references {sym} (depends on the sketch's USB options)")
        bad = 1
    res = subprocess.run(["grep", "-rnE", BOARD_MACROS, "src/prosdk.h", "src/prosdk",
                          "--exclude-dir=thirdparty"], cwd=ROOT, capture_output=True, text=True)
    for ln in res.stdout.splitlines():
        if ln.split(":", 1)[0] not in SKETCH_SIDE:
            print(f"ABI check: archive source uses a board-option macro: {ln}")
            bad = 1
    print("ABI check: " + ("FAILED" if bad else
                           "archive independent of the USB/PSRAM board options"))
    return bad


def check_fs_symbols(elf: Path) -> int:
    """Fails (1) when a weak reference of the LittleFS auto-mount would stay null."""
    nm = arduino_recipe.paths()["toolchain"] / "xtensa-esp32s3-elf-nm"
    res = subprocess.run([str(nm), str(elf)], capture_output=True, text=True)
    if res.returncode != 0:
        print(f"FS check: {nm.name} failed: {res.stderr.strip()}")
        return 1
    kinds: dict[str, list[str]] = {}
    for ln in res.stdout.splitlines():
        f = ln.split()
        if len(f) >= 2 and f[-1] in FS_WEAK_SYMBOLS:
            kinds.setdefault(f[-1], []).append(f[-2])
    bad = 0
    for sym in FS_WEAK_SYMBOLS:
        # a defined global: B/D/T/R (or a weak definition V/W); 'w', 'U' or absent = unresolved
        if not any(k in "BDTRGSVW" for k in kinds.get(sym, [])):
            print(f"FS check: {sym} is not defined in the link ({kinds.get(sym, 'absent')}); "
                  "prosdk's auto-mount would not see the sketch's file system library")
            bad = 1
    print("FS check: " + ("FAILED" if bad else
                          f"{', '.join(FS_WEAK_SYMBOLS)} defined (weak references resolve)"))
    return bad


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, type=Path, help="work folder (sketch, libraries, build)")
    ap.add_argument("--compile", action="store_true", help="link the sketch with arduino-cli")
    ap.add_argument("--core", type=Path, default=ROOT / ".upstream" / "probot-core",
                    help="probot-core library folder (default: .upstream/probot-core)")
    args = ap.parse_args()

    headers, names = parse()
    out = args.out.resolve()
    sketch_dir = out / SKETCH
    sketch_dir.mkdir(parents=True, exist_ok=True)
    write_if_changed(sketch_dir / f"{SKETCH}.ino", sketch_source(headers, names))
    print(f"{sketch_dir / (SKETCH + '.ino')}: {len(names)} HAL functions from {len(headers)} "
          "headers, plus GetDefaultDriverStationImpl and 2 SimDevice constructors")
    if not args.compile:
        return 0

    libs = out / "libraries"
    libs.mkdir(exist_ok=True)
    for name, target in (("prosdk", ROOT), ("probot", args.core.resolve())):
        link = libs / name
        if link.is_symlink() and Path(os.readlink(link)) != target:
            link.unlink()
        if not link.exists():
            link.symlink_to(target, target_is_directory=True)
    lock = ROOT / ".build" / "build.lock"
    cmd = ["flock", str(lock), "nice", "-n", "19", "ionice", "-c3",
           "arduino-cli", "compile", "--fqbn", FQBN, "--libraries", str(libs),
           "--build-path", str(out / "build"), "--jobs", "2", "--warnings", "default",
           str(sketch_dir)]
    print(" ".join(cmd), flush=True)
    rc = subprocess.run(cmd).returncode
    print(f"arduino-cli exit {rc}")
    if rc:
        return rc
    abi = check_archive_abi()
    return check_fs_symbols(out / "build" / f"{SKETCH}.ino.elf") or abi


if __name__ == "__main__":
    sys.exit(main())
