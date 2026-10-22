#!/usr/bin/env python3
"""List the HAL C functions a HAL backend must define (design/hal-esp32-functions.txt).

Scans the vendored public HAL headers (src/prosdk/hal, from tools/upstream/generated.txt) for
`HAL_*` and `HALSIM_*` function declarations (a name followed by its parameter list and `;`;
inline definitions in headers are skipped) and removes the functions the backend-independent code
already defines (src/prosdk/impl/hal and the hand-written src/prosdk/platform/hal/*.cpp). What is
left is the contract of a backend: the sim backend (host/hal/sim) defines all of it on the host;
the ESP32 backend (src/prosdk/platform/hal/esp32) must define all of it on the target.

Usage: tools/build/hal_functions.py [--out FILE]   (default: print to stdout)
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GENERATED = ROOT / "tools" / "upstream" / "generated.txt"
NAME_RE = re.compile(r"\b((?:HAL|HALSIM)_[A-Za-z0-9_]+)\s*\(")

# C++ (not extern "C") links between the backend-independent code and a backend. Kept by hand:
# the scanner above only sees the C API.
CXX_HOOKS = [
    "## C++ hooks between the common code and a backend",
    "backend defines:  prosdk::hal::MrcLibDs* prosdk::hal::GetDefaultDriverStationImpl()"
    "  (hal/cpp/MrcLibDs.hpp; sim: host/hal/sim/DriverStation.cpp, systemcore: DriverStation.cpp)",
    "common defines, the backend's HAL_Initialize calls: prosdk::hal::InitializeDriverStation()"
    " (FIRSTDriverStation.cpp; calls the dashboard-opmode no-op, then"
    " GetDefaultDriverStationImpl()), prosdk::hal::init::InitializeMain() (Main.cpp)",
    "common defines, for the backend: prosdk::hal::ForceDsInstance(MrcLibDs*),"
    " prosdk::hal::WaitForInitialPacket(), prosdk::hal::DefaultSendErrorImpl(...),"
    " prosdk::hal::MakeError/MakeErrorIndexOutOfRange/MakeErrorPreviouslyAllocated"
    " (ErrorHandling.hpp), handle helpers (handles/HandlesInternal.hpp, HandlesInternal.cpp)",
    "declared, defined nowhere upstream (unused): prosdk::hal::GetMrcLibDs()",
    "",
    "## declared, but the sim backend does not define them (upstream; systemcore does)",
    "HAL_StartPowerDistributionStream HAL_GetPowerDistributionStreamData"
    " HAL_FreePowerDistributionStreamData HAL_StopPowerDistributionStream"
    "  (nothing in wpilibc calls them; checked with nm against the host hal_sim objects)",
]

NOT_TYPE = {"return", "else", "case", "sizeof", "decltype", "define", "if", "while", "for"}


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", lambda m: " " * len(m.group(0)), text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    # preprocessor lines (macros such as HAL_ENUM are not functions)
    return re.sub(r"^[ \t]*#(?:[^\n]*\\\n)*[^\n]*", "", text, flags=re.M)


def functions(text: str) -> dict[str, tuple[str, str]]:
    """name -> (kind 'decl'|'def', one-line declaration) for every HAL function in text."""
    text = strip_comments(text)
    out: dict[str, tuple[str, str]] = {}
    for m in NAME_RE.finditer(text):
        name = m.group(1)
        # previous token must end a return type (identifier or '*' / '&')
        before = text[:m.start()].rstrip()
        prev = re.search(r"([A-Za-z0-9_]+|[*&])$", before)
        if not prev or prev.group(1) in NOT_TYPE:
            continue
        # matching ')'
        depth, i = 0, m.end() - 1
        while i < len(text):
            if text[i] == "(":
                depth += 1
            elif text[i] == ")":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        rest = text[i + 1:].lstrip()
        rest = re.sub(r"^(?:noexcept|const|override)\b\s*", "", rest)
        if not rest or rest[0] not in ";{":
            continue
        # declaration text: back to the start of the statement
        cuts = [before.rfind(c) + len(c) for c in (";", "}", "{", '"C"') if c in before]
        start = max(cuts, default=0)
        decl = " ".join((text[start:i + 1]).split())
        decl = re.sub(r"^(?:extern \"C\"\s*)", "", decl)
        kind = "def" if rest[0] == "{" else "decl"
        if name not in out or kind == "def":
            out[name] = (kind, decl)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    gen = [ln.split(" ", 1)[1] for ln in GENERATED.read_text().splitlines()
           if ln.startswith("hal ")]
    headers = sorted(p for p in gen if p.startswith("src/prosdk/hal/") and p.endswith(".h"))
    common = sorted(p for p in gen if p.startswith("src/prosdk/impl/hal/") and p.endswith(".cpp"))
    common += sorted(p.relative_to(ROOT).as_posix()
                     for p in (ROOT / "src/prosdk/platform/hal").glob("*.cpp"))

    declared: dict[str, tuple[str, str]] = {}
    inline: set[str] = set()
    for h in headers:
        for name, (kind, decl) in functions((ROOT / h).read_text()).items():
            if kind == "def":
                inline.add(name)
            else:
                declared.setdefault(name, (h[len("src/"):], decl))
    defined_common: dict[str, str] = {}
    for c in common:
        for name, (kind, decl) in functions((ROOT / c).read_text()).items():
            # a C++ overload (std:: parameters, e.g. HAL_ReportUsage(std::string_view, int, ...))
            # does not define the C function of the same name
            if kind == "def" and "std::" not in decl:
                defined_common[name] = c
    need = {n: v for n, v in declared.items() if n not in inline and n not in defined_common}

    lines = [
        "# HAL functions a HAL backend must define. Generated by tools/build/hal_functions.py from",
        "# the vendored headers (src/prosdk/hal); do not edit by hand, re-run the script.",
        "# Host: host/hal/sim defines all of them except the four listed at the end. ESP32:",
        "# src/prosdk/platform/hal/esp32 must define all of them.",
        "# HALSIM_* (simulation/*.h) are needed on the target too: wpilibc's device *Sim classes",
        "# are compiled into the archive and reference them (systemcore/mockdata/*: stubs that",
        "# use HAL_SIMDATAVALUE_STUB_CAPI are the reference).",
        f"# {len(need)} functions to define; {len(defined_common)} already defined by the "
        f"common code; {len(inline)} inline in headers.",
        "",
    ]
    by_header: dict[str, list[str]] = {}
    for n in sorted(need):
        h, decl = need[n]
        by_header.setdefault(h, []).append(decl)
    for h in sorted(by_header, key=lambda x: ("/simulation/" in x, x)):
        lines.append(f"## {h} ({len(by_header[h])})")
        lines.extend(sorted(by_header[h], key=lambda d: NAME_RE.search(d).group(1)))
        lines.append("")
    lines.append(f"## already defined by the backend-independent code ({len(defined_common)})")
    for n in sorted(defined_common):
        if n in declared:
            lines.append(f"{n}  ({defined_common[n]})")
    lines.append("")
    lines.append(f"## inline in the headers ({len(inline)})")
    lines.append(" ".join(sorted(inline)))
    lines.append("")
    lines.extend(CXX_HOOKS)
    text = "\n".join(lines) + "\n"
    if a.out:
        a.out.write_text(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
