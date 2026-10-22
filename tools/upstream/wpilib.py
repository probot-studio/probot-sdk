#!/usr/bin/env python3
"""Vendor the pinned allwpilib commit into prosdk (design/PORT.md sections 2, 4, 5, 6).

Pipeline per module (data in tools/upstream/modules/<module>.py):
  1. fetch the pinned commit into a cache (.upstream/allwpilib, sparse, blob-filtered clone);
  2. copy the selected files;
  3. mechanical rename (section 4), C++23 shim rewrite (section 6) and Arduino user-identifier
     renames (section 5);
  4. apply tools/upstream/patches/<module>/NNNN-*.patch in order (git apply), authored against
     the output of steps 1-3 (--emit-unpatched writes it);
  5. wrap every public header in the Arduino macro guard (after the patches) and reject macro
     definitions whose replacement text names an Arduino macro;
  6. write tools/upstream/generated.txt (every produced file).

Vendored files are never edited by hand. Re-running gives byte-identical output; the script only
deletes files listed in the previous generated.txt.

Usage:
  wpilib.py [--module NAME ...] [--upstream PATH] [--check] [--emit-unpatched DIR]
"""
from __future__ import annotations

import argparse
import difflib
import fnmatch
import importlib.util
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = ROOT / "tools" / "upstream"
LOCK = HERE / "wpilib.lock"
GENERATED = HERE / "generated.txt"
MACROS_FILE = HERE / "arduino-macros.txt"
MODULES_DIR = HERE / "modules"
PATCHES_DIR = HERE / "patches"
BUILD_LOCK = ROOT / ".build" / "build.lock"

sys.path.insert(0, str(ROOT / "tools" / "build"))
import arduino_recipe  # noqa: E402

# --------------------------------------------------------------------------------------
# Section 4.5: names that must never change (wire formats, C ABI). The engine only renames
# the identifier token `wpi`, the `fs` namespace and `wpi::` inside literals/comments, so
# these never match; they are verified after every transform (count before == after).
PROTECTED_PATTERNS = [
    r"wpi\.proto\.",      # protobuf package names
    r"\bwpi_proto_\w*",   # nanopb C identifiers
    r"struct:\w+",        # struct type strings
    r"proto:\w+",         # protobuf type strings
    r"\bWPI_\w+",         # wpiutil C API
    r"\bHAL_\w+",         # HAL C API
    r"\bWPILIB_\w+",      # macro names
]

# Section 6: C++23 library names rewritten to prosdk::cxx (std::NAME -> prosdk::cxx::NAME),
# with the shim header that provides them.
CXX_SHIMS = {
    "expected": "prosdk/cxx/expected.hpp",
    "unexpected": "prosdk/cxx/expected.hpp",
    "unexpect": "prosdk/cxx/expected.hpp",
    "unexpect_t": "prosdk/cxx/expected.hpp",
    "bad_expected_access": "prosdk/cxx/expected.hpp",
    "print": "prosdk/cxx/print.hpp",
    "println": "prosdk/cxx/print.hpp",
    "byteswap": "prosdk/cxx/bit.hpp",
    "to_underlying": "prosdk/cxx/utility.hpp",
    "unreachable": "prosdk/cxx/utility.hpp",
    "invoke_r": "prosdk/cxx/functional.hpp",
}
# C++23 standard headers replaced by the shim headers.
CXX_SHIM_HEADERS = {
    "expected": "prosdk/cxx/expected.hpp",
    "print": "prosdk/cxx/print.hpp",
}

# Section 5: identifiers users must type that collide with Arduino macros. Every enumerator of an
# enum with at least one collision is renamed (k prefix, WPILib C++ style) so the enum stays
# consistent. Applied to every module as `<Qualifier>::<NAME>` and inside the enum body in the
# defining header(s). Listed in design/API-DIFFERENCES.md.
#
# Optional keys: `define_file` may be a list (several classes with a nested enum of the same
# name); `scope` (fnmatch globs on the upstream path) limits the qualified renames to those files,
# for enum names such as `Button` that other classes use with different enumerators; `members`
# renames a token followed by `(` (a member function) in the scoped files.
_PS_BUTTONS = {
    "CROSS": "kCross", "CIRCLE": "kCircle", "SQUARE": "kSquare", "TRIANGLE": "kTriangle",
    "SHARE": "kShare", "CREATE": "kCreate", "PS": "kPS", "OPTIONS": "kOptions",
    "L1": "kL1", "R1": "kR1", "L2": "kL2", "R2": "kR2", "L3": "kL3", "R3": "kR3",
    "DPAD_UP": "kDpadUp", "DPAD_DOWN": "kDpadDown", "DPAD_LEFT": "kDpadLeft",
    "DPAD_RIGHT": "kDpadRight", "TOUCHPAD": "kTouchpad", "MICROPHONE": "kMicrophone",
    "RIGHT_PADDLE_1": "kRightPaddle1", "LEFT_PADDLE_1": "kLeftPaddle1",
    "LEFT_FUNCTION": "kLeftFunction", "RIGHT_FUNCTION": "kRightFunction",
}
_PS_CONTROLLERS = ["DualShock4Controller", "DualSenseController", "DualSenseEdgeController",
                   "NiDsPS4Controller", "NiDsPS5Controller"]
ARDUINO_RENAMES = [
    {
        "enum": "Level",
        "define_file": "wpi/util/Alert.hpp",
        "names": {"HIGH": "kHigh", "MEDIUM": "kMedium", "LOW": "kLow"},
    },
    {
        "enum": "Polling",
        "define_file": "wpi/tunables/TunableConfig.hpp",
        "names": {"DEFAULT": "kDefault", "ALWAYS_GET": "kAlwaysGet",
                  "GET_ON_CHANGE": "kGetOnChange"},
    },
    {
        "enum": "Direction",
        "define_file": "wpi/hal/SimDevice.hpp",
        "names": {"INPUT": "kInput", "OUTPUT": "kOutput", "BIDIR": "kBidir"},
    },
    {
        "enum": "DebounceType",
        "define_file": "wpi/math/filter/Debouncer.hpp",
        "names": {"RISING": "kRising", "FALLING": "kFalling", "BOTH": "kBoth"},
    },
    {
        "enum": "CompressorConfigType",
        "define_file": "wpi/hardware/pneumatic/CompressorConfigType.hpp",
        "names": {"DISABLED": "kDisabled", "DIGITAL": "kDigital", "ANALOG": "kAnalog",
                  "HYBRID": "kHybrid"},
    },
    {
        # `PS` (PlayStation button) is an Xtensa special-register macro (xtensa/config/specreg.h)
        "enum": "Button",
        "define_file": [f"wpi/driverstation/{c}.hpp" for c in _PS_CONTROLLERS],
        "scope": [f"*{c}*" for c in _PS_CONTROLLERS],
        "names": _PS_BUTTONS,
        "members": {"PS": "PSButton"},
    },
]


def _define_files(r: dict) -> list[str]:
    d = r["define_file"]
    return [d] if isinstance(d, str) else list(d)


def _in_scope(r: dict, up_file: str) -> bool:
    return "scope" not in r or any(fnmatch.fnmatchcase(up_file, g) for g in r["scope"])

# Baseline for the Arduino macro set: every macro defined after <Arduino.h> that is not already
# defined by the C/C++ standard library (and POSIX headers vendored code uses) under the same flags.
BASELINE_HEADERS = """
algorithm any array atomic bit bitset cassert cctype cerrno cfloat charconv chrono cinttypes
climits clocale cmath compare complex concepts condition_variable csetjmp csignal cstdarg cstddef
cstdint cstdio cstdlib cstring ctime cwchar cwctype deque exception filesystem format forward_list
fstream functional future initializer_list iomanip ios iosfwd iostream istream iterator limits list
locale map memory memory_resource mutex new numbers numeric optional ostream queue random ranges
ratio regex scoped_allocator set shared_mutex source_location span sstream stack stdexcept
stop_token streambuf string string_view system_error thread tuple type_traits typeindex typeinfo
unordered_map unordered_set utility valarray variant vector version
cxxabi.h pthread.h unistd.h fcntl.h sys/stat.h sys/types.h dirent.h sys/uio.h sys/time.h
""".split()

HEADER_EXTS = {".h", ".hpp", ".hh", ".inc", ".ipp", ".tpp", ""}
GUARD_BEGIN = "// prosdk: Arduino macro guard begin (generated by tools/upstream/wpilib.py)"
GUARD_END = "// prosdk: Arduino macro guard end"


# --------------------------------------------------------------------------------------
# Lexer (lossless). Token kinds: ws, comment, string, char, ident, number, punct, include.

_INCLUDE_RE = re.compile(r'#[ \t]*(?:include|include_next|import)\b[^\n]*')
_RAW_PREFIX = {"R", "u8R", "uR", "UR", "LR"}
_STR_PREFIX = {"u8", "u", "U", "L"}


def lex(text: str) -> list[list[str]]:
    toks: list[list[str]] = []
    i, n = 0, len(text)
    line_start = True
    while i < n:
        c = text[i]
        # preprocessor include directive (whole line)
        if c == "#" and line_start:
            m = _INCLUDE_RE.match(text, i)
            if m:
                toks.append(["include", m.group(0)])
                i = m.end()
                line_start = False
                continue
        if c in " \t\r\n\f\v" or (c == "\\" and i + 1 < n and text[i + 1] == "\n"):
            j = i
            while j < n and (text[j] in " \t\r\n\f\v" or (text[j] == "\\" and j + 1 < n and text[j + 1] == "\n")):
                j += 1
            s = text[i:j]
            toks.append(["ws", s])
            if "\n" in s:
                line_start = True
            i = j
            continue
        if text.startswith("//", i):
            j = i
            while True:
                e = text.find("\n", j)
                if e == -1:
                    e = n
                    break
                if e > 0 and text[e - 1] == "\\":
                    j = e + 1
                    continue
                break
            toks.append(["comment", text[i:e]])
            i = e
            continue
        if text.startswith("/*", i):
            e = text.find("*/", i + 2)
            e = n if e == -1 else e + 2
            toks.append(["comment", text[i:e]])
            i = e
            line_start = False
            continue
        if c.isalpha() or c == "_" or c == "$":
            j = i + 1
            while j < n and (text[j].isalnum() or text[j] in "_$"):
                j += 1
            word = text[i:j]
            if j < n and text[j] == '"' and word in _RAW_PREFIX:
                m = re.compile(r'"([^ ()\\\t\n]{0,16})\(').match(text, j)
                if m:
                    delim = m.group(1)
                    end = text.find(")" + delim + '"', m.end())
                    end = n if end == -1 else end + len(delim) + 2
                    toks.append(["string", text[i:end]])
                    i = end
                    line_start = False
                    continue
            if j < n and text[j] in "\"'" and word in _STR_PREFIX:
                q = text[j]
                e = _scan_quoted(text, j, q)
                toks.append(["string" if q == '"' else "char", text[i:e]])
                i = e
                line_start = False
                continue
            toks.append(["ident", word])
            i = j
            line_start = False
            continue
        if c.isdigit() or (c == "." and i + 1 < n and text[i + 1].isdigit()):
            j = i + 1
            while j < n:
                d = text[j]
                if d in "eEpP" and j + 1 < n and text[j + 1] in "+-":
                    j += 2
                    continue
                if d.isalnum() or d in "_.":
                    j += 1
                    continue
                if d == "'" and j + 1 < n and text[j + 1].isalnum():
                    j += 2
                    continue
                break
            toks.append(["number", text[i:j]])
            i = j
            line_start = False
            continue
        if c == '"' or c == "'":
            e = _scan_quoted(text, i, c)
            toks.append(["string" if c == '"' else "char", text[i:e]])
            i = e
            line_start = False
            continue
        if text.startswith("::", i) or text.startswith("->", i):
            toks.append(["punct", text[i:i + 2]])
            i += 2
            line_start = False
            continue
        toks.append(["punct", c])
        i += 1
        if c != "#":
            line_start = False
    return toks


def _scan_quoted(text: str, i: int, q: str) -> int:
    j = i + 1
    n = len(text)
    while j < n:
        d = text[j]
        if d == "\\":
            j += 2
            continue
        if d == q:
            return j + 1
        if d == "\n":
            return j
        j += 1
    return n


def _sig(toks, i, step):
    j = i + step
    while 0 <= j < len(toks) and toks[j][0] in ("ws", "comment"):
        j += step
    return j if 0 <= j < len(toks) else None


def _tok(toks, j):
    return toks[j] if j is not None else ["", ""]


# --------------------------------------------------------------------------------------
# Module data


@dataclass
class Rule:
    kind: str                # header | source | test | test-header
    src: str                 # upstream dir (repo-relative)
    dst: str                 # destination dir (prosdk repo-relative)
    include: list[str]
    exclude: dict[str, str] = field(default_factory=dict)  # glob -> reason
    include_base: str | None = None  # registers include names relative to this dest dir


@dataclass
class Module:
    name: str
    sparse: list[str]
    rules: list[Rule]
    transforms: dict[str, bool]
    path: Path


DEFAULT_TRANSFORMS = {"rename": True, "fs": False, "cxx23": True, "arduino_renames": True,
                      "guard": True, "verbatim": False, "config_include": [],
                      "multi_subscript": False, "guard_pasted": False}

# Section 7.1: header that carries the configuration vendored headers need (Eigen limits, Sleipnir
# switches). The `config_include` transform (list of regexes on the destination path) inserts it
# before the first #include of every matching file, so a sketch TU and the archive see the same
# configuration without -D flags.
CONFIG_HEADER = "prosdk/platform/config.hpp"


# Section 7.1: C++23 multidimensional subscript (P2128). The target language mode is gnu++20, where
# `m[i, j]` is not an error but a deprecated comma expression that silently means `m[j]`, and a
# multi-parameter `operator[]` does not compile. The `multi_subscript` transform rewrites every
# subscript whose top-level contents hold a comma to a call (`m[i, j]` -> `m(i, j)`) and every
# multi-parameter `operator[](a, b)` declaration to `operator()(a, b)`. Eigen already provides
# operator()(i, j) with the same meaning; for Sleipnir the rewritten declarations provide it.
# Lambda introducers, structured bindings and attributes are not subscripts and are left alone.
_NOT_SUBSCRIPT_BEFORE = {"auto", "return", "co_return", "co_yield", "case", "new", "delete",
                         "throw", "else", "do", "and", "or", "not", "xor", "bitand", "bitor",
                         "compl", "and_eq", "or_eq", "xor_eq", "not_eq", "sizeof", "alignof",
                         "decltype", "noexcept", "requires", "typename", "template", "constexpr",
                         "const", "volatile", "static", "inline", "mutable"}


def _match_close(toks, i: int) -> tuple[int, bool]:
    """Index of the bracket closing toks[i] and whether a comma appears at its top level."""
    depth = 0
    comma = False
    for k in range(i, len(toks)):
        kind, s = toks[k]
        if kind != "punct":
            continue
        if s in "([{":
            depth += 1
        elif s in ")]}":
            depth -= 1
            if depth == 0:
                return k, comma
        elif s == "," and depth == 1:
            comma = True
    raise SystemExit("error: unbalanced bracket in multi_subscript transform")


def rewrite_multi_subscripts(toks) -> int:
    count = 0
    for i, t in enumerate(toks):
        if t[0] != "punct" or t[1] != "[":
            continue
        p = _sig(toks, i, -1)
        nx = _sig(toks, i, 1)
        prev = _tok(toks, p)
        if prev[1] == "[" or _tok(toks, nx)[1] == "[":
            continue  # [[attribute]]
        if prev[1] == "operator":
            if _tok(toks, nx)[1] != "]":
                continue
            lp = _sig(toks, nx, 1)
            if _tok(toks, lp)[1] != "(":
                continue
            _, comma = _match_close(toks, lp)
            if comma:
                t[1], toks[nx][1] = "(", ")"
                count += 1
            continue
        if not (prev[1] in (")", "]") or (prev[0] == "ident" and prev[1] not in _NOT_SUBSCRIPT_BEFORE)):
            continue
        close, comma = _match_close(toks, i)
        if comma:
            t[1], toks[close][1] = "(", ")"
            count += 1
    return count


def load_modules() -> dict[str, Module]:
    mods: dict[str, Module] = {}
    for f in sorted(MODULES_DIR.glob("*.py")):
        spec = importlib.util.spec_from_file_location(f"prosdk_module_{f.stem}", f)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)  # type: ignore[union-attr]
        tr = dict(DEFAULT_TRANSFORMS)
        tr.update(getattr(m, "TRANSFORMS", {}))
        rules = [Rule(**r) for r in m.RULES]
        mods[m.NAME] = Module(m.NAME, list(m.SPARSE), rules, tr, f)
    return mods


def rename_path(rel: str) -> str:
    return "prosdk/" + rel[4:] if rel.startswith("wpi/") else rel


def select_files(up: Path, rule: Rule) -> list[tuple[str, str]]:
    """Return sorted (upstream_relpath_in_rule, dest_repo_relpath)."""
    base = up / rule.src
    if not base.is_dir():
        raise SystemExit(f"error: upstream dir missing: {rule.src} (sparse checkout incomplete?)")
    out = []
    if not any(c in g for g in rule.include for c in "*?["):
        # literal file list (no directory walk)
        for rel in sorted(rule.include):
            if not (base / rel).is_file():
                raise SystemExit(f"error: upstream file missing: {rule.src}/{rel}")
            out.append((rel, f"{rule.dst}/{rename_path(rel)}"))
        return out
    for p in sorted(base.rglob("*")):
        if not p.is_file():
            continue
        rel = p.relative_to(base).as_posix()
        if not any(fnmatch.fnmatchcase(rel, g) for g in rule.include):
            continue
        if any(fnmatch.fnmatchcase(rel, g) for g in rule.exclude):
            continue
        out.append((rel, f"{rule.dst}/{rename_path(rel)}"))
    return out


# --------------------------------------------------------------------------------------
# Transform


class Ctx:
    def __init__(self, up: Path, include_map: dict[str, str], macros: set[str]):
        self.up = up
        self.include_map = include_map
        self.macros = macros


_INC_LINE = re.compile(r'^(#[ \t]*(?:include|include_next|import)[ \t]*)([<"])([^>"]+)([>"])(.*)$', re.S)
_HAS_INC = re.compile(r'(__has_include(?:_next)?[ \t]*\([ \t]*)([<"])([^>"]+)([>"])')


def map_include(ctx: Ctx, up_file: str, name: str, quoted: bool) -> str:
    if quoted and (ctx.up / os.path.dirname(up_file) / name).is_file():
        return name  # relative include next to the file: copied with the same layout
    if name in ctx.include_map:
        return ctx.include_map[name]
    if name.startswith("wpi/"):
        return rename_path(name)
    if name in CXX_SHIM_HEADERS:
        return CXX_SHIM_HEADERS[name]
    return name


def transform(ctx: Ctx, mod: Module, up_file: str, dest: str, text: str) -> str:
    tr = mod.transforms
    before = {p: len(re.findall(p, text)) for p in PROTECTED_PATTERNS}
    toks = lex(text)
    assert "".join(t[1] for t in toks) == text, f"lexer is not lossless for {up_file}"
    used_shims: set[str] = set()
    defined_shim_headers: set[str] = set()
    define_file = None
    for r in ARDUINO_RENAMES:
        for d in _define_files(r):
            if up_file.endswith("/" + d) or up_file == d:
                define_file = r
    scoped = [r for r in ARDUINO_RENAMES if _in_scope(r, up_file)]

    # enum bodies to rename in the defining file: token index ranges
    enum_ranges: list[tuple[int, int, dict]] = []
    if tr["arduino_renames"] and define_file is not None:
        for i, t in enumerate(toks):
            if t[0] == "ident" and t[1] == define_file["enum"]:
                p = _sig(toks, i, -1)
                if _tok(toks, p)[1] in ("enum", "class", "struct"):
                    j = _sig(toks, i, 1)
                    while j is not None and toks[j][1] != "{" and toks[j][1] != ";":
                        j = _sig(toks, j, 1)
                    if j is not None and toks[j][1] == "{":
                        k = j
                        while k < len(toks) and toks[k][1] != "}":
                            k += 1
                        enum_ranges.append((j, k, define_file["names"]))

    if tr["multi_subscript"]:
        rewrite_multi_subscripts(toks)

    for i, t in enumerate(toks):
        kind, s = t
        if kind == "include":
            m = _INC_LINE.match(s)
            if m:
                name = map_include(ctx, up_file, m.group(3), m.group(2) == '"')
                if tr["cxx23"] and m.group(3) in CXX_SHIM_HEADERS:
                    name = CXX_SHIM_HEADERS[m.group(3)]
                t[1] = m.group(1) + m.group(2) + name + m.group(4) + m.group(5)
                if name.startswith("prosdk/cxx/"):
                    defined_shim_headers.add(name)
            continue
        if kind == "comment":
            if tr["rename"]:
                s = re.sub(r"\bwpi::", "prosdk::", s)
                s = re.sub(r'([<"])wpi/', r"\1prosdk/", s)
            if tr["arduino_renames"]:
                s = _qualified_renames(s, scoped)
            t[1] = s
            continue
        if kind == "string":
            if tr["rename"]:
                t[1] = re.sub(r"\bwpi::", "prosdk::", s)
            continue
        if kind != "ident":
            continue
        # __has_include(<...>) probes
        if s in ("__has_include", "__has_include_next"):
            j = _sig(toks, i, 1)
            if _tok(toks, j)[1] == "(":
                # reassemble up to ')'
                k = j + 1
                buf = ""
                while k < len(toks) and toks[k][1] != ")":
                    buf += toks[k][1]
                    k += 1
                m = re.match(r'^\s*([<"])([^>"]+)([>"])\s*$', buf)
                if m:
                    name = map_include(ctx, up_file, m.group(2), m.group(1) == '"')
                    if tr["cxx23"] and m.group(2) in CXX_SHIM_HEADERS:
                        name = CXX_SHIM_HEADERS[m.group(2)]
                    toks[j + 1][1] = m.group(1) + name + m.group(3)
                    toks[j + 1][0] = "include"
                    for q in range(j + 2, k):
                        toks[q][1] = ""
                        toks[q][0] = "include"
            continue
        if tr["rename"] and s == "wpi":
            t[1] = "prosdk"
            continue
        if tr["fs"] and s == "fs":
            p = _sig(toks, i, -1)
            nx = _sig(toks, i, 1)
            ptxt = _tok(toks, p)[1]
            ntxt = _tok(toks, nx)[1]
            if ptxt == "namespace":
                pp = _tok(toks, _sig(toks, p, -1))[1]
                if ntxt == "=":
                    pass  # local alias: namespace fs = std::filesystem;
                elif pp == "using" or ntxt in ("{", "::"):
                    t[1] = "prosdk::fs"
            elif ntxt == "::" and ptxt not in (".", "->") and _tok(toks, p)[0] != "ident":
                if ptxt == "::":
                    ppk = _tok(toks, _sig(toks, p, -1))
                    if ppk[0] == "ident" or ppk[1] == ">":
                        continue  # X::fs:: (member of another scope)
                t[1] = "prosdk::fs"
            continue
        if tr["cxx23"] and s == "std":
            nx = _sig(toks, i, 1)
            if _tok(toks, nx)[1] == "::":
                nn = _sig(toks, nx, 1)
                name = _tok(toks, nn)[1]
                if name in CXX_SHIMS:
                    p = _sig(toks, i, -1)
                    if _tok(toks, p)[1] not in (".", "->"):
                        t[1] = "prosdk::cxx"
                        used_shims.add(CXX_SHIMS[name])
            continue
        if tr["arduino_renames"]:
            for r in scoped:
                if s in r["names"]:
                    p = _sig(toks, i, -1)
                    if _tok(toks, p)[1] == "::":
                        q = _sig(toks, p, -1)
                        if _tok(toks, q)[1] == r["enum"]:
                            t[1] = r["names"][s]
                            break
                if s in r.get("members", {}) and _tok(toks, _sig(toks, i, 1))[1] == "(":
                    t[1] = r["members"][s]
                    break
            for (a, b, names) in enum_ranges:
                if a < i < b and s in names:
                    nx = _sig(toks, i, 1)
                    if _tok(toks, nx)[1] in (",", "=", "}"):
                        t[1] = names[s]

    out = "".join(t[1] for t in toks)

    # add shim includes that are used but not included
    missing = sorted(h for h in used_shims if h not in defined_shim_headers)
    if missing:
        out = _insert_includes(out, [f"#include <{h}>" for h in missing])

    if any(re.search(rx, dest) for rx in tr["config_include"]):
        out = _insert_includes(out, [f"#include <{CONFIG_HEADER}>  // prosdk: configuration"])

    after = {p: len(re.findall(p, out)) for p in PROTECTED_PATTERNS}
    for p in PROTECTED_PATTERNS:
        if before[p] != after[p]:
            raise SystemExit(f"error: protected pattern {p!r} changed in {up_file} "
                             f"({before[p]} -> {after[p]})")
    return out


def _qualified_renames(s: str, renames: list[dict]) -> str:
    for r in renames:
        for old, new in r["names"].items():
            s = re.sub(rf"\b{r['enum']}::{old}\b", f"{r['enum']}::{new}", s)
    return s


def _insert_includes(text: str, lines: list[str]) -> str:
    block = "\n".join(lines) + "\n"
    m = re.search(r"^#pragma once[^\n]*\n", text, re.M)
    if m:
        return text[:m.end()] + "\n" + block + text[m.end():]
    m = re.search(r"^#[ \t]*include\b", text, re.M)
    if m:
        return text[:m.start()] + block + text[m.start():]
    m = re.search(r"^#ifndef (\w+)\n#define \1[^\n]*\n", text, re.M)
    if m:
        return text[:m.end()] + block + text[m.end():]
    return block + text


def is_header(dest: str) -> bool:
    return Path(dest).suffix in HEADER_EXTS


def code_identifiers(text: str) -> set[str]:
    return {t[1] for t in lex(text) if t[0] == "ident"}


def defined_macros(text: str) -> set[str]:
    return set(re.findall(r"^[ \t]*#[ \t]*define[ \t]+(\w+)", text, re.M))


def add_guard(text: str, names: list[str], dest: str) -> str:
    if not names:
        return text
    pro = [GUARD_BEGIN]
    for nme in names:
        pro.append(f'#pragma push_macro("{nme}")')
        pro.append(f"#undef {nme}")
    epi = [GUARD_END]
    for nme in reversed(names):
        epi.append(f'#pragma pop_macro("{nme}")')
    # after the leading comment block (license header)
    lines = text.split("\n")
    k = 0
    in_block = False
    while k < len(lines):
        ln = lines[k].strip()
        if in_block:
            if "*/" in ln:
                in_block = False
            k += 1
            continue
        if ln.startswith("//") or ln == "":
            k += 1
            continue
        if ln.startswith("/*"):
            in_block = "*/" not in ln
            k += 1
            continue
        break
    # back up over trailing blank lines so the guard sits right after the comment block
    while k > 0 and lines[k - 1].strip() == "":
        k -= 1
    head = "\n".join(lines[:k])
    body = "\n".join(lines[k:]).lstrip("\n")
    if not body.endswith("\n"):
        body += "\n"
    prefix = head + "\n\n" if head else ""
    return prefix + "\n".join(pro) + "\n\n" + body + "\n" + "\n".join(epi) + "\n"


# Macros defined in vendored headers whose replacement text names an Arduino macro would expand
# that name at the use site, outside the per-header guard. Allowed only when the definition is in
# a branch never compiled for ESP32/host (header -> {macro: reason}).
MACRO_BODY_ALLOW = {
    "src/prosdk/thirdparty/nanopb/pb.h": {
        "PB_PROGMEM": "inside #if defined(__AVR__) only",
        "PB_PROGMEM_READU32": "inside #if defined(__AVR__) only",
    },
}

_DEFINE_RE = re.compile(r"^[ \t]*#[ \t]*define[ \t]+(\w+)(\([^)]*\))?(.*)$", re.M)


def macro_body_hazards(text: str, macros: set[str]) -> dict[str, list[str]]:
    joined = text.replace("\\\n", " ")
    out = {}
    for m in _DEFINE_RE.finditer(joined):
        params = set(re.findall(r"\w+", m.group(2) or ""))
        bad = (code_identifiers(m.group(3)) - params) & macros
        if bad:
            out[m.group(1)] = sorted(bad)
    return out


_PASTE_RE = re.compile(r"(\w+)\s*##\s*(\w+)")
Affixes = dict[str, tuple[set[str], set[str]]]


def paste_affixes(texts: list[str]) -> Affixes:
    """Per function-like macro of these headers: the fixed fragments it pastes onto a parameter
    (`P ## abbreviation` -> prefix "P", `nameSingular ## _t` -> suffix "_t")."""
    out: Affixes = {}
    for text in texts:
        for m in _DEFINE_RE.finditer(text.replace("\\\n", " ")):
            if not m.group(2):
                continue
            params = {x.strip() for x in m.group(2)[1:-1].split(",")}
            pre, suf = out.setdefault(m.group(1), (set(), set()))
            for a, b in _PASTE_RE.findall(m.group(3)):
                if b in params and a not in params:
                    pre.add(a)
                elif a in params and b not in params:
                    suf.add(b)
    return {k: v for k, v in out.items() if v[0] or v[1]}


def pasted_identifiers(text: str, affixes: Affixes) -> set[str]:
    """Identifiers formed by invoking a pasting macro: affixes x identifier arguments."""
    toks = [t for t in lex(text) if t[0] not in ("ws", "comment")]
    out: set[str] = set()
    for i, t in enumerate(toks):
        if t[0] != "ident" or t[1] not in affixes or i + 1 >= len(toks) or toks[i + 1][1] != "(":
            continue
        pre, suf = affixes[t[1]]
        depth = 0
        for k in range(i + 1, len(toks)):
            s = toks[k][1]
            if s == "(":
                depth += 1
            elif s == ")":
                depth -= 1
                if depth == 0:
                    break
            elif toks[k][0] == "ident":
                out |= {a + s for a in pre} | {s + b for b in suf}
    return out


def guard_header(path: Path, dest: str, macros: set[str], affixes: Affixes | None = None) -> None:
    """Section 5: wrap a vendored public header in push_macro/undef ... pop_macro for exactly the
    Arduino macros its tokens use. Runs after the patches, so patches see unguarded text.

    With `guard_pasted`, identifiers that an invocation of a token-pasting macro of the module
    forms count as used too: `UNIT_ADD_WITH_METRIC_PREFIXES(..., S, ...)` forms `PS`
    (petasiemens), an Xtensa special-register macro (xtensa/config/specreg.h)."""
    text = path.read_text()
    idents = code_identifiers(text)
    pasted = pasted_identifiers(text, affixes) if affixes else set()
    names = sorted((idents | pasted) & macros)
    clash = defined_macros(text) & set(names)
    if clash:
        raise SystemExit(f"error: {dest} defines Arduino macro names {sorted(clash)}; "
                         f"add a patch or rename")
    allow = MACRO_BODY_ALLOW.get(dest, {})
    hazards = {k: v for k, v in macro_body_hazards(text, macros).items() if k not in allow}
    if hazards:
        raise SystemExit(f"error: {dest}: macro bodies name Arduino macros (expanded outside the "
                         f"guard): {hazards}; add a patch")
    if names:
        path.write_text(add_guard(text, names, dest))


# --------------------------------------------------------------------------------------
# Arduino macro set


def compute_arduino_macros() -> list[str] | None:
    p = arduino_recipe.paths()
    gxx = p["toolchain"] / "xtensa-esp32s3-elf-g++"
    if not gxx.exists():
        return None
    flags = arduino_recipe.cxx_flags(p)
    with tempfile.TemporaryDirectory(prefix="prosdk-macros-") as td:
        td = Path(td)
        (td / "ard.cpp").write_text("#include <Arduino.h>\n")
        (td / "base.cpp").write_text("".join(f"#include <{h}>\n" for h in BASELINE_HEADERS))
        res = {}
        for name in ("ard", "base"):
            cmd = _low_prio() + [str(gxx)] + flags + ["-dM", "-E", str(td / f"{name}.cpp")]
            r = subprocess.run(cmd, capture_output=True, text=True)
            if r.returncode != 0:
                raise SystemExit(f"error: preprocessing failed:\n{r.stderr[-2000:]}")
            res[name] = set(re.findall(r"^#define (\w+)", r.stdout, re.M))
    diff = res["ard"] - res["base"]
    # reserved names (_X, __x) are platform internals, never user/vendored identifiers
    return sorted(m for m in diff if not m.startswith("_"))


def _low_prio() -> list[str]:
    BUILD_LOCK.parent.mkdir(parents=True, exist_ok=True)
    cmd = []
    if shutil.which("flock"):
        cmd += ["flock", str(BUILD_LOCK)]
    cmd += ["nice", "-n", "19"]
    if shutil.which("ionice"):
        cmd += ["ionice", "-c3"]
    return cmd


def macros_text(macros: list[str]) -> str:
    return ("# Arduino-ESP32 macros that can collide with C++ identifiers.\n"
            "# Generated by tools/upstream/wpilib.py from <Arduino.h> (exact esp32s3 recipe)\n"
            "# minus the macros of the C/C++ standard headers; reserved names dropped.\n"
            + "".join(m + "\n" for m in macros))


def read_macros_file() -> list[str]:
    if not MACROS_FILE.exists():
        raise SystemExit("error: no Arduino toolchain and no tools/upstream/arduino-macros.txt")
    return [ln.strip() for ln in MACROS_FILE.read_text().splitlines()
            if ln.strip() and not ln.startswith("#")]


# --------------------------------------------------------------------------------------
# Upstream checkout


def read_lock() -> dict[str, str]:
    d = {}
    for ln in LOCK.read_text().splitlines():
        if "=" in ln:
            k, v = ln.split("=", 1)
            d[k.strip()] = v.strip()
    return d


def git(up: Path, *args: str, check=True) -> str:
    r = subprocess.run(["git", "-C", str(up), *args], capture_output=True, text=True)
    if check and r.returncode != 0:
        raise SystemExit(f"error: git {' '.join(args)} failed:\n{r.stderr}")
    return r.stdout.strip()


def ensure_upstream(path: Path, lock: dict[str, str], sparse: list[str]) -> Path:
    if not path.exists():
        print(f"fetching {lock['repo']} @ {lock['commit'][:12]} into {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "clone", "--filter=blob:none", "--no-checkout", "--sparse",
                        lock["repo"], str(path)], check=True)
        git(path, "sparse-checkout", "set", *sparse)
        git(path, "checkout", "--detach", lock["commit"])
    head = git(path, "rev-parse", "HEAD")
    if head != lock["commit"]:
        raise SystemExit(f"error: {path} is at {head}, the lock pins {lock['commit']}")
    have = set(git(path, "sparse-checkout", "list", check=False).split())
    missing = [s for s in sparse if s not in have and not (path / s).exists()]
    if missing:
        print(f"adding sparse paths to {path}: {' '.join(missing)}")
        git(path, "sparse-checkout", "add", *missing)
    dirty = git(path, "status", "--porcelain", "--untracked-files=no", "--", *sparse)
    if dirty:
        raise SystemExit(f"error: upstream checkout has local changes:\n{dirty}")
    return path


# --------------------------------------------------------------------------------------
# Generation


def build_include_map(up: Path, mods: dict[str, Module]) -> dict[str, str]:
    imap: dict[str, str] = {}
    for mod in mods.values():
        for rule in mod.rules:
            if rule.include_base is None:
                continue
            if not (up / rule.src).is_dir():
                continue
            for rel, dest in select_files(up, rule):
                name = os.path.relpath(dest, rule.include_base).replace(os.sep, "/")
                imap.setdefault(rel, name)
    return imap


def generate(up: Path, mods: dict[str, Module], selected: list[str], macros: set[str],
             stage: Path, unpatched: Path | None) -> dict[str, list[str]]:
    imap = build_include_map(up, {k: mods[k] for k in mods})
    ctx = Ctx(up, imap, macros)
    produced: dict[str, list[str]] = {}
    for name in selected:
        mod = mods[name]
        files: list[str] = []
        for rule in mod.rules:
            for rel, dest in select_files(up, rule):
                up_file = f"{rule.src}/{rel}"
                raw = (up / up_file).read_bytes()
                try:
                    text = raw.decode("utf-8")
                except UnicodeDecodeError:
                    text = None
                if text is None or mod.transforms["verbatim"]:
                    out_bytes = raw
                else:
                    out_bytes = transform(ctx, mod, up_file, dest, text).encode("utf-8")
                if dest in files or any(dest in v for v in produced.values()):
                    raise SystemExit(f"error: two rules produce {dest}")
                (stage / dest).parent.mkdir(parents=True, exist_ok=True)
                (stage / dest).write_bytes(out_bytes)
                files.append(dest)
        if unpatched is not None:
            for f in files:
                (unpatched / f).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(stage / f, unpatched / f)
        apply_patches(name, stage)
        if mod.transforms["guard"]:
            headers = [d for d in files if d.startswith("src/") and is_header(d)]
            affixes: Affixes | None = None
            if mod.transforms["guard_pasted"]:
                affixes = paste_affixes([(stage / d).read_text() for d in headers])
            for dest in headers:
                guard_header(stage / dest, dest, macros, affixes)
        produced[name] = sorted(files)
    return produced


def apply_patches(module: str, stage: Path) -> None:
    pdir = PATCHES_DIR / module
    if not pdir.is_dir():
        return
    for patch in sorted(pdir.glob("*.patch")):
        env = dict(os.environ, GIT_CEILING_DIRECTORIES=str(stage.parent))
        r = subprocess.run(["git", "apply", "--whitespace=nowarn", "-p1", str(patch)],
                           cwd=stage, capture_output=True, text=True, env=env)
        if r.returncode != 0:
            raise SystemExit(f"error: patch {patch.relative_to(ROOT)} does not apply:\n{r.stderr}")


def read_generated() -> dict[str, list[str]]:
    d: dict[str, list[str]] = {}
    if not GENERATED.exists():
        return d
    for ln in GENERATED.read_text().splitlines():
        if not ln or ln.startswith("#"):
            continue
        mod, path = ln.split(" ", 1)
        d.setdefault(mod, []).append(path)
    return d


def generated_text(d: dict[str, list[str]]) -> str:
    lines = ["# Files produced by tools/upstream/wpilib.py (module path). Do not edit by hand.\n"]
    for mod in sorted(d):
        for p in sorted(d[mod]):
            lines.append(f"{mod} {p}\n")
    return "".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="Vendor the pinned allwpilib commit into prosdk.")
    ap.add_argument("--module", action="append", dest="modules", metavar="NAME",
                    help="only (re)generate this module (repeatable); default: all")
    ap.add_argument("--upstream", type=Path,
                    default=Path(os.environ["PROSDK_UPSTREAM"]) if os.environ.get("PROSDK_UPSTREAM")
                    else ROOT / ".upstream" / "allwpilib",
                    help="allwpilib checkout at the pinned commit (env PROSDK_UPSTREAM)")
    ap.add_argument("--check", action="store_true",
                    help="regenerate into a temp dir and diff against the tree (exit 1 on drift)")
    ap.add_argument("--emit-unpatched", type=Path, metavar="DIR",
                    help="also write the pre-patch output here (for writing patches)")
    args = ap.parse_args()

    mods = load_modules()
    selected = args.modules or sorted(mods)
    for s in selected:
        if s not in mods:
            raise SystemExit(f"error: unknown module {s}; have {', '.join(sorted(mods))}")
    lock = read_lock()
    sparse = sorted({s for m in mods.values() for s in m.sparse})
    up = ensure_upstream(args.upstream.resolve(), lock, sparse)

    computed = compute_arduino_macros()
    macro_list = computed if computed is not None else read_macros_file()
    if computed is None:
        print("note: Arduino toolchain not found; using tools/upstream/arduino-macros.txt")
    macros = set(macro_list)

    prev = read_generated()
    with tempfile.TemporaryDirectory(prefix="prosdk-vendor-") as td:
        stage = Path(td) / "tree"
        stage.mkdir()
        produced = generate(up, mods, selected, macros, stage, args.emit_unpatched)
        new_gen = dict(prev)
        for k, v in produced.items():
            new_gen[k] = v
        gen_txt = generated_text(new_gen)
        mac_txt = macros_text(macro_list)

        if args.check:
            drift = []
            for mod, files in produced.items():
                for f in files:
                    a = (ROOT / f).read_bytes() if (ROOT / f).exists() else None
                    b = (stage / f).read_bytes()
                    if a != b:
                        drift.append(f)
                        if a is not None:
                            d = difflib.unified_diff(a.decode(errors="replace").splitlines(),
                                                     b.decode(errors="replace").splitlines(),
                                                     f"a/{f}", f"b/{f}", lineterm="", n=1)
                            print("\n".join(list(d)[:40]))
                        else:
                            print(f"missing: {f}")
                for f in prev.get(mod, []):
                    if f not in files and (ROOT / f).exists():
                        drift.append(f)
                        print(f"stale (no longer generated): {f}")
            if not GENERATED.exists() or GENERATED.read_text() != gen_txt:
                drift.append(str(GENERATED.relative_to(ROOT)))
                print("drift: tools/upstream/generated.txt")
            if computed is not None and (not MACROS_FILE.exists() or MACROS_FILE.read_text() != mac_txt):
                drift.append(str(MACROS_FILE.relative_to(ROOT)))
                print("drift: tools/upstream/arduino-macros.txt")
            n = sum(len(v) for v in produced.values())
            if drift:
                print(f"CHECK FAILED: {len(drift)} of {n} generated files differ")
                return 1
            print(f"check OK: {n} generated files in {len(produced)} modules match "
                  f"({', '.join(sorted(produced))})")
            return 0

        written = removed = 0
        for mod, files in produced.items():
            for f in files:
                b = (stage / f).read_bytes()
                dst = ROOT / f
                if not dst.exists() or dst.read_bytes() != b:
                    dst.parent.mkdir(parents=True, exist_ok=True)
                    dst.write_bytes(b)
                    written += 1
            for f in prev.get(mod, []):
                if f not in files and (ROOT / f).exists():
                    (ROOT / f).unlink()
                    removed += 1
                    _prune_empty_dirs((ROOT / f).parent)
        if not GENERATED.exists() or GENERATED.read_text() != gen_txt:
            GENERATED.write_text(gen_txt)
        if computed is not None and (not MACROS_FILE.exists() or MACROS_FILE.read_text() != mac_txt):
            MACROS_FILE.write_text(mac_txt)
        total = sum(len(v) for v in produced.values())
        print(f"generated {total} files in {len(produced)} modules "
              f"({', '.join(f'{k}={len(v)}' for k, v in sorted(produced.items()))}); "
              f"{written} written, {removed} removed, {total - written} unchanged")
    return 0


def _prune_empty_dirs(d: Path) -> None:
    while d != ROOT and d.is_dir() and not any(d.iterdir()):
        d.rmdir()
        d = d.parent


if __name__ == "__main__":
    sys.exit(main())
