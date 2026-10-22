#!/usr/bin/env python3
# Copyright (c) Probot Studio. Hand-written; see LICENSE.
"""Parses the UART logs of the QEMU speed benchmark (test/target/QemuBench, design/qemu-bench.md).

  tools/qemu/bench.py [--json OUT] [--compile-log LOG] [--check BASELINE [--threshold PCT]]
                      [--save-baseline BASELINE] UART_LOG [UART_LOG ...]

Prints the BENCH lines of the first log as a table, writes them with the memory lines (MEM,
STACK, TASK), the clock line and the static RAM of the compile log to JSON, and with two or more
logs compares every benchmark's instr_per_op between the boots: QEMU's -icount makes the counts
deterministic for one binary, so any difference exits 1. --check exits 1 when a benchmark of the
baseline is missing or its instr_per_op grew by more than PCT percent (default 10); it reads the
first log, which stands for all of them once the boots are identical. --save-baseline writes the
baseline only when everything else passed. Run by tools/qemu/run.sh --bench.
"""

import argparse
import datetime
import json
import re
import sys

ROBOT_KEYS = ("max_us", "jitter_max_us", "overruns", "headroom_pct", "headroom_min_pct")


def number(text):
    try:
        return int(text)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            return text


def fields(words):
    out = {}
    for word in words:
        key, sep, value = word.partition("=")
        if sep:
            out[key] = number(value)
    return out


def parse(path):
    run = {"benchmarks": {}, "order": [], "mem": [], "stack": {}, "tasks": [], "clock": {},
           "checks": [], "info": []}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            words = line.split()
            if not words:
                continue
            tag = words[0]
            if tag == "BENCH" and len(words) > 1:
                run["benchmarks"][words[1]] = fields(words[2:])
                run["order"].append(words[1])
            elif tag == "MEM" and len(words) > 1:
                run["mem"].append(dict(where=words[1], **fields(words[2:])))
            elif tag == "STACK" and len(words) > 1:
                used, _, size = str(fields(words[2:]).get("used", "")).partition("/")
                run["stack"][words[1]] = {"used": number(used), "size": number(size)}
            elif tag == "TASK" and len(words) > 1:
                run["tasks"].append(dict(name=words[1], **fields(words[2:])))
            elif tag == "CLOCK":
                run["clock"] = fields(words[1:])
            elif tag == "BENCH-CHECK":
                run["checks"].append(" ".join(words[1:]))
            elif tag == "BENCH-INFO":
                run["info"].append(" ".join(words[1:]))
    return run


def static_memory(path):
    out = {}
    if not path:
        return out
    try:
        text = open(path, encoding="utf-8", errors="replace").read()
    except OSError:
        return out
    m = re.search(r"Sketch uses (\d+) bytes", text)
    if m:
        out["flash_bytes"] = int(m.group(1))
    m = re.search(r"Global variables use (\d+) bytes", text)
    if m:
        out["static_ram_bytes"] = int(m.group(1))
    return out


def table(run):
    rows = [("benchmark", "iters", "per_op_us", "instr_per_op", "notes")]
    for name in run["order"]:
        b = run["benchmarks"][name]
        notes = " ".join(f"{k}={b[k]}" for k in ROBOT_KEYS if k in b)
        rows.append((name, str(b.get("iters", "")), f"{b.get('per_op_us', 0):.3f}",
                     f"{b.get('instr_per_op', 0):.1f}", notes))
    widths = [max(len(r[i]) for r in rows) for i in range(4)]
    for r in rows:
        print("   " + "  ".join(r[i].ljust(widths[i]) if i == 0 else r[i].rjust(widths[i])
                                for i in range(4)) + ("  " + r[4] if r[4] else ""))


def compare_boots(runs):
    first = runs[0]["benchmarks"]
    worst = 0.0
    diffs = []
    for i, run in enumerate(runs[1:], start=2):
        for name, b in first.items():
            other = run["benchmarks"].get(name)
            if other is None:
                diffs.append(f"boot {i}: {name} missing")
                worst = float("inf")
                continue
            a, c = b.get("instr_per_op", 0.0), other.get("instr_per_op", 0.0)
            rel = abs(c - a) / a if a else (0.0 if c == a else float("inf"))
            if c != a:
                diffs.append(f"boot {i}: {name} {a} -> {c} ({100 * rel:+.4f} %)")
            worst = max(worst, rel)
    return {"boots": len(runs), "identical": not diffs, "max_rel_diff_pct": 100 * worst,
            "diffs": diffs}


def check(run, baseline_path, threshold):
    with open(baseline_path, encoding="utf-8") as f:
        baseline = json.load(f)["benchmarks"]
    failures = []
    notes = []
    for name, base in baseline.items():
        cur = run["benchmarks"].get(name)
        if cur is None:
            failures.append(f"{name}: missing from this run")
            continue
        a, c = base["instr_per_op"], cur["instr_per_op"]
        change = 100.0 * (c - a) / a if a else 0.0
        if change > threshold:
            failures.append(f"{name}: instr_per_op {a} -> {c} ({change:+.1f} %)")
        elif change < -threshold:
            notes.append(f"{name}: instr_per_op {a} -> {c} ({change:+.1f} %, faster)")
    for name in run["benchmarks"]:
        if name not in baseline:
            notes.append(f"{name}: not in the baseline")
    return failures, notes


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--json")
    ap.add_argument("--compile-log")
    ap.add_argument("--check")
    ap.add_argument("--threshold", type=float, default=10.0)
    ap.add_argument("--save-baseline")
    args = ap.parse_args()

    runs = [parse(p) for p in args.logs]
    run = runs[0]
    if not run["benchmarks"]:
        print("bench: no BENCH lines in " + args.logs[0], file=sys.stderr)
        return 1
    print(f"== benchmarks ({len(run['order'])}; instr = guest ns / 4 under -icount shift=2)")
    table(run)

    result = {
        "generated": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        "clock": run["clock"],
        "benchmarks": {name: run["benchmarks"][name] for name in run["order"]},
        "checks": run["checks"],
        "info": run["info"],
        "memory": {"mem": run["mem"], "stack": run["stack"], "tasks": run["tasks"],
                   **static_memory(args.compile_log)},
    }
    status = 0
    if len(runs) > 1:
        det = compare_boots(runs)
        result["determinism"] = det
        if det["identical"]:
            print(f"== determinism: {det['boots']} boots, every instr_per_op identical")
        else:
            print(f"== determinism: {det['boots']} boots, {len(det['diffs'])} differences, "
                  f"largest {det['max_rel_diff_pct']:.4f} %")
            for d in det["diffs"]:
                print("   " + d)
            status = 1

    if args.check:
        failures, notes = check(run, args.check, args.threshold)
        result["check"] = {"baseline": args.check, "threshold_pct": args.threshold,
                           "failures": failures, "notes": notes}
        for n in notes:
            print("   note: " + n)
        for f in failures:
            print("   REGRESSION: " + f)
        print(f"== check against {args.check} (threshold {args.threshold:g} %): "
              + ("FAIL" if failures else "PASS"))
        if failures:
            status = 1

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
            f.write("\n")
        print("== json: " + args.json)

    if args.save_baseline and status != 0:
        print("== baseline NOT written (the boots differ or the check failed): "
              + args.save_baseline)
    elif args.save_baseline:
        baseline = {
            "about": "QemuBench baseline (tools/qemu/run.sh --bench --check): guest "
                     "instructions per operation under QEMU -icount shift=2; design/qemu-bench.md",
            "generated": result["generated"],
            "benchmarks": {name: {"instr_per_op": run["benchmarks"][name]["instr_per_op"]}
                           for name in run["order"]},
        }
        with open(args.save_baseline, "w", encoding="utf-8") as f:
            json.dump(baseline, f, indent=2)
            f.write("\n")
        print("== baseline written: " + args.save_baseline)
    return status


if __name__ == "__main__":
    sys.exit(main())
