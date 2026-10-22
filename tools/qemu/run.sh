#!/usr/bin/env bash
# Copyright (c) Probot Studio. Hand-written; see LICENSE.
#
# prosdk on-target smoke test in Espressif's QEMU (machine esp32s3); design/PORT.md section 11,
# test/target/README.md. With --bench, the speed benchmark instead (design/qemu-bench.md).
#
#   tools/qemu/run.sh [--no-build] [--boots N] [--timeout S]
#   tools/qemu/run.sh --bench [--no-build] [--boots N] [--timeout S] [--check] [--threshold PCT]
#                     [--save-baseline]
#
# 1. installs/verifies the pinned QEMU (tools/qemu/setup.sh -> tools/bin/qemu)
# 2. compiles test/target/QemuSmoke with arduino-cli (huge_app, -j2, under the shared build lock
#    at the lowest CPU/IO priority; persistent build folder .build/qemu/QemuSmoke). No libraries
#    folder: probot-core is not available to the sketch. Arduino's library discovery keys on a
#    library's top-level header (prosdk.h, which pulls in <probot.h>), so the sketch reaches
#    prosdk the way a discovered precompiled library would: -I<repo>/src for every TU and
#    src/esp32s3/libprosdk.a inside the link group (build.extra_libs). -I<repo>/test/target/shared
#    gives both sketches the one copy of the fake core, the UART lines and the task helpers
#    (Arduino compiles only the sketch folder).
# 3. builds a 4 MB flash image with esptool merge-bin (bootloader, partition table, boot_app0,
#    app; the rest erased) in .build/qemu/flash.bin
# 4. writes the QEMU eFuse image (.build/qemu/efuse.bin, see below) and boots QEMU headless N
#    times (default 2) on the SAME flash image (NVS and LittleFS persist between boots), UART0
#    to .build/qemu/uart-<n>.log, until "QEMU-SMOKE END", a crash or the timeout; guest time is
#    instruction-count time (-icount, see below), independent of the host's load
# 5. parses the CASE lines: exit 0 only if every boot printed BEGIN/END with the expected boot
#    number and every case passed.
#
# --bench: the same for test/target/QemuBench (build folder .build/qemu/QemuBench, files
# bench-compile.log, bench-flash.bin, bench-uart-<n>.log; default 1 boot). Every boot starts from
# a freshly merged flash image, so boots are comparable. tools/qemu/bench.py then parses the BENCH
# lines into a table and .build/qemu/bench.json and, with 2 or more boots, compares the boots
# (instruction counts are deterministic under -icount; any difference fails). --check fails
# (exit 1) when a benchmark's instr_per_op grew by more than --threshold percent (default 10) over
# the committed baseline tools/qemu/bench-baseline.json; --save-baseline rewrites that baseline
# from this run, only when the run passed.
#
# Needs: src/esp32s3/libprosdk.a (tools/build/build.sh esp32s3), arduino-cli with
# esp32:esp32@3.3.10.

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$ROOT/.build/qemu"
# DIO: QEMU's SPI flash model does not take the QIO enable sequence (the default qio
# build asserts in esp_flash init); the flash image header is DIO either way
FQBN="esp32:esp32:esp32s3:PartitionScheme=huge_app,FlashMode=dio"
JOBS=2
build=1
boots=
timeout_s=300
bench=0
check=0
threshold=
save_baseline=0
usage() {
  echo "usage: $0 [--no-build] [--boots N] [--timeout S]" >&2
  echo "       $0 --bench [--no-build] [--boots N] [--timeout S] [--check] [--threshold PCT]" \
       "[--save-baseline]" >&2
  exit 2
}

while [ $# -gt 0 ]; do
  case "$1" in
    --no-build) build=0 ;;
    --boots) boots="$2"; shift ;;
    --timeout) timeout_s="$2"; shift ;;
    --bench) bench=1 ;;
    --check) check=1 ;;
    --threshold) threshold="$2"; shift ;;
    --save-baseline) save_baseline=1 ;;
    *) usage ;;
  esac
  shift
done
# --check, --threshold and --save-baseline belong to --bench; the smoke test has no baseline
if [ "$bench" = 0 ] && { [ "$check" = 1 ] || [ -n "$threshold" ] || [ "$save_baseline" = 1 ]; }; then
  usage
fi
threshold="${threshold:-10}"

if [ "$bench" = 1 ]; then
  NAME=QemuBench; PFX=bench-; MARK=QEMU-BENCH; boots="${boots:-1}"
else
  NAME=QemuSmoke; PFX=; MARK=QEMU-SMOKE; boots="${boots:-2}"
fi
SKETCH="$ROOT/test/target/$NAME"

mkdir -p "$OUT"
"$ROOT/tools/qemu/setup.sh" >/dev/null
QEMU="${QEMU_DIR:-$ROOT/tools/bin/qemu}/bin/qemu-system-xtensa"

ESP32_HW="${ARDUINO_ESP32_DIR:-$HOME/.arduino15/packages/esp32/hardware/esp32/3.3.10}"
ESPTOOL="$(ls -d "$HOME"/.arduino15/packages/esp32/tools/esptool_py/*/esptool 2>/dev/null | tail -1)"
[ -x "$ESPTOOL" ] || { echo "esptool not found under ~/.arduino15/packages/esp32/tools/esptool_py" >&2; exit 2; }

if [ "$build" = 1 ]; then
  [ -f "$ROOT/src/esp32s3/libprosdk.a" ] || {
    echo "src/esp32s3/libprosdk.a is missing: run tools/build/build.sh esp32s3 first" >&2; exit 2; }
  echo "== compile $NAME"
  flock "$ROOT/.build/build.lock" nice -n 19 ionice -c3 \
    arduino-cli compile --fqbn "$FQBN" \
      --build-property "compiler.cpp.extra_flags=-I$ROOT/src -I$ROOT/test/target/shared" \
      --build-property "build.extra_libs=$ROOT/src/esp32s3/libprosdk.a" \
      --build-path "$OUT/$NAME" --jobs "$JOBS" --warnings default "$SKETCH" \
      > "$OUT/${PFX}compile.log" 2>&1 || {
    grep -E "error|Error" "$OUT/${PFX}compile.log" | head -40 >&2
    echo "compile FAILED (full log: $OUT/${PFX}compile.log)" >&2
    exit 1
  }
  grep -E "^(Sketch uses|Global variables)" "$OUT/${PFX}compile.log"
fi

B="$OUT/$NAME/$NAME.ino"
for f in "$B.bootloader.bin" "$B.partitions.bin" "$B.bin"; do
  [ -f "$f" ] || { echo "missing $f (build first)" >&2; exit 2; }
done
FLASH="$OUT/${PFX}flash.bin"
merge_flash() {
  "$ESPTOOL" --chip esp32s3 merge-bin -o "$FLASH" --pad-to-size 4MB \
    --flash-mode keep --flash-freq keep --flash-size keep \
    0x0 "$B.bootloader.bin" 0x8000 "$B.partitions.bin" \
    0xe000 "$ESP32_HW/tools/partitions/boot_app0.bin" 0x10000 "$B.bin" | tail -1
}
echo "== flash image"
merge_flash

# eFuse image (QEMU only). QEMU's default eFuses are all zero; ESP-IDF's esp_adc constructor
# then finds no factory ADC calibration (BLK_VERSION_MAJOR 0) and self-calibrates on the SAR ADC,
# which QEMU does not emulate, so the app hangs before setup(). BLK_VERSION_MAJOR = 1 (eFuse BLK2
# bit 128, register 0x6000706C, file offset 0x40: file offset = register address - 0x6000702C) is
# what a production chip carries; the calibration values stay 0. A real chip is untouched by this.
python3 -c 'import sys; b = bytearray(1024); b[0x40] = 1; open(sys.argv[1], "wb").write(b)' \
  "$OUT/efuse.bin"

# Virtual time (QEMU only): -icount shift=2,sleep=off. Without it QEMU's virtual clock follows the
# host clock, so a guest vCPU thread that the loaded host does not schedule for 30 ms sees 30 ms
# pass between two instructions: periods are skipped and the timing checks of cases c and d
# measured host load (case d failed at load 10.5). With it the virtual clock advances 2^2 = 4 ns
# per executed guest instruction (about 250 MIPS, the order of a 240 MHz LX7) and, with
# sleep=off, jumps straight to the next timer deadline while both cores wait (waiti). Guest time
# no longer depends on the host: a boot pinned to one CPU next to seven busy loops took 235 s
# instead of 29 s and printed the same case c and d numbers. icount runs the two cores
# round-robin on one TCG thread (no MTTCG). Timing values in the CASE lines are therefore
# instruction-count time, not ESP32-S3 time (no caches, flash wait states or pipeline stalls).
ICOUNT=(-icount shift=2,sleep=off)

status=0
logs=()
for n in $(seq 1 "$boots"); do
  log="$OUT/${PFX}uart-$n.log"
  logs+=("$log")
  : > "$log"
  # the benchmark starts every boot from the same image (the smoke test keeps NVS and LittleFS)
  if [ "$bench" = 1 ] && [ "$n" -gt 1 ]; then merge_flash >/dev/null; fi
  echo "== QEMU boot $n (timeout ${timeout_s}s)"
  start=$(date +%s)
  # low priority like the builds: the emulator is CPU-bound, -icount keeps its timings unaffected
  nice -n 19 ionice -c3 "$QEMU" -nographic -machine esp32s3 -m 4M "${ICOUNT[@]}" \
    -drive "file=$FLASH,if=mtd,format=raw" \
    -drive "file=$OUT/efuse.bin,if=none,format=raw,id=efuse" \
    -global driver=nvram.esp32s3.efuse,property=drive,value=efuse \
    -serial "file:$log" -monitor none -no-reboot &
  qpid=$!
  result=timeout
  while kill -0 "$qpid" 2>/dev/null; do
    if grep -q "^$MARK END" "$log"; then result=end; break; fi
    if grep -qE "Guru Meditation|abort\(\) was called|stack overflow|Backtrace:|Rebooting\.\.\." "$log"; then
      sleep 2; result=crash; break
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout_s" ]; then break; fi
    sleep 1
  done
  kill "$qpid" 2>/dev/null || true
  wait "$qpid" 2>/dev/null || true
  echo "   $(( $(date +%s) - start )) s, $result"
  if [ "$bench" = 1 ]; then
    grep -E "^(QEMU-BENCH|CLOCK|BENCH-FAIL|BENCH-CHECK)" "$log" | sed 's/^/   /' || true
  else
    grep -E "^(QEMU-SMOKE|CASE|  CHECK FAILED|TASK|QEMU-PRINT)" "$log" | sed 's/^/   /' || true
  fi
  if [ "$result" != end ]; then
    echo "   boot $n: no END line ($result); last UART lines:"
    tail -n 30 "$log" | sed 's/^/   | /'
    status=1
    continue
  fi
  if [ "$bench" = 1 ]; then
    if grep -q "^BENCH-FAIL" "$log"; then status=1; fi
    grep -q "^QEMU-BENCH END boot=1 groups=[0-9]* failed=0" "$log" || status=1
    continue
  fi
  grep -q "^QEMU-SMOKE BEGIN boot=$n " "$log" || { echo "   boot $n: wrong boot number"; status=1; }
  if grep -q "^CASE .* FAIL" "$log"; then status=1; fi
  grep -q "^QEMU-SMOKE END boot=$n pass=[0-9]* fail=0" "$log" || status=1
done

if [ "$bench" = 1 ]; then
  args=(--json "$OUT/bench.json" --compile-log "$OUT/bench-compile.log")
  [ "$check" = 1 ] && args+=(--check "$ROOT/tools/qemu/bench-baseline.json" --threshold "$threshold")
  [ "$save_baseline" = 1 ] && args+=(--save-baseline "$ROOT/tools/qemu/bench-baseline.json")
  if [ "$status" = 0 ]; then
    python3 "$ROOT/tools/qemu/bench.py" "${args[@]}" "${logs[@]}" || status=1
  fi
  if [ "$status" = 0 ]; then echo "QEMU bench: PASS ($boots boots)"; else echo "QEMU bench: FAIL"; fi
  exit $status
fi

if [ "$status" = 0 ]; then echo "QEMU smoke: PASS ($boots boots)"; else echo "QEMU smoke: FAIL"; fi
exit $status
