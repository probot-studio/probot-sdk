#!/usr/bin/env bash
# prosdk build wrapper (design/PORT.md section 9).
#
#   tools/build/build.sh esp32s3 [targets...]          -> src/esp32s3/libprosdk.a (default target)
#   tools/build/build.sh esp32s3 header_check size
#   tools/build/build.sh host [targets...] [--test]    -> host libraries and tests; --test runs them
#
# Configures once into .build/<cfg> and builds incrementally. Every configure/build step runs
# under the shared build lock at the lowest CPU and I/O priority with at most 2 jobs:
#   flock .build/build.lock nice -n 19 ionice -c3 ...
# Uses ninja (tools/venv) and ccache (tools/bin) when present; tools/build/bootstrap.sh installs
# them.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
JOBS=2

cfg="${1:-}"
case "$cfg" in
  esp32s3|host) shift ;;
  *) echo "usage: $0 {esp32s3|host} [targets...] [--test]" >&2; exit 2 ;;
esac

run_tests=0
targets=()
for a in "$@"; do
  case "$a" in
    --test) run_tests=1 ;;
    *) targets+=("$a") ;;
  esac
done

mkdir -p "$ROOT/.build"
LOCK="$ROOT/.build/build.lock"
low() { flock "$LOCK" nice -n 19 ionice -c3 "$@"; }

BUILD="$ROOT/.build/$cfg"
gen=()
if [ -x "$ROOT/tools/venv/bin/ninja" ]; then
  gen=(-G Ninja -DCMAKE_MAKE_PROGRAM="$ROOT/tools/venv/bin/ninja")
elif command -v ninja >/dev/null 2>&1; then
  gen=(-G Ninja)
fi

if [ ! -f "$BUILD/CMakeCache.txt" ]; then
  args=(-S "$ROOT/tools/build" -B "$BUILD" "${gen[@]}" -DCMAKE_BUILD_TYPE=)
  if [ "$cfg" = esp32s3 ]; then
    args+=(-DCMAKE_TOOLCHAIN_FILE="$ROOT/tools/build/esp32s3.cmake")
  else
    args+=(-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++)
  fi
  low cmake "${args[@]}"
fi

if [ ${#targets[@]} -eq 0 ]; then
  if [ "$cfg" = esp32s3 ]; then targets=(prosdk); else targets=(all); fi
fi
low cmake --build "$BUILD" -j "$JOBS" --target "${targets[@]}"

if [ "$run_tests" = 1 ]; then
  [ "$cfg" = host ] || { echo "--test is only for the host build" >&2; exit 2; }
  status=0
  for t in prosdk_tests prosdk_dashboard_tests prosdk_robot_tests wpiutil_tests telemetry_tests tunables_tests hal_tests datalog_tests units_tests wpimath_tests fields_tests apriltag_tests wpilibc_tests commandsv2_tests prosdk_commands2_tests; do
    if [ -x "$BUILD/$t" ]; then
      echo "== $t"
      low "$BUILD/$t" || status=1
    fi
  done
  exit $status
fi
