#!/usr/bin/env bash
# Builds the prosdk release zip for Library Manager (design/audit-final.md section 6.3, item 16).
#
#   tools/release/package.sh [--no-smoke]
#
# 1. Brings src/esp32s3/libprosdk.a up to date with the incremental build
#    (tools/build/build.sh esp32s3 prosdk; no clean rebuild, ninja does nothing when it is current).
# 2. Stages the tracked tree at HEAD (`git archive`) plus that archive in
#    .build/package/prosdk-<version>/. Library Manager serves a clone of the release tag, so the
#    zip has the tag's content only if the archive is committed in the tag (it is gitignored today;
#    release layout: audit-final.md section 6.3, item 15).
# 3. Checks library.properties for a precompiled library (name, precompiled=full, architectures,
#    includes, depends on probot-core's library name) and the archive's place src/{build.mcu}.
# 4. Writes .build/package/prosdk-<version>.zip and prints the sizes.
# 5. Smoke test (skipped with --no-smoke): unpacks the zip into .build/package/smoke/install/prosdk
#    and compiles its examples/TemplateTimed against it with tools/build/build.sh examples
#    (probot-core from $PROBOT_CORE, default .upstream/probot-core), then checks that arduino-cli
#    linked the unpacked library's precompiled archive.
#
# Publishes nothing: no tag, no GitHub release, no Library Manager request.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
smoke=1
case "${1:-}" in
  "") ;;
  --no-smoke) smoke=0 ;;
  *) echo "usage: $0 [--no-smoke]" >&2; exit 2 ;;
esac

prop() { sed -n "s/^$1=//p" "$ROOT/library.properties"; }
fail() { echo "package: $*" >&2; exit 1; }

version="$(prop version)"
[ -n "$version" ] || fail "library.properties has no version"
core="$(cd "${PROBOT_CORE:-$ROOT/.upstream/probot-core}" && pwd)"
core_name="$(sed -n 's/^name=//p' "$core/library.properties")"

# library.properties (https://arduino.github.io/arduino-cli/latest/library-specification/)
[ "$(prop name)" = prosdk ] || fail "name is not prosdk"
[ "$(prop precompiled)" = full ] || fail "precompiled is not full"
[[ ",$(prop architectures)," == *,esp32,* ]] || fail "architectures does not list esp32"
[ -f "$ROOT/src/$(prop includes)" ] || fail "includes=$(prop includes) is not in src/"
dep="$(prop depends)"
[ "${dep%% (*}" = "$core_name" ] || fail "depends=$dep, but probot-core's library name is $core_name"

if [ -n "$(git -C "$ROOT" status --porcelain --untracked-files=no)" ]; then
  echo "package: warning: tracked files differ from HEAD; the zip holds HEAD, the archive the" \
       "working tree" >&2
fi

"$ROOT/tools/build/build.sh" esp32s3 prosdk
archive="$ROOT/src/esp32s3/libprosdk.a"

out="$ROOT/.build/package"
stage="$out/prosdk-$version"
zipfile="$out/prosdk-$version.zip"
rm -rf "$stage" "$zipfile" "$out/smoke/install"
mkdir -p "$stage/src/esp32s3"
git -C "$ROOT" archive --format=tar HEAD | tar -x -C "$stage"
cp "$archive" "$stage/src/esp32s3/libprosdk.a"
(cd "$out" && nice -n 19 zip -q -r -X "$zipfile" "prosdk-$version")

echo "commit:  $(git -C "$ROOT" rev-parse --short HEAD)"
echo "archive: $(stat -c %s "$archive") bytes (src/esp32s3/libprosdk.a, debug-stripped)"
echo "tree:    $(du -sb "$stage" | cut -f1) bytes unpacked"
echo "zip:     $(stat -c %s "$zipfile") bytes ($zipfile)"

[ "$smoke" = 1 ] || exit 0
lib="$out/smoke/install/prosdk"
mkdir -p "$out/smoke/install"
unzip -q "$zipfile" -d "$out/smoke/install"
mv "$out/smoke/install/prosdk-$version" "$lib"
log="$out/smoke/TemplateTimed.log"
# build.sh links $lib as .build/package/smoke/build/libraries/prosdk
PROSDK_LIBRARY="$lib" PROSDK_EXAMPLES_OUT="$out/smoke/build" PROBOT_CORE="$core" \
  "$ROOT/tools/build/build.sh" examples TemplateTimed 2>&1 | tee "$log"
# arduino-cli names the folder of the precompiled archive it linked: the unpacked zip's
grep -qF "Using precompiled library in $out/smoke/build/libraries/prosdk/src/esp32s3" "$log" ||
  fail "arduino-cli did not link the unpacked library's archive (see $log)"
echo "smoke:   TemplateTimed compiled against the unpacked zip ($log)"
