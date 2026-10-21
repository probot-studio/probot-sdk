#!/usr/bin/env bash
# Install the optional build helpers without root:
#   ninja  -> python venv in tools/venv (pip install ninja)
#   ccache -> official static binary from GitHub releases, installed into tools/bin
# Both are optional: build.sh falls back to make and no ccache when they are missing.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CCACHE_VERSION="4.14.2"
CCACHE_TARBALL="ccache-${CCACHE_VERSION}-linux-x86_64-musl-static.tar.xz"
CCACHE_URL="https://github.com/ccache/ccache/releases/download/v${CCACHE_VERSION}/${CCACHE_TARBALL}"

status=0

if [ -x "$ROOT/tools/venv/bin/ninja" ]; then
  echo "ninja: present ($("$ROOT/tools/venv/bin/ninja" --version))"
else
  echo "ninja: creating venv in tools/venv"
  if python3 -m venv "$ROOT/tools/venv" && "$ROOT/tools/venv/bin/pip" install -q ninja; then
    echo "ninja: installed ($("$ROOT/tools/venv/bin/ninja" --version))"
  else
    echo "ninja: install FAILED (build.sh will use make)"; status=1
  fi
fi

if [ -x "$ROOT/tools/bin/ccache" ]; then
  echo "ccache: present ($("$ROOT/tools/bin/ccache" --version | head -1))"
else
  # Download into its own new, empty directory; never execute anything from inside it
  # except the single verified binary.
  dl="$(mktemp -d "${TMPDIR:-/tmp}/prosdk-ccache.XXXXXX")"
  echo "ccache: downloading $CCACHE_URL"
  if curl -fsSL --retry 2 -o "$dl/$CCACHE_TARBALL" "$CCACHE_URL" \
     && tar -C "$dl" -xJf "$dl/$CCACHE_TARBALL" \
     && [ -f "$dl/ccache-${CCACHE_VERSION}-linux-x86_64-musl-static/ccache" ] \
     && "$dl/ccache-${CCACHE_VERSION}-linux-x86_64-musl-static/ccache" --version >/dev/null; then
    mkdir -p "$ROOT/tools/bin"
    install -m 0755 "$dl/ccache-${CCACHE_VERSION}-linux-x86_64-musl-static/ccache" "$ROOT/tools/bin/ccache"
    echo "ccache: installed ($("$ROOT/tools/bin/ccache" --version | head -1))"
  else
    echo "ccache: install FAILED (build.sh will build without ccache)"; status=1
  fi
  rm -rf "$dl"
fi
exit $status
