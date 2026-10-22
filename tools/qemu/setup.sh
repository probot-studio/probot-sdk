#!/usr/bin/env bash
# Copyright (c) Probot Studio. Hand-written; see LICENSE.
#
# Installs Espressif's QEMU (qemu-system-xtensa with the esp32s3 machine) into tools/bin/qemu
# (gitignored). Pinned release + sha256; re-running with the same pin is a no-op.
#
#   tools/qemu/setup.sh          install (or verify) the pinned build
#   QEMU_DIR=<dir> tools/qemu/setup.sh   install somewhere else (CI cache)
#
# Release: https://github.com/espressif/qemu/releases/tag/esp-develop-9.2.2-20260417
# The tarball needs libgcrypt20, libglib2.0-0, libpixman-1-0, libsdl2-2.0-0 and libslirp0 on the
# host (Ubuntu package names).

set -euo pipefail

QEMU_TAG="esp-develop-9.2.2-20260417"
QEMU_ASSET="qemu-xtensa-softmmu-esp_develop_9.2.2_20260417-x86_64-linux-gnu.tar.xz"
QEMU_SHA256="0eecb2a34a5586c0e59110f77b9343b7b336e82fdb0e1a30e1dc1bab8a547e35"
QEMU_URL="https://github.com/espressif/qemu/releases/download/${QEMU_TAG}/${QEMU_ASSET}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
QEMU_DIR="${QEMU_DIR:-${ROOT}/tools/bin/qemu}"
STAMP="${QEMU_DIR}/.installed"
BIN="${QEMU_DIR}/bin/qemu-system-xtensa"

if [[ "$(uname -s)-$(uname -m)" != "Linux-x86_64" ]]; then
  echo "setup.sh: only the Linux x86_64 build is pinned (host is $(uname -s)-$(uname -m))" >&2
  exit 1
fi

if [[ -x "${BIN}" && -f "${STAMP}" && "$(cat "${STAMP}")" == "${QEMU_SHA256}" ]]; then
  echo "qemu: ${QEMU_TAG} already in ${QEMU_DIR}"
else
  tmp="$(mktemp -d)"
  trap 'rm -rf "${tmp}"' EXIT
  echo "qemu: downloading ${QEMU_URL}"
  curl -fsSL --retry 3 -o "${tmp}/${QEMU_ASSET}" "${QEMU_URL}"
  echo "${QEMU_SHA256}  ${tmp}/${QEMU_ASSET}" | sha256sum -c --quiet -
  rm -rf "${QEMU_DIR}"
  mkdir -p "${QEMU_DIR}"
  # the tarball has one top-level directory (qemu/)
  tar -xJf "${tmp}/${QEMU_ASSET}" -C "${QEMU_DIR}" --strip-components=1
  echo "${QEMU_SHA256}" > "${STAMP}"
fi

if ! "${BIN}" -machine help | grep -qw esp32s3; then
  echo "qemu: ${BIN} has no esp32s3 machine" >&2
  "${BIN}" -machine help >&2 || true
  exit 1
fi
"${BIN}" --version | head -1
echo "qemu: esp32s3 machine available"
