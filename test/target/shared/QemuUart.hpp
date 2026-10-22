// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// UART0 line output of the QEMU sketches (QemuSmoke, QemuBench): one flushed line per call, which
// tools/qemu/run.sh parses. Header-only, shared through -I<repo>/test/target/shared.

#pragma once

#include <cstdio>
#include <format>
#include <string_view>
#include <utility>

namespace qemu {

/** Writes one line to UART0 (stdout) and flushes it. */
inline void Line(std::string_view text) {
  std::fwrite(text.data(), 1, text.size(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
}

template <class... Args>
void Linef(std::format_string<Args...> fmt, Args&&... args) {
  Line(std::format(fmt, std::forward<Args>(args)...));
}

}  // namespace qemu
