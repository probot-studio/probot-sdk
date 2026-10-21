// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Stack trace capture for prosdk::util::GetStackTrace on the ESP32 target (design/PORT.md
// section 7): program counters from the Xtensa backtrace walker, never execinfo. The addresses
// can be decoded with xtensa-esp32s3-elf-addr2line against the sketch ELF.

#pragma once

#include <cstddef>
#include <cstdint>

namespace prosdk::platform {

/**
 * Stores up to maxFrames return addresses of the calling thread, innermost first, and returns
 * how many were stored. Returns 0 where capture is not supported (host build).
 */
std::size_t CaptureBacktrace(std::uintptr_t* pcs, std::size_t maxFrames) noexcept;

}  // namespace prosdk::platform
