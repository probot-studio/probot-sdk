// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/Backtrace.hpp"

#include "prosdk/platform/config.hpp"

#if PROSDK_TARGET_HW
#include <esp_cpu_utils.h>
#include <esp_debug_helpers.h>
#endif

namespace prosdk::platform {

std::size_t CaptureBacktrace(std::uintptr_t* pcs, std::size_t maxFrames) noexcept {
#if PROSDK_TARGET_HW
  esp_backtrace_frame_t frame{};
  esp_backtrace_get_start(&frame.pc, &frame.sp, &frame.next_pc);
  std::size_t n = 0;
  // the first frame is this function; skip it
  while (n < maxFrames && esp_backtrace_get_next_frame(&frame)) {
    std::uintptr_t pc = esp_cpu_process_stack_pc(frame.pc);
    if (pc == 0) {
      break;
    }
    pcs[n++] = pc;
    if (frame.next_pc == 0) {
      break;
    }
  }
  return n;
#else
  (void)pcs;
  (void)maxFrames;
  return 0;
#endif
}

}  // namespace prosdk::platform
