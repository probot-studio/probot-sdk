// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/Console.hpp"

#include <atomic>
#include <cstdio>

#include "prosdk/platform/config.hpp"

#if PROSDK_TARGET_HW
#include <freertos/FreeRTOS.h>
#endif

namespace prosdk::platform {

static void DefaultSink(std::string_view text) {
  std::fwrite(text.data(), 1, text.size(), stderr);
  std::fflush(stderr);
}

// One of the two is in use: a leveled sink when set, otherwise the text sink.
static std::atomic<LeveledConsoleSink> gLeveledSink{nullptr};
static std::atomic<ConsoleSink> gSink{DefaultSink};
static std::atomic<ConsoleSink> gTee{nullptr};
// a write from inside the tee (for example an error report while it runs) is not passed to the
// tee again
static thread_local bool tInTee = false;

void SetConsoleSink(ConsoleSink sink) noexcept {
  gSink.store(sink != nullptr ? sink : DefaultSink);
  gLeveledSink.store(nullptr);
}

void SetLeveledConsoleSink(LeveledConsoleSink sink) noexcept {
  gSink.store(DefaultSink);
  gLeveledSink.store(sink);
}

void SetConsoleTee(ConsoleSink tee) noexcept {
  gTee.store(tee);
}

void ConsoleWrite(std::string_view text, ConsoleLevel level) noexcept {
#if PROSDK_TARGET_HW
  // the sinks take locks and may allocate: never from an interrupt handler
  if (xPortInIsrContext()) {
    return;
  }
#endif
  try {
    if (LeveledConsoleSink leveled = gLeveledSink.load()) {
      leveled(level, text);
    } else {
      gSink.load()(text);
    }
  } catch (...) {
  }
  ConsoleSink tee = gTee.load();
  if (tee != nullptr && !tInTee) {
    tInTee = true;
    try {
      tee(text);
    } catch (...) {
    }
    tInTee = false;
  }
}

}  // namespace prosdk::platform
