// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/Console.hpp"

#include <atomic>
#include <cstdio>

namespace prosdk::platform {

static void DefaultSink(std::string_view text) {
  std::fwrite(text.data(), 1, text.size(), stderr);
  std::fflush(stderr);
}

static std::atomic<ConsoleSink> gSink{DefaultSink};
static std::atomic<ConsoleSink> gTee{nullptr};
// a write from inside the tee (for example an error report while it runs) is not passed to the
// tee again
static thread_local bool tInTee = false;

void SetConsoleSink(ConsoleSink sink) noexcept {
  gSink.store(sink != nullptr ? sink : DefaultSink);
}

void SetConsoleTee(ConsoleSink tee) noexcept {
  gTee.store(tee);
}

void ConsoleWrite(std::string_view text) noexcept {
  try {
    gSink.load()(text);
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
