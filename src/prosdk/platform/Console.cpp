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

void SetConsoleSink(ConsoleSink sink) noexcept {
  gSink.store(sink != nullptr ? sink : DefaultSink);
}

void ConsoleWrite(std::string_view text) noexcept {
  try {
    gSink.load()(text);
  } catch (...) {
  }
}

}  // namespace prosdk::platform
