// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Thread creation settings for every thread prosdk starts (design/PORT.md section 7).
//
// On ESP32 a std::thread gets the ESP-IDF pthread defaults (small stack, priority 5), which
// would preempt the probot-core control loop (priority 4). Every prosdk creation site wraps the
// std::thread construction in a ScopedThreadConfig, which applies an explicit stack size, a
// priority below core's loop and core 1 through esp_pthread_set_cfg. On the host it is a no-op.

#pragma once

#include <cstddef>
#include <memory>

#include "prosdk/platform/config.hpp"

// Defaults, overridable with -D (the precompiled archive uses these values).
#ifndef PROSDK_THREAD_STACK_SIZE
#define PROSDK_THREAD_STACK_SIZE 4096
#endif
#ifndef PROSDK_THREAD_PRIORITY
#define PROSDK_THREAD_PRIORITY 3
#endif
#ifndef PROSDK_THREAD_CORE
#define PROSDK_THREAD_CORE 1
#endif

namespace prosdk::platform {

/** Settings for one thread. Only used on the ESP32 target. */
struct ThreadConfig {
  /** Thread (FreeRTOS task) name; must outlive the thread's creation. nullptr keeps the default. */
  const char* name = nullptr;
  /** Stack size in bytes. */
  std::size_t stackSize = PROSDK_THREAD_STACK_SIZE;
  /** FreeRTOS priority; must stay below 4 (the probot-core control loop). */
  int priority = PROSDK_THREAD_PRIORITY;
  /** Core to pin the thread to (0 or 1), or -1 for no affinity. */
  int core = PROSDK_THREAD_CORE;
};

/**
 * Applies a ThreadConfig to the threads the calling thread creates while this object is alive
 * and restores the previous configuration afterwards. Create it immediately before constructing
 * a std::thread.
 */
class ScopedThreadConfig {
 public:
  explicit ScopedThreadConfig(const ThreadConfig& config = {});
  ~ScopedThreadConfig();
  ScopedThreadConfig(const ScopedThreadConfig&) = delete;
  ScopedThreadConfig& operator=(const ScopedThreadConfig&) = delete;

 private:
  struct Saved;
  std::unique_ptr<Saved> m_saved;
};

}  // namespace prosdk::platform
