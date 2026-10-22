// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/Random.hpp"

#include <mutex>
#include <random>

#if PROSDK_TARGET_HW
#include <esp_random.h>
#endif

namespace prosdk::platform {

std::uint32_t RandomSeed() {
#if PROSDK_TARGET_HW
  return esp_random();
#else
  std::random_device rd;
  return rd();
#endif
}

namespace {
struct SharedEngine {
  std::mutex mutex;
  std::mt19937 engine{RandomSeed()};
};

SharedEngine& GetSharedEngine() {
  static SharedEngine shared;
  return shared;
}
}  // namespace

RandomEngine::result_type RandomEngine::operator()() {
  auto& shared = GetSharedEngine();
  std::scoped_lock lock{shared.mutex};
  return static_cast<result_type>(shared.engine());
}

}  // namespace prosdk::platform
