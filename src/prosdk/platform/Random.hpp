// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Shared random number engine (design/PORT.md section 7, `<random>`).
//
// Upstream wpimath creates `std::random_device rd; std::mt19937 gen{rd()};` on the stack at every
// call (about 5 KB of stack on libstdc++). prosdk patches those sites to use RandomEngine instead:
// an empty handle that draws from one process-wide std::mt19937, seeded once from RandomSeed()
// (esp_random() on the ESP32, std::random_device on the host). Draws are serialized by a mutex,
// so the handle can be used from any thread.

#pragma once

#include <cstdint>

#include "prosdk/platform/config.hpp"

namespace prosdk::platform {

/** A seed from the platform entropy source (esp_random() on ESP32, std::random_device on host). */
std::uint32_t RandomSeed();

/**
 * UniformRandomBitGenerator over the shared engine. Constructing one is free; every handle draws
 * from the same std::mt19937, which is seeded once on first use.
 */
class RandomEngine {
 public:
  using result_type = std::uint32_t;

  static constexpr result_type min() { return 0; }
  static constexpr result_type max() { return 0xFFFFFFFFu; }

  /** Returns the next value of the shared engine. Thread-safe. */
  result_type operator()();
};

}  // namespace prosdk::platform
