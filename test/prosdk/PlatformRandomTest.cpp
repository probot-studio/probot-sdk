// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// prosdk::platform::RandomEngine (shared engine behind the patched wpimath <random> sites).

#include <cstdint>
#include <random>
#include <set>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/platform/Random.hpp"

static_assert(std::uniform_random_bit_generator<prosdk::platform::RandomEngine>);

TEST_CASE("RandomEngine handles share one engine", "[prosdk][platform]") {
  prosdk::platform::RandomEngine a;
  prosdk::platform::RandomEngine b;
  std::set<std::uint32_t> seen;
  for (int i = 0; i < 1000; ++i) {
    seen.insert(a());
    seen.insert(b());
  }
  // one shared mt19937 stream: 2000 draws are (practically) all distinct
  CHECK(seen.size() > 1990);
}

TEST_CASE("RandomEngine works with std distributions", "[prosdk][platform]") {
  prosdk::platform::RandomEngine gen;
  std::normal_distribution<double> normal{0.0, 1.0};
  double sum = 0.0;
  constexpr int kN = 20000;
  for (int i = 0; i < kN; ++i) {
    sum += normal(gen);
  }
  CHECK(sum / kN < 0.05);
  CHECK(sum / kN > -0.05);
}

TEST_CASE("RandomEngine is usable from several threads", "[prosdk][platform]") {
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([] {
      prosdk::platform::RandomEngine gen;
      for (int i = 0; i < 10000; ++i) {
        (void)gen();
      }
    });
  }
  for (auto& th : threads) {
    th.join();
  }
  SUCCEED();
}
