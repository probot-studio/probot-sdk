// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// wpimath/0002: the default MathShared reports errors and warnings to the platform console.

#include <string>
#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/util/MathShared.hpp"
#include "prosdk/platform/Console.hpp"

namespace {
std::string gCaptured;
void Capture(std::string_view text) { gCaptured += text; }
}  // namespace

TEST_CASE("Default MathShared writes to the platform console", "[prosdk][wpimath]") {
  gCaptured.clear();
  prosdk::platform::SetConsoleSink(Capture);
  prosdk::math::MathSharedStore::ReportError("kV must be non-negative, got {}!", -1.5);
  prosdk::math::MathSharedStore::ReportWarning("kV defaulted to {}.", 0);
  prosdk::platform::SetConsoleSink(nullptr);
  CHECK(gCaptured == "Error: kV must be non-negative, got -1.5!\nWarning: kV defaulted to 0.\n");
}

TEST_CASE("Default MathShared timestamp advances", "[prosdk][wpimath]") {
  auto t0 = prosdk::math::MathSharedStore::GetTimestamp();
  auto t1 = prosdk::math::MathSharedStore::GetTimestamp();
  CHECK(t1 >= t0);
  CHECK(t0.value() > 0.0);
}
