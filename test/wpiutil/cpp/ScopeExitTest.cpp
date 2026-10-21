// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <utility>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_range_equals.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>

#include "prosdk/util/scope"

TEST_CASE("ScopeExitTest ScopeExit", "[wpiutil]") {
  int exitCount = 0;

  {
    prosdk::util::scope_exit exit{[&] { ++exitCount; }};

    CHECK(0 == exitCount);
  }

  CHECK(1 == exitCount);
}

TEST_CASE("ScopeExitTest Release", "[wpiutil]") {
  int exitCount = 0;

  {
    prosdk::util::scope_exit exit1{[&] { ++exitCount; }};
    prosdk::util::scope_exit exit2 = std::move(exit1);
    // NOLINTNEXTLINE(clang-analyzer-cplusplus.Move)
    prosdk::util::scope_exit exit3 = std::move(exit1);
    CHECK(0 == exitCount);
  }
  CHECK(1 == exitCount);

  {
    prosdk::util::scope_exit exit{[&] { ++exitCount; }};
    exit.release();
  }
  CHECK(1 == exitCount);
}
