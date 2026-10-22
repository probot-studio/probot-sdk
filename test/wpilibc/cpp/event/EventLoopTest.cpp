// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/event/EventLoop.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/system/Errors.hpp"

using namespace prosdk;

TEST_CASE("EventLoopTest ConcurrentModification", "[wpilibc][event]") {
  EventLoop loop;

  loop.Bind(
      [&loop] { REQUIRE_THROWS_AS(loop.Bind([] {}), prosdk::RuntimeError); });

  loop.Poll();

  loop.Clear();

  loop.Bind([&loop] { REQUIRE_THROWS_AS(loop.Clear(), prosdk::RuntimeError); });

  loop.Poll();
}
