// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/util/Synchronization.hpp"

#include <thread>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_range_equals.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>

TEST_CASE("EventTest AutoReset", "[wpiutil]") {
  auto event = prosdk::util::MakeEvent(false, false);
  std::thread thr([&] { prosdk::util::SetEvent(event); });
  prosdk::util::WaitForObject(event);
  thr.join();
  bool timedOut;
  prosdk::util::WaitForObject(event, 0, &timedOut);
  REQUIRE(timedOut == true);
}

TEST_CASE("EventTest ManualReset", "[wpiutil]") {
  auto event = prosdk::util::MakeEvent(true, false);
  int done = 0;
  std::thread thr([&] {
    prosdk::util::SetEvent(event);
    ++done;
  });
  prosdk::util::WaitForObject(event);
  thr.join();
  REQUIRE(done == 1);
  bool timedOut;
  prosdk::util::WaitForObject(event, 0, &timedOut);
  REQUIRE(timedOut == false);
}

TEST_CASE("EventTest InitialSet", "[wpiutil]") {
  auto event = prosdk::util::MakeEvent(false, true);
  bool timedOut;
  prosdk::util::WaitForObject(event, 0, &timedOut);
  REQUIRE(timedOut == false);
}

TEST_CASE("EventTest WaitMultiple", "[wpiutil]") {
  auto event1 = prosdk::util::MakeEvent(false, false);
  auto event2 = prosdk::util::MakeEvent(false, false);
  std::thread thr([&] { prosdk::util::SetEvent(event2); });
  WPI_Handle signaled[2];
  auto result1 = prosdk::util::WaitForObjects({event1, event2}, signaled);
  thr.join();
  REQUIRE(result1.size() == 1u);
  REQUIRE(result1[0] == event2);
  bool timedOut;
  auto result2 =
      prosdk::util::WaitForObjects({event1, event2}, signaled, 0, &timedOut);
  REQUIRE(timedOut == true);
  REQUIRE(result2.size() == 0u);
}
