// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/filter/EdgeCounterFilter.hpp"

#include <stdint.h>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/units/time.hpp"
#include "prosdk/util/timestamp.h"

static prosdk::units::second_t now = 0_s;

class EdgeCounterFilterTest {
 protected:
  EdgeCounterFilterTest() {
    WPI_SetNowImpl([] { return prosdk::units::nanosecond_t{now}.to<int64_t>(); });
    now = 0_ms;
  }

  ~EdgeCounterFilterTest() {
    now = 0_ms;
    WPI_SetNowImpl(nullptr);
  }
};

TEST_CASE_METHOD(EdgeCounterFilterTest,
                 "EdgeCounterFilterTest EdgeCounterFilterActivated",
                 "[wpimath]") {
  prosdk::math::EdgeCounterFilter filter{2, 0.2_s};

  CHECK_FALSE(filter.Calculate(true));  // First edge

  now = 50_ms;
  CHECK_FALSE(filter.Calculate(false));  // First edge ended

  now = 100_ms;
  CHECK(filter.Calculate(true));  // Second edge

  now = 150_ms;
  CHECK(filter.Calculate(true));  // Still true

  now = 250_ms;
  CHECK(filter.Calculate(true));  // Still true

  now = 300_ms;
  CHECK_FALSE(filter.Calculate(false));  // Input false, should reset
}

TEST_CASE_METHOD(EdgeCounterFilterTest,
                 "EdgeCounterFilterTest EdgeCounterFilterExpired",
                 "[wpimath]") {
  prosdk::math::EdgeCounterFilter filter{2, 0.2_s};

  CHECK_FALSE(filter.Calculate(true));  // First edge

  now = 50_ms;
  filter.Calculate(false);  // First edge ended

  now = 250_ms;
  CHECK_FALSE(filter.Calculate(true));  // Second edge after window expired

  now = 300_ms;
  CHECK_FALSE(filter.Calculate(true));  // Still false
}

TEST_CASE_METHOD(EdgeCounterFilterTest,
                 "EdgeCounterFilterTest EdgeCounterFilterParams", "[wpimath]") {
  prosdk::math::EdgeCounterFilter filter{2, 0.2_s};

  CHECK(filter.GetRequiredEdges() == 2);
  CHECK(filter.GetWindowTime() == 0.2_s);

  filter.SetRequiredEdges(3);

  CHECK(filter.GetRequiredEdges() == 3);

  filter.SetWindowTime(0.5_s);

  CHECK(filter.GetWindowTime() == 0.5_s);
}
