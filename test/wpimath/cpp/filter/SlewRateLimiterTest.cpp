// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/filter/SlewRateLimiter.hpp"

#include <stdint.h>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/timestamp.h"

static prosdk::units::second_t now = 0_s;

class SlewRateLimiterTest {
 public:
  SlewRateLimiterTest() {
    WPI_SetNowImpl([] { return prosdk::units::nanosecond_t{now}.to<int64_t>(); });
  }

  ~SlewRateLimiterTest() { WPI_SetNowImpl(nullptr); }
};

TEST_CASE_METHOD(SlewRateLimiterTest, "SlewRateLimiterTest SlewRateLimit",
                 "[wpimath]") {
  WPI_SetNowImpl([] { return prosdk::units::nanosecond_t{now}.to<int64_t>(); });

  prosdk::math::SlewRateLimiter<prosdk::units::meters> limiter(1_mps);

  now += 1_s;

  CHECK(limiter.Calculate(2_m) < 2_m);
}

TEST_CASE_METHOD(SlewRateLimiterTest, "SlewRateLimiterTest SlewRateNoLimit",
                 "[wpimath]") {
  prosdk::math::SlewRateLimiter<prosdk::units::meters> limiter(1_mps);

  now += 1_s;

  CHECK(limiter.Calculate(0.5_m) == 0.5_m);
}

TEST_CASE_METHOD(SlewRateLimiterTest,
                 "SlewRateLimiterTest SlewRatePositiveNegativeLimit",
                 "[wpimath]") {
  prosdk::math::SlewRateLimiter<prosdk::units::meters> limiter(1_mps, -0.5_mps);

  now += 1_s;

  CHECK(limiter.Calculate(2_m) == 1_m);

  now += 1_s;

  CHECK(limiter.Calculate(0_m) == 0.5_m);
}
