// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/AntiTipping.hpp"

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/base.hpp"
#include "prosdk/units/velocity.hpp"

static constexpr double TOLERANCE = 1e-6;

// Shared constructor parameters used by all tests
static constexpr prosdk::units::unit_t<prosdk::math::AntiTipping::kp_unit> KP{0.1};
static constexpr prosdk::units::radian_t THRESHOLD = 3_deg;
static constexpr prosdk::units::meters_per_second_t MAX_SPEED = 2_mps;

TEST_CASE("AntiTippingTest BelowThresholdGeneratesNoCorrection", "[wpimath]") {
  prosdk::math::AntiTipping antiTipping{KP, THRESHOLD, MAX_SPEED};
  auto correction =
      antiTipping.Calculate(prosdk::math::Rotation3d{1_deg, 1_deg, 0_deg});

  CHECK_NEAR(0.0, correction.vx.value(), TOLERANCE);
  CHECK_NEAR(0.0, correction.vy.value(), TOLERANCE);
}

TEST_CASE("AntiTippingTest ForwardTipDrivesForward", "[wpimath]") {
  prosdk::math::AntiTipping antiTipping{KP, THRESHOLD, MAX_SPEED};
  auto correction =
      antiTipping.Calculate(prosdk::math::Rotation3d{0_deg, 10_deg, 0_deg});

  CHECK(correction.vx.value() > 0.0);
  CHECK_NEAR(0.0, correction.vy.value(), TOLERANCE);
}

TEST_CASE("AntiTippingTest BackwardTipDrivesBackward", "[wpimath]") {
  prosdk::math::AntiTipping antiTipping{KP, THRESHOLD, MAX_SPEED};
  auto correction =
      antiTipping.Calculate(prosdk::math::Rotation3d{0_deg, -10_deg, 0_deg});

  CHECK(correction.vx.value() < 0.0);
  CHECK_NEAR(0.0, correction.vy.value(), TOLERANCE);
}

TEST_CASE("AntiTippingTest RightRollDrivesRight", "[wpimath]") {
  prosdk::math::AntiTipping antiTipping{KP, THRESHOLD, MAX_SPEED};
  auto correction =
      antiTipping.Calculate(prosdk::math::Rotation3d{15_deg, 0_deg, 0_deg});

  CHECK_NEAR(0.0, correction.vx.value(), TOLERANCE);
  CHECK(correction.vy.value() < 0.0);
}

TEST_CASE("AntiTippingTest LeftRollDrivesLeft", "[wpimath]") {
  prosdk::math::AntiTipping antiTipping{KP, THRESHOLD, MAX_SPEED};
  auto correction =
      antiTipping.Calculate(prosdk::math::Rotation3d{-15_deg, 0_deg, 0_deg});

  CHECK_NEAR(0.0, correction.vx.value(), TOLERANCE);
  CHECK(correction.vy.value() > 0.0);
}
