// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/system/NumericalIntegration.hpp"

#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/units/time.hpp"

// Test that integrating dx/dt = eˣ works
TEST_CASE("NumericalIntegrationTest Exponential", "[wpimath]") {
  prosdk::math::Vectord<1> y0{0.0};

  prosdk::math::Vectord<1> y1 = prosdk::math::RK4(
      [](const prosdk::math::Vectord<1>& x) {
        return prosdk::math::Vectord<1>{std::exp(x(0))};
      },
      y0, 0.1_s);
  CHECK_NEAR(y1(0), std::exp(0.1) - std::exp(0), 1e-3);
}

// Test that integrating dx/dt = eˣ works when we provide a u
TEST_CASE("NumericalIntegrationTest ExponentialWithU", "[wpimath]") {
  prosdk::math::Vectord<1> y0{0.0};

  prosdk::math::Vectord<1> y1 = prosdk::math::RK4(
      [](const prosdk::math::Vectord<1>& x, const prosdk::math::Vectord<1>& u) {
        return prosdk::math::Vectord<1>{std::exp(u(0) * x(0))};
      },
      y0, prosdk::math::Vectord<1>{1.0}, 0.1_s);
  CHECK_NEAR(y1(0), std::exp(0.1) - std::exp(0), 1e-3);
}

// Tests RK4 with a time varying solution. From
// http://www2.hawaii.edu/~jmcfatri/math407/RungeKuttaTest.html:
//
//   dx/dt = x (2 / (eᵗ + 1) - 1)
//
// The true (analytical) solution is:
//
//   x(t) = 12eᵗ/(eᵗ + 1)²
TEST_CASE("NumericalIntegrationTest RK4TimeVarying", "[wpimath]") {
  prosdk::math::Vectord<1> y0{12.0 * std::exp(5.0) /
                           std::pow(std::exp(5.0) + 1.0, 2.0)};

  prosdk::math::Vectord<1> y1 = prosdk::math::RK4(
      [](prosdk::units::second_t t, const prosdk::math::Vectord<1>& x) {
        return prosdk::math::Vectord<1>{x(0) *
                                     (2.0 / (std::exp(t.value()) + 1.0) - 1.0)};
      },
      5_s, y0, 1_s);
  CHECK_NEAR(y1(0), 12.0 * std::exp(6.0) / std::pow(std::exp(6.0) + 1.0, 2.0),
             1e-3);
}

// Tests that integrating dx/dt = 0 works with RKDP
TEST_CASE("NumericalIntegrationTest ZeroRKDP", "[wpimath]") {
  prosdk::math::Vectord<1> y1 = prosdk::math::RKDP(
      [](const prosdk::math::Vectord<1>& x, const prosdk::math::Vectord<1>& u) {
        return prosdk::math::Vectord<1>::Zero();
      },
      prosdk::math::Vectord<1>{0.0}, prosdk::math::Vectord<1>{0.0}, 0.1_s);
  CHECK_NEAR(y1(0), 0.0, 1e-3);
}

// Tests that integrating dx/dt = eˣ works with RKDP
TEST_CASE("NumericalIntegrationTest ExponentialRKDP", "[wpimath]") {
  prosdk::math::Vectord<1> y0{0.0};

  prosdk::math::Vectord<1> y1 = prosdk::math::RKDP(
      [](const prosdk::math::Vectord<1>& x, const prosdk::math::Vectord<1>& u) {
        return prosdk::math::Vectord<1>{std::exp(x(0))};
      },
      y0, prosdk::math::Vectord<1>{0.0}, 0.1_s);
  CHECK_NEAR(y1(0), std::exp(0.1) - std::exp(0), 1e-3);
}

// Tests RKDP with a time varying solution. From
// http://www2.hawaii.edu/~jmcfatri/math407/RungeKuttaTest.html:
//
//   dx/dt = x(2/(eᵗ + 1) - 1)
//
// The true (analytical) solution is:
//
//   x(t) = 12eᵗ/(eᵗ + 1)²
TEST_CASE("NumericalIntegrationTest RKDPTimeVarying", "[wpimath]") {
  prosdk::math::Vectord<1> y0{12.0 * std::exp(5.0) /
                           std::pow(std::exp(5.0) + 1.0, 2.0)};

  prosdk::math::Vectord<1> y1 = prosdk::math::RKDP(
      [](prosdk::units::second_t t, const prosdk::math::Vectord<1>& x) {
        return prosdk::math::Vectord<1>{x(0) *
                                     (2.0 / (std::exp(t.value()) + 1.0) - 1.0)};
      },
      5_s, y0, 1_s, 1e-12);
  CHECK_NEAR(y1(0), 12.0 * std::exp(6.0) / std::pow(std::exp(6.0) + 1.0, 2.0),
             1e-3);
}
