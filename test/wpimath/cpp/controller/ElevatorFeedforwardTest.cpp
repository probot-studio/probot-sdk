// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/ElevatorFeedforward.hpp"

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/controller/LinearPlantInversionFeedforward.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"

static constexpr auto Ks = 0.5_V;
static constexpr auto Kv = 1.5_V * 1_s / 1_m;
static constexpr auto Ka = 2_V * 1_s * 1_s / 1_m;
static constexpr auto Kg = 1_V;

TEST_CASE("ElevatorFeedforwardTest Calculate", "[wpimath]") {
  prosdk::math::ElevatorFeedforward elevatorFF{Ks, Kg, Kv, Ka};

  CHECK_NEAR(elevatorFF.Calculate(0_m / 1_s).value(), Kg.value(), 0.002);
  CHECK_NEAR(elevatorFF.Calculate(2_m / 1_s).value(), 4.5, 0.002);

  prosdk::math::Matrixd<1, 1> A{-Kv.value() / Ka.value()};
  prosdk::math::Matrixd<1, 1> B{1.0 / Ka.value()};
  constexpr prosdk::units::second_t dt = 20_ms;
  prosdk::math::LinearPlantInversionFeedforward<1, 1> plantInversion{A, B, dt};

  prosdk::math::Vectord<1> r{2.0};
  prosdk::math::Vectord<1> nextR{3.0};
  CHECK_NEAR(plantInversion.Calculate(r, nextR)(0) + Ks.value() + Kg.value(),
             elevatorFF.Calculate(2_mps, 3_mps).value(), 0.002);
}

TEST_CASE("ElevatorFeedforwardTest AchievableVelocity", "[wpimath]") {
  prosdk::math::ElevatorFeedforward elevatorFF{Ks, Kg, Kv, Ka};
  CHECK_NEAR(elevatorFF.MaxAchievableVelocity(11_V, 1_m / 1_s / 1_s).value(), 5,
             0.002);
  CHECK_NEAR(elevatorFF.MinAchievableVelocity(11_V, 1_m / 1_s / 1_s).value(),
             -9, 0.002);
}

TEST_CASE("ElevatorFeedforwardTest AchievableAcceleration", "[wpimath]") {
  prosdk::math::ElevatorFeedforward elevatorFF{Ks, Kg, Kv, Ka};
  CHECK_NEAR(elevatorFF.MaxAchievableAcceleration(12_V, 2_m / 1_s).value(),
             3.75, 0.002);
  CHECK_NEAR(elevatorFF.MaxAchievableAcceleration(12_V, -2_m / 1_s).value(),
             7.25, 0.002);
  CHECK_NEAR(elevatorFF.MinAchievableAcceleration(12_V, 2_m / 1_s).value(),
             -8.25, 0.002);
  CHECK_NEAR(elevatorFF.MinAchievableAcceleration(12_V, -2_m / 1_s).value(),
             -4.75, 0.002);
}

TEST_CASE("ElevatorFeedforwardTest NegativeGains", "[wpimath]") {
  prosdk::math::ElevatorFeedforward elevatorFF{Ks, Kg, -Kv, -Ka};
  CHECK(elevatorFF.GetKv().value() == 0);
  CHECK(elevatorFF.GetKa().value() == 0);
}
