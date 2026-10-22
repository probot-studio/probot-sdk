// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/SimpleMotorFeedforward.hpp"

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/controller/LinearPlantInversionFeedforward.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"

namespace prosdk::math {

TEST_CASE("SimpleMotorFeedforwardTest Calculate", "[wpimath]") {
  constexpr auto Ks = 0.5_V;
  constexpr auto Kv = 3_V / 1_mps;
  constexpr auto Ka = 0.6_V / 1_mps_sq;
  constexpr prosdk::units::second_t dt = 20_ms;

  constexpr Matrixd<1, 1> A{{-Kv.value() / Ka.value()}};
  constexpr Matrixd<1, 1> B{{1.0 / Ka.value()}};

  prosdk::math::LinearPlantInversionFeedforward<1, 1> plantInversion{A, B, dt};
  prosdk::math::SimpleMotorFeedforward<prosdk::units::meter> simpleMotor{Ks, Kv, Ka};

  constexpr Vectord<1> r{{2.0}};
  constexpr Vectord<1> nextR{{3.0}};

  CHECK_NEAR((37.524995834325161_V + Ks).value(),
             simpleMotor.Calculate(2_mps, 3_mps).value(), 0.002);
  CHECK_NEAR(plantInversion.Calculate(r, nextR)(0) + Ks.value(),
             simpleMotor.Calculate(2_mps, 3_mps).value(), 0.002);

  // These won't match exactly. It's just an approximation to make sure they're
  // in the same ballpark.
  CHECK_NEAR(plantInversion.Calculate(r, nextR)(0) + Ks.value(),
             simpleMotor.Calculate(2_mps, 3_mps).value(), 2.0);
}

TEST_CASE("SimpleMotorFeedforwardTest NegativeGains", "[wpimath]") {
  constexpr auto Ks = 0.5_V;
  constexpr auto Kv = -3_V / 1_mps;
  constexpr auto Ka = -0.6_V / 1_mps_sq;
  constexpr prosdk::units::second_t dt = 0_ms;
  prosdk::math::SimpleMotorFeedforward<prosdk::units::meter> simpleMotor{Ks, Kv, Ka,
                                                                   dt};
  CHECK(simpleMotor.GetKv().value() == 0);
  CHECK(simpleMotor.GetKa().value() == 0);
  CHECK(simpleMotor.GetDt().value() == 0.02);
}

}  // namespace prosdk::math
