// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/estimator/MerweScaledSigmaPoints.hpp"

#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/linalg/EigenCore.hpp"

TEST_CASE("MerweScaledSigmaPointsTest ZeroMean", "[wpimath]") {
  prosdk::math::MerweScaledSigmaPoints<2> sigmaPoints;
  auto points = sigmaPoints.SquareRootSigmaPoints(
      prosdk::math::Vectord<2>{0.0, 0.0},
      prosdk::math::Matrixd<2, 2>{{1.0, 0.0}, {0.0, 1.0}});

  CHECK((points -
         prosdk::math::Matrixd<2, 5>{{0.0, 0.00173205, 0.0, -0.00173205, 0.0},
                                  {0.0, 0.0, 0.00173205, 0.0, -0.00173205}})
            .norm() < 1e-3);
}

TEST_CASE("MerweScaledSigmaPointsTest NonzeroMean", "[wpimath]") {
  prosdk::math::MerweScaledSigmaPoints<2> sigmaPoints;
  auto points = sigmaPoints.SquareRootSigmaPoints(
      prosdk::math::Vectord<2>{1.0, 2.0},
      prosdk::math::Matrixd<2, 2>{{1.0, 0.0}, {0.0, std::sqrt(10.0)}});

  CHECK(
      (points - prosdk::math::Matrixd<2, 5>{{1.0, 1.00173205, 1.0, 0.998268, 1.0},
                                         {2.0, 2.0, 2.00548, 2.0, 1.99452}})
          .norm() < 1e-3);
}
