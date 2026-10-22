// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/LinearPlantInversionFeedforward.hpp"

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk::math {

TEST_CASE("LinearPlantInversionFeedforwardTest Calculate", "[wpimath]") {
  Matrixd<2, 2> A{{1, 0}, {0, 1}};
  Matrixd<2, 1> B{0, 1};

  prosdk::math::LinearPlantInversionFeedforward<2, 1> feedforward{A, B, 20_ms};

  Vectord<2> r{2, 2};
  Vectord<2> nextR{3, 3};

  CHECK_NEAR(47.502599, feedforward.Calculate(r, nextR)(0, 0), 0.002);
}

}  // namespace prosdk::math
