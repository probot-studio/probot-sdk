// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/controller/BangBangController.hpp"

TEST_CASE("BangBangInputOutputTest ShouldOutput", "[wpimath]") {
  prosdk::math::BangBangController controller;

  CHECK_DOUBLE_EQ(controller.Calculate(0, 1), 1);
}

TEST_CASE("BangBangInputOutputTest ShouldNotOutput", "[wpimath]") {
  prosdk::math::BangBangController controller;

  CHECK_DOUBLE_EQ(controller.Calculate(1, 0), 0);
}
