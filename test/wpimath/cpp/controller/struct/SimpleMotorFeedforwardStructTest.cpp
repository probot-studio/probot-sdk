// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>

#include "../../StructTestBase.hpp"
#include "prosdk/math/controller/SimpleMotorFeedforward.hpp"

using namespace prosdk::math;

template <typename T>
struct SimpleMotorFeedforwardStructTestData {
  using Type = SimpleMotorFeedforward<T>;

  inline static const Type TEST_DATA = {
      prosdk::units::volt_t{0.4},
      prosdk::units::volt_t{4.0} / (prosdk::units::unit_t<T>{1} / 1_s),
      prosdk::units::volt_t{0.7} / (prosdk::units::unit_t<T>{1} / 1_s / 1_s), 25_ms};

  static void CheckEq(const Type& testData, const Type& data) {
    CHECK(testData.GetKs().value() == data.GetKs().value());
    CHECK(testData.GetKv().value() == data.GetKv().value());
    CHECK(testData.GetKa().value() == data.GetKa().value());
    CHECK(testData.GetDt().value() == data.GetDt().value());
  }
};

INSTANTIATE_CATCH_TYPED_TEST_SUITE_P(
    SimpleMotorFeedforwardMeters, StructTest,
    SimpleMotorFeedforwardStructTestData<prosdk::units::meters>);
INSTANTIATE_CATCH_TYPED_TEST_SUITE_P(
    SimpleMotorFeedforwardFeet, StructTest,
    SimpleMotorFeedforwardStructTestData<prosdk::units::feet>);
INSTANTIATE_CATCH_TYPED_TEST_SUITE_P(
    SimpleMotorFeedforwardRadians, StructTest,
    SimpleMotorFeedforwardStructTestData<prosdk::units::radians>);
